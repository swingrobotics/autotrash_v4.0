"""Offline model preview against synchronized RECORD session inputs.

Unlike the arbitrary-video preview, this path replays the camera, LiDAR, IMU
and (for AUTO_GPS) GNSS/route context captured in a real RECORD session. It is
strictly diagnostic: inference results are rendered to a new video/CSV and
never receive vehicle control authority.
"""

from __future__ import annotations

import bisect
import csv
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
from statistics import fmean, median

from autonomous_car.recording.log_replay import LogReplay
from autonomous_car.routes import GpsRouteFeatureExtractor, NormalizedGpsRoute

from .gps_runtime import GpsAiRuntime
from .runtime import AutoAiRuntime
from .video_preview import command_path_points


RECORD_PREVIEW_AUTHORITY = "NONE"
MAXIMUM_LIDAR_SKEW_SECONDS = 0.20
MAXIMUM_IMU_SKEW_SECONDS = 0.12
MAXIMUM_GNSS_SKEW_SECONDS = 0.20


@dataclass(frozen=True)
class RecordPreviewSummary:
    session: str
    policy_type: str
    route_id: str | None
    output_video: str
    output_csv: str
    source_frames: int
    inferred_frames: int
    skipped_frames: int
    source_fps: float
    mean_abs_steering_error_degrees: float | None
    maximum_abs_steering_error_degrees: float | None
    mean_abs_throttle_error: float | None
    lidar_sync_ratio: float
    imu_sync_ratio: float
    gnss_sync_ratio: float | None
    control_authority: str = RECORD_PREVIEW_AUTHORITY

    def as_dict(self):
        return asdict(self)


def _number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _nearest(rows, timestamps, timestamp):
    if not rows:
        return None, math.inf
    position = bisect.bisect_left(timestamps, timestamp)
    candidates = []
    if position < len(rows):
        candidates.append(position)
    if position > 0:
        candidates.append(position - 1)
    if not candidates:
        return None, math.inf
    index = min(candidates, key=lambda item: abs(timestamps[item] - timestamp))
    return rows[index], abs(timestamps[index] - timestamp)


def _timed_rows(rows):
    result = []
    for row in rows or []:
        timestamp = _number(row.get("monotonic"))
        if timestamp is None:
            continue
        result.append((timestamp, row))
    result.sort(key=lambda item: item[0])
    return [row for _, row in result], [timestamp for timestamp, _ in result]


def _camera_rows(session_path):
    path = Path(session_path) / "camera_timestamps.csv"
    if not path.is_file():
        raise FileNotFoundError(f"camera_timestamps.csv not found: {session_path}")
    values = []
    with open(path, "r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            timestamp = _number(row.get("monotonic"))
            frame_number = _number(row.get("frame_number"))
            if timestamp is None or frame_number is None or frame_number <= 0:
                continue
            row["_timestamp"] = timestamp
            row["_frame_number"] = int(frame_number)
            values.append(row)
    values.sort(key=lambda row: row["_timestamp"])
    if not values:
        raise ValueError("RECORD_CAMERA_TIMESTAMPS_EMPTY")
    return values


def _fps_from_rows(rows):
    deltas = [
        rows[index]["_timestamp"] - rows[index - 1]["_timestamp"]
        for index in range(1, len(rows))
        if rows[index]["_timestamp"] > rows[index - 1]["_timestamp"]
    ]
    if not deltas:
        return 10.0
    delta = median(deltas)
    if not math.isfinite(delta) or delta <= 1e-6:
        return 10.0
    return max(1.0, min(60.0, 1.0 / delta))


def _safe_saved_frame(session_path, filename):
    raw = str(filename or "").strip().replace("\\", "/")
    if not raw or raw.startswith("/") or "\x00" in raw:
        return None
    relative = raw if raw.startswith("camera_frames/") else "camera_frames/" + raw
    parts = relative.split("/")
    if any(part in {"", ".", ".."} for part in parts):
        return None
    root = os.path.realpath(session_path)
    candidate = os.path.realpath(os.path.join(root, *parts))
    try:
        if os.path.commonpath([root, candidate]) != root:
            return None
    except ValueError:
        return None
    return candidate if os.path.isfile(candidate) else None


def _human_labels(row):
    target = _number(row.get("target_steering_angle_degrees"))
    actual = _number(row.get("steering_angle_degrees"))
    requested = _number(row.get("requested_throttle"))
    final = _number(row.get("final_throttle"))
    return (
        target if target is not None else actual,
        actual,
        requested if requested is not None else final,
    )


def _manifest(model_path, manifest_path):
    resolved = os.path.abspath(
        manifest_path
        or os.path.join(os.path.dirname(os.path.abspath(model_path)), "model_manifest.json")
    )
    with open(resolved, "r", encoding="utf-8") as handle:
        document = json.load(handle)
    if not isinstance(document, dict):
        raise ValueError("MODEL_MANIFEST_OBJECT_REQUIRED")
    policy = str(document.get("policy_type") or "AUTO_AI").strip().upper()
    if policy not in {"AUTO_AI", "AUTO_GPS"}:
        raise ValueError(f"RECORD_PREVIEW_POLICY_UNSUPPORTED:{policy}")
    return resolved, document, policy


def _resolve_route_path(route_path, route_id):
    if route_path:
        resolved = os.path.abspath(str(route_path))
        if not os.path.isfile(resolved):
            raise FileNotFoundError(resolved)
        return resolved
    if not route_id:
        raise ValueError("AUTO_GPS_RECORD_PREVIEW_ROUTE_REQUIRED")
    roots = [
        os.environ.get("AUTONOMY_GPS_ROUTES_PATH"),
        os.path.join(os.getcwd(), "gps-routes"),
    ]
    for root in roots:
        if not root:
            continue
        candidate = os.path.abspath(os.path.join(root, f"{route_id}.json"))
        if os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(
        f"AUTO_GPS route {route_id!r} not found; pass route_path explicitly"
    )


def _draw_path(frame, normalized_steering, cv2, color, *, dashed=False):
    height, width = frame.shape[:2]
    points = command_path_points(width, height, normalized_steering)
    for index, (first, second) in enumerate(zip(points, points[1:])):
        if dashed and index % 2:
            continue
        cv2.line(
            frame,
            (int(round(first[0])), int(round(first[1]))),
            (int(round(second[0])), int(round(second[1]))),
            color,
            max(2, width // 420),
            cv2.LINE_AA,
        )


def _draw_record_preview(
    frame,
    inference,
    cv2,
    *,
    policy_type,
    human_steering,
    actual_steering,
    human_throttle,
    maximum_steering_degrees,
    status,
):
    height, width = frame.shape[:2]
    if inference is not None:
        # Orange/yellow: model prediction.
        _draw_path(frame, inference.normalized_steering, cv2, (60, 210, 255))
    if human_steering is not None:
        # Green dashed: human target command used as the imitation label.
        normalized = max(
            -1.0,
            min(1.0, human_steering / max(1e-6, maximum_steering_degrees)),
        )
        _draw_path(frame, normalized, cv2, (110, 235, 110), dashed=True)
    if actual_steering is not None:
        # Cyan: actual encoder-measured wheel angle. New temporal models consume
        # its history rather than feeding their own predictions back as state.
        normalized_actual = max(
            -1.0,
            min(1.0, actual_steering / max(1e-6, maximum_steering_degrees)),
        )
        _draw_path(frame, normalized_actual, cv2, (235, 220, 70))

    model_text = (
        f"MODEL steer {inference.steering_degrees:+.1f} deg / throttle {inference.throttle:+.3f}"
        if inference is not None
        else f"MODEL skipped / {status}"
    )
    human_text = (
        f"HUMAN target {human_steering:+.1f} deg / throttle {human_throttle:+.3f}"
        if human_steering is not None and human_throttle is not None
        else "HUMAN label unavailable"
    )
    actual_text = (
        f"ACTUAL encoder {actual_steering:+.1f} deg"
        if actual_steering is not None
        else "ACTUAL encoder unavailable"
    )
    cv2.putText(
        frame,
        model_text,
        (18, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.56,
        (230, 245, 230),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        human_text,
        (18, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (210, 240, 210),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        actual_text,
        (18, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.50,
        (230, 230, 190),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"{policy_type} RECORD REPLAY / synchronized recorded sensors / CONTROL NONE",
        (18, max(22, height - 18)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.40,
        (180, 215, 255),
        1,
        cv2.LINE_AA,
    )
    return frame


def preview_record_session(
    session_path,
    model_path,
    manifest_path=None,
    *,
    route_path=None,
    output_video=None,
    output_csv=None,
    sample_every=1,
    jpeg_quality=92,
    progress_callback=None,
    cancelled=None,
):
    """Render model-vs-human preview from one stored RECORD session.

    AUTO_AI reuses recorded Camera + LiDAR + IMU. AUTO_GPS additionally replays
    the recorded GNSS position and IMU heading against the normalized route
    bound to the selected model. Measured-steering temporal GPS models receive
    the recorded encoder steering angle as their previous-state signal.

    ``progress_callback(done, total)`` is optional and is called after each
    source camera row. ``cancelled()`` is optional and is checked between frames
    so a Worker preview job can stop without waiting for the full recording.
    """

    try:
        import cv2
    except ImportError as error:  # pragma: no cover
        raise RuntimeError("OpenCV is required for RECORD model preview") from error

    session_path = os.path.abspath(str(session_path))
    if not os.path.isdir(session_path):
        raise FileNotFoundError(session_path)
    session_name = os.path.basename(session_path.rstrip(os.sep))
    manifest_path, manifest, policy_type = _manifest(model_path, manifest_path)
    route_id = str(manifest.get("route_id") or "").strip() or None

    if policy_type == "AUTO_GPS":
        resolved_route = _resolve_route_path(route_path, route_id)
        route = NormalizedGpsRoute.load(resolved_route)
        if route_id and route.route_id != route_id:
            raise ValueError(
                f"AUTO_GPS_RECORD_PREVIEW_ROUTE_MISMATCH:model={route_id},route={route.route_id}"
            )
        runtime = GpsAiRuntime(model_path, manifest_path=manifest_path)
        route_extractor = GpsRouteFeatureExtractor(route)
    else:
        runtime = AutoAiRuntime(model_path, manifest_path=manifest_path)
        route_extractor = None

    replay = LogReplay(session_path)
    camera_rows = _camera_rows(session_path)
    source_fps = _fps_from_rows(camera_rows)
    sample_every = max(1, int(sample_every))
    if policy_type == "AUTO_GPS" and getattr(runtime, "requires_measured_steering", False):
        # Temporal history is defined at the normal camera cadence. Skipping
        # frames would silently change the meaning of the 0.5 s feedback window.
        sample_every = 1
    jpeg_quality = max(70, min(100, int(jpeg_quality)))

    imu_rows, imu_times = _timed_rows(replay.streams.get("imu") or [])
    gnss_rows, gnss_times = _timed_rows(replay.streams.get("gnss") or [])
    lidar_rows = []
    lidar_times = []
    for timestamp, document in replay.iter_lidar_raw() or ():
        points = document.get("safety_points")
        source = "safety_points"
        if points is None:
            points = document.get("points") or []
            source = "legacy_raw_points"
        lidar_times.append(float(timestamp))
        lidar_rows.append({"points": points or [], "source": source})

    video_path = os.path.join(session_path, "camera.mp4")
    source_video = cv2.VideoCapture(video_path) if os.path.isfile(video_path) else None
    if source_video is not None and not source_video.isOpened():
        source_video.release()
        source_video = None

    stem = os.path.join(os.path.dirname(session_path), session_name + ".record-preview")
    output_video = os.path.abspath(str(output_video or (stem + ".mp4")))
    output_csv = os.path.abspath(str(output_csv or (stem + ".csv")))
    Path(output_video).parent.mkdir(parents=True, exist_ok=True)
    Path(output_csv).parent.mkdir(parents=True, exist_ok=True)

    writer = None
    rows = []
    inferred = 0
    skipped = 0
    lidar_synced = 0
    imu_synced = 0
    gnss_synced = 0
    steering_errors = []
    throttle_errors = []
    last_inference = None
    encode_parameters = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
    total_camera_rows = len(camera_rows)

    def report(done):
        if callable(progress_callback):
            progress_callback(int(done), int(total_camera_rows))

    def require_not_cancelled():
        if callable(cancelled) and cancelled():
            raise RuntimeError("JOB_CANCELLED")

    try:
        for source_index, camera_row in enumerate(camera_rows):
            require_not_cancelled()
            timestamp = float(camera_row["_timestamp"])
            frame_number = int(camera_row["_frame_number"])
            saved = _safe_saved_frame(session_path, camera_row.get("filename"))
            frame = cv2.imread(saved) if saved else None
            if frame is None and source_video is not None:
                source_video.set(cv2.CAP_PROP_POS_FRAMES, max(0, frame_number - 1))
                ok, frame = source_video.read()
                if not ok:
                    frame = None
            if frame is None:
                last_inference = None
                if policy_type == "AUTO_GPS" and hasattr(runtime, "reset_temporal_state"):
                    runtime.reset_temporal_state()
                skipped += 1
                rows.append(
                    {
                        "frame_index": frame_number - 1,
                        "timestamp_monotonic": timestamp,
                        "time_seconds": timestamp - camera_rows[0]["_timestamp"],
                        "status": "CAMERA_FRAME_UNAVAILABLE",
                        "policy_type": policy_type,
                        "control_authority": RECORD_PREVIEW_AUTHORITY,
                    }
                )
                report(source_index + 1)
                continue

            if writer is None:
                height, width = frame.shape[:2]
                writer = cv2.VideoWriter(
                    output_video,
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    source_fps,
                    (width, height),
                )
                if not writer.isOpened():
                    raise OSError(f"Could not create preview video: {output_video}")

            lidar_row, lidar_skew = _nearest(lidar_rows, lidar_times, timestamp)
            lidar_ok = lidar_row is not None and lidar_skew <= MAXIMUM_LIDAR_SKEW_SECONDS
            if lidar_ok:
                lidar_synced += 1
            imu_row, imu_skew = _nearest(imu_rows, imu_times, timestamp)
            imu_ok = imu_row is not None and imu_skew <= MAXIMUM_IMU_SKEW_SECONDS
            if imu_ok:
                imu_synced += 1
            imu_yaw_rate = _number((imu_row or {}).get("yaw_rate_dps")) if imu_ok else None
            human_steering, actual_steering, human_throttle = _human_labels(camera_row)

            status = "OK"
            route_features = None
            gnss_skew = None
            if not lidar_ok:
                status = "LIDAR_NOT_SYNCHRONIZED"
            elif policy_type == "AUTO_GPS":
                gnss_row, gnss_skew = _nearest(gnss_rows, gnss_times, timestamp)
                gnss_ok = gnss_row is not None and gnss_skew <= MAXIMUM_GNSS_SKEW_SECONDS
                if gnss_ok:
                    gnss_synced += 1
                if not gnss_ok:
                    status = "GNSS_NOT_SYNCHRONIZED"
                elif not imu_ok:
                    status = "GPS_HEADING_IMU_NOT_SYNCHRONIZED"
                else:
                    latitude = _number(gnss_row.get("latitude"))
                    longitude = _number(gnss_row.get("longitude"))
                    heading = _number(imu_row.get("yaw_degrees"))
                    if heading is None:
                        heading = _number(imu_row.get("global_heading_degrees"))
                    if latitude is None or longitude is None:
                        status = "GNSS_POSITION_MISSING"
                    elif heading is None:
                        status = "GPS_HEADING_MISSING"
                    else:
                        # Keep preview feature construction identical to GPS
                        # training: each sample performs an unrestricted route
                        # projection rather than inheriting temporal index state.
                        route_features = route_extractor.extract(
                            latitude,
                            longitude,
                            heading,
                        )

            if status != "OK":
                # Never carry a command or temporal state across a sensor sync
                # gap. The next valid frame starts a fresh measured-state window.
                last_inference = None
                if policy_type == "AUTO_GPS" and hasattr(runtime, "reset_temporal_state"):
                    runtime.reset_temporal_state()

            should_infer = status == "OK" and (
                last_inference is None or source_index % sample_every == 0
            )
            inference = last_inference if status == "OK" else None
            reused = False
            if should_infer:
                encoded_ok, encoded = cv2.imencode(".jpg", frame, encode_parameters)
                if not encoded_ok:
                    status = "JPEG_ENCODE_FAILED"
                    inference = None
                    last_inference = None
                    if policy_type == "AUTO_GPS" and hasattr(runtime, "reset_temporal_state"):
                        runtime.reset_temporal_state()
                elif policy_type == "AUTO_GPS":
                    inference = runtime.infer_jpeg(
                        encoded.tobytes(),
                        lidar_row["points"],
                        imu_yaw_rate,
                        route_features.as_dict(),
                        person_hazard=False,
                        measured_steering_degrees=actual_steering,
                    )
                else:
                    inference = runtime.infer_jpeg(
                        encoded.tobytes(),
                        lidar_row["points"],
                        imu_yaw_rate,
                        person_hazard=False,
                    )
                if inference is not None:
                    last_inference = inference
                    inferred += 1
            elif inference is not None:
                reused = True

            if inference is None:
                skipped += 1
            steering_error = None
            throttle_error = None
            if inference is not None and human_steering is not None:
                steering_error = abs(float(inference.steering_degrees) - human_steering)
                steering_errors.append(steering_error)
            if inference is not None and human_throttle is not None:
                throttle_error = abs(float(inference.throttle) - human_throttle)
                throttle_errors.append(throttle_error)

            rendered = _draw_record_preview(
                frame.copy(),
                inference,
                cv2,
                policy_type=policy_type,
                human_steering=human_steering,
                actual_steering=actual_steering,
                human_throttle=human_throttle,
                maximum_steering_degrees=runtime.maximum_steering_degrees,
                status=status,
            )
            writer.write(rendered)

            rows.append(
                {
                    "frame_index": frame_number - 1,
                    "timestamp_monotonic": timestamp,
                    "time_seconds": timestamp - camera_rows[0]["_timestamp"],
                    "status": status,
                    "policy_type": policy_type,
                    "route_id": route_id,
                    "model_steering_degrees": (
                        None if inference is None else float(inference.steering_degrees)
                    ),
                    "human_steering_degrees": human_steering,
                    "actual_steering_degrees": actual_steering,
                    "steering_abs_error_degrees": steering_error,
                    "model_throttle": (
                        None if inference is None else float(inference.throttle)
                    ),
                    "human_throttle": human_throttle,
                    "throttle_abs_error": throttle_error,
                    "inference_seconds": (
                        None if inference is None else float(inference.inference_seconds)
                    ),
                    "inference_reused": reused,
                    "lidar_skew_seconds": (
                        None if not math.isfinite(lidar_skew) else lidar_skew
                    ),
                    "imu_skew_seconds": (
                        None if not math.isfinite(imu_skew) else imu_skew
                    ),
                    "gnss_skew_seconds": (
                        None
                        if gnss_skew is None or not math.isfinite(gnss_skew)
                        else gnss_skew
                    ),
                    "lidar_source": (
                        None if lidar_row is None else lidar_row.get("source")
                    ),
                    "control_authority": RECORD_PREVIEW_AUTHORITY,
                }
            )
            report(source_index + 1)
    finally:
        if source_video is not None:
            source_video.release()
        if writer is not None:
            writer.release()

    if writer is None:
        raise ValueError("RECORD_PREVIEW_NO_DECODABLE_CAMERA_FRAMES")

    fieldnames = [
        "frame_index",
        "timestamp_monotonic",
        "time_seconds",
        "status",
        "policy_type",
        "route_id",
        "model_steering_degrees",
        "human_steering_degrees",
        "actual_steering_degrees",
        "steering_abs_error_degrees",
        "model_throttle",
        "human_throttle",
        "throttle_abs_error",
        "inference_seconds",
        "inference_reused",
        "lidar_skew_seconds",
        "imu_skew_seconds",
        "gnss_skew_seconds",
        "lidar_source",
        "control_authority",
    ]
    with open(output_csv, "w", newline="", encoding="utf-8") as handle:
        writer_csv = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer_csv.writeheader()
        writer_csv.writerows(rows)

    total = max(1, len(camera_rows))
    return RecordPreviewSummary(
        session=session_name,
        policy_type=policy_type,
        route_id=route_id,
        output_video=output_video,
        output_csv=output_csv,
        source_frames=len(camera_rows),
        inferred_frames=inferred,
        skipped_frames=skipped,
        source_fps=source_fps,
        mean_abs_steering_error_degrees=(
            fmean(steering_errors) if steering_errors else None
        ),
        maximum_abs_steering_error_degrees=(
            max(steering_errors) if steering_errors else None
        ),
        mean_abs_throttle_error=(
            fmean(throttle_errors) if throttle_errors else None
        ),
        lidar_sync_ratio=lidar_synced / total,
        imu_sync_ratio=imu_synced / total,
        gnss_sync_ratio=(
            gnss_synced / total if policy_type == "AUTO_GPS" else None
        ),
    )


__all__ = [
    "MAXIMUM_GNSS_SKEW_SECONDS",
    "MAXIMUM_IMU_SKEW_SECONDS",
    "MAXIMUM_LIDAR_SKEW_SECONDS",
    "RECORD_PREVIEW_AUTHORITY",
    "RecordPreviewSummary",
    "preview_record_session",
]
