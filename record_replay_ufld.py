"""Offline UFLD analysis for finalized RECORD videos.

A sidecar UFLD analysis can be regenerated for any finalized RECORD while the
vehicle is DISARMED. The source MJPEG MP4 may have nominal fixed-FPS timestamps,
so analysis uses camera_timestamps.csv as the authoritative camera timeline.
The sidecar is replay-only and never changes vehicle/control authority.
"""

from __future__ import annotations

import bisect
import csv
import json
import math
import os
import threading
import time
from urllib.parse import parse_qs, urlsplit

import server_v2_release as release
from autonomous_car import DriveMode
from autonomous_car.control.hybrid_lane_controller import HybridLaneController
from record_replay_media import _camera_timing, _finalized_media_path, _session_path

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


_INSTALLED = False
_JOB_LOCK = threading.RLock()
_JOB = {
    "session": None,
    "state": "idle",
    "progress": 0.0,
    "processed": 0,
    "total": 0,
    "error": None,
    "started_at": None,
    "finished_at": None,
}
_RESULT_CACHE = {}

_RESULT_FIELDS = [
    "offset_seconds",
    "video_frame_index",
    "lane_detected",
    "lane_confidence",
    "lane_lateral_error_m",
    "lane_heading_error_degrees",
    "lane_correction_angle_degrees",
    "lane_error",
    "lane_backend",
    "lane_marking",
    "lane_control_authority",
    "lane_inference_ms",
    "lane_latency_allowed",
    "lane_left_json",
    "lane_right_json",
    "lane_center_json",
    "lane_image_size_json",
    "lane_roi_json",
]


def _period_seconds():
    try:
        value = float(os.environ.get("SWING_UFLD_REPLAY_PERIOD_SECONDS", "0.50"))
    except (TypeError, ValueError):
        value = 0.50
    return max(0.20, min(2.0, value))


def _result_path(session_path):
    return os.path.join(session_path, "ufld_replay.csv")


def _metadata_path(session_path):
    return os.path.join(session_path, "ufld_analysis.json")


def _json_compact(value):
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"))


def _safe_number(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _analysis_allowed():
    manager = release.full.legacy.record_manager
    if manager.active:
        return False, "STOP_RECORDING_BEFORE_UFLD_ANALYSIS"
    mode = release.full.legacy.vehicle_state_machine.mode
    canonical = getattr(mode, "canonical", mode)
    if canonical != DriveMode.DISARMED:
        return False, "UFLD_ANALYSIS_REQUIRES_DISARMED"
    hybrid = getattr(release.full, "HYBRID_LANE_CONTROLLER", None)
    if hybrid is None:
        return False, "UFLD_RUNTIME_UNAVAILABLE"
    # Worker UFLD is configured as neural_enabled even while the vehicle is
    # DISARMED. That flag does not mean control is active, so do not reject
    # replay analysis solely because it is True.
    pretrained = getattr(hybrid, "pretrained", None)
    if pretrained is None or not getattr(pretrained, "available", False):
        return False, "UFLD_MODEL_UNAVAILABLE"
    return True, None


def _offline_controller():
    source = getattr(release.full, "HYBRID_LANE_CONTROLLER", None)
    if source is None:
        raise RuntimeError("UFLD_RUNTIME_UNAVAILABLE")
    return HybridLaneController(
        source.pretrained,
        camera_calibration=source.camera_calibration,
        expected_lane_width_m=source.expected_lane_width_m,
        vehicle_width_m=source.vehicle_width_m,
        processing_width=source.processing_width,
        processing_height=source.processing_height,
        maximum_neural_inference_ms=source.maximum_neural_inference_ms,
    )


def _row_from_result(offset_seconds, frame_index, lane, diagnostics):
    return {
        "offset_seconds": f"{float(offset_seconds):.6f}",
        "video_frame_index": int(frame_index),
        "lane_detected": bool(lane.get("detected")),
        "lane_confidence": lane.get("confidence"),
        "lane_lateral_error_m": lane.get("lateral_error_m"),
        "lane_heading_error_degrees": lane.get("heading_error_degrees"),
        "lane_correction_angle_degrees": lane.get("correction_angle_degrees"),
        "lane_error": lane.get("error"),
        "lane_backend": lane.get("backend") or "UFLD_ONNX",
        "lane_marking": lane.get("marking"),
        "lane_control_authority": "NONE",
        "lane_inference_ms": diagnostics.get("inference_ms"),
        "lane_latency_allowed": diagnostics.get("latency_allowed"),
        "lane_left_json": _json_compact(lane.get("left_line") or {}),
        "lane_right_json": _json_compact(lane.get("right_line") or {}),
        "lane_center_json": _json_compact(lane.get("center_line") or {}),
        "lane_image_size_json": _json_compact(lane.get("image_size") or []),
        "lane_roi_json": _json_compact(lane.get("roi") or {}),
    }


def _write_metadata(session_path, document):
    temporary = _metadata_path(session_path) + ".tmp"
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump(document, file, ensure_ascii=False, indent=2)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, _metadata_path(session_path))


def _set_job(**updates):
    with _JOB_LOCK:
        _JOB.update(updates)
        return dict(_JOB)


def _frame_offset(frame_index, camera_offsets, fallback_fps):
    if 0 <= frame_index < len(camera_offsets):
        value = camera_offsets[frame_index]
        if value is not None:
            return max(0.0, float(value))
    return frame_index / max(0.1, float(fallback_fps))


def _analyze_session(session_name, media_path, session_path):
    period = _period_seconds()
    output = _result_path(session_path)
    temporary = output + ".tmp"
    capture = None
    row_count = 0
    sampled = 0
    started = time.time()
    try:
        if cv2 is None:
            raise RuntimeError("OPENCV_UNAVAILABLE")

        controller = _offline_controller()
        capture = cv2.VideoCapture(media_path)
        if not capture.isOpened():
            raise RuntimeError("RECORDED_VIDEO_OPEN_FAILED")

        fps = _safe_number(capture.get(cv2.CAP_PROP_FPS)) or 10.0
        if fps <= 0.1:
            fps = 10.0
        frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
        timing = _camera_timing(session_path)
        camera_offsets = list(timing.get("offsets") or [])
        recorded_duration = _safe_number(timing.get("duration_seconds"))
        if recorded_duration is not None and recorded_duration > 0:
            total = max(1, int(math.ceil(recorded_duration / period)) + 1)
        else:
            step = max(1, int(round(fps * period)))
            total = max(1, int(math.ceil(frame_count / step))) if frame_count > 0 else 0
        _set_job(total=total)

        with open(temporary, "w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=_RESULT_FIELDS)
            writer.writeheader()
            frame_index = 0
            next_sample_offset = 0.0
            while True:
                allowed, reason = _analysis_allowed()
                if not allowed:
                    raise RuntimeError(f"UFLD_ANALYSIS_INTERRUPTED:{reason}")

                ok, frame = capture.read()
                if not ok or frame is None:
                    break

                offset = _frame_offset(frame_index, camera_offsets, fps)
                should_sample = offset + 1e-6 >= next_sample_offset
                if should_sample:
                    encoded_ok, encoded = cv2.imencode(
                        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                    )
                    if not encoded_ok:
                        lane = {
                            "detected": False,
                            "confidence": 0.0,
                            "backend": "UFLD_ONNX",
                            "marking": "OFFLINE_REPLAY",
                            "error": "RECORDED_FRAME_JPEG_FAILED",
                        }
                        diagnostics = {}
                    else:
                        result = controller.analyze_neural_preview_jpeg(encoded.tobytes())
                        lane = result.as_dict()
                        diagnostics = dict(controller.preview_snapshot() or {})

                    writer.writerow(_row_from_result(offset, frame_index, lane, diagnostics))
                    row_count += 1
                    sampled += 1
                    next_sample_offset = offset + period
                    progress = min(1.0, sampled / total) if total > 0 else 0.0
                    _set_job(processed=sampled, progress=progress)
                    if row_count % 20 == 0:
                        file.flush()

                frame_index += 1

            file.flush()
            os.fsync(file.fileno())

        if row_count <= 0:
            raise RuntimeError("RECORDED_VIDEO_HAS_NO_DECODABLE_FRAMES")
        os.replace(temporary, output)
        _RESULT_CACHE.pop(session_name, None)

        metadata = {
            "state": "completed",
            "session": session_name,
            "created_at": time.time(),
            "analysis_elapsed_seconds": max(0.0, time.time() - started),
            "recorded_duration_seconds": recorded_duration,
            "sample_period_seconds": period,
            "rows": row_count,
            "source": "camera.mp4",
            "timeline": "camera_timestamps.csv" if camera_offsets else "nominal_video_fps",
            "result": os.path.basename(output),
            "backend": "UFLD_ONNX",
            "control_authority": "NONE",
        }
        _write_metadata(session_path, metadata)
        with _JOB_LOCK:
            prior_total = int(_JOB.get("total") or 0)
        _set_job(
            state="completed",
            progress=1.0,
            processed=row_count,
            total=max(row_count, prior_total),
            error=None,
            finished_at=time.time(),
        )
    except Exception as error:
        try:
            if os.path.exists(temporary):
                os.remove(temporary)
        except OSError:
            pass
        _set_job(
            state="failed",
            error=f"{type(error).__name__}: {error}",
            finished_at=time.time(),
        )
    finally:
        if capture is not None:
            capture.release()


def _read_metadata(session_path):
    path = _metadata_path(session_path)
    if not os.path.isfile(path):
        return {}
    try:
        with open(path, "r", encoding="utf-8") as file:
            document = json.load(file)
        return document if isinstance(document, dict) else {}
    except (OSError, ValueError, TypeError):
        return {}


def _load_rows(session_name, session_path):
    path = _result_path(session_path)
    if not os.path.isfile(path):
        return [], []
    mtime = os.path.getmtime(path)
    cached = _RESULT_CACHE.get(session_name)
    if cached and cached.get("mtime") == mtime:
        return cached["rows"], cached["offsets"]

    rows = []
    offsets = []
    with open(path, "r", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            offset = _safe_number(row.get("offset_seconds"))
            if offset is None:
                continue
            rows.append(row)
            offsets.append(offset)
    _RESULT_CACHE[session_name] = {
        "mtime": mtime,
        "rows": rows,
        "offsets": offsets,
    }
    return rows, offsets


def _row_at(session_name, session_path, offset_seconds):
    rows, offsets = _load_rows(session_name, session_path)
    if not rows:
        return None
    offset = max(0.0, float(offset_seconds or 0.0))
    index = bisect.bisect_right(offsets, offset) - 1
    if index < 0:
        index = 0
    return dict(rows[index])


def _native_ufld_available(session_path):
    """Require actual UFLD rows, not merely UFLD-shaped CSV column names."""
    path = os.path.join(session_path, "perception.csv")
    if not os.path.isfile(path):
        return False
    try:
        with open(path, "r", newline="", encoding="utf-8") as file:
            reader = csv.DictReader(file)
            fields = set(reader.fieldnames or [])
            if "lane_backend" not in fields:
                return False
            for row in reader:
                backend = str(row.get("lane_backend") or "").upper()
                if "UFLD" in backend:
                    return True
        return False
    except OSError:
        return False


def analysis_status(session_name, offset_seconds=None):
    session_path = _session_path(session_name)
    result_exists = os.path.isfile(_result_path(session_path))
    metadata = _read_metadata(session_path)

    with _JOB_LOCK:
        same_session = _JOB.get("session") == session_name
        job = dict(_JOB) if same_session else None

    if job and job.get("state") == "running":
        state = "running"
        progress = float(job.get("progress") or 0.0)
        processed = int(job.get("processed") or 0)
        total = int(job.get("total") or 0)
        error = job.get("error")
    elif job and job.get("state") == "failed":
        state = "failed"
        progress = float(job.get("progress") or 0.0)
        processed = int(job.get("processed") or 0)
        total = int(job.get("total") or 0)
        error = job.get("error")
    elif result_exists:
        state = "completed"
        progress = 1.0
        processed = int(metadata.get("rows") or 0)
        total = processed
        error = None
    else:
        state = "not_analyzed"
        progress = 0.0
        processed = 0
        total = 0
        error = None

    payload = {
        "session": session_name,
        "state": state,
        "available": result_exists,
        "native_ufld": _native_ufld_available(session_path),
        "progress": progress,
        "processed": processed,
        "total": total,
        "error": error,
        "metadata": metadata,
        "control_authority": "NONE",
    }
    if offset_seconds is not None and result_exists:
        payload["row"] = _row_at(session_name, session_path, offset_seconds)
    return payload


def start_analysis(session_name, force=False):
    session_path = _session_path(session_name)
    media_path = _finalized_media_path(session_name)
    allowed, reason = _analysis_allowed()
    if not allowed:
        raise RuntimeError(reason)

    output = _result_path(session_path)
    if os.path.isfile(output) and not force:
        return analysis_status(session_name)

    with _JOB_LOCK:
        if _JOB.get("state") == "running":
            if _JOB.get("session") == session_name:
                return dict(_JOB)
            raise RuntimeError(f"UFLD_ANALYSIS_BUSY:{_JOB.get('session')}")
        _JOB.update(
            {
                "session": session_name,
                "state": "running",
                "progress": 0.0,
                "processed": 0,
                "total": 0,
                "error": None,
                "started_at": time.time(),
                "finished_at": None,
            }
        )

    thread = threading.Thread(
        target=_analyze_session,
        args=(session_name, media_path, session_path),
        name=f"ufld-replay-{session_name}",
        daemon=True,
    )
    thread.start()
    return analysis_status(session_name)


def install_record_replay_ufld_endpoints():
    global _INSTALLED
    if _INSTALLED:
        return True

    replay_owner = release.full.legacy
    original_replay_state = replay_owner.recording_replay_state
    if not getattr(original_replay_state, "_swing_offline_ufld_replay", False):
        def recording_replay_state_with_offline_ufld(session_name, offset_seconds=0.0):
            result = original_replay_state(session_name, offset_seconds)
            state = dict(result.get("state") or {})
            perception = dict(state.get("perception") or {})
            try:
                offline = analysis_status(
                    result.get("session") or session_name,
                    result.get("offset_seconds", offset_seconds),
                )
                row = offline.get("row") if offline.get("available") else None
            except Exception:
                row = None
            # A user-triggered sidecar analysis intentionally overrides the old
            # replay-only lane row, even if the original CSV contains a stale
            # Classical/UFLD backend field. It never changes live control state.
            if row:
                perception.update(row)
                perception["lane_source"] = "OFFLINE_UFLD_REPLAY"
                state["perception"] = perception
                result = dict(result)
                result["state"] = state
            return result

        recording_replay_state_with_offline_ufld._swing_offline_ufld_replay = True
        replay_owner.recording_replay_state = recording_replay_state_with_offline_ufld

    handler = release.full.legacy.CameraHandler
    original_do_get = handler.do_GET
    original_do_post = handler.do_POST

    if getattr(original_do_get, "_swing_record_replay_ufld", False):
        _INSTALLED = True
        return True

    def do_get_with_record_replay_ufld(self):
        parsed = urlsplit(str(self.path or ""))
        if parsed.path != "/api/recordings/ufld-analysis":
            return original_do_get(self)
        params = parse_qs(parsed.query, keep_blank_values=False)
        session = (params.get("session") or [""])[0]
        offset = (params.get("offset") or [None])[0]
        try:
            self._send_json(
                analysis_status(
                    session,
                    None if offset is None else float(offset),
                )
            )
        except FileNotFoundError as error:
            self._send_json({"error": str(error)}, 404)
        except (TypeError, ValueError, OSError) as error:
            self._send_json({"error": str(error)}, 400)

    def do_post_with_record_replay_ufld(self):
        path = str(self.path or "").split("?", 1)[0]
        if path != "/api/recordings/ufld-analysis":
            return original_do_post(self)
        try:
            payload = self._read_json()
            result = start_analysis(
                payload.get("session"),
                force=bool(payload.get("force", False)),
            )
            self._send_json(result, 202 if result.get("state") == "running" else 200)
        except FileNotFoundError as error:
            self._send_json({"error": str(error)}, 404)
        except RuntimeError as error:
            text = str(error)
            status = 409 if (
                text.startswith("UFLD_ANALYSIS_")
                or text.startswith("STOP_RECORDING_")
            ) else 503
            self._send_json({"error": text}, status)
        except (TypeError, ValueError, OSError) as error:
            self._send_json({"error": str(error)}, 400)

    do_get_with_record_replay_ufld._swing_record_replay_ufld = True
    do_post_with_record_replay_ufld._swing_record_replay_ufld = True
    handler.do_GET = do_get_with_record_replay_ufld
    handler.do_POST = do_post_with_record_replay_ufld
    _INSTALLED = True
    return True


__all__ = [
    "analysis_status",
    "install_record_replay_ufld_endpoints",
    "start_analysis",
]
