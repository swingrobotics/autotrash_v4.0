"""Offline visual preview for a trained AUTO_AI model on arbitrary video.

This module is deliberately diagnostic-only.  AUTO_AI is a multi-modal model
(camera + LiDAR + IMU); an arbitrary video does not contain the synchronized
LiDAR/IMU inputs seen during normal driving.  Therefore ``preview_video`` runs
with explicit neutral/missing sensor inputs and labels the result CAMERA_ONLY.
The drawn curve is a visualization of the model's steering command, not a
calibrated world-coordinate vehicle trajectory and never has motor authority.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import csv
import math
import os
from pathlib import Path
from statistics import fmean

from .runtime import AutoAiRuntime


PREVIEW_SENSOR_MODE = "CAMERA_ONLY_NEUTRAL_LIDAR_IMU"
PREVIEW_AUTHORITY = "NONE"


@dataclass(frozen=True)
class VideoPreviewSummary:
    video_path: str
    output_video: str
    output_csv: str
    analyzed_frames: int
    source_frames: int
    source_fps: float
    mean_abs_steering_degrees: float
    maximum_abs_steering_degrees: float
    mean_throttle: float
    left_command_ratio: float
    straight_command_ratio: float
    right_command_ratio: float
    sensor_mode: str = PREVIEW_SENSOR_MODE
    control_authority: str = PREVIEW_AUTHORITY

    def as_dict(self):
        return asdict(self)


def command_path_points(width, height, normalized_steering, *, count=28):
    """Return image-space points using the vehicle steering sign convention.

    SWING stores positive steering as LEFT and negative steering as RIGHT. Image
    x grows to the right, so positive steering must subtract from x. The curve
    remains illustrative steering intent, not a calibrated vehicle trajectory.
    """

    width = max(2, int(width))
    height = max(2, int(height))
    count = max(4, int(count))
    steering = max(-1.0, min(1.0, float(normalized_steering)))
    center_x = width * 0.5
    bottom_y = height * 0.96
    top_y = height * 0.42
    maximum_shift = width * 0.30
    points = []
    for index in range(count):
        progress = index / float(count - 1)
        lateral = -steering * maximum_shift * (progress ** 1.7)
        x = center_x + lateral
        y = bottom_y + (top_y - bottom_y) * progress
        points.append((float(x), float(y)))
    return points


def _draw_preview(frame, inference, cv2):
    height, width = frame.shape[:2]
    points = command_path_points(width, height, inference.normalized_steering)
    polygon = []
    half_width_near = width * 0.065
    half_width_far = width * 0.018
    left = []
    right = []
    for index, (x, y) in enumerate(points):
        progress = index / max(1.0, float(len(points) - 1))
        half = half_width_near + (half_width_far - half_width_near) * progress
        left.append((int(round(x - half)), int(round(y))))
        right.append((int(round(x + half)), int(round(y))))
    polygon.extend(left)
    polygon.extend(reversed(right))
    if len(polygon) >= 3:
        import numpy as np

        overlay = frame.copy()
        cv2.fillPoly(overlay, [np.asarray(polygon, dtype=np.int32)], (48, 160, 230))
        cv2.addWeighted(overlay, 0.18, frame, 0.82, 0.0, frame)
    for first, second in zip(points, points[1:]):
        cv2.line(
            frame,
            (int(round(first[0])), int(round(first[1]))),
            (int(round(second[0])), int(round(second[1]))),
            (60, 210, 255),
            max(2, width // 420),
            cv2.LINE_AA,
        )
    steering_text = f"STEER {inference.steering_degrees:+.1f} deg"
    throttle_text = f"THROTTLE {inference.throttle:+.3f}"
    cv2.putText(frame, steering_text, (18, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (230, 245, 230), 2, cv2.LINE_AA)
    cv2.putText(frame, throttle_text, (18, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 245, 230), 2, cv2.LINE_AA)
    cv2.putText(
        frame,
        "CAMERA-ONLY WHAT-IF / LiDAR+IMU neutral / CONTROL NONE",
        (18, max(22, height - 18)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (170, 210, 255),
        1,
        cv2.LINE_AA,
    )
    return frame


def preview_video(
    video_path,
    model_path,
    manifest_path=None,
    *,
    output_video=None,
    output_csv=None,
    sample_every=1,
    jpeg_quality=92,
):
    """Run a trained AUTO_AI model on video and render steering-intent preview."""

    try:
        import cv2
    except ImportError as error:  # pragma: no cover - runtime environment
        raise RuntimeError("OpenCV is required for AI video preview") from error

    video_path = os.path.abspath(str(video_path))
    if not os.path.isfile(video_path):
        raise FileNotFoundError(video_path)
    runtime = AutoAiRuntime(model_path, manifest_path=manifest_path)
    sample_every = max(1, int(sample_every))
    jpeg_quality = max(70, min(100, int(jpeg_quality)))

    source = cv2.VideoCapture(video_path)
    if not source.isOpened():
        raise OSError(f"Could not open video: {video_path}")
    fps = float(source.get(cv2.CAP_PROP_FPS) or 0.0)
    if not math.isfinite(fps) or fps <= 0.0:
        fps = 20.0
    width = int(source.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(source.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    if width <= 0 or height <= 0:
        source.release()
        raise ValueError("VIDEO_DIMENSIONS_INVALID")

    stem = str(Path(video_path).with_suffix(""))
    output_video = os.path.abspath(str(output_video or (stem + ".ai-preview.mp4")))
    output_csv = os.path.abspath(str(output_csv or (stem + ".ai-preview.csv")))
    Path(output_video).parent.mkdir(parents=True, exist_ok=True)
    Path(output_csv).parent.mkdir(parents=True, exist_ok=True)

    writer = cv2.VideoWriter(
        output_video,
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        source.release()
        raise OSError(f"Could not create preview video: {output_video}")

    rows = []
    steering_values = []
    throttle_values = []
    source_index = 0
    analyzed = 0
    last_inference = None
    encode_parameters = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
    try:
        while True:
            ok, frame = source.read()
            if not ok:
                break
            source_index += 1
            should_infer = last_inference is None or ((source_index - 1) % sample_every == 0)
            if should_infer:
                encoded_ok, encoded = cv2.imencode(".jpg", frame, encode_parameters)
                if not encoded_ok:
                    raise RuntimeError(f"JPEG_ENCODE_FAILED:{source_index}")
                last_inference = runtime.infer_jpeg(
                    encoded.tobytes(),
                    lidar_points=[],
                    imu_yaw_rate_dps=None,
                    person_hazard=False,
                )
                analyzed += 1
                steering_values.append(float(last_inference.steering_degrees))
                throttle_values.append(float(last_inference.throttle))
                rows.append(
                    {
                        "frame_index": source_index - 1,
                        "time_seconds": (source_index - 1) / fps,
                        "steering_degrees": float(last_inference.steering_degrees),
                        "normalized_steering": float(last_inference.normalized_steering),
                        "throttle": float(last_inference.throttle),
                        "inference_seconds": float(last_inference.inference_seconds),
                        "sensor_mode": PREVIEW_SENSOR_MODE,
                        "control_authority": PREVIEW_AUTHORITY,
                    }
                )
            rendered = _draw_preview(frame.copy(), last_inference, cv2)
            writer.write(rendered)
    finally:
        source.release()
        writer.release()

    fieldnames = [
        "frame_index",
        "time_seconds",
        "steering_degrees",
        "normalized_steering",
        "throttle",
        "inference_seconds",
        "sensor_mode",
        "control_authority",
    ]
    with open(output_csv, "w", newline="", encoding="utf-8") as handle:
        csv_writer = csv.DictWriter(handle, fieldnames=fieldnames)
        csv_writer.writeheader()
        csv_writer.writerows(rows)

    abs_steering = [abs(value) for value in steering_values]
    straight_threshold = max(1.0, runtime.maximum_steering_degrees * 0.10)
    total = max(1, len(steering_values))
    left = sum(value > straight_threshold for value in steering_values)
    right = sum(value < -straight_threshold for value in steering_values)
    straight = len(steering_values) - left - right
    return VideoPreviewSummary(
        video_path=video_path,
        output_video=output_video,
        output_csv=output_csv,
        analyzed_frames=analyzed,
        source_frames=source_index,
        source_fps=fps,
        mean_abs_steering_degrees=fmean(abs_steering) if abs_steering else 0.0,
        maximum_abs_steering_degrees=max(abs_steering) if abs_steering else 0.0,
        mean_throttle=fmean(throttle_values) if throttle_values else 0.0,
        left_command_ratio=left / total,
        straight_command_ratio=straight / total,
        right_command_ratio=right / total,
    )


__all__ = [
    "PREVIEW_AUTHORITY",
    "PREVIEW_SENSOR_MODE",
    "VideoPreviewSummary",
    "command_path_points",
    "preview_video",
]
