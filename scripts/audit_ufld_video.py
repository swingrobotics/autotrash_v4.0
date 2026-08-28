#!/usr/bin/env python3
"""Compare reference UFLD decoding with SWING's production confidence gate.

The core UFLD v1/TuSimple griding decoder is shared in both columns. The
REFERENCE column uses confidence_threshold=0, matching the original decoder's
point/support acceptance without SWING's lane-level confidence gate. The SWING
column applies the configured production lane confidence threshold (0.55 by
default). This isolates whether apparent misses are caused by the ONNX model
itself or by SWING's extra lane-level gate.

This script does not exercise later ego-lane pair/geometry safety rejection;
offline RECORD replay already preserves those rejected candidates separately.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import statistics
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from third_party.ufld import decode_tusimple_output, prepare_tusimple_input


def _session(model_path, threads):
    try:
        import onnxruntime as ort
    except ImportError as error:
        raise RuntimeError("onnxruntime is required") from error
    options = ort.SessionOptions()
    options.intra_op_num_threads = max(1, int(threads))
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    session = ort.InferenceSession(
        os.path.abspath(model_path),
        sess_options=options,
        providers=["CPUExecutionProvider"],
    )
    if len(session.get_inputs()) != 1 or len(session.get_outputs()) != 1:
        raise ValueError("UFLD_ONNX_CONTRACT_EXPECTS_ONE_INPUT_AND_ONE_OUTPUT")
    return session


def _draw_lanes(frame, lanes, cv2, *, reference):
    for lane in lanes:
        points = lane.get("points") or []
        if len(points) < 2:
            continue
        for first, second in zip(points, points[1:]):
            p1 = (int(round(first[0])), int(round(first[1])))
            p2 = (int(round(second[0])), int(round(second[1])))
            if reference:
                cv2.line(frame, p1, p2, (120, 120, 120), 2, cv2.LINE_AA)
            else:
                cv2.line(frame, p1, p2, (80, 220, 120), 3, cv2.LINE_AA)
    return frame


def main():
    parser = argparse.ArgumentParser(
        description="Audit UFLD raw/reference decoding against SWING confidence gating"
    )
    parser.add_argument("video_path")
    parser.add_argument("model_path")
    parser.add_argument("--threshold", type=float, default=0.55)
    parser.add_argument("--sample-every", type=int, default=1)
    parser.add_argument("--maximum-frames", type=int, default=0)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--output-csv")
    parser.add_argument("--output-video")
    arguments = parser.parse_args()

    try:
        import cv2
    except ImportError as error:
        raise RuntimeError("OpenCV is required") from error

    video_path = os.path.abspath(arguments.video_path)
    model_path = os.path.abspath(arguments.model_path)
    threshold = max(0.0, min(1.0, float(arguments.threshold)))
    sample_every = max(1, int(arguments.sample_every))
    session = _session(model_path, arguments.threads)
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    capture = cv2.VideoCapture(video_path)
    if not capture.isOpened():
        raise OSError(f"Could not open video: {video_path}")
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0) or 20.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    if width <= 0 or height <= 0:
        capture.release()
        raise ValueError("VIDEO_DIMENSIONS_INVALID")

    stem = str(Path(video_path).with_suffix(""))
    csv_path = os.path.abspath(arguments.output_csv or (stem + ".ufld-audit.csv"))
    video_out = os.path.abspath(arguments.output_video or (stem + ".ufld-audit.mp4"))
    Path(csv_path).parent.mkdir(parents=True, exist_ok=True)
    Path(video_out).parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        video_out,
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps / sample_every,
        (width, height),
    )
    if not writer.isOpened():
        capture.release()
        raise OSError(f"Could not create audit video: {video_out}")

    rows = []
    source_index = 0
    analyzed = 0
    reference_with_lane = 0
    swing_with_lane = 0
    gate_drop_frames = 0
    inference_ms = []
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            frame_index = source_index
            source_index += 1
            if frame_index % sample_every:
                continue

            tensor = prepare_tusimple_input(frame, cv2)
            started = time.perf_counter()
            output = session.run([output_name], {input_name: tensor})[0]
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            inference_ms.append(elapsed_ms)

            reference, reference_confidences = decode_tusimple_output(
                output,
                (width, height),
                confidence_threshold=0.0,
            )
            swing, swing_confidences = decode_tusimple_output(
                output,
                (width, height),
                confidence_threshold=threshold,
            )
            reference_count = len(reference)
            swing_count = len(swing)
            reference_with_lane += int(reference_count > 0)
            swing_with_lane += int(swing_count > 0)
            dropped = max(0, reference_count - swing_count)
            gate_drop_frames += int(dropped > 0)
            analyzed += 1
            rows.append(
                {
                    "frame_index": frame_index,
                    "time_seconds": frame_index / fps,
                    "reference_lane_count": reference_count,
                    "swing_gated_lane_count": swing_count,
                    "dropped_by_confidence_gate": dropped,
                    "threshold": threshold,
                    "inference_ms": elapsed_ms,
                    "reference_confidences": json.dumps(
                        [round(float(value), 5) for value in reference_confidences]
                    ),
                    "swing_confidences": json.dumps(
                        [round(float(value), 5) for value in swing_confidences]
                    ),
                }
            )

            rendered = frame.copy()
            _draw_lanes(rendered, reference, cv2, reference=True)
            _draw_lanes(rendered, swing, cv2, reference=False)
            cv2.putText(
                rendered,
                f"REFERENCE raw {reference_count} / SWING gate {swing_count} @ {threshold:.2f}",
                (16, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (245, 245, 245),
                2,
                cv2.LINE_AA,
            )
            if dropped:
                cv2.putText(
                    rendered,
                    f"confidence gate dropped {dropped} lane(s)",
                    (16, 58),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (80, 180, 255),
                    2,
                    cv2.LINE_AA,
                )
            writer.write(rendered)
            if arguments.maximum_frames and analyzed >= int(arguments.maximum_frames):
                break
    finally:
        capture.release()
        writer.release()

    with open(csv_path, "w", newline="", encoding="utf-8") as handle:
        fields = list(rows[0].keys()) if rows else [
            "frame_index",
            "time_seconds",
            "reference_lane_count",
            "swing_gated_lane_count",
            "dropped_by_confidence_gate",
            "threshold",
            "inference_ms",
            "reference_confidences",
            "swing_confidences",
        ]
        output_csv = csv.DictWriter(handle, fieldnames=fields)
        output_csv.writeheader()
        output_csv.writerows(rows)

    summary = {
        "video_path": video_path,
        "model_path": model_path,
        "analyzed_frames": analyzed,
        "threshold": threshold,
        "reference_detection_ratio": reference_with_lane / analyzed if analyzed else 0.0,
        "swing_gate_detection_ratio": swing_with_lane / analyzed if analyzed else 0.0,
        "frames_with_confidence_gate_drop": gate_drop_frames,
        "confidence_gate_drop_ratio": gate_drop_frames / analyzed if analyzed else 0.0,
        "mean_inference_ms": statistics.fmean(inference_ms) if inference_ms else 0.0,
        "output_csv": csv_path,
        "output_video": video_out,
        "next_stage": (
            "Use RECORD replay raw-candidate overlay to distinguish remaining "
            "ego-pair/geometry rejection from detector misses."
        ),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
