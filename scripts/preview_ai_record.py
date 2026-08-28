#!/usr/bin/env python3
"""Render synchronized RECORD replay preview for AUTO_AI/AUTO_GPS models."""

from __future__ import annotations

import argparse
import json
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from autonomous_car.ai.record_preview import preview_record_session


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Replay one stored RECORD session through a trained AUTO_AI or AUTO_GPS "
            "ONNX model. Camera/LiDAR/IMU are read from the recording; AUTO_GPS also "
            "replays GNSS + IMU heading against the model's normalized route. The "
            "result is an annotated model-vs-human preview only and has no control authority."
        )
    )
    parser.add_argument("session_path", help="Stored RECORD session directory")
    parser.add_argument("model_path", help="Trained AUTO_AI/AUTO_GPS .onnx file")
    parser.add_argument(
        "--manifest",
        dest="manifest_path",
        help="model_manifest.json (default: next to the ONNX model)",
    )
    parser.add_argument(
        "--route",
        dest="route_path",
        help="Normalized GPS route JSON. Required for AUTO_GPS unless it can be resolved from AUTONOMY_GPS_ROUTES_PATH.",
    )
    parser.add_argument("--output-video", help="Annotated MP4 output path")
    parser.add_argument("--output-csv", help="Frame-by-frame comparison CSV output path")
    parser.add_argument(
        "--sample-every",
        type=int,
        default=1,
        help="Run model inference every N camera frames; intermediate valid frames reuse the last command",
    )
    parser.add_argument("--jpeg-quality", type=int, default=92)
    arguments = parser.parse_args()

    summary = preview_record_session(
        arguments.session_path,
        arguments.model_path,
        manifest_path=arguments.manifest_path,
        route_path=arguments.route_path,
        output_video=arguments.output_video,
        output_csv=arguments.output_csv,
        sample_every=arguments.sample_every,
        jpeg_quality=arguments.jpeg_quality,
    )
    document = summary.as_dict()
    document["warning"] = (
        "Offline RECORD replay only. The preview compares the model with recorded human "
        "controls using synchronized stored sensors; it never commands motors."
    )
    print(json.dumps(document, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
