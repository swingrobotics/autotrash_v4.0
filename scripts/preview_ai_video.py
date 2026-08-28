#!/usr/bin/env python3
"""Render a diagnostic steering-path preview for a trained AUTO_AI model."""

from __future__ import annotations

import argparse
import json
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from autonomous_car.ai.video_preview import preview_video


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Run an AUTO_AI ONNX model on an arbitrary camera video and create "
            "an annotated steering-intent video plus CSV. This is CAMERA-ONLY "
            "what-if validation: LiDAR and IMU are intentionally treated as missing."
        )
    )
    parser.add_argument("video_path", help="Input MP4/AVI/MOV video")
    parser.add_argument("model_path", help="Trained AUTO_AI .onnx file")
    parser.add_argument(
        "--manifest",
        dest="manifest_path",
        help="model_manifest.json (default: next to the ONNX model)",
    )
    parser.add_argument("--output-video", help="Annotated MP4 output path")
    parser.add_argument("--output-csv", help="Frame inference CSV output path")
    parser.add_argument(
        "--sample-every",
        type=int,
        default=1,
        help="Run inference every N source frames; intermediate frames reuse the last command",
    )
    parser.add_argument("--jpeg-quality", type=int, default=92)
    arguments = parser.parse_args()

    summary = preview_video(
        arguments.video_path,
        arguments.model_path,
        manifest_path=arguments.manifest_path,
        output_video=arguments.output_video,
        output_csv=arguments.output_csv,
        sample_every=arguments.sample_every,
        jpeg_quality=arguments.jpeg_quality,
    )
    document = summary.as_dict()
    document["warning"] = (
        "CAMERA_ONLY what-if result. AUTO_AI normally consumes synchronized Camera + "
        "LiDAR + IMU. The overlay visualizes predicted steering intent and is not a "
        "metric/calibrated vehicle trajectory."
    )
    print(json.dumps(document, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
