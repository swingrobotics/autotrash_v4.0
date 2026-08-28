#!/usr/bin/env python3

import argparse
import json
import os

from autonomous_car.ai import OnnxExportConfig, OnnxExporter


def main():
    parser = argparse.ArgumentParser(description="Export an AUTO_AI checkpoint to ONNX")
    parser.add_argument("checkpoint_path")
    parser.add_argument("output_path")
    parser.add_argument("--model-filename", default="drive_model.onnx")
    parser.add_argument("--skip-verify", action="store_true")
    args = parser.parse_args()

    result = OnnxExporter().export(
        os.path.abspath(args.checkpoint_path),
        os.path.abspath(args.output_path),
        OnnxExportConfig(
            verify=not args.skip_verify,
            model_filename=args.model_filename,
        ),
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
