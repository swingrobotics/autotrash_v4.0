#!/usr/bin/env python3

import argparse
import json

from autonomous_car.ai import MODEL_LIFECYCLE, ModelRegistry


def main():
    parser = argparse.ArgumentParser(description="Update AUTO_AI model validation lifecycle")
    parser.add_argument("model_id")
    parser.add_argument("stage", choices=MODEL_LIFECYCLE)
    parser.add_argument(
        "--models-root",
        default="/home/gnss/camera-stream/models",
    )
    parser.add_argument("--metrics", default=None)
    args = parser.parse_args()

    metrics = None
    if args.metrics:
        with open(args.metrics, "r", encoding="utf-8") as file:
            metrics = json.load(file)
    result = ModelRegistry(args.models_root).update_lifecycle(
        args.model_id,
        args.stage,
        metrics=metrics,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
