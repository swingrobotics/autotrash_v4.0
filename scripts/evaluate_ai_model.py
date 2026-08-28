#!/usr/bin/env python3

import argparse
import json
import os

from autonomous_car.ai import EvaluationCriteria, Evaluator


def main():
    parser = argparse.ArgumentParser(description="Evaluate an AUTO_AI checkpoint on a held-out split")
    parser.add_argument("dataset_path")
    parser.add_argument("checkpoint_path")
    parser.add_argument("--split", default="test", choices=["train", "validation", "test"])
    parser.add_argument("--output-path", default=None)
    parser.add_argument("--recordings-root", default=None)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--max-steering-mae-deg", type=float, default=None)
    parser.add_argument("--max-throttle-mae", type=float, default=None)
    args = parser.parse_args()

    criteria = EvaluationCriteria(
        maximum_steering_mae_degrees=args.max_steering_mae_deg,
        maximum_throttle_mae=args.max_throttle_mae,
    )
    result = Evaluator().evaluate(
        os.path.abspath(args.dataset_path),
        os.path.abspath(args.checkpoint_path),
        split=args.split,
        output_path=(os.path.abspath(args.output_path) if args.output_path else None),
        recordings_root_override=(
            os.path.abspath(args.recordings_root) if args.recordings_root else None
        ),
        criteria=criteria,
        device=args.device,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
