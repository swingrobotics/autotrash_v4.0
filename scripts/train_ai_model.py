#!/usr/bin/env python3

import argparse
import json
import os

from autonomous_car.ai import Trainer, TrainingConfig


def main():
    parser = argparse.ArgumentParser(description="Train an AUTO_AI model from a built dataset")
    parser.add_argument("dataset_path")
    parser.add_argument("output_path")
    parser.add_argument("--recordings-root", default=None)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--device", default="auto")
    parser.add_argument(
        "--no-balance-scenarios",
        action="store_true",
        help="Disable inverse-frequency scenario sampling (enabled by default)",
    )
    parser.add_argument(
        "--scenario-balance-exponent",
        type=float,
        default=0.70,
        help="0 disables reweighting; 1 approaches full inverse-frequency balancing",
    )
    parser.add_argument(
        "--max-scenario-weight-ratio",
        type=float,
        default=8.0,
        help="Cap oversampling weight for rare scenarios",
    )
    args = parser.parse_args()

    trainer = Trainer(
        config=TrainingConfig(
            epochs=max(1, args.epochs),
            batch_size=max(1, args.batch_size),
            learning_rate=max(1e-7, args.learning_rate),
            device=args.device,
            balance_scenarios=not args.no_balance_scenarios,
            scenario_balance_exponent=max(0.0, args.scenario_balance_exponent),
            maximum_scenario_weight_ratio=max(1.0, args.max_scenario_weight_ratio),
        )
    )
    result = trainer.train(
        os.path.abspath(args.dataset_path),
        os.path.abspath(args.output_path),
        recordings_root_override=(
            os.path.abspath(args.recordings_root) if args.recordings_root else None
        ),
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
