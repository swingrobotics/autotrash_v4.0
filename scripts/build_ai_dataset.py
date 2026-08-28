#!/usr/bin/env python3

import argparse
import json
import os

from autonomous_car.ai import DatasetBuildConfig, DatasetBuilder, LidarSectorizer


def main():
    parser = argparse.ArgumentParser(
        description="Build a timestamp-aligned AUTO_AI dataset from RECORD sessions"
    )
    parser.add_argument("sessions", nargs="+", help="RECORD session directory names")
    parser.add_argument(
        "--recordings-root",
        default="/home/gnss/camera-stream/recordings",
    )
    parser.add_argument(
        "--output-root",
        default="/home/gnss/camera-stream/ai-datasets",
    )
    parser.add_argument("--dataset-id", default=None)
    parser.add_argument("--allow-missing-lidar", action="store_true")
    parser.add_argument("--require-imu", action="store_true")
    parser.add_argument("--minimum-throttle", type=float, default=0.0)
    args = parser.parse_args()

    config = DatasetBuildConfig(
        require_lidar=not args.allow_missing_lidar,
        require_imu=args.require_imu,
        minimum_absolute_throttle=max(0.0, args.minimum_throttle),
    )
    builder = DatasetBuilder(
        os.path.abspath(args.recordings_root),
        os.path.abspath(args.output_root),
        config=config,
        sectorizer=LidarSectorizer(minimum_confidence=35),
    )
    result = builder.build(args.sessions, dataset_id=args.dataset_id)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
