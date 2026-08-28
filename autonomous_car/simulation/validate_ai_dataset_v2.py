"""Synthetic regression validation for the AUTO_AI DatasetBuilder.

Run with:
    python3 -m autonomous_car.simulation.validate_ai_dataset_v2
"""

import csv
import json
import os
import struct
import tempfile
import zlib

from autonomous_car.ai import DatasetBuilder, LidarSectorizer


def _write_csv(path, fields, rows):
    with open(path, "w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_lidar(path, timestamps, include_safety_points=True):
    with open(path, "wb") as file:
        for timestamp in timestamps:
            raw_points = [
                {"bearing_degrees": 0.0, "distance_mm": 1200, "confidence": 100},
                {"bearing_degrees": 25.0, "distance_mm": 2100, "confidence": 100},
                {"bearing_degrees": -50.0, "distance_mm": 3200, "confidence": 100},
            ]
            row = {
                "monotonic": timestamp,
                "points": raw_points,
            }
            if include_safety_points:
                # Deliberately different front distance so the regression can
                # prove the V2 builder prefers the same stabilized feature set
                # used during rover inference.
                row["safety_points"] = [
                    {"bearing_degrees": 0.0, "distance_mm": 1800, "confidence": 100},
                    {"bearing_degrees": 25.0, "distance_mm": 2100, "confidence": 100},
                    {"bearing_degrees": -50.0, "distance_mm": 3200, "confidence": 100},
                ]
            payload = zlib.compress(json.dumps(row).encode("utf-8"))
            file.write(struct.pack("<dI", timestamp, len(payload)))
            file.write(payload)


def _make_session(root, name, offset, record_gps, include_safety_points=True):
    path = os.path.join(root, name)
    os.makedirs(path)
    with open(os.path.join(path, "metadata.json"), "w", encoding="utf-8") as file:
        json.dump({"session": name, "record_gps": record_gps}, file)
    open(os.path.join(path, "camera.mp4"), "wb").close()

    camera_times = [offset + 1.0, offset + 1.1]
    _write_csv(
        os.path.join(path, "camera_timestamps.csv"),
        [
            "frame_number",
            "source_sequence",
            "monotonic",
            "wall_time",
            "filename",
            "steering_angle_degrees",
            "target_steering_angle_degrees",
            "requested_throttle",
            "final_throttle",
        ],
        [
            {
                "frame_number": 1,
                "source_sequence": 10,
                "monotonic": camera_times[0],
                "wall_time": 0,
                "filename": "",
                "steering_angle_degrees": 0.2,
                "target_steering_angle_degrees": 0.0,
                # Deliberately differ requested/final so the regression proves
                # imitation learning follows the human request, not a post-
                # Safety/limit actuator value.
                "requested_throttle": 0.2,
                "final_throttle": 0.05,
            },
            {
                "frame_number": 2,
                "source_sequence": 11,
                "monotonic": camera_times[1],
                "wall_time": 0,
                "filename": "",
                "steering_angle_degrees": 5.5,
                "target_steering_angle_degrees": 6.0,
                "requested_throttle": 0.15,
                "final_throttle": 0.04,
            },
        ],
    )
    _write_csv(
        os.path.join(path, "imu.csv"),
        ["monotonic", "yaw_rate_dps"],
        [
            {"monotonic": camera_times[0] + 0.01, "yaw_rate_dps": 1.5},
            {"monotonic": camera_times[1] + 0.01, "yaw_rate_dps": 2.0},
        ],
    )
    _write_csv(
        os.path.join(path, "control.csv"),
        ["monotonic", "target_speed_mps"],
        [
            {"monotonic": camera_times[0], "target_speed_mps": 0.2},
            {"monotonic": camera_times[1], "target_speed_mps": 0.15},
        ],
    )
    if record_gps:
        _write_csv(
            os.path.join(path, "gnss.csv"),
            ["monotonic", "speed_mps"],
            [
                {"monotonic": camera_times[0], "speed_mps": 0.21},
                {"monotonic": camera_times[1], "speed_mps": 0.16},
            ],
        )
    _write_lidar(
        os.path.join(path, "lidar_raw.bin"),
        camera_times,
        include_safety_points=include_safety_points,
    )


def validate():
    with tempfile.TemporaryDirectory() as directory:
        recordings = os.path.join(directory, "recordings")
        datasets = os.path.join(directory, "datasets")
        os.makedirs(recordings)
        _make_session(recordings, "run_0", 0.0, True, include_safety_points=True)
        # Historical-recording compatibility: no safety_points key.
        _make_session(recordings, "run_1", 10.0, False, include_safety_points=False)
        _make_session(recordings, "run_2", 20.0, True, include_safety_points=True)

        sectorizer = LidarSectorizer(minimum_confidence=35)
        sectors = sectorizer.transform(
            [
                {"bearing_degrees": 0.0, "distance_mm": 900, "confidence": 100},
                {"bearing_degrees": 50.0, "distance_mm": 2500, "confidence": 100},
            ]
        )
        assert sectors.observed["front"]
        assert abs(sectors.distances_m["front"] - 0.9) < 1e-6
        assert sectors.observed["left"]
        assert not sectors.observed["right"]

        builder = DatasetBuilder(recordings, datasets, sectorizer=sectorizer)
        result = builder.build(
            ["run_0", "run_1", "run_2"],
            dataset_id="synthetic_v2",
        )
        assert result["accepted_samples"] == 6
        assert result["rejected_samples"] == 0
        assert set(result["split_counts"]) == {"train", "validation", "test"}
        assert all(result["split_counts"][name] == 2 for name in result["split_counts"])
        assert "safety_points" in result["feature_contract"]["lidar_source_preference"]
        assert "requested throttle" in result["feature_contract"]["throttle_label"]
        assert "target steering" in result["feature_contract"]["steering_label_degrees"]

        manifest_path = os.path.join(datasets, "synthetic_v2", "samples.jsonl")
        with open(manifest_path, "r", encoding="utf-8") as file:
            samples = [json.loads(line) for line in file if line.strip()]
        assert len(samples) == 6
        assert all(sample["learned_features"]["speed_mps"] is None for sample in samples)
        assert any(sample["evaluation_only"]["gnss_speed_mps"] is None for sample in samples)
        assert any(sample["evaluation_only"]["gnss_speed_mps"] is not None for sample in samples)
        assert all(sample["learned_features"]["lidar"]["observed"]["front"] for sample in samples)
        assert samples[0]["camera"]["video_frame_index"] == 0
        assert all(
            sample["labels"]["throttle_label_source"] == "requested_throttle"
            for sample in samples
        )
        assert {round(float(sample["labels"]["throttle"]), 2) for sample in samples} == {
            0.15,
            0.20,
        }
        assert {round(float(sample["labels"]["final_throttle"]), 2) for sample in samples} == {
            0.04,
            0.05,
        }

        by_session = {}
        for sample in samples:
            by_session.setdefault(sample["session"], []).append(sample)
        assert all(
            abs(sample["learned_features"]["lidar"]["distances_m"]["front"] - 1.8) < 1e-6
            for name in ("run_0", "run_2")
            for sample in by_session[name]
        )
        assert all(
            abs(sample["learned_features"]["lidar"]["distances_m"]["front"] - 1.2) < 1e-6
            for sample in by_session["run_1"]
        )

        splits_by_session = {}
        for sample in samples:
            splits_by_session.setdefault(sample["session"], set()).add(sample["split"])
        assert all(len(splits) == 1 for splits in splits_by_session.values())

        return {
            "lidar_sector_features": "PASS",
            "stabilized_lidar_alignment": "PASS",
            "legacy_lidar_fallback": "PASS",
            "timestamp_alignment": "PASS",
            "human_requested_throttle_labels": "PASS",
            "dataset_label_metadata_contract": "PASS",
            "gps_excluded_from_learned_features": "PASS",
            "session_level_split": "PASS",
            "manifest_generation": "PASS",
        }


def main():
    result = validate()
    print("AUTO_AI DatasetBuilder V2 validation: PASS")
    for name, status in result.items():
        print(f"- {name}: {status}")


if __name__ == "__main__":
    main()
