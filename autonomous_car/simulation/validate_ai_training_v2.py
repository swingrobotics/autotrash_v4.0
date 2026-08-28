"""End-to-end AUTO_AI smoke test for the training-capable CI job.

This intentionally uses a tiny synthetic dataset. It validates software wiring,
not driving quality or real-world model approval.
"""

import csv
import json
import math
import os
import struct
import tempfile
import zlib

from autonomous_car.ai import (
    AutoAiRuntime,
    DatasetBuilder,
    EvaluationCriteria,
    Evaluator,
    OnnxExporter,
    Trainer,
    TrainingConfig,
)


def _deps():
    import cv2
    import numpy as np
    return cv2, np


def _write_csv(path, fields, rows):
    with open(path, "w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _write_lidar(path, timestamps):
    with open(path, "wb") as file:
        for index, timestamp in enumerate(timestamps):
            front = 900 + index * 80
            row = {
                "monotonic": timestamp,
                "points": [
                    {"bearing_degrees": 0.0, "distance_mm": front - 100, "confidence": 100},
                    {"bearing_degrees": 35.0, "distance_mm": 2200, "confidence": 100},
                    {"bearing_degrees": -35.0, "distance_mm": 2500, "confidence": 100},
                ],
                "safety_points": [
                    {"bearing_degrees": 0.0, "distance_mm": front, "confidence": 100},
                    {"bearing_degrees": 35.0, "distance_mm": 2200, "confidence": 100},
                    {"bearing_degrees": -35.0, "distance_mm": 2500, "confidence": 100},
                ],
            }
            payload = zlib.compress(json.dumps(row).encode("utf-8"), level=1)
            file.write(struct.pack("<dI", timestamp, len(payload)))
            file.write(payload)


def _make_session(root, name, offset, steering_values):
    cv2, np = _deps()
    path = os.path.join(root, name)
    frames = os.path.join(path, "camera_frames")
    os.makedirs(frames)
    with open(os.path.join(path, "metadata.json"), "w", encoding="utf-8") as file:
        json.dump({"session": name, "purpose": "RECORD", "record_gps": False}, file)
    open(os.path.join(path, "camera.mp4"), "wb").close()

    timestamps = [offset + 1.0 + index * 0.1 for index in range(len(steering_values))]
    camera_rows = []
    imu_rows = []
    control_rows = []
    state_rows = []
    for index, (timestamp, steering) in enumerate(zip(timestamps, steering_values), start=1):
        image = np.zeros((90, 160, 3), dtype=np.uint8)
        image[:, :, 0] = min(255, 30 + index * 25)
        image[:, :, 1] = min(255, 60 + int(abs(steering) * 8))
        cv2.line(
            image,
            (80, 89),
            (80 + int(steering * 2), 10),
            (255, 255, 255),
            3,
        )
        filename = f"frame_{index:08d}.jpg"
        if not cv2.imwrite(os.path.join(frames, filename), image):
            raise RuntimeError("Synthetic training JPEG could not be written")
        throttle = 0.18 if abs(steering) < 8 else 0.12
        camera_rows.append(
            {
                "frame_number": index,
                "source_sequence": index,
                "monotonic": timestamp,
                "wall_time": timestamp,
                "filename": filename,
                "steering_angle_degrees": steering,
                "target_steering_angle_degrees": steering,
                "requested_throttle": throttle,
                "final_throttle": throttle,
            }
        )
        imu_rows.append({"monotonic": timestamp + 0.005, "yaw_rate_dps": steering * 0.5})
        control_rows.append({"monotonic": timestamp, "target_speed_mps": 0.2})
        state_rows.append(
            {
                "monotonic": timestamp,
                "mode": "RECORD",
                "system_state": "ACTIVE",
                "manual_override": False,
                "emergency_stop": False,
                "fault_code": "",
            }
        )

    _write_csv(
        os.path.join(path, "camera_timestamps.csv"),
        [
            "frame_number", "source_sequence", "monotonic", "wall_time", "filename",
            "steering_angle_degrees", "target_steering_angle_degrees",
            "requested_throttle", "final_throttle",
        ],
        camera_rows,
    )
    _write_csv(os.path.join(path, "imu.csv"), ["monotonic", "yaw_rate_dps"], imu_rows)
    _write_csv(os.path.join(path, "control.csv"), ["monotonic", "target_speed_mps"], control_rows)
    _write_csv(
        os.path.join(path, "vehicle_state.csv"),
        ["monotonic", "mode", "system_state", "manual_override", "emergency_stop", "fault_code"],
        state_rows,
    )
    _write_lidar(os.path.join(path, "lidar_raw.bin"), timestamps)


def validate():
    cv2, _ = _deps()
    with tempfile.TemporaryDirectory() as directory:
        recordings = os.path.join(directory, "recordings")
        datasets = os.path.join(directory, "datasets")
        training = os.path.join(directory, "training")
        exported = os.path.join(directory, "export")
        evaluation = os.path.join(directory, "evaluation")
        os.makedirs(recordings)

        _make_session(recordings, "run_train", 0.0, [0.0, 3.5, -3.5, 10.0, -10.0, 0.5])
        _make_session(recordings, "run_validation", 10.0, [0.0, 4.0, -4.0, 9.0, -9.0, 0.3])
        _make_session(recordings, "run_test", 20.0, [0.0, 3.0, -3.0, 8.5, -8.5, 0.2])

        dataset = DatasetBuilder(recordings, datasets).build(
            ["run_train", "run_validation", "run_test"],
            "training_smoke",
        )
        dataset_path = os.path.join(datasets, dataset["dataset_id"])
        assert dataset["accepted_samples"] == 18
        assert all(dataset["split_counts"][name] == 6 for name in ("train", "validation", "test"))

        trainer = Trainer(
            config=TrainingConfig(
                epochs=1,
                batch_size=2,
                learning_rate=1e-3,
                device="cpu",
                balance_scenarios=True,
            )
        )
        train_metrics = trainer.train(dataset_path, training)
        assert train_metrics["train_samples"] == 6
        assert train_metrics["validation_samples"] == 6
        assert train_metrics["scenario_balancing"]["enabled"]

        checkpoint = os.path.join(training, "checkpoint.pt")
        evaluator = Evaluator()
        eval_metrics = evaluator.evaluate(
            dataset_path,
            checkpoint,
            split="test",
            output_path=evaluation,
            criteria=EvaluationCriteria(
                maximum_steering_mae_degrees=25.0,
                maximum_throttle_mae=1.5,
            ),
            device="cpu",
        )
        assert eval_metrics["samples"] == 6
        assert eval_metrics["criteria_passed"] is True
        assert math.isfinite(eval_metrics["steering_mae_degrees"])
        assert math.isfinite(eval_metrics["throttle_mae"])

        manifest = OnnxExporter().export(checkpoint, exported)
        model_path = os.path.join(exported, manifest["model_file"])
        manifest_path = os.path.join(exported, "model_manifest.json")
        runtime = AutoAiRuntime(model_path, manifest_path)

        image_path = os.path.join(recordings, "run_test", "camera_frames", "frame_00000001.jpg")
        frame = cv2.imread(image_path)
        ok, jpeg = cv2.imencode(".jpg", frame)
        assert ok
        lidar = [
            {"bearing_degrees": 0.0, "distance_mm": 1200, "confidence": 100},
            {"bearing_degrees": 35.0, "distance_mm": 2300, "confidence": 100},
            {"bearing_degrees": -35.0, "distance_mm": 2500, "confidence": 100},
        ]
        inference = runtime.infer_jpeg(jpeg.tobytes(), lidar, 1.0)
        assert math.isfinite(inference.steering_degrees)
        assert math.isfinite(inference.throttle)
        assert inference.person_stop is False

        person = runtime.infer_jpeg(jpeg.tobytes(), lidar, 1.0, person_hazard=True)
        assert person.person_stop is True
        assert person.throttle == 0.0

        return {
            "dataset": "PASS",
            "scenario_balanced_training": "PASS",
            "held_out_evaluation": "PASS",
            "onnx_export_verification": "PASS",
            "onnxruntime_inference": "PASS",
            "person_stop_override": "PASS",
            "inference_seconds": inference.inference_seconds,
        }


def main():
    result = validate()
    print("AUTO_AI training/export/inference smoke: PASS")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
