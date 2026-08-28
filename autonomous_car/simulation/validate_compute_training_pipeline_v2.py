"""Real smoke for the PC Compute Worker BASE -> QUICK training pipeline.

Uses tiny synthetic human RECORD sessions in the same JPEG-first segmented camera
layout used by the rover. It validates wiring and regression-set preservation
only; it is not a driving-quality benchmark or vehicle approval.
"""

import csv
import json
import os
import tempfile

from autonomous_car.simulation.validate_ai_training_v2 import _make_session
from swing_compute.frame_training_compat import install_frame_training_compat
from swing_compute.training_pipeline import TrainingPipeline


install_frame_training_compat()


def _convert_to_segmented_jpeg_session(session_path):
    """Remove legacy MP4 and make synthetic filenames match production V2."""
    video = os.path.join(session_path, "camera.mp4")
    try:
        os.remove(video)
    except FileNotFoundError:
        pass

    camera_root = os.path.join(session_path, "camera_frames")
    segment = os.path.join(camera_root, "segment_0000")
    os.makedirs(segment, exist_ok=True)
    moved = {}
    for name in sorted(os.listdir(camera_root)):
        source = os.path.join(camera_root, name)
        if not os.path.isfile(source) or not name.endswith(".jpg"):
            continue
        destination = os.path.join(segment, name)
        os.replace(source, destination)
        moved[name] = f"camera_frames/segment_0000/{name}"

    timestamps = os.path.join(session_path, "camera_timestamps.csv")
    with open(timestamps, "r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))
        fields = list(rows[0].keys()) if rows else []
    for row in rows:
        name = str(row.get("filename") or "")
        if name in moved:
            row["filename"] = moved[name]
    with open(timestamps, "w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _session(root, name, offset, steering):
    _make_session(root, name, offset, steering)
    _convert_to_segmented_jpeg_session(os.path.join(root, name))


def validate():
    with tempfile.TemporaryDirectory() as directory:
        recordings = os.path.join(directory, "recordings")
        os.makedirs(recordings)
        _session(recordings, "base_a", 0.0, [0.0, 3.0, -3.0, 9.0, -9.0, 0.5])
        _session(recordings, "base_b", 10.0, [0.0, 4.0, -4.0, 8.0, -8.0, 0.2])
        _session(recordings, "base_c", 20.0, [0.0, 3.5, -3.5, 10.0, -10.0, 0.3])

        for name in ("base_a", "base_b", "base_c"):
            assert not os.path.exists(os.path.join(recordings, name, "camera.mp4"))
            assert os.path.isdir(
                os.path.join(recordings, name, "camera_frames", "segment_0000")
            )

        base_root = os.path.join(directory, "base")
        base = TrainingPipeline(recordings, base_root).run(
            model_id="pipeline-base",
            mode="BASE",
            sessions=["base_a", "base_b", "base_c"],
            epochs=1,
            target_hz=10.0,
        )
        assert os.path.isfile(base["checkpoint"])
        assert os.path.isfile(base["model"])
        with open(base["context"], "r", encoding="utf-8") as file:
            base_context = json.load(file)
        assert len(base_context["session_splits"]) == 3
        held_out = {
            session: split
            for session, split in base_context["session_splits"].items()
            if split in {"validation", "test"}
        }
        assert held_out

        with open(
            os.path.join(base_root, "package", "training_package.json"),
            "r",
            encoding="utf-8",
        ) as file:
            package = json.load(file)
        assert package["source_counts"]["SEGMENTED_JPEG"] > 0
        assert package["source_counts"]["LEGACY_MP4"] == 0

        _session(recordings, "correction_left", 30.0, [11.0, 12.0, 10.5, 11.5, 12.5, 10.0])
        quick_root = os.path.join(directory, "quick")
        quick = TrainingPipeline(recordings, quick_root).run(
            model_id="pipeline-quick",
            mode="QUICK",
            sessions=["base_a", "base_b", "base_c", "correction_left"],
            correction_sessions=["correction_left"],
            base_model_id="pipeline-base",
            base_checkpoint=base["checkpoint"],
            base_context=base["context"],
            epochs=1,
            target_hz=10.0,
        )
        assert os.path.isfile(quick["checkpoint"])
        assert os.path.isfile(quick["model"])
        assert quick["regression_comparison"] is not None
        with open(quick["context"], "r", encoding="utf-8") as file:
            quick_context = json.load(file)
        assert quick_context["session_splits"]["correction_left"] == "train"
        for session, split in held_out.items():
            assert quick_context["session_splits"][session] == split
        assert quick_context["training_metrics"]["image_encoder_frozen"] is True
        assert quick_context["training_metrics"]["quick_new_data_target_fraction"] == 0.30
        return {
            "jpeg_only_record_input": "PASS",
            "segmented_camera_paths": "PASS",
            "base": "PASS",
            "quick_warm_start": "PASS",
            "fixed_regression_split": "PASS",
            "image_encoder_freeze": "PASS",
            "replay_target": "70_old_30_new",
            "onnx_export": "PASS",
        }


def main():
    result = validate()
    print("SWING Compute JPEG-first training pipeline smoke: PASS")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
