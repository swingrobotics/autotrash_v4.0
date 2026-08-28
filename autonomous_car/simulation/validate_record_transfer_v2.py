"""Regression for JPEG-first RECORD transfer and dataset compatibility."""

import csv
import json
import os
import tempfile

from autonomous_car.ai import DatasetBuildConfig, DatasetBuilder
from autonomous_car.recording.record_transfer import (
    iter_record_source_files,
    normalize_record_relative_path,
    record_source_path,
)


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def _write_csv(path, fields, rows):
    with open(path, "w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _expect_reject(value):
    try:
        normalize_record_relative_path(value)
    except ValueError:
        return
    raise AssertionError(f"unsafe RECORD path accepted: {value}")


def main():
    with tempfile.TemporaryDirectory() as directory:
        recordings = os.path.join(directory, "recordings")
        datasets = os.path.join(directory, "datasets")
        session = os.path.join(recordings, "run_jpeg")
        segment = os.path.join(session, "camera_frames", "segment_0000")
        os.makedirs(segment, exist_ok=True)

        frame_name = "camera_frames/segment_0000/frame_00000001.jpg"
        with open(os.path.join(session, *frame_name.split("/")), "wb") as file:
            file.write(b"synthetic-jpeg-placeholder")
        with open(os.path.join(session, "metadata.json"), "w", encoding="utf-8") as file:
            json.dump({"session": "run_jpeg", "record_gps": False}, file)
        with open(os.path.join(session, "record_manifest.json"), "w", encoding="utf-8") as file:
            json.dump({"state": "FINALIZED", "schema": "swing_record_manifest_v2"}, file)
        # Derived Worker/browser output must not be re-sent as a rover source.
        with open(os.path.join(session, "camera_browser_v2.mp4"), "wb") as file:
            file.write(b"derived")
        with open(os.path.join(segment, "ignored.part"), "wb") as file:
            file.write(b"partial")

        camera_fields = [
            "frame_number",
            "source_sequence",
            "monotonic",
            "wall_time",
            "filename",
            "steering_angle_degrees",
            "target_steering_angle_degrees",
            "requested_throttle",
            "final_throttle",
        ]
        _write_csv(
            os.path.join(session, "camera_timestamps.csv"),
            camera_fields,
            [
                {
                    "frame_number": 1,
                    "source_sequence": 101,
                    "monotonic": 10.0,
                    "wall_time": 1000.0,
                    "filename": frame_name,
                    "steering_angle_degrees": 2.0,
                    "target_steering_angle_degrees": 2.0,
                    "requested_throttle": 0.2,
                    "final_throttle": 0.2,
                },
                {
                    "frame_number": 2,
                    "source_sequence": 102,
                    "monotonic": 10.1,
                    "wall_time": 1000.1,
                    "filename": "camera_frames/segment_0000/frame_00000002.jpg",
                    "steering_angle_degrees": 3.0,
                    "target_steering_angle_degrees": 3.0,
                    "requested_throttle": 0.2,
                    "final_throttle": 0.2,
                },
            ],
        )

        source_files = dict(iter_record_source_files(session))
        _require("metadata.json" in source_files, source_files)
        _require("record_manifest.json" in source_files, source_files)
        _require("camera_timestamps.csv" in source_files, source_files)
        _require(frame_name in source_files, source_files)
        _require("camera_browser_v2.mp4" not in source_files, source_files)
        _require(
            all(not name.endswith(".part") for name in source_files),
            source_files,
        )
        _require(
            record_source_path(session, frame_name).is_file(),
            "nested JPEG cannot be resolved",
        )

        _require(normalize_record_relative_path(frame_name) == frame_name, frame_name)
        for unsafe in (
            "../metadata.json",
            "/etc/passwd",
            "camera_frames/../metadata.json",
            "camera_frames\\segment_0000\\frame.jpg",
            "camera_frames/segment_0000/not-image.bin",
            "camera_browser_v2.mp4",
        ):
            _expect_reject(unsafe)

        builder = DatasetBuilder(
            recordings,
            datasets,
            config=DatasetBuildConfig(require_lidar=False),
        )
        document = builder.build(["run_jpeg"], "jpeg_only_dataset")
        _require(document["accepted_samples"] == 1, document)
        _require(document["rejected_samples"] == 1, document)
        session_summary = document["sessions"][0]
        _require(
            session_summary["rejected_reasons"].get("MISSING_CAMERA_FRAME") == 1,
            session_summary,
        )
        with open(
            os.path.join(datasets, "jpeg_only_dataset", "samples.jsonl"),
            "r",
            encoding="utf-8",
        ) as file:
            sample = json.loads(next(line for line in file if line.strip()))
        _require(sample["camera"]["video_path"] is None, sample["camera"])
        _require(
            sample["camera"]["saved_frame_path"]
            == "run_jpeg/camera_frames/segment_0000/frame_00000001.jpg",
            sample["camera"],
        )

        rover_source = open("compute_rover_api.py", encoding="utf-8").read()
        worker_source = open(
            "swing_compute/record_worker_extensions.py",
            encoding="utf-8",
        ).read()
        release_source = open("server_v2_release.py", encoding="utf-8").read()
        dashboard_tools_source = open(
            "unified_dashboard_data_tools.py",
            encoding="utf-8",
        ).read()
        _require(
            "recording_session_path" in rover_source and "iter_record_source_files" in rover_source,
            "Compute rover API is not using the removable-storage recursive manifest",
        )
        _require(
            "worker_class.sync_recordings = sync_recordings" in worker_source,
            "Worker recursive sync patch is not installed",
        )
        _require(
            "destination.parent.mkdir(parents=True, exist_ok=True)" in worker_source,
            "Worker cannot materialize nested camera frame directories",
        )
        _require(
            "state[\"dirty\"] >= 250" in worker_source,
            "Worker cache metadata is still rewritten per frame",
        )
        _require(
            "def _recordings_root_for_sessions(session_names):" in release_source
            and "recordings_root = _recordings_root_for_sessions(self.sessions)" in release_source,
            "Pi local dataset build is still pinned to the legacy microSD root",
        )
        _require(
            "from record_storage_runtime import install_record_storage_runtime"
            in dashboard_tools_source
            and "install_record_storage_runtime(_release.full.legacy)"
            in dashboard_tools_source,
            "Final runtime does not install the removable RECORD storage bridge",
        )

    print("JPEG RECORD transfer/dataset V2 regression: PASS")
    print(
        {
            "usb_runtime_resolution": "PASS",
            "recursive_camera_frames": "PASS",
            "path_escape_guard": "PASS",
            "derived_artifact_exclusion": "PASS",
            "jpeg_only_dataset": "PASS",
            "missing_frame_rejection": "PASS",
            "worker_incremental_cache": "PASS",
            "pi_local_usb_dataset_root": "PASS",
            "final_runtime_storage_hook": "PASS",
        }
    )


if __name__ == "__main__":
    main()
