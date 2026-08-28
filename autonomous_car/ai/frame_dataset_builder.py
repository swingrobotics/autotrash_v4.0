"""Dataset compatibility for low-latency segmented-JPEG RECORD sessions."""

from __future__ import annotations

import csv
import json
import os

from .aligned_dataset_builder import DatasetBuilder as _AlignedDatasetBuilder
from .dataset_builder import SessionBuildSummary


class DatasetBuilder(_AlignedDatasetBuilder):
    """Prefer timestamped saved JPEGs while preserving legacy MP4 sessions."""

    def _build_session(self, session_name, split):
        self._validate_manual_record_session(session_name)
        session_path = self._session_path(session_name)
        camera_path = os.path.join(session_path, "camera_timestamps.csv")
        video_path = os.path.join(session_path, "camera.mp4")
        lidar_path = os.path.join(session_path, "lidar_raw.bin")
        imu_path = os.path.join(session_path, "imu.csv")
        gnss_path = os.path.join(session_path, "gnss.csv")
        control_path = os.path.join(session_path, "control.csv")
        metadata_path = os.path.join(session_path, "metadata.json")

        if not os.path.isfile(camera_path):
            raise FileNotFoundError(f"{session_name}: camera_timestamps.csv not found")
        if self.config.require_lidar and not os.path.isfile(lidar_path):
            raise FileNotFoundError(f"{session_name}: lidar_raw.bin not found")

        camera_rows = []
        with open(camera_path, "r", encoding="utf-8", newline="") as file:
            camera_rows = list(csv.DictReader(file))
        if not camera_rows:
            raise ValueError(f"{session_name}: camera_timestamps.csv has no frames")

        has_video = os.path.isfile(video_path)
        has_saved_frame = any(
            self._saved_frame_absolute(session_path, row.get("filename")) is not None
            for row in camera_rows
        )
        if not has_video and not has_saved_frame:
            raise FileNotFoundError(
                f"{session_name}: neither segmented camera frames nor camera.mp4 were found"
            )

        metadata = self._read_json_file(metadata_path) if os.path.isfile(metadata_path) else {}
        imu_rows = self._read_timed_csv(imu_path) if os.path.isfile(imu_path) else []
        gnss_rows = self._read_timed_csv(gnss_path) if os.path.isfile(gnss_path) else []
        control_rows = self._read_timed_csv(control_path) if os.path.isfile(control_path) else []
        lidar_rows = list(self._read_lidar_raw(lidar_path)) if os.path.isfile(lidar_path) else []

        imu_index = self._time_index(imu_rows)
        gnss_index = self._time_index(gnss_rows)
        control_index = self._time_index(control_rows)
        lidar_index = self._time_index(lidar_rows)

        accepted = []
        rejected = {}
        scenarios = {}
        for camera_row in camera_rows:
            sample, reason = self._sample_from_camera_row(
                session_name,
                session_path,
                split,
                camera_row,
                imu_index,
                lidar_index,
                gnss_index,
                control_index,
            )
            if sample is None:
                rejected[reason] = rejected.get(reason, 0) + 1
                continue
            camera = sample.get("camera") or {}
            saved = camera.get("saved_frame_path")
            if not saved and not has_video:
                rejected["CAMERA_FRAME_FILE_MISSING"] = (
                    rejected.get("CAMERA_FRAME_FILE_MISSING", 0) + 1
                )
                continue
            accepted.append(sample)
            scenario = sample["scenario"]
            scenarios[scenario] = scenarios.get(scenario, 0) + 1

        summary = SessionBuildSummary(
            session=session_name,
            split=split,
            accepted_samples=len(accepted),
            rejected_samples=sum(rejected.values()),
            rejected_reasons=rejected,
            scenario_counts=scenarios,
            record_gps=metadata.get("record_gps"),
        )
        return summary, accepted

    def _sample_from_camera_row(
        self,
        session_name,
        session_path,
        split,
        camera_row,
        imu_index,
        lidar_index,
        gnss_index,
        control_index,
    ):
        sample, reason = super()._sample_from_camera_row(
            session_name,
            session_path,
            split,
            camera_row,
            imu_index,
            lidar_index,
            gnss_index,
            control_index,
        )
        if sample is None:
            return None, reason

        camera = sample.setdefault("camera", {})
        absolute = self._saved_frame_absolute(session_path, camera_row.get("filename"))
        if absolute is not None:
            relative_session = os.path.relpath(session_path, self.recordings_root)
            relative_frame = os.path.relpath(absolute, session_path)
            camera["saved_frame_path"] = os.path.join(
                relative_session, relative_frame
            ).replace("\\", "/")
        else:
            camera["saved_frame_path"] = None

        if not os.path.isfile(os.path.join(session_path, "camera.mp4")):
            camera["video_path"] = None
            camera["video_frame_index"] = None
        camera["source_kind"] = (
            "SEGMENTED_JPEG" if camera.get("saved_frame_path") else "LEGACY_MP4"
        )
        return sample, None

    def build(self, session_names, dataset_id=None):
        document = super().build(session_names, dataset_id)
        contract = document.setdefault("feature_contract", {})
        contract["camera"] = (
            "timestamped saved JPEG preferred; legacy camera.mp4 + frame index fallback"
        )
        output_path = os.path.join(
            self.output_root, document["dataset_id"], "dataset.json"
        )
        with open(output_path, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2)
        return document

    @staticmethod
    def _saved_frame_absolute(session_path, filename):
        raw = str(filename or "").strip().replace("\\", "/")
        if not raw:
            return None
        if raw.startswith("camera_frames/"):
            relative = raw
        else:
            relative = "camera_frames/" + raw.lstrip("/")
        root = os.path.realpath(session_path)
        candidate = os.path.realpath(os.path.join(root, *relative.split("/")))
        try:
            if os.path.commonpath([root, candidate]) != root:
                return None
        except ValueError:
            return None
        return candidate if os.path.isfile(candidate) else None


__all__ = ["DatasetBuilder"]
