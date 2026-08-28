import csv
import json
import os

from .dataset_builder import (
    DatasetBuildConfig,
    DatasetBuilder as _BaseDatasetBuilder,
    SessionBuildSummary,
)


class DatasetBuilder(_BaseDatasetBuilder):
    """V2 dataset builder aligned with AUTO_AI inference and label purity.

    New RECORD sessions store both raw `points` and temporally stabilized
    `safety_points` in each lidar_raw frame. AUTO_AI inference uses stabilized
    points, so training prefers the same representation. Historical recordings
    without `safety_points` remain usable and fall back to their raw points.

    Training data must represent human driving. Sessions explicitly produced by
    autonomous modes, or sessions containing autonomous/fault/e-stop states,
    are rejected instead of silently contaminating imitation-learning labels.
    """

    BLOCKED_MODES = {
        "AUTO_AI",
        "AUTO_GPS",
        "AUTO_LOCAL",
        "AUTO",
        "AUTO_ROUTE",
        "AUTO_HYBRID",
        "EMERGENCY_STOP",
        "FAULT",
    }
    MAXIMUM_LABEL_SKEW_SECONDS = max(
        0.01,
        float(os.environ.get("AI_MAX_LABEL_SKEW_SECONDS", "0.12")),
    )

    def _build_session(self, session_name, split):
        self._validate_manual_record_session(session_name)
        return super()._build_session(session_name, split)

    def _validate_manual_record_session(self, session_name):
        session_path = self._session_path(session_name)
        metadata_path = os.path.join(session_path, "metadata.json")
        if os.path.isfile(metadata_path):
            metadata = self._read_json_file(metadata_path)
            purpose = str(metadata.get("purpose") or "").strip().upper()
            if purpose.startswith("AUTO"):
                raise ValueError(
                    f"{session_name}: autonomous-purpose recordings cannot be used for AUTO_AI training"
                )

        state_path = os.path.join(session_path, "vehicle_state.csv")
        if not os.path.isfile(state_path):
            # Historical recordings may predate vehicle_state.csv. Keep them
            # usable, but modern V2 RECORD sessions are expected to have it.
            return

        blocked = set()
        with open(state_path, "r", encoding="utf-8", newline="") as file:
            for row in csv.DictReader(file):
                mode = str(row.get("mode") or "").strip().upper()
                if mode in self.BLOCKED_MODES:
                    blocked.add(mode)
        if blocked:
            raise ValueError(
                f"{session_name}: training session contains non-human states: {sorted(blocked)}"
            )

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
        # New RECORD sessions carry explicit source-frame vs control/steering
        # timestamp skew. Reject stale imitation labels before any learned
        # feature construction. Historical recordings without these columns are
        # accepted but remain explicitly marked as unverified below.
        steering_skew = self._float(camera_row.get("steering_skew_seconds"))
        control_skew = self._float(camera_row.get("control_skew_seconds"))
        if (
            steering_skew is not None
            and steering_skew > self.MAXIMUM_LABEL_SKEW_SECONDS
        ):
            return None, "STEERING_LABEL_NOT_SYNCHRONIZED"
        if (
            control_skew is not None
            and control_skew > self.MAXIMUM_LABEL_SKEW_SECONDS
        ):
            return None, "CONTROL_LABEL_NOT_SYNCHRONIZED"

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

        # Hard Safety remains outside the learned controller. Frames where the
        # supervisor actively stopped the car are not imitation-learning labels.
        timestamp = sample["timestamp_monotonic"]
        control_row, _ = self._nearest(control_index, timestamp)
        stop_reason = str((control_row or {}).get("stop_reason") or "").strip()
        if stop_reason:
            return None, "SAFETY_STOP_FRAME"

        # Steering already prefers the human target angle in the base builder.
        # For throttle, learn the operator's request rather than a post-Safety
        # limited value. Runtime SafetySupervisor applies hard limits separately.
        requested = sample["labels"].get("requested_throttle")
        if requested is not None:
            requested = float(requested)
            if abs(requested) < self.config.minimum_absolute_throttle:
                return None, "BELOW_MINIMUM_REQUESTED_THROTTLE"
            sample["labels"]["throttle"] = requested
            sample["labels"]["throttle_label_source"] = "requested_throttle"
        else:
            sample["labels"]["throttle_label_source"] = "legacy_final_throttle"
        sample["labels"]["steering_label_source"] = (
            "target_steering_angle_degrees"
            if sample["labels"].get("target_steering_degrees") is not None
            else "actual_steering_degrees"
        )
        sample.setdefault("synchronization", {}).update(
            {
                "steering_label_skew_seconds": steering_skew,
                "control_label_skew_seconds": control_skew,
                "label_alignment_verified": (
                    steering_skew is not None and control_skew is not None
                ),
            }
        )
        return sample, None

    @staticmethod
    def _read_lidar_raw(path):
        for row in _BaseDatasetBuilder._read_lidar_raw(path):
            document = dict(row)
            if "safety_points" in document:
                document["points"] = document.get("safety_points") or []
                document["_feature_points_source"] = "safety_points"
            else:
                document["_feature_points_source"] = "legacy_raw_points"
            yield document

    def build(self, session_names, dataset_id=None):
        document = super().build(session_names, dataset_id)
        contract = document.setdefault("feature_contract", {})
        # The base compatibility builder historically described final_throttle
        # as the primary label. The public V2 builder deliberately rewrites the
        # sample label to the human request, so publish the same contract in
        # dataset.json rather than leaving contradictory metadata behind.
        contract["steering_label_degrees"] = (
            "human target steering preferred; actual steering legacy fallback"
        )
        contract["throttle_label"] = (
            "human requested throttle preferred; legacy final throttle fallback"
        )
        contract["lidar_source_preference"] = (
            "recorded safety_points when available; legacy raw points fallback"
        )
        contract["label_source_policy"] = (
            "human RECORD only; target steering + requested throttle; "
            "autonomous/fault/e-stop sessions and Safety-stop frames rejected"
        )
        contract["label_alignment"] = {
            "maximum_skew_seconds": self.MAXIMUM_LABEL_SKEW_SECONDS,
            "modern_recordings": "reject camera/control or camera/steering skew above threshold",
            "legacy_recordings": "accepted when skew columns are absent; alignment marked unverified",
        }
        output_path = os.path.join(self.output_root, document["dataset_id"], "dataset.json")
        with open(output_path, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2)
        return document


__all__ = ["DatasetBuildConfig", "DatasetBuilder", "SessionBuildSummary"]
