import json
import math
import os

from .dataset_builder import DatasetBuildConfig, SessionBuildSummary
from .frame_dataset_builder import DatasetBuilder as HumanDatasetBuilder
from .gps_quality import GpsTrainingQualityPolicy
from autonomous_car.routes.gps_route import GpsRouteFeatureExtractor, NormalizedGpsRoute, ROUTE_FEATURE_ORDER


_STRAIGHT_STEERING_DEGREES = 2.0
_SHARP_STEERING_DEGREES = 8.0
_TRANSITION_RATE_DEGREES_PER_SECOND = 15.0
_TRANSITION_MAX_GAP_SECONDS = 0.50
_TRANSITION_CONTEXT_SECONDS = 0.25


class GpsDatasetBuilder(HumanDatasetBuilder):
    """Build AUTO_GPS imitation data from GPS-ON human RECORD sessions.

    The normalized reference route prefers RTK FIXED and may use DGPS/DGNSS as
    a fallback when a source session lacks enough RTK FIXED samples. Training
    samples additionally allow route-qualified RTK FLOAT / DGPS positions.
    Live AUTO_GPS runtime safety remains RTK FIXED only.

    GPS training keeps complete RECORD sessions isolated across splits. With the
    minimum useful set of exactly three sessions, two are used for training and
    one complete session is held out for validation. This avoids the generic
    1-train/1-validation/1-test rounding that would otherwise leave the model
    learning from only one drive.
    """

    def __init__(self, recordings_root, output_root, route_path, config=None, sectorizer=None,
                 maximum_route_deviation_m=3.0, maximum_gnss_skew_seconds=0.20):
        super().__init__(recordings_root, output_root, config=config, sectorizer=sectorizer)
        self.route_path = os.path.abspath(route_path)
        self.route = NormalizedGpsRoute.load(self.route_path)
        self.route_extractor = GpsRouteFeatureExtractor(self.route)
        self.maximum_route_deviation_m = max(0.25, float(maximum_route_deviation_m))
        self.maximum_gnss_skew_seconds = max(0.02, float(maximum_gnss_skew_seconds))
        self.gps_quality_policy = GpsTrainingQualityPolicy.from_environment(
            self.maximum_route_deviation_m
        )
        self._gps_quality_stats = {}

    def _assign_session_splits(self, sessions):
        result = super()._assign_session_splits(sessions)
        if len(result) == 3:
            test_sessions = [
                session for session, split in result.items() if split == "test"
            ]
            if len(test_sessions) == 1:
                result[test_sessions[0]] = "train"
        return result

    def _session_quality_stats(self, session_name):
        return self._gps_quality_stats.setdefault(
            session_name,
            {
                "candidate_frames": 0,
                "gnss_matched_frames": 0,
                "status_counts": {},
                "accepted_by_status": {},
                "rejected_by_reason": {},
            },
        )

    @staticmethod
    def _increment(mapping, key):
        mapping[key] = int(mapping.get(key) or 0) + 1

    def _quality_reject(self, session_name, reason):
        stats = self._session_quality_stats(session_name)
        self._increment(stats["rejected_by_reason"], reason)
        return None, reason

    @staticmethod
    def _steering_group(steering_degrees):
        magnitude = abs(float(steering_degrees))
        if magnitude < _STRAIGHT_STEERING_DEGREES:
            return "straight"
        if magnitude < _SHARP_STEERING_DEGREES:
            return "gentle"
        return "sharp"

    @classmethod
    def _annotate_steering_context(cls, samples):
        if not samples:
            return

        event_indices = set()
        for index, sample in enumerate(samples):
            labels = sample.get("labels") or {}
            steering = float(labels.get("steering_degrees") or 0.0)
            context = sample.setdefault("training_context", {})
            context.update(
                {
                    "steering_group": cls._steering_group(steering),
                    "steering_transition": False,
                    "steering_transition_event": False,
                    "previous_steering_delta_degrees": None,
                    "previous_steering_rate_dps": None,
                }
            )
            if index == 0:
                continue

            previous = samples[index - 1]
            previous_labels = previous.get("labels") or {}
            previous_steering = float(previous_labels.get("steering_degrees") or 0.0)
            try:
                timestamp = float(sample.get("timestamp_monotonic"))
                previous_timestamp = float(previous.get("timestamp_monotonic"))
            except (TypeError, ValueError):
                continue
            delta_time = timestamp - previous_timestamp
            if delta_time <= 0.0 or delta_time > _TRANSITION_MAX_GAP_SECONDS:
                continue

            delta = steering - previous_steering
            rate = delta / delta_time
            context["previous_steering_delta_degrees"] = delta
            context["previous_steering_rate_dps"] = rate
            group_changed = (
                cls._steering_group(steering)
                != cls._steering_group(previous_steering)
            )
            rapid_change = abs(rate) >= _TRANSITION_RATE_DEGREES_PER_SECOND
            if group_changed or rapid_change:
                event_indices.update((index - 1, index))
                context["steering_transition_event"] = True

        expanded = set(event_indices)
        for index in tuple(event_indices):
            center_time = samples[index].get("timestamp_monotonic")
            try:
                center_time = float(center_time)
            except (TypeError, ValueError):
                continue
            for neighbor in (index - 1, index + 1):
                if neighbor < 0 or neighbor >= len(samples):
                    continue
                try:
                    neighbor_time = float(samples[neighbor].get("timestamp_monotonic"))
                except (TypeError, ValueError):
                    continue
                if abs(neighbor_time - center_time) <= _TRANSITION_CONTEXT_SECONDS:
                    expanded.add(neighbor)

        for index in expanded:
            samples[index].setdefault("training_context", {})[
                "steering_transition"
            ] = True

    def _build_session(self, session_name, split):
        metadata_path = os.path.join(self._session_path(session_name), "metadata.json")
        if os.path.isfile(metadata_path):
            metadata = self._read_json_file(metadata_path)
            if metadata.get("record_gps") is False:
                raise ValueError(f"{session_name}: AUTO_GPS training requires GPS ON RECORD")
        if not os.path.isfile(os.path.join(self._session_path(session_name), "gnss.csv")):
            raise ValueError(f"{session_name}: AUTO_GPS training requires gnss.csv")
        summary, samples = super()._build_session(session_name, split)
        self._annotate_steering_context(samples)
        return summary, samples

    def _sample_from_camera_row(self, session_name, session_path, split, camera_row,
                                imu_index, lidar_index, gnss_index, control_index):
        sample, reason = super()._sample_from_camera_row(
            session_name, session_path, split, camera_row,
            imu_index, lidar_index, gnss_index, control_index,
        )
        if sample is None:
            return None, reason

        stats = self._session_quality_stats(session_name)
        stats["candidate_frames"] += 1
        timestamp = sample["timestamp_monotonic"]
        gnss_row, gnss_skew = self._nearest(gnss_index, timestamp)
        if gnss_row is None or gnss_skew > self.maximum_gnss_skew_seconds:
            return self._quality_reject(session_name, "GNSS_NOT_SYNCHRONIZED")

        stats["gnss_matched_frames"] += 1
        quality = self.gps_quality_policy.evaluate_row(gnss_row)
        self._increment(stats["status_counts"], quality["status"])
        if not quality["accepted"]:
            return self._quality_reject(session_name, quality["reason"])

        try:
            latitude = float(gnss_row["latitude"])
            longitude = float(gnss_row["longitude"])
        except (KeyError, TypeError, ValueError):
            return self._quality_reject(session_name, "GNSS_POSITION_MISSING")

        imu_row, imu_skew = self._nearest(imu_index, timestamp)
        if imu_row is None or imu_skew > self.config.maximum_imu_skew_seconds:
            return self._quality_reject(session_name, "GPS_AI_IMU_NOT_SYNCHRONIZED")
        heading = imu_row.get("yaw_degrees")
        if heading in {None, ""}:
            heading = imu_row.get("global_heading_degrees")
        try:
            heading = float(heading)
        except (TypeError, ValueError):
            return self._quality_reject(session_name, "GPS_AI_HEADING_MISSING")
        if not all(math.isfinite(value) for value in (latitude, longitude, heading)):
            return self._quality_reject(session_name, "GPS_AI_POSITION_NONFINITE")

        route = self.route_extractor.extract(latitude, longitude, heading)
        route_limit = self.gps_quality_policy.route_deviation_limit_m(quality["status"])
        if abs(route.cross_track_error_m) > route_limit:
            reason = (
                "TOO_FAR_FROM_NORMALIZED_ROUTE"
                if quality["tier"] == "FIXED"
                else "CONDITIONAL_FIX_TOO_FAR_FROM_ROUTE"
            )
            return self._quality_reject(session_name, reason)

        self._increment(stats["accepted_by_status"], quality["status"])
        sample["schema"] = "autonomy_gps_ai_sample_v1"
        sample["learned_features"]["route"] = route.as_dict()
        sample["synchronization"]["gnss_skew_seconds"] = gnss_skew
        sample["synchronization"]["gps_heading_imu_skew_seconds"] = imu_skew
        sample["evaluation_only"]["latitude"] = latitude
        sample["evaluation_only"]["longitude"] = longitude
        sample["evaluation_only"]["rtk_status"] = quality["status"]
        sample["evaluation_only"]["gps_quality_tier"] = quality["tier"]
        sample["evaluation_only"]["gnss_hdop"] = quality["hdop"]
        sample["evaluation_only"]["route_deviation_limit_m"] = route_limit
        return sample, None

    def _quality_summary(self):
        sessions = []
        total_candidate = 0
        total_matched = 0
        status_counts = {}
        accepted_by_status = {}
        rejected_by_reason = {}
        for session_name in sorted(self._gps_quality_stats):
            value = self._gps_quality_stats[session_name]
            accepted = sum(int(item) for item in value["accepted_by_status"].values())
            candidate = int(value["candidate_frames"])
            total_candidate += candidate
            total_matched += int(value["gnss_matched_frames"])
            for key, count in value["status_counts"].items():
                status_counts[key] = int(status_counts.get(key) or 0) + int(count)
            for key, count in value["accepted_by_status"].items():
                accepted_by_status[key] = int(accepted_by_status.get(key) or 0) + int(count)
            for key, count in value["rejected_by_reason"].items():
                rejected_by_reason[key] = int(rejected_by_reason.get(key) or 0) + int(count)
            sessions.append(
                {
                    "session": session_name,
                    **value,
                    "accepted_frames": accepted,
                    "eligibility_ratio": accepted / candidate if candidate else 0.0,
                }
            )
        total_accepted = sum(accepted_by_status.values())
        return {
            "policy": self.gps_quality_policy.as_dict(),
            "candidate_frames": total_candidate,
            "gnss_matched_frames": total_matched,
            "accepted_frames": total_accepted,
            "rejected_frames_after_base_filters": max(0, total_candidate - total_accepted),
            "eligibility_ratio": total_accepted / total_candidate if total_candidate else 0.0,
            "status_counts": status_counts,
            "accepted_by_status": accepted_by_status,
            "rejected_by_reason": rejected_by_reason,
            "sessions": sessions,
        }

    def build(self, session_names, dataset_id=None):
        self._gps_quality_stats = {}
        document = super().build(session_names, dataset_id)
        document["schema"] = "autonomy_gps_ai_dataset_v1"
        document["policy_type"] = "AUTO_GPS"
        route_fix_policy = str(
            self.route.quality.get("reference_fix_policy") or "RTK_FIXED_ONLY"
        )
        document["route"] = {
            "route_id": self.route.route_id,
            "route_path": self.route_path,
            "source_sessions": list(self.route.source_sessions),
            "quality": dict(self.route.quality),
            "fix_policy": route_fix_policy,
        }
        document["gps_quality"] = self._quality_summary()
        document["gps_training_policy"] = {
            "three_session_split": "2_train_1_validation_no_test",
            "session_isolation": True,
            "steering_groups_degrees": {
                "straight_abs_lt": _STRAIGHT_STEERING_DEGREES,
                "gentle_abs_lt": _SHARP_STEERING_DEGREES,
                "sharp_abs_gte": _SHARP_STEERING_DEGREES,
            },
            "transition_detection": {
                "group_boundary_change": True,
                "minimum_abs_steering_rate_dps": _TRANSITION_RATE_DEGREES_PER_SECOND,
                "maximum_neighbor_gap_seconds": _TRANSITION_MAX_GAP_SECONDS,
                "context_seconds": _TRANSITION_CONTEXT_SECONDS,
            },
        }
        document["feature_contract"]["gps_route"] = {
            "raw_lat_lon_as_model_input": False,
            "feature_order": list(ROUTE_FEATURE_ORDER),
            "contract": (
                "normalized route-relative features from an RTK-FIXED-preferred, "
                "DGPS-fallback reference route + IMU heading; training samples allow "
                "RTK FIXED and route-qualified RTK FLOAT/DGPS"
            ),
            "reference_fix_policy": route_fix_policy,
            "runtime_fix_policy": "RTK_FIXED_ONLY",
        }
        document["feature_contract"]["training_context"] = {
            "model_input": False,
            "purpose": "curve/transition-aware loss weighting and validation only",
            "fields": [
                "steering_group",
                "steering_transition",
                "steering_transition_event",
                "previous_steering_delta_degrees",
                "previous_steering_rate_dps",
            ],
        }
        output_path = os.path.join(self.output_root, document["dataset_id"], "dataset.json")
        with open(output_path, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2)
        return document


__all__ = ["DatasetBuildConfig", "GpsDatasetBuilder", "SessionBuildSummary"]
