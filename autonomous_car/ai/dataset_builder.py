import bisect
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import struct
import zlib

from .features import LidarSectorizer


@dataclass(frozen=True)
class DatasetBuildConfig:
    maximum_imu_skew_seconds: float = 0.12
    maximum_lidar_skew_seconds: float = 0.20
    require_lidar: bool = True
    require_imu: bool = False
    minimum_absolute_throttle: float = 0.0
    train_ratio: float = 0.75
    validation_ratio: float = 0.125
    test_ratio: float = 0.125

    def validated(self):
        ratios = [self.train_ratio, self.validation_ratio, self.test_ratio]
        if any(value < 0.0 for value in ratios):
            raise ValueError("Dataset split ratios must be non-negative")
        if abs(sum(ratios) - 1.0) > 1e-6:
            raise ValueError("Dataset split ratios must sum to 1.0")
        return self


@dataclass(frozen=True)
class SessionBuildSummary:
    session: str
    split: str
    accepted_samples: int
    rejected_samples: int
    rejected_reasons: dict[str, int]
    scenario_counts: dict[str, int]
    record_gps: bool | None


class DatasetBuilder:
    """Build a timestamp-aligned AUTO_AI manifest from RECORD sessions.

    New RECORD sessions keep frame-addressable segmented JPEGs and no longer
    require an MP4 on the rover. Historical camera.mp4 sessions remain usable.
    Samples therefore prefer a verified saved_frame_path and carry the legacy
    zero-based video frame index only when camera.mp4 exists.
    """

    SCHEMA = "autonomy_ai_dataset_v1"

    def __init__(self, recordings_root, output_root, config=None, sectorizer=None):
        self.recordings_root = os.path.abspath(recordings_root)
        self.output_root = os.path.abspath(output_root)
        self.config = (config or DatasetBuildConfig()).validated()
        self.sectorizer = sectorizer or LidarSectorizer()

    def build(self, session_names, dataset_id=None):
        sessions = self._validate_sessions(session_names)
        if not sessions:
            raise ValueError("At least one RECORD session is required")

        dataset_id = self._safe_dataset_id(dataset_id or self._default_dataset_id())
        output_path = os.path.join(self.output_root, dataset_id)
        if os.path.exists(output_path):
            raise FileExistsError(f"Dataset already exists: {dataset_id}")
        os.makedirs(output_path, exist_ok=False)

        split_by_session = self._assign_session_splits(sessions)
        samples_path = os.path.join(output_path, "samples.jsonl")
        summaries = []
        total_samples = 0
        total_rejected = 0
        split_counts = {"train": 0, "validation": 0, "test": 0}
        scenario_counts = {}

        try:
            with open(samples_path, "w", encoding="utf-8") as manifest:
                for session in sessions:
                    summary, samples = self._build_session(
                        session,
                        split_by_session[session],
                    )
                    summaries.append(summary)
                    for sample in samples:
                        manifest.write(json.dumps(sample, separators=(",", ":")) + "\n")
                        total_samples += 1
                        split_counts[sample["split"]] += 1
                        scenario = sample["scenario"]
                        scenario_counts[scenario] = scenario_counts.get(scenario, 0) + 1
                    total_rejected += summary.rejected_samples

            document = {
                "schema": self.SCHEMA,
                "dataset_id": dataset_id,
                "created_at": datetime.now(timezone.utc).isoformat(),
                "recordings_root": self.recordings_root,
                "sample_manifest": "samples.jsonl",
                "session_split_policy": "whole_session",
                "config": asdict(self.config),
                "feature_contract": {
                    "camera": (
                        "segmented JPEG saved_frame_path preferred; legacy camera.mp4 "
                        "+ zero_based_video_frame_index fallback"
                    ),
                    "lidar": "7 sector nearest distance meters + observed mask",
                    "lidar_sector_order": [
                        "far_right",
                        "right",
                        "front_right",
                        "front",
                        "front_left",
                        "left",
                        "far_left",
                    ],
                    "speed_mps": "optional; only non-null when a recorded source exists",
                    "imu_yaw_rate_dps": "optional timestamp-aligned IMU feature",
                    "steering_label_degrees": "target steering preferred, actual steering fallback",
                    "throttle_label": "final throttle preferred, requested throttle fallback",
                    "gps": "not a learned-driving input",
                },
                "sessions": [asdict(summary) for summary in summaries],
                "accepted_samples": total_samples,
                "rejected_samples": total_rejected,
                "split_counts": split_counts,
                "scenario_counts": scenario_counts,
            }
            with open(os.path.join(output_path, "dataset.json"), "w", encoding="utf-8") as file:
                json.dump(document, file, ensure_ascii=False, indent=2)
            return document
        except Exception:
            # Keep failure deterministic: an incomplete dataset is never left
            # looking valid. Files remain for diagnostics but no dataset.json
            # is emitted unless the build reaches the end successfully.
            raise

    def _build_session(self, session_name, split):
        session_path = self._session_path(session_name)
        camera_path = os.path.join(session_path, "camera_timestamps.csv")
        video_path = os.path.join(session_path, "camera.mp4")
        camera_frames_path = os.path.join(session_path, "camera_frames")
        lidar_path = os.path.join(session_path, "lidar_raw.bin")
        imu_path = os.path.join(session_path, "imu.csv")
        gnss_path = os.path.join(session_path, "gnss.csv")
        control_path = os.path.join(session_path, "control.csv")
        metadata_path = os.path.join(session_path, "metadata.json")

        if not os.path.isfile(camera_path):
            raise FileNotFoundError(f"{session_name}: camera_timestamps.csv not found")
        if not os.path.isfile(video_path) and not os.path.isdir(camera_frames_path):
            raise FileNotFoundError(
                f"{session_name}: neither segmented camera frames nor camera.mp4 found"
            )
        if self.config.require_lidar and not os.path.isfile(lidar_path):
            raise FileNotFoundError(f"{session_name}: lidar_raw.bin not found")

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
        with open(camera_path, "r", encoding="utf-8", newline="") as file:
            for camera_row in csv.DictReader(file):
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
        timestamp = self._float(camera_row.get("monotonic"))
        frame_number = self._int(camera_row.get("frame_number"))
        if timestamp is None or frame_number is None or frame_number <= 0:
            return None, "INVALID_CAMERA_TIMESTAMP"

        target_steering = self._float(camera_row.get("target_steering_angle_degrees"))
        actual_steering = self._float(camera_row.get("steering_angle_degrees"))
        steering_label = target_steering if target_steering is not None else actual_steering
        if steering_label is None:
            return None, "MISSING_STEERING_LABEL"

        final_throttle = self._float(camera_row.get("final_throttle"))
        requested_throttle = self._float(camera_row.get("requested_throttle"))
        throttle_label = final_throttle if final_throttle is not None else requested_throttle
        if throttle_label is None:
            return None, "MISSING_THROTTLE_LABEL"
        if abs(throttle_label) < self.config.minimum_absolute_throttle:
            return None, "BELOW_MINIMUM_THROTTLE"

        imu_row, imu_skew = self._nearest(imu_index, timestamp)
        if self.config.require_imu and (
            imu_row is None or imu_skew > self.config.maximum_imu_skew_seconds
        ):
            return None, "IMU_NOT_SYNCHRONIZED"
        if imu_row is not None and imu_skew > self.config.maximum_imu_skew_seconds:
            imu_row = None

        lidar_row, lidar_skew = self._nearest(lidar_index, timestamp)
        if self.config.require_lidar and (
            lidar_row is None or lidar_skew > self.config.maximum_lidar_skew_seconds
        ):
            return None, "LIDAR_NOT_SYNCHRONIZED"
        if lidar_row is not None and lidar_skew > self.config.maximum_lidar_skew_seconds:
            lidar_row = None

        lidar = self.sectorizer.transform((lidar_row or {}).get("points") or [])
        gnss_row, _ = self._nearest(gnss_index, timestamp)
        control_row, _ = self._nearest(control_index, timestamp)

        # GNSS speed is retained only as evaluation metadata. It is explicitly
        # not included in learned_features because AUTO_AI must work without GPS.
        evaluation_speed = self._float((gnss_row or {}).get("speed_mps"))
        commanded_speed = self._float((control_row or {}).get("target_speed_mps"))
        imu_yaw_rate = self._float((imu_row or {}).get("yaw_rate_dps"))

        scenario = self._scenario(steering_label, lidar.distances_m, lidar.observed)
        relative_session = os.path.relpath(session_path, self.recordings_root).replace(os.sep, "/")
        saved_filename = str(camera_row.get("filename") or "").strip()
        saved_relative = self._saved_frame_relative(session_path, saved_filename)
        video_exists = os.path.isfile(os.path.join(session_path, "camera.mp4"))
        if saved_filename and saved_relative is None and not video_exists:
            return None, "MISSING_CAMERA_FRAME"
        if saved_relative is None and not video_exists:
            return None, "MISSING_CAMERA_SOURCE"

        saved_frame_path = (
            f"{relative_session}/{saved_relative}" if saved_relative is not None else None
        )
        video_relative = f"{relative_session}/camera.mp4" if video_exists else None
        return {
            "schema": "autonomy_ai_sample_v1",
            "session": session_name,
            "split": split,
            "timestamp_monotonic": timestamp,
            "camera": {
                "video_path": video_relative,
                "video_frame_index": frame_number - 1 if video_exists else None,
                "source_sequence": self._int(camera_row.get("source_sequence")),
                "saved_frame_path": saved_frame_path,
            },
            "learned_features": {
                "lidar": lidar.as_dict(),
                "imu_yaw_rate_dps": imu_yaw_rate,
                # A non-GPS speed sensor may populate this in a future recorder
                # revision without changing the dataset schema.
                "speed_mps": None,
            },
            "labels": {
                "steering_degrees": steering_label,
                "throttle": throttle_label,
                "actual_steering_degrees": actual_steering,
                "target_steering_degrees": target_steering,
                "requested_throttle": requested_throttle,
                "final_throttle": final_throttle,
            },
            "evaluation_only": {
                "gnss_speed_mps": evaluation_speed,
                "commanded_target_speed_mps": commanded_speed,
            },
            "synchronization": {
                "imu_skew_seconds": imu_skew if imu_row is not None else None,
                "lidar_skew_seconds": lidar_skew if lidar_row is not None else None,
            },
            "scenario": scenario,
        }, None

    @staticmethod
    def _saved_frame_relative(session_path, value):
        raw = str(value or "").strip().replace("\\", "/")
        if not raw or raw.startswith("/") or "\x00" in raw:
            return None
        relative = raw if raw.startswith("camera_frames/") else "camera_frames/" + raw
        parts = relative.split("/")
        if any(part in {"", ".", ".."} for part in parts):
            return None
        root = os.path.realpath(session_path)
        candidate = os.path.realpath(os.path.join(root, *parts))
        try:
            if os.path.commonpath([root, candidate]) != root:
                return None
        except ValueError:
            return None
        if not os.path.isfile(candidate):
            return None
        return "/".join(parts)

    def _assign_session_splits(self, sessions):
        ordered = sorted(
            sessions,
            key=lambda name: hashlib.sha256(name.encode("utf-8")).hexdigest(),
        )
        count = len(ordered)
        if count == 1:
            return {ordered[0]: "train"}
        if count == 2:
            return {ordered[0]: "train", ordered[1]: "validation"}

        test_count = max(1, round(count * self.config.test_ratio)) if self.config.test_ratio else 0
        validation_count = (
            max(1, round(count * self.config.validation_ratio))
            if self.config.validation_ratio
            else 0
        )
        while test_count + validation_count >= count:
            if validation_count > 1:
                validation_count -= 1
            elif test_count > 1:
                test_count -= 1
            else:
                break
        train_count = count - validation_count - test_count
        result = {}
        for index, name in enumerate(ordered):
            if index < train_count:
                result[name] = "train"
            elif index < train_count + validation_count:
                result[name] = "validation"
            else:
                result[name] = "test"
        return result

    @staticmethod
    def _scenario(steering_degrees, distances, observed):
        steering = float(steering_degrees)
        front_observed = observed.get("front", False)
        front = distances.get("front")
        obstacle_context = front_observed and front is not None and front < 1.5
        absolute = abs(steering)
        if absolute < 2.0:
            steering_class = "straight"
        elif absolute < 8.0:
            steering_class = "gentle_left" if steering > 0 else "gentle_right"
        else:
            steering_class = "sharp_left" if steering > 0 else "sharp_right"
        return f"obstacle_{steering_class}" if obstacle_context else steering_class

    def _validate_sessions(self, session_names):
        result = []
        seen = set()
        for value in session_names or ():
            name = os.path.basename(str(value or "").strip())
            if not name or name in {".", ".."} or name != str(value).strip():
                raise ValueError(f"Invalid RECORD session name: {value}")
            if name in seen:
                continue
            self._session_path(name)
            seen.add(name)
            result.append(name)
        return result

    def _session_path(self, session_name):
        path = os.path.abspath(os.path.join(self.recordings_root, session_name))
        if os.path.commonpath([self.recordings_root, path]) != self.recordings_root:
            raise ValueError("Session path escapes recordings root")
        if not os.path.isdir(path):
            raise FileNotFoundError(f"RECORD session not found: {session_name}")
        return path

    @staticmethod
    def _read_json_file(path):
        with open(path, "r", encoding="utf-8") as file:
            value = json.load(file)
        return value if isinstance(value, dict) else {}

    @staticmethod
    def _read_timed_csv(path):
        rows = []
        with open(path, "r", encoding="utf-8", newline="") as file:
            for row in csv.DictReader(file):
                timestamp = DatasetBuilder._float(row.get("monotonic"))
                if timestamp is None:
                    continue
                row["_timestamp"] = timestamp
                rows.append(row)
        rows.sort(key=lambda row: row["_timestamp"])
        return rows

    @staticmethod
    def _read_lidar_raw(path):
        with open(path, "rb") as file:
            while True:
                header = file.read(12)
                if not header:
                    return
                if len(header) != 12:
                    raise ValueError("Truncated lidar_raw.bin header")
                timestamp, payload_size = struct.unpack("<dI", header)
                if payload_size <= 0 or payload_size > 32 * 1024 * 1024:
                    raise ValueError("Invalid lidar_raw.bin payload size")
                payload = file.read(payload_size)
                if len(payload) != payload_size:
                    raise ValueError("Truncated lidar_raw.bin payload")
                try:
                    row = json.loads(zlib.decompress(payload).decode("utf-8"))
                except (ValueError, zlib.error, UnicodeDecodeError) as error:
                    raise ValueError(f"Invalid lidar_raw.bin payload: {error}") from error
                if not isinstance(row, dict):
                    continue
                row["_timestamp"] = float(timestamp)
                yield row

    @staticmethod
    def _time_index(rows):
        return ([row["_timestamp"] for row in rows], rows)

    @staticmethod
    def _nearest(index, timestamp):
        times, rows = index
        if not times:
            return None, math.inf
        position = bisect.bisect_left(times, timestamp)
        candidates = []
        if position < len(times):
            candidates.append(position)
        if position > 0:
            candidates.append(position - 1)
        best = min(candidates, key=lambda item: abs(times[item] - timestamp))
        return rows[best], abs(times[best] - timestamp)

    @staticmethod
    def _float(value):
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    @staticmethod
    def _int(value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _safe_dataset_id(value):
        value = str(value or "").strip()
        if not value or value in {".", ".."} or os.path.basename(value) != value:
            raise ValueError("Invalid dataset ID")
        allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.")
        if any(character not in allowed for character in value):
            raise ValueError("Dataset ID may contain only letters, numbers, -, _, and .")
        return value

    @staticmethod
    def _default_dataset_id():
        return datetime.now().strftime("dataset_%Y-%m-%d_%H-%M-%S")
