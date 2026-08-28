"""Temporal AUTO_GPS training built on synchronized human RECORD sessions.

New GPS candidates use a compact 0.5 s history at the normal 10 Hz control
rate: five IMU yaw-rate observations plus five *previous* steering commands.
Each scalar has a presence bit, producing a 20-value auxiliary vector. The
normalized route remains a reference line; legitimate human correction events
are retained separately instead of forcing a one-off correction into the route.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import csv
import hashlib
import json
import math
import os

from .dataset_builder import DatasetBuildConfig, SessionBuildSummary
from .gps_dataset import GpsDatasetBuilder as _LegacyGpsDatasetBuilder
from .gps_quality import GpsTrainingQualityPolicy, RTK_FIXED
from .training import ManifestDataset, TrainingConfig, require_training_dependencies
from autonomous_car.routes.gps_route import ROUTE_FEATURE_ORDER


TEMPORAL_HISTORY_STEPS = 5
TEMPORAL_AUXILIARY_VALUES_PER_STEP = 4
TEMPORAL_AUXILIARY_SIZE = TEMPORAL_HISTORY_STEPS * TEMPORAL_AUXILIARY_VALUES_PER_STEP
TEMPORAL_MAX_GAP_SECONDS = 0.30
RECOVERY_STEERING_MIN_DEGREES = 2.0
DEFAULT_FIXED_RECOVERY_DEVIATION_M = 4.5
MINIMUM_CURVE_COVERAGE_FRAMES = 3


@dataclass(frozen=True)
class GpsDrivingModelSpec:
    image_width: int = 160
    image_height: int = 90
    maximum_steering_degrees: float = 20.0
    maximum_abs_yaw_rate_dps: float = 90.0
    lidar_maximum_distance_m: float = 8.0
    route_feature_size: int = len(ROUTE_FEATURE_ORDER)
    output_size: int = 2
    temporal_history_steps: int = TEMPORAL_HISTORY_STEPS
    auxiliary_feature_size: int = TEMPORAL_AUXILIARY_SIZE


@dataclass(frozen=True)
class GpsTrainingPolicy:
    straight_steering_loss_weight: float = 1.0
    gentle_steering_loss_weight: float = 1.5
    sharp_steering_loss_weight: float = 2.5
    transition_steering_loss_weight: float = 2.0
    recovery_steering_loss_weight: float = 3.0
    maximum_steering_loss_weight: float = 3.0
    early_stopping_patience: int = 5
    early_stopping_min_delta: float = 5e-4


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _steering_group(value):
    value = float(value)
    magnitude = abs(value)
    if magnitude < 2.0:
        return "straight"
    side = "left" if value > 0.0 else "right"
    return ("gentle_" if magnitude < 8.0 else "sharp_") + side


def _pad(values, size):
    values = list(values or [])[-size:]
    return [None] * max(0, size - len(values)) + values


def encode_temporal_auxiliary(sample, spec):
    """Encode oldest->newest yaw and previous-steering history."""
    steps = max(1, int(spec.temporal_history_steps))
    temporal = ((sample.get("learned_features") or {}).get("temporal") or {})
    yaw = temporal.get("yaw_rate_history_dps")
    steering = temporal.get("previous_steering_history_degrees")
    if not isinstance(yaw, list):
        yaw = [(sample.get("learned_features") or {}).get("imu_yaw_rate_dps")]
    if not isinstance(steering, list):
        steering = []
    yaw = _pad(yaw, steps)
    steering = _pad(steering, steps)
    maximum_yaw = max(1e-6, abs(float(spec.maximum_abs_yaw_rate_dps)))
    maximum_steering = max(1e-6, abs(float(spec.maximum_steering_degrees)))
    result = []
    for yaw_value, steering_value in zip(yaw, steering):
        y = _finite(yaw_value)
        s = _finite(steering_value)
        result.extend(
            [
                0.0 if y is None else max(-1.0, min(1.0, y / maximum_yaw)),
                0.0 if y is None else 1.0,
                0.0 if s is None else max(-1.0, min(1.0, s / maximum_steering)),
                0.0 if s is None else 1.0,
            ]
        )
    if len(result) != int(spec.auxiliary_feature_size):
        raise ValueError(
            f"Temporal auxiliary size mismatch: got {len(result)}, expected {spec.auxiliary_feature_size}"
        )
    return result


class GpsDatasetBuilder(_LegacyGpsDatasetBuilder):
    """GPS dataset that protects rare direction/recovery demonstrations."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.nominal_fixed_route_deviation_m = float(
            self.gps_quality_policy.maximum_fixed_route_deviation_m
        )
        requested = max(
            self.nominal_fixed_route_deviation_m,
            float(
                os.environ.get(
                    "GPS_AI_MAX_FIXED_RECOVERY_ROUTE_DEVIATION_M",
                    str(DEFAULT_FIXED_RECOVERY_DEVIATION_M),
                )
            ),
        )
        self.maximum_fixed_recovery_deviation_m = requested
        policy = self.gps_quality_policy
        self.gps_quality_policy = GpsTrainingQualityPolicy(
            maximum_conditional_hdop=policy.maximum_conditional_hdop,
            maximum_fixed_route_deviation_m=requested,
            maximum_conditional_route_deviation_m=policy.maximum_conditional_route_deviation_m,
        )

    def _raw_steering_counts(self, session_name):
        path = os.path.join(self._session_path(session_name), "camera_timestamps.csv")
        counts = Counter()
        if not os.path.isfile(path):
            return counts
        with open(path, "r", encoding="utf-8", newline="") as handle:
            for row in csv.DictReader(handle):
                value = _finite(row.get("target_steering_angle_degrees"))
                if value is None:
                    value = _finite(row.get("steering_angle_degrees"))
                if value is not None:
                    counts[_steering_group(value)] += 1
        return counts

    def _assign_session_splits(self, sessions):
        # With the minimum three runs, choose the held-out run so a direction
        # demonstrated in only one run is not accidentally removed from train.
        sessions = list(sessions)
        if len(sessions) != 3:
            return super()._assign_session_splits(sessions)
        counts = {session: self._raw_steering_counts(session) for session in sessions}
        total = Counter()
        for values in counts.values():
            total.update(values)
        all_curve_groups = {
            group
            for group, count in total.items()
            if group != "straight" and count >= MINIMUM_CURVE_COVERAGE_FRAMES
        }
        ordered = sorted(
            sessions,
            key=lambda name: hashlib.sha256(name.encode("utf-8")).hexdigest(),
        )
        candidates = []
        for validation in ordered:
            train = [name for name in ordered if name != validation]
            train_counts = Counter()
            for name in train:
                train_counts.update(counts[name])
            missing = sorted(
                group for group in all_curve_groups if train_counts.get(group, 0) <= 0
            )
            minimum_curve_count = min(
                (train_counts.get(group, 0) for group in all_curve_groups),
                default=0,
            )
            validation_curve_count = sum(
                count for group, count in counts[validation].items() if group != "straight"
            )
            candidates.append(
                (
                    len(missing),
                    -minimum_curve_count,
                    -validation_curve_count,
                    ordered.index(validation),
                    validation,
                )
            )
        validation = min(candidates)[-1]
        return {
            session: ("validation" if session == validation else "train")
            for session in sessions
        }

    def _sample_from_camera_row(self, *args, **kwargs):
        sample, reason = super()._sample_from_camera_row(*args, **kwargs)
        if sample is None:
            return None, reason
        route = ((sample.get("learned_features") or {}).get("route") or {})
        deviation = abs(float(route.get("cross_track_error_m") or 0.0))
        status = str((sample.get("evaluation_only") or {}).get("rtk_status") or "")
        recovery = status == RTK_FIXED and deviation > self.nominal_fixed_route_deviation_m
        if recovery:
            steering = abs(float((sample.get("labels") or {}).get("steering_degrees") or 0.0))
            if steering < RECOVERY_STEERING_MIN_DEGREES:
                session_name = args[0] if args else kwargs.get("session_name")
                if session_name:
                    stats = self._session_quality_stats(session_name)
                    accepted = int(stats["accepted_by_status"].get(status) or 0)
                    if accepted > 0:
                        stats["accepted_by_status"][status] = accepted - 1
                    self._increment(
                        stats["rejected_by_reason"],
                        "FIXED_ROUTE_DEVIATION_WITHOUT_HUMAN_CORRECTION",
                    )
                return None, "FIXED_ROUTE_DEVIATION_WITHOUT_HUMAN_CORRECTION"
        sample.setdefault("training_context", {})["route_recovery"] = recovery
        sample["training_context"]["route_deviation_m"] = deviation
        sample["training_context"]["nominal_route_limit_m"] = (
            self.nominal_fixed_route_deviation_m
        )
        return sample, None

    @staticmethod
    def _annotate_temporal_context(samples):
        for index, sample in enumerate(samples):
            current_time = _finite(sample.get("timestamp_monotonic"))
            chain = []
            previous_time = current_time
            cursor = index
            while cursor >= 0 and len(chain) < TEMPORAL_HISTORY_STEPS + 1:
                candidate = samples[cursor]
                timestamp = _finite(candidate.get("timestamp_monotonic"))
                if timestamp is None:
                    break
                if previous_time is not None and previous_time - timestamp > TEMPORAL_MAX_GAP_SECONDS:
                    break
                chain.append(candidate)
                previous_time = timestamp
                cursor -= 1
            chain.reverse()
            yaw_samples = chain[-TEMPORAL_HISTORY_STEPS:]
            steering_samples = chain[:-1][-TEMPORAL_HISTORY_STEPS:]
            yaw_history = [
                _finite((item.get("learned_features") or {}).get("imu_yaw_rate_dps"))
                for item in yaw_samples
            ]
            steering_history = [
                _finite((item.get("labels") or {}).get("steering_degrees"))
                for item in steering_samples
            ]
            yaw_history = _pad(yaw_history, TEMPORAL_HISTORY_STEPS)
            steering_history = _pad(steering_history, TEMPORAL_HISTORY_STEPS)
            sample.setdefault("learned_features", {})["temporal"] = {
                "history_steps": TEMPORAL_HISTORY_STEPS,
                "yaw_rate_history_dps": yaw_history,
                "previous_steering_history_degrees": steering_history,
                "current_steering_excluded": True,
                "maximum_neighbor_gap_seconds": TEMPORAL_MAX_GAP_SECONDS,
            }

    def _build_session(self, session_name, split):
        summary, samples = super()._build_session(session_name, split)
        self._annotate_temporal_context(samples)
        return summary, samples

    def build(self, session_names, dataset_id=None):
        normalized_sessions = list(dict.fromkeys(str(value) for value in session_names or []))
        raw_counts = Counter()
        for session in normalized_sessions:
            raw_counts.update(self._raw_steering_counts(session))
        required_raw_curves = {
            group
            for group, count in raw_counts.items()
            if group != "straight" and count >= MINIMUM_CURVE_COVERAGE_FRAMES
        }

        document = super().build(session_names, dataset_id)
        path = os.path.join(self.output_root, document["dataset_id"], "samples.jsonl")
        coverage = {"train": Counter(), "validation": Counter(), "test": Counter()}
        recovery_counts = {"train": 0, "validation": 0, "test": 0}
        with open(path, "r", encoding="utf-8") as handle:
            for line in handle:
                if not line.strip():
                    continue
                sample = json.loads(line)
                split = str(sample.get("split") or "train")
                group = _steering_group(
                    (sample.get("labels") or {}).get("steering_degrees") or 0.0
                )
                coverage.setdefault(split, Counter())[group] += 1
                if (sample.get("training_context") or {}).get("route_recovery"):
                    recovery_counts[split] = recovery_counts.get(split, 0) + 1
        missing_train = sorted(
            group for group in required_raw_curves if coverage["train"].get(group, 0) <= 0
        )
        if missing_train:
            raise ValueError(
                "GPS_TRAINING_CURVE_COVERAGE_MISSING:" + ",".join(missing_train)
            )
        document["gps_training_policy"].update(
            {
                "temporal_context": {
                    "history_steps": TEMPORAL_HISTORY_STEPS,
                    "nominal_rate_hz": 10,
                    "features_per_step": [
                        "imu_yaw_rate_normalized",
                        "imu_yaw_rate_present",
                        "previous_steering_normalized",
                        "previous_steering_present",
                    ],
                    "auxiliary_feature_size": TEMPORAL_AUXILIARY_SIZE,
                    "current_steering_excluded": True,
                },
                "three_session_split": "2_train_1_validation_curve_coverage_aware",
                "minimum_curve_coverage_frames": MINIMUM_CURVE_COVERAGE_FRAMES,
                "route_correction_retention": {
                    "reference_route_unchanged": True,
                    "nominal_fixed_route_limit_m": self.nominal_fixed_route_deviation_m,
                    "maximum_fixed_recovery_deviation_m": self.maximum_fixed_recovery_deviation_m,
                    "minimum_abs_human_steering_degrees": RECOVERY_STEERING_MIN_DEGREES,
                    "conditional_fix_limits_unchanged": True,
                },
            }
        )
        document["steering_coverage_by_split"] = {
            split: dict(values) for split, values in coverage.items()
        }
        document["raw_steering_coverage"] = dict(raw_counts)
        document["required_curve_groups"] = sorted(required_raw_curves)
        document["route_recovery_samples_by_split"] = recovery_counts
        training_contract = document.setdefault("feature_contract", {}).setdefault(
            "training_context", {}
        )
        fields = list(training_contract.get("fields") or [])
        for field in ["route_recovery", "route_deviation_m", "temporal"]:
            if field not in fields:
                fields.append(field)
        training_contract["fields"] = fields
        output_path = os.path.join(self.output_root, document["dataset_id"], "dataset.json")
        with open(output_path, "w", encoding="utf-8") as handle:
            json.dump(document, handle, ensure_ascii=False, indent=2)
        return document


def create_gps_torch_model(spec=None):
    _, _, torch, nn, _, _ = require_training_dependencies()
    spec = spec or GpsDrivingModelSpec()
    if int(spec.auxiliary_feature_size) != int(spec.temporal_history_steps) * 4:
        raise ValueError("AUTO_GPS temporal auxiliary contract mismatch")

    class GpsDrivingNetwork(nn.Module):
        def __init__(self):
            super().__init__()
            self.image_encoder = nn.Sequential(
                nn.Conv2d(3,16,5,2,2), nn.ReLU(inplace=True),
                nn.Conv2d(16,24,5,2,2), nn.ReLU(inplace=True),
                nn.Conv2d(24,32,3,2,1), nn.ReLU(inplace=True),
                nn.Conv2d(32,48,3,2,1), nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d((3,5)), nn.Flatten(),
                nn.Linear(48*3*5,128), nn.ReLU(inplace=True),
            )
            self.lidar_encoder = nn.Sequential(
                nn.Linear(14,32), nn.ReLU(inplace=True),
                nn.Linear(32,32), nn.ReLU(inplace=True),
            )
            self.aux_encoder = nn.Sequential(
                nn.Linear(spec.auxiliary_feature_size,32), nn.ReLU(inplace=True),
                nn.Linear(32,16), nn.ReLU(inplace=True),
            )
            self.route_encoder = nn.Sequential(
                nn.Linear(spec.route_feature_size,32), nn.ReLU(inplace=True),
                nn.Linear(32,32), nn.ReLU(inplace=True),
            )
            self.control_head = nn.Sequential(
                nn.Linear(128+32+16+32,128), nn.ReLU(inplace=True),
                nn.Dropout(0.10), nn.Linear(128,64), nn.ReLU(inplace=True),
                nn.Linear(64,spec.output_size), nn.Tanh(),
            )

        def forward(self, image, lidar, auxiliary, route):
            return self.control_head(torch.cat((
                self.image_encoder(image), self.lidar_encoder(lidar),
                self.aux_encoder(auxiliary), self.route_encoder(route),
            ), dim=1))

    return GpsDrivingNetwork()


def GpsManifestDataset(dataset_path, split, model_spec=None, recordings_root_override=None):
    _, np, torch, _, _, Dataset = require_training_dependencies()
    spec = model_spec or GpsDrivingModelSpec()
    base = ManifestDataset(dataset_path, split, spec, recordings_root_override)
    with open(os.path.join(os.path.abspath(dataset_path), "dataset.json"), "r", encoding="utf-8") as handle:
        document = json.load(handle)
    if document.get("policy_type") != "AUTO_GPS":
        base.close()
        raise ValueError("GPS trainer requires an AUTO_GPS dataset")

    class _GpsDataset(Dataset):
        def __init__(self):
            self.base = base
            self.samples = base.samples
        def __len__(self):
            return len(self.base)
        def __getitem__(self, index):
            image, lidar, _legacy_aux, target, source_index = self.base[index]
            sample = self.samples[index]
            auxiliary = np.asarray(
                encode_temporal_auxiliary(sample, spec), dtype=np.float32
            )
            route = np.asarray(
                sample["learned_features"]["route"]["normalized"], dtype=np.float32
            )
            if route.shape != (spec.route_feature_size,):
                raise ValueError(f"Route feature shape mismatch: {route.shape}")
            return (
                image,
                lidar,
                torch.from_numpy(auxiliary),
                torch.from_numpy(route),
                target,
                source_index,
            )
        def scenario_counts(self):
            return self.base.scenario_counts()
        def close(self):
            self.base.close()
    return _GpsDataset()


class GpsTrainer:
    def __init__(self, model_spec=None, config=None, policy=None):
        self.model_spec = model_spec or GpsDrivingModelSpec()
        self.config = config or TrainingConfig()
        self.policy = policy or GpsTrainingPolicy()

    @staticmethod
    def _group(sample):
        context = sample.get("training_context") or {}
        group = str(context.get("steering_group") or "").lower()
        if group in {"straight", "gentle", "sharp"}:
            return group
        scenario = str(sample.get("scenario") or "").lower()
        if "sharp" in scenario:
            return "sharp"
        if "gentle" in scenario:
            return "gentle"
        return "straight"

    def _weight(self, sample):
        group = self._group(sample)
        weight = {
            "straight": self.policy.straight_steering_loss_weight,
            "gentle": self.policy.gentle_steering_loss_weight,
            "sharp": self.policy.sharp_steering_loss_weight,
        }[group]
        context = sample.get("training_context") or {}
        if context.get("steering_transition"):
            weight = max(weight, self.policy.transition_steering_loss_weight)
        if context.get("route_recovery"):
            weight = max(weight, self.policy.recovery_steering_loss_weight)
        return min(
            self.policy.maximum_steering_loss_weight,
            max(1e-6, float(weight)),
        )

    def _weights(self, torch, dataset, indices, device):
        values = [
            self._weight(dataset.samples[int(index)])
            for index in indices.detach().cpu().tolist()
        ]
        return torch.tensor(values, dtype=torch.float32, device=device)

    def _sampler(self, torch, dataset):
        counts = dataset.scenario_counts()
        if not self.config.balance_scenarios or len(counts) <= 1:
            return None, counts
        maximum = max(counts.values())
        exponent = max(0.0, float(self.config.scenario_balance_exponent))
        limit = max(1.0, float(self.config.maximum_scenario_weight_ratio))
        weights = []
        for sample in dataset.samples:
            count = max(
                1,
                counts.get(str(sample.get("scenario") or "unknown"), 1),
            )
            weights.append(min(limit, max(1.0, (maximum / count) ** exponent)))
        generator = torch.Generator()
        generator.manual_seed(self.config.seed)
        return (
            torch.utils.data.WeightedRandomSampler(
                weights,
                len(weights),
                replacement=True,
                generator=generator,
            ),
            counts,
        )

    def train(self, dataset_path, output_path, recordings_root_override=None):
        _, _, torch, _, DataLoader, _ = require_training_dependencies()
        os.makedirs(output_path, exist_ok=True)
        torch.manual_seed(self.config.seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(self.config.seed)
        device = self._device(torch)
        train_dataset = GpsManifestDataset(
            dataset_path, "train", self.model_spec, recordings_root_override
        )
        val_dataset = GpsManifestDataset(
            dataset_path, "validation", self.model_spec, recordings_root_override
        )
        if len(train_dataset) == 0:
            train_dataset.close()
            val_dataset.close()
            raise ValueError("Training split contains no samples")
        sampler, counts = self._sampler(torch, train_dataset)
        train_loader = DataLoader(
            train_dataset,
            batch_size=self.config.batch_size,
            shuffle=sampler is None,
            sampler=sampler,
            num_workers=self.config.num_workers,
        )
        val_loader = (
            DataLoader(
                val_dataset,
                batch_size=self.config.batch_size,
                shuffle=False,
                num_workers=self.config.num_workers,
            )
            if len(val_dataset)
            else None
        )
        model = create_gps_torch_model(self.model_spec).to(device)
        optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=self.config.learning_rate,
            weight_decay=self.config.weight_decay,
        )
        history = []
        best = math.inf
        best_state = None
        best_epoch = None
        best_validation = None
        stale = 0
        stopped = False
        patience = max(0, int(self.policy.early_stopping_patience))
        delta = max(0.0, float(self.policy.early_stopping_min_delta))
        for epoch in range(1, self.config.epochs + 1):
            model.train()
            total = 0.0
            batches = 0
            for image, lidar, auxiliary, route, target, indices in train_loader:
                image, lidar, auxiliary, route, target = [
                    value.to(device)
                    for value in (image, lidar, auxiliary, route, target)
                ]
                weights = self._weights(torch, train_dataset, indices, device)
                optimizer.zero_grad(set_to_none=True)
                prediction = model(image, lidar, auxiliary, route)
                steering_error = torch.square(prediction[:, 0] - target[:, 0])
                steering = (
                    (steering_error * weights).sum()
                    / torch.clamp(weights.sum(), min=1e-6)
                )
                throttle = torch.square(prediction[:, 1] - target[:, 1]).mean()
                loss = (
                    self.config.steering_loss_weight * steering
                    + self.config.throttle_loss_weight * throttle
                )
                loss.backward()
                optimizer.step()
                total += float(loss.detach().cpu())
                batches += 1
            validation = (
                self._validate(model, val_loader, val_dataset, device, torch)
                if val_loader is not None
                else None
            )
            if validation is not None:
                value = float(validation["loss"])
                if value < best - delta:
                    best = value
                    best_epoch = epoch
                    stale = 0
                    best_validation = validation
                    best_state = {
                        key: value.detach().cpu().clone()
                        for key, value in model.state_dict().items()
                    }
                else:
                    stale += 1
            history.append(
                {
                    "epoch": epoch,
                    "train_loss": total / max(1, batches),
                    "validation_loss": (
                        None if validation is None else validation["loss"]
                    ),
                    "validation": validation,
                }
            )
            if val_loader is not None and patience and stale >= patience:
                stopped = True
                break
        if best_state is not None:
            model.load_state_dict(best_state)
        with open(
            os.path.join(dataset_path, "dataset.json"),
            "r",
            encoding="utf-8",
        ) as handle:
            dataset_document = json.load(handle)
        checkpoint = os.path.join(output_path, "checkpoint.pt")
        torch.save(
            {
                "schema": "autonomy_gps_ai_checkpoint_v2",
                "policy_type": "AUTO_GPS",
                "route_id": (dataset_document.get("route") or {}).get("route_id"),
                "model_spec": asdict(self.model_spec),
                "training_config": asdict(self.config),
                "gps_training_policy": asdict(self.policy),
                "best_epoch": best_epoch,
                "model_state_dict": model.state_dict(),
                "created_at": datetime.now(timezone.utc).isoformat(),
            },
            checkpoint,
        )
        metrics = {
            "schema": "autonomy_gps_ai_training_metrics_v2",
            "policy_type": "AUTO_GPS",
            "route_id": (dataset_document.get("route") or {}).get("route_id"),
            "device": str(device),
            "train_samples": len(train_dataset),
            "validation_samples": len(val_dataset),
            "best_validation_loss": None if best == math.inf else best,
            "best_epoch": best_epoch,
            "best_validation": best_validation,
            "epochs_completed": len(history),
            "early_stopping": {
                "enabled": bool(val_loader is not None and patience > 0),
                "patience": patience,
                "minimum_delta": delta,
                "stopped_early": stopped,
            },
            "temporal_auxiliary": {
                "history_steps": self.model_spec.temporal_history_steps,
                "feature_size": self.model_spec.auxiliary_feature_size,
                "lookback_seconds_nominal": 0.5,
            },
            "curve_aware_steering_loss": {
                "loss": "weighted_mse",
                **asdict(self.policy),
            },
            "scenario_balancing": {
                "enabled": bool(self.config.balance_scenarios),
                "scenario_counts": counts,
                "exponent": self.config.scenario_balance_exponent,
                "maximum_weight_ratio": self.config.maximum_scenario_weight_ratio,
            },
            "history": history,
            "checkpoint": os.path.basename(checkpoint),
        }
        with open(
            os.path.join(output_path, "training_metrics.json"),
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(metrics, handle, indent=2)
        train_dataset.close()
        val_dataset.close()
        return metrics

    def _validate(self, model, loader, dataset, device, torch):
        model.eval()
        count = 0
        steer_sq = 0.0
        steer_w = 0.0
        throttle_sq = 0.0
        steer_abs = 0.0
        groups = {}
        transitions = {"samples": 0, "steering_sum": 0.0, "throttle_sum": 0.0}
        recoveries = {"samples": 0, "steering_sum": 0.0, "throttle_sum": 0.0}
        with torch.no_grad():
            for image, lidar, auxiliary, route, target, indices in loader:
                image, lidar, auxiliary, route, target = [
                    value.to(device)
                    for value in (image, lidar, auxiliary, route, target)
                ]
                prediction = model(image, lidar, auxiliary, route)
                weights = self._weights(torch, dataset, indices, device)
                se = torch.square(prediction[:, 0] - target[:, 0])
                te = torch.square(prediction[:, 1] - target[:, 1])
                steer_sq += float((se * weights).sum().cpu())
                steer_w += float(weights.sum().cpu())
                throttle_sq += float(te.sum().cpu())
                prediction_cpu = prediction.detach().cpu()
                target_cpu = target.detach().cpu()
                for row, index in enumerate(indices.detach().cpu().tolist()):
                    sample = dataset.samples[int(index)]
                    steering_error = (
                        abs(float(prediction_cpu[row, 0] - target_cpu[row, 0]))
                        * self.model_spec.maximum_steering_degrees
                    )
                    throttle_error = abs(
                        float(prediction_cpu[row, 1] - target_cpu[row, 1])
                    )
                    steer_abs += steering_error
                    count += 1
                    group = self._group(sample)
                    bucket = groups.setdefault(
                        group,
                        {"samples": 0, "steering_sum": 0.0, "throttle_sum": 0.0},
                    )
                    bucket["samples"] += 1
                    bucket["steering_sum"] += steering_error
                    bucket["throttle_sum"] += throttle_error
                    context = sample.get("training_context") or {}
                    for enabled, extra_bucket in (
                        (context.get("steering_transition"), transitions),
                        (context.get("route_recovery"), recoveries),
                    ):
                        if enabled:
                            extra_bucket["samples"] += 1
                            extra_bucket["steering_sum"] += steering_error
                            extra_bucket["throttle_sum"] += throttle_error

        def bucket_metrics(bucket):
            if not bucket["samples"]:
                return None
            return {
                "samples": bucket["samples"],
                "steering_mae_degrees": (
                    bucket["steering_sum"] / bucket["samples"]
                ),
                "throttle_mae": bucket["throttle_sum"] / bucket["samples"],
            }

        steering_mse = steer_sq / max(1e-6, steer_w)
        throttle_mse = throttle_sq / max(1, count)
        return {
            "loss": (
                self.config.steering_loss_weight * steering_mse
                + self.config.throttle_loss_weight * throttle_mse
            ),
            "weighted_steering_mse": steering_mse,
            "throttle_mse": throttle_mse,
            "steering_mae_degrees": steer_abs / max(1, count),
            "steering_group_metrics": {
                name: bucket_metrics(value) for name, value in groups.items()
            },
            "transition_metrics": bucket_metrics(transitions),
            "recovery_metrics": bucket_metrics(recoveries),
        }

    def _device(self, torch):
        if self.config.device != "auto":
            return torch.device(self.config.device)
        if torch.cuda.is_available():
            return torch.device("cuda")
        if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
            return torch.device("mps")
        return torch.device("cpu")


class GpsEvaluator:
    def evaluate(
        self,
        dataset_path,
        checkpoint_path,
        split="test",
        recordings_root_override=None,
        output_path=None,
        maximum_steering_mae_degrees=None,
        maximum_throttle_mae=None,
        device="auto",
    ):
        _, _, torch, _, DataLoader, _ = require_training_dependencies()
        resolved = torch.device(
            "cuda"
            if device == "auto" and torch.cuda.is_available()
            else ("cpu" if device == "auto" else device)
        )
        checkpoint = torch.load(
            checkpoint_path,
            map_location=resolved,
            weights_only=True,
        )
        spec = GpsDrivingModelSpec(**checkpoint["model_spec"])
        model = create_gps_torch_model(spec).to(resolved)
        model.load_state_dict(checkpoint["model_state_dict"])
        model.eval()
        dataset = GpsManifestDataset(
            dataset_path,
            split,
            spec,
            recordings_root_override,
        )
        if len(dataset) == 0:
            dataset.close()
            raise ValueError(f"Evaluation split contains no samples: {split}")
        loader = DataLoader(dataset, batch_size=64, shuffle=False, num_workers=0)
        steering = 0.0
        throttle = 0.0
        count = 0
        scenarios = {}
        groups = {}
        transitions = {"samples": 0, "steering_sum": 0.0, "throttle_sum": 0.0}
        recoveries = {"samples": 0, "steering_sum": 0.0, "throttle_sum": 0.0}
        trainer = GpsTrainer(model_spec=spec)
        with torch.no_grad():
            for image, lidar, auxiliary, route, target, indices in loader:
                prediction = model(
                    image.to(resolved),
                    lidar.to(resolved),
                    auxiliary.to(resolved),
                    route.to(resolved),
                ).cpu()
                for row in range(prediction.shape[0]):
                    steering_error = (
                        abs(float(prediction[row, 0] - target[row, 0]))
                        * spec.maximum_steering_degrees
                    )
                    throttle_error = abs(float(prediction[row, 1] - target[row, 1]))
                    steering += steering_error
                    throttle += throttle_error
                    count += 1
                    sample = dataset.samples[int(indices[row])]
                    scenario = str(sample.get("scenario") or "unknown")
                    scenario_bucket = scenarios.setdefault(
                        scenario,
                        {"samples": 0, "steering_sum": 0.0, "throttle_sum": 0.0},
                    )
                    scenario_bucket["samples"] += 1
                    scenario_bucket["steering_sum"] += steering_error
                    scenario_bucket["throttle_sum"] += throttle_error
                    group = trainer._group(sample)
                    group_bucket = groups.setdefault(
                        group,
                        {"samples": 0, "steering_sum": 0.0, "throttle_sum": 0.0},
                    )
                    group_bucket["samples"] += 1
                    group_bucket["steering_sum"] += steering_error
                    group_bucket["throttle_sum"] += throttle_error
                    context = sample.get("training_context") or {}
                    for enabled, extra_bucket in (
                        (context.get("steering_transition"), transitions),
                        (context.get("route_recovery"), recoveries),
                    ):
                        if enabled:
                            extra_bucket["samples"] += 1
                            extra_bucket["steering_sum"] += steering_error
                            extra_bucket["throttle_sum"] += throttle_error

        def convert(values):
            result = {}
            for name, bucket in values.items():
                result[name] = {
                    "samples": bucket["samples"],
                    "steering_mae_degrees": (
                        bucket["steering_sum"] / bucket["samples"]
                    ),
                    "throttle_mae": bucket["throttle_sum"] / bucket["samples"],
                }
            return result

        def extra(bucket):
            if not bucket["samples"]:
                return None
            return {
                "samples": bucket["samples"],
                "steering_mae_degrees": bucket["steering_sum"] / bucket["samples"],
                "throttle_mae": bucket["throttle_sum"] / bucket["samples"],
            }

        steering_mae = steering / count
        throttle_mae = throttle / count
        checks = {}
        if maximum_steering_mae_degrees is not None:
            checks["steering_mae"] = steering_mae <= maximum_steering_mae_degrees
        if maximum_throttle_mae is not None:
            checks["throttle_mae"] = throttle_mae <= maximum_throttle_mae
        result = {
            "schema": "autonomy_gps_ai_evaluation_v2",
            "policy_type": "AUTO_GPS",
            "route_id": checkpoint.get("route_id"),
            "split": split,
            "samples": count,
            "steering_mae_degrees": steering_mae,
            "throttle_mae": throttle_mae,
            "scenario_metrics": convert(scenarios),
            "steering_group_metrics": convert(groups),
            "transition_metrics": extra(transitions),
            "recovery_metrics": extra(recoveries),
            "criteria_passed": all(checks.values()) if checks else None,
            "checks": checks,
        }
        if output_path:
            os.makedirs(output_path, exist_ok=True)
            with open(
                os.path.join(output_path, "evaluation_metrics.json"),
                "w",
                encoding="utf-8",
            ) as handle:
                json.dump(result, handle, indent=2)
        dataset.close()
        return result


class GpsOnnxExporter:
    def export(
        self,
        checkpoint_path,
        output_path,
        verify=True,
        model_filename="gps_drive_model.onnx",
        parity_absolute_tolerance=1e-4,
        parity_relative_tolerance=1e-4,
    ):
        from .exporter import _prepare_unicode_progress_output, _remove_stale_external_data
        from .onnx_parity import verify_onnx_parity

        _, _, torch, _, _, _ = require_training_dependencies()
        os.makedirs(output_path, exist_ok=True)
        checkpoint = torch.load(
            checkpoint_path,
            map_location="cpu",
            weights_only=True,
        )
        if checkpoint.get("policy_type") != "AUTO_GPS":
            raise ValueError("GPS ONNX exporter requires an AUTO_GPS checkpoint")
        spec = GpsDrivingModelSpec(**checkpoint["model_spec"])
        model = create_gps_torch_model(spec)
        model.load_state_dict(checkpoint["model_state_dict"])
        model.eval()
        generator = torch.Generator(device="cpu")
        generator.manual_seed(20260825)
        image = torch.rand(
            (1, 3, spec.image_height, spec.image_width),
            dtype=torch.float32,
            generator=generator,
        )
        lidar = torch.rand((1, 14), dtype=torch.float32, generator=generator)
        auxiliary = torch.linspace(
            -0.5,
            0.5,
            steps=spec.auxiliary_feature_size,
            dtype=torch.float32,
        ).reshape(1, spec.auxiliary_feature_size)
        route = torch.linspace(
            -0.75,
            0.75,
            steps=spec.route_feature_size,
            dtype=torch.float32,
        ).reshape(1, spec.route_feature_size)
        path = os.path.join(output_path, model_filename)
        sidecar = _remove_stale_external_data(path)
        _prepare_unicode_progress_output()
        torch.onnx.export(
            model,
            (image, lidar, auxiliary, route),
            f=path,
            input_names=["image", "lidar", "auxiliary", "route"],
            output_names=["control"],
            dynamo=True,
            verify=bool(verify),
            external_data=False,
        )
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            raise OSError("AUTO_GPS ONNX export failed")
        if os.path.exists(sidecar):
            raise OSError("AUTO_GPS ONNX exporter created unexpected external data sidecar")
        parity = None
        if verify:
            parity = verify_onnx_parity(
                model,
                path,
                {
                    "image": image,
                    "lidar": lidar,
                    "auxiliary": auxiliary,
                    "route": route,
                },
                torch=torch,
                absolute_tolerance=parity_absolute_tolerance,
                relative_tolerance=parity_relative_tolerance,
            )
        manifest = {
            "schema": "autonomy_gps_ai_onnx_manifest_v2",
            "policy_type": "AUTO_GPS",
            "route_id": checkpoint.get("route_id"),
            "model_file": os.path.basename(path),
            "model_spec": asdict(spec),
            "inputs": {
                "image": {
                    "shape": [1, 3, spec.image_height, spec.image_width],
                    "dtype": "float32",
                    "normalization": "RGB / 255.0",
                },
                "lidar": {
                    "shape": [1, 14],
                    "dtype": "float32",
                    "maximum_distance_m": spec.lidar_maximum_distance_m,
                },
                "auxiliary": {
                    "shape": [1, spec.auxiliary_feature_size],
                    "dtype": "float32",
                    "contract": (
                        f"{spec.temporal_history_steps} chronological steps: "
                        "normalized IMU yaw rate + presence + previous steering + presence"
                    ),
                    "history_steps": spec.temporal_history_steps,
                    "maximum_abs_yaw_rate_dps": spec.maximum_abs_yaw_rate_dps,
                    "maximum_steering_degrees": spec.maximum_steering_degrees,
                },
                "route": {
                    "shape": [1, spec.route_feature_size],
                    "dtype": "float32",
                    "feature_order": list(ROUTE_FEATURE_ORDER),
                },
            },
            "output": {
                "control": {
                    "shape": [1, 2],
                    "dtype": "float32",
                    "index_0": "normalized steering",
                    "index_1": "throttle",
                }
            },
            "export": {
                "backend": "torch.onnx.export",
                "dynamo": True,
                "verify": bool(verify),
                "external_data": False,
                "self_contained": True,
                "onnxruntime_parity": parity,
            },
        }
        with open(
            os.path.join(output_path, "model_manifest.json"),
            "w",
            encoding="utf-8",
        ) as handle:
            json.dump(manifest, handle, indent=2)
        return manifest


__all__ = [
    "DatasetBuildConfig",
    "GpsDatasetBuilder",
    "GpsDrivingModelSpec",
    "GpsEvaluator",
    "GpsManifestDataset",
    "GpsOnnxExporter",
    "GpsTrainer",
    "GpsTrainingPolicy",
    "MINIMUM_CURVE_COVERAGE_FRAMES",
    "SessionBuildSummary",
    "TEMPORAL_AUXILIARY_SIZE",
    "TEMPORAL_HISTORY_STEPS",
    "create_gps_torch_model",
    "encode_temporal_auxiliary",
]
