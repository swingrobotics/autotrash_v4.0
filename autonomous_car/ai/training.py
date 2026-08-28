from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
import os

from .dataset_integrity import validate_dataset_split_integrity
from .features import SECTOR_DEFINITIONS


@dataclass(frozen=True)
class DrivingModelSpec:
    image_width: int = 160
    image_height: int = 90
    maximum_steering_degrees: float = 20.0
    maximum_abs_yaw_rate_dps: float = 90.0
    lidar_maximum_distance_m: float = 8.0
    output_size: int = 2


@dataclass(frozen=True)
class TrainingConfig:
    epochs: int = 30
    batch_size: int = 32
    learning_rate: float = 1e-3
    weight_decay: float = 1e-5
    steering_loss_weight: float = 1.0
    throttle_loss_weight: float = 0.35
    num_workers: int = 0
    seed: int = 1337
    device: str = "auto"
    balance_scenarios: bool = True
    scenario_balance_exponent: float = 0.70
    maximum_scenario_weight_ratio: float = 8.0


class TrainingDependencyError(RuntimeError):
    pass


def require_training_dependencies():
    try:
        import cv2
        import numpy as np
        import torch
        from torch import nn
        from torch.utils.data import DataLoader, Dataset
    except ImportError as error:
        raise TrainingDependencyError(
            "AUTO_AI training requires PyTorch, OpenCV and NumPy on the training machine"
        ) from error
    return cv2, np, torch, nn, DataLoader, Dataset


def create_torch_model(spec=None):
    _, _, torch, nn, _, _ = require_training_dependencies()
    spec = spec or DrivingModelSpec()

    class DrivingNetwork(nn.Module):
        def __init__(self):
            super().__init__()
            self.image_encoder = nn.Sequential(
                nn.Conv2d(3, 16, kernel_size=5, stride=2, padding=2),
                nn.ReLU(inplace=True),
                nn.Conv2d(16, 24, kernel_size=5, stride=2, padding=2),
                nn.ReLU(inplace=True),
                nn.Conv2d(24, 32, kernel_size=3, stride=2, padding=1),
                nn.ReLU(inplace=True),
                nn.Conv2d(32, 48, kernel_size=3, stride=2, padding=1),
                nn.ReLU(inplace=True),
                nn.AdaptiveAvgPool2d((3, 5)),
                nn.Flatten(),
                nn.Linear(48 * 3 * 5, 128),
                nn.ReLU(inplace=True),
            )
            self.lidar_encoder = nn.Sequential(
                nn.Linear(14, 32),
                nn.ReLU(inplace=True),
                nn.Linear(32, 32),
                nn.ReLU(inplace=True),
            )
            # auxiliary = normalized yaw rate + presence bit. A future
            # non-GPS speed sensor can extend the model spec/schema explicitly.
            self.aux_encoder = nn.Sequential(
                nn.Linear(2, 16),
                nn.ReLU(inplace=True),
            )
            self.control_head = nn.Sequential(
                nn.Linear(128 + 32 + 16, 128),
                nn.ReLU(inplace=True),
                nn.Dropout(p=0.10),
                nn.Linear(128, 64),
                nn.ReLU(inplace=True),
                nn.Linear(64, spec.output_size),
                nn.Tanh(),
            )

        def forward(self, image, lidar, auxiliary):
            image_features = self.image_encoder(image)
            lidar_features = self.lidar_encoder(lidar)
            auxiliary_features = self.aux_encoder(auxiliary)
            fused = torch.cat(
                (image_features, lidar_features, auxiliary_features),
                dim=1,
            )
            return self.control_head(fused)

    return DrivingNetwork()


class ManifestDataset:
    """Thin wrapper that becomes a torch Dataset only on a training machine."""

    def __new__(
        cls,
        dataset_path,
        split,
        model_spec=None,
        recordings_root_override=None,
    ):
        cv2, np, torch, _, _, Dataset = require_training_dependencies()
        spec = model_spec or DrivingModelSpec()
        dataset_path = os.path.abspath(dataset_path)

        with open(os.path.join(dataset_path, "dataset.json"), "r", encoding="utf-8") as file:
            document = json.load(file)
        integrity = validate_dataset_split_integrity(dataset_path, document=document)
        recordings_root = os.path.abspath(
            recordings_root_override or document["recordings_root"]
        )
        samples = []
        with open(
            os.path.join(dataset_path, document.get("sample_manifest", "samples.jsonl")),
            "r",
            encoding="utf-8",
        ) as file:
            for line in file:
                if not line.strip():
                    continue
                sample = json.loads(line)
                if sample.get("split") == split:
                    samples.append(sample)

        class _TorchManifestDataset(Dataset):
            def __init__(self):
                self.samples = samples
                self.recordings_root = recordings_root
                self.spec = spec
                self.integrity = dict(integrity)
                self._video_captures = {}

            def __len__(self):
                return len(self.samples)

            def __getitem__(self, index):
                sample = self.samples[index]
                image = self._read_image(sample)
                lidar = self._lidar_vector(sample)
                auxiliary = self._auxiliary_vector(sample)
                labels = sample["labels"]
                steering = max(
                    -1.0,
                    min(
                        1.0,
                        float(labels["steering_degrees"])
                        / max(1e-6, self.spec.maximum_steering_degrees),
                    ),
                )
                throttle = max(-1.0, min(1.0, float(labels["throttle"])))
                target = np.asarray([steering, throttle], dtype=np.float32)
                return (
                    torch.from_numpy(image),
                    torch.from_numpy(lidar),
                    torch.from_numpy(auxiliary),
                    torch.from_numpy(target),
                    index,
                )

            def scenario_counts(self):
                result = {}
                for sample in self.samples:
                    scenario = str(sample.get("scenario") or "unknown")
                    result[scenario] = result.get(scenario, 0) + 1
                return result

            def _read_image(self, sample):
                camera = sample["camera"]
                saved = camera.get("saved_frame_path")
                frame = None
                if saved:
                    frame = cv2.imread(os.path.join(self.recordings_root, saved))
                if frame is None:
                    video_path = os.path.join(
                        self.recordings_root,
                        camera["video_path"],
                    )
                    capture = self._video_captures.get(video_path)
                    if capture is None:
                        capture = cv2.VideoCapture(video_path)
                        if not capture.isOpened():
                            raise OSError(f"Unable to open training video: {video_path}")
                        self._video_captures[video_path] = capture
                    capture.set(
                        cv2.CAP_PROP_POS_FRAMES,
                        int(camera["video_frame_index"]),
                    )
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise OSError(
                            f"Unable to decode frame {camera['video_frame_index']} from {video_path}"
                        )
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame = cv2.resize(
                    frame,
                    (self.spec.image_width, self.spec.image_height),
                    interpolation=cv2.INTER_AREA,
                )
                image = frame.astype(np.float32) / 255.0
                return np.transpose(image, (2, 0, 1))

            def _lidar_vector(self, sample):
                lidar = sample["learned_features"]["lidar"]
                distances = lidar["distances_m"]
                observed = lidar["observed"]
                maximum = max(0.01, self.spec.lidar_maximum_distance_m)
                vector = [
                    max(0.0, min(maximum, float(distances[name]))) / maximum
                    for name, _ in SECTOR_DEFINITIONS
                ]
                vector.extend(
                    1.0 if observed[name] else 0.0
                    for name, _ in SECTOR_DEFINITIONS
                )
                return np.asarray(vector, dtype=np.float32)

            def _auxiliary_vector(self, sample):
                value = sample["learned_features"].get("imu_yaw_rate_dps")
                if value is None:
                    return np.asarray([0.0, 0.0], dtype=np.float32)
                maximum = max(1e-6, self.spec.maximum_abs_yaw_rate_dps)
                normalized = max(-1.0, min(1.0, float(value) / maximum))
                return np.asarray([normalized, 1.0], dtype=np.float32)

            def close(self):
                for capture in self._video_captures.values():
                    capture.release()
                self._video_captures.clear()

            def __del__(self):
                try:
                    self.close()
                except Exception:
                    pass

        return _TorchManifestDataset()


class Trainer:
    def __init__(self, model_spec=None, config=None):
        self.model_spec = model_spec or DrivingModelSpec()
        self.config = config or TrainingConfig()

    def train(self, dataset_path, output_path, recordings_root_override=None):
        _, _, torch, nn, DataLoader, _ = require_training_dependencies()
        os.makedirs(output_path, exist_ok=True)
        self._seed(torch)
        device = self._device(torch)

        train_dataset = ManifestDataset(
            dataset_path,
            "train",
            self.model_spec,
            recordings_root_override,
        )
        validation_dataset = ManifestDataset(
            dataset_path,
            "validation",
            self.model_spec,
            recordings_root_override,
        )
        if len(train_dataset) == 0:
            train_dataset.close()
            validation_dataset.close()
            raise ValueError("Training split contains no samples")

        scenario_counts = train_dataset.scenario_counts()
        sampler = self._scenario_sampler(torch, train_dataset, scenario_counts)
        train_loader = DataLoader(
            train_dataset,
            batch_size=self.config.batch_size,
            shuffle=sampler is None,
            sampler=sampler,
            num_workers=self.config.num_workers,
        )
        validation_loader = DataLoader(
            validation_dataset,
            batch_size=self.config.batch_size,
            shuffle=False,
            num_workers=self.config.num_workers,
        ) if len(validation_dataset) else None

        model = create_torch_model(self.model_spec).to(device)
        optimizer = torch.optim.AdamW(
            model.parameters(),
            lr=self.config.learning_rate,
            weight_decay=self.config.weight_decay,
        )
        criterion = nn.MSELoss(reduction="mean")
        history = []
        best_validation = math.inf
        best_state = None

        for epoch in range(1, self.config.epochs + 1):
            model.train()
            train_total = 0.0
            train_batches = 0
            for image, lidar, auxiliary, target, _ in train_loader:
                image = image.to(device)
                lidar = lidar.to(device)
                auxiliary = auxiliary.to(device)
                target = target.to(device)
                optimizer.zero_grad(set_to_none=True)
                prediction = model(image, lidar, auxiliary)
                steering_loss = criterion(prediction[:, 0], target[:, 0])
                throttle_loss = criterion(prediction[:, 1], target[:, 1])
                loss = (
                    self.config.steering_loss_weight * steering_loss
                    + self.config.throttle_loss_weight * throttle_loss
                )
                loss.backward()
                optimizer.step()
                train_total += float(loss.detach().cpu())
                train_batches += 1

            validation_loss = None
            if validation_loader is not None:
                validation_loss = self._validation_loss(
                    model,
                    validation_loader,
                    device,
                    criterion,
                    torch,
                )
                if validation_loss < best_validation:
                    best_validation = validation_loss
                    best_state = {
                        key: value.detach().cpu().clone()
                        for key, value in model.state_dict().items()
                    }

            history.append(
                {
                    "epoch": epoch,
                    "train_loss": train_total / max(1, train_batches),
                    "validation_loss": validation_loss,
                }
            )

        if best_state is not None:
            model.load_state_dict(best_state)

        checkpoint_path = os.path.join(output_path, "checkpoint.pt")
        torch.save(
            {
                "schema": "autonomy_ai_checkpoint_v1",
                "model_spec": asdict(self.model_spec),
                "training_config": asdict(self.config),
                "model_state_dict": model.state_dict(),
                "created_at": datetime.now(timezone.utc).isoformat(),
            },
            checkpoint_path,
        )
        metrics = {
            "schema": "autonomy_ai_training_metrics_v1",
            "device": str(device),
            "train_samples": len(train_dataset),
            "validation_samples": len(validation_dataset),
            "best_validation_loss": (
                None if best_validation == math.inf else best_validation
            ),
            "dataset_integrity": train_dataset.integrity,
            "scenario_balancing": {
                "enabled": bool(self.config.balance_scenarios),
                "scenario_counts": scenario_counts,
                "exponent": self.config.scenario_balance_exponent,
                "maximum_weight_ratio": self.config.maximum_scenario_weight_ratio,
            },
            "history": history,
            "checkpoint": os.path.basename(checkpoint_path),
        }
        with open(
            os.path.join(output_path, "training_metrics.json"),
            "w",
            encoding="utf-8",
        ) as file:
            json.dump(metrics, file, indent=2)
        train_dataset.close()
        validation_dataset.close()
        return metrics

    def _scenario_sampler(self, torch, dataset, counts):
        if not self.config.balance_scenarios or not counts or len(counts) <= 1:
            return None
        maximum_count = max(counts.values())
        exponent = max(0.0, float(self.config.scenario_balance_exponent))
        maximum_ratio = max(1.0, float(self.config.maximum_scenario_weight_ratio))
        weights = []
        for sample in dataset.samples:
            scenario = str(sample.get("scenario") or "unknown")
            count = max(1, counts.get(scenario, 1))
            ratio = (maximum_count / count) ** exponent
            weights.append(min(maximum_ratio, max(1.0, ratio)))
        generator = torch.Generator()
        generator.manual_seed(self.config.seed)
        return torch.utils.data.WeightedRandomSampler(
            weights=weights,
            num_samples=len(weights),
            replacement=True,
            generator=generator,
        )

    def _validation_loss(self, model, loader, device, criterion, torch):
        model.eval()
        total = 0.0
        batches = 0
        with torch.no_grad():
            for image, lidar, auxiliary, target, _ in loader:
                image = image.to(device)
                lidar = lidar.to(device)
                auxiliary = auxiliary.to(device)
                target = target.to(device)
                prediction = model(image, lidar, auxiliary)
                steering_loss = criterion(prediction[:, 0], target[:, 0])
                throttle_loss = criterion(prediction[:, 1], target[:, 1])
                loss = (
                    self.config.steering_loss_weight * steering_loss
                    + self.config.throttle_loss_weight * throttle_loss
                )
                total += float(loss.cpu())
                batches += 1
        return total / max(1, batches)

    def _device(self, torch):
        if self.config.device != "auto":
            return torch.device(self.config.device)
        if torch.cuda.is_available():
            return torch.device("cuda")
        if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
            return torch.device("mps")
        return torch.device("cpu")

    def _seed(self, torch):
        torch.manual_seed(self.config.seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(self.config.seed)
