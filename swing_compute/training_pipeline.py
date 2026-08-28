"""End-to-end AUTO_AI training pipeline used by the PC Compute Worker.

This module deliberately keeps the current rover ONNX input contract. It makes
training cheaper by materializing the accepted video samples once at the model
input resolution and supports checkpoint warm-start for QUICK correction jobs.
"""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil

from autonomous_car.ai import (
    DatasetBuilder,
    DrivingModelSpec,
    Evaluator,
    OnnxExporter,
    TrainingConfig,
    create_torch_model,
)
from autonomous_car.ai.training import ManifestDataset, require_training_dependencies


PACKAGE_SCHEMA = "swing_training_package_v1"
CONTEXT_SCHEMA = "swing_training_context_v1"


def _json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump(value, file, ensure_ascii=False, indent=2, sort_keys=True)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)


def _load_json(path):
    with open(path, "r", encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def _safe_id(value):
    text = str(value or "").strip()
    if not text or len(text) > 96:
        raise ValueError("INVALID_ID")
    if any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_." for ch in text):
        raise ValueError("INVALID_ID")
    return text


class TrainingPipeline:
    def __init__(self, recordings_root, work_root, progress=None, cancelled=None):
        self.recordings_root = Path(recordings_root).resolve()
        self.work_root = Path(work_root).resolve()
        self.progress = progress or (lambda phase, percent, message=None, **extra: None)
        self.cancelled = cancelled or (lambda: False)
        self.work_root.mkdir(parents=True, exist_ok=True)

    def check_cancelled(self):
        if self.cancelled():
            raise RuntimeError("JOB_CANCELLED")

    def run(
        self,
        *,
        model_id,
        mode,
        sessions,
        correction_sessions=None,
        base_model_id=None,
        base_checkpoint=None,
        base_context=None,
        epochs=None,
        target_hz=10.0,
    ):
        model_id = _safe_id(model_id)
        mode = str(mode or "BASE").strip().upper()
        if mode not in {"BASE", "QUICK"}:
            raise ValueError("TRAINING_MODE_MUST_BE_BASE_OR_QUICK")
        sessions = list(dict.fromkeys(_safe_id(item) for item in sessions or []))
        corrections = set(_safe_id(item) for item in correction_sessions or [])
        if mode == "BASE" and len(sessions) < 3:
            raise ValueError("BASE_TRAINING_REQUIRES_AT_LEAST_3_RECORD_SESSIONS")
        if mode == "QUICK":
            if not base_model_id or not base_checkpoint or not base_context:
                raise ValueError("QUICK_TRAINING_REQUIRES_BASE_CHECKPOINT_AND_CONTEXT")
            if not corrections:
                raise ValueError("QUICK_TRAINING_REQUIRES_CORRECTION_SESSIONS")

        base_document = _load_json(base_context) if base_context else {}
        split_override = None
        if mode == "QUICK":
            split_override = dict(base_document.get("session_splits") or {})
            # New correction demonstrations are training-only. Existing held-out
            # sessions remain untouched, so the regression set stays fixed.
            for session in corrections:
                split_override[session] = "train"

        self.progress("BUILDING_DATASET", 18, "카메라·LiDAR·IMU·조작 시점을 맞추는 중")
        package_path, package_document = self.build_package(
            model_id=model_id,
            sessions=sessions,
            split_override=split_override,
            target_hz=target_hz,
        )
        self.check_cancelled()

        session_splits = {
            str(item.get("session")): str(item.get("split"))
            for item in package_document.get("sessions") or []
            if item.get("session") and item.get("split")
        }
        evaluation_split = (
            "test"
            if int((package_document.get("split_counts") or {}).get("test") or 0) > 0
            else "validation"
        )
        if int((package_document.get("split_counts") or {}).get(evaluation_split) or 0) <= 0:
            raise ValueError("A held-out validation/test session is required")

        output = self.work_root / "output"
        output.mkdir(parents=True, exist_ok=True)
        checkpoint = output / "checkpoint.pt"
        training_metrics = output / "training_metrics.json"

        configured_epochs = int(epochs or (3 if mode == "QUICK" else 20))
        configured_epochs = max(1, min(configured_epochs, 60 if mode == "BASE" else 10))
        training_config = TrainingConfig(
            epochs=configured_epochs,
            batch_size=32,
            learning_rate=3e-4 if mode == "QUICK" else 1e-3,
            weight_decay=1e-5,
            num_workers=0,
            device="auto",
            balance_scenarios=(mode == "BASE"),
        )
        self.progress("TRAINING", 38, f"{mode} 학습 시작 · {configured_epochs} epoch")
        metrics = self.train(
            package_path,
            checkpoint,
            training_metrics,
            config=training_config,
            mode=mode,
            base_checkpoint=base_checkpoint,
            new_sessions=corrections,
            base_model_id=base_model_id,
        )
        self.check_cancelled()

        self.progress("EVALUATING", 76, f"고정 {evaluation_split} 구간에서 후보 모델 평가 중")
        evaluator = Evaluator()
        evaluation_dir = output / "evaluation"
        candidate_eval = evaluator.evaluate(
            str(package_path),
            str(checkpoint),
            split=evaluation_split,
            output_path=str(evaluation_dir),
            recordings_root_override=str(package_path),
            device="auto",
        )
        comparison = None
        if mode == "QUICK":
            base_eval = evaluator.evaluate(
                str(package_path),
                str(base_checkpoint),
                split=evaluation_split,
                recordings_root_override=str(package_path),
                device="auto",
            )
            comparison = self.compare_evaluations(base_eval, candidate_eval)
            _json(output / "comparison.json", comparison)

        self.check_cancelled()
        self.progress("EXPORTING", 88, "ONNX 변환과 PyTorch/ONNX 일치 검증 중")
        export_dir = output / "export"
        manifest = OnnxExporter().export(str(checkpoint), str(export_dir))

        context = {
            "schema": CONTEXT_SCHEMA,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "model_id": model_id,
            "mode": mode,
            "base_model_id": base_model_id,
            "parent_model_id": base_model_id,
            "source_sessions": sessions,
            "correction_sessions": sorted(corrections),
            "session_splits": session_splits,
            "evaluation_split": evaluation_split,
            "training_package": package_document.get("training_package") or {},
            "model_spec": asdict(DrivingModelSpec()),
            "training_config": asdict(training_config),
            "training_metrics": metrics,
            "candidate_evaluation": candidate_eval,
            "regression_comparison": comparison,
        }
        context_path = output / "training_context.json"
        _json(context_path, context)
        self.progress("READY_TO_INSTALL", 98, "후보 모델 패키지 준비 완료")
        return {
            "model_id": model_id,
            "mode": mode,
            "package_path": str(package_path),
            "checkpoint": str(checkpoint),
            "training_metrics": str(training_metrics),
            "model": str(export_dir / manifest["model_file"]),
            "manifest": str(export_dir / "model_manifest.json"),
            "evaluation": str(evaluation_dir / "evaluation_metrics.json"),
            "context": str(context_path),
            "comparison": None if comparison is None else str(output / "comparison.json"),
            "evaluation_summary": candidate_eval,
            "regression_comparison": comparison,
        }

    def build_package(self, *, model_id, sessions, split_override=None, target_hz=10.0):
        self.check_cancelled()
        raw_root = self.work_root / "aligned"
        if raw_root.exists():
            shutil.rmtree(raw_root)
        raw_root.mkdir(parents=True)
        dataset_id = f"{model_id}-aligned"
        aligned = DatasetBuilder(str(self.recordings_root), str(raw_root)).build(
            sessions, dataset_id
        )
        aligned_path = raw_root / aligned["dataset_id"]
        package = self.work_root / "package"
        if package.exists():
            shutil.rmtree(package)
        (package / "frames").mkdir(parents=True)

        samples = []
        with open(aligned_path / "samples.jsonl", "r", encoding="utf-8") as file:
            for line in file:
                if line.strip():
                    samples.append(json.loads(line))

        interval = 0.0 if not target_hz or target_hz <= 0 else 1.0 / float(target_hz)
        last_by_session = {}
        thinned = []
        for sample in samples:
            session = str(sample.get("session") or "")
            timestamp = float(sample.get("timestamp_monotonic") or 0.0)
            previous = last_by_session.get(session)
            if previous is not None and interval and timestamp - previous < interval - 1e-6:
                continue
            last_by_session[session] = timestamp
            if split_override and session in split_override:
                sample["split"] = str(split_override[session])
            thinned.append(sample)
        samples = thinned

        self.progress("MATERIALIZING", 27, f"학습용 저해상도 프레임 {len(samples)}개 준비 중")
        cv2, _, _, _, _, _ = require_training_dependencies()
        grouped = {}
        for sample in samples:
            camera = sample["camera"]
            video_rel = camera["video_path"]
            grouped.setdefault(video_rel, []).append(sample)

        spec = DrivingModelSpec()
        completed = 0
        for video_rel, rows in grouped.items():
            self.check_cancelled()
            video = self.recordings_root / video_rel
            capture = cv2.VideoCapture(str(video))
            if not capture.isOpened():
                raise OSError(f"Unable to open training video: {video}")
            try:
                rows.sort(key=lambda item: int(item["camera"]["video_frame_index"]))
                session = str(rows[0].get("session") or "session")
                frame_dir = package / "frames" / session
                frame_dir.mkdir(parents=True, exist_ok=True)
                for sample in rows:
                    self.check_cancelled()
                    index = int(sample["camera"]["video_frame_index"])
                    capture.set(cv2.CAP_PROP_POS_FRAMES, index)
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise OSError(f"Unable to decode frame {index} from {video}")
                    frame = cv2.resize(
                        frame,
                        (spec.image_width, spec.image_height),
                        interpolation=cv2.INTER_AREA,
                    )
                    filename = f"{index:08d}.jpg"
                    destination = frame_dir / filename
                    if not cv2.imwrite(str(destination), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88]):
                        raise OSError(f"Unable to write {destination}")
                    sample["camera"]["saved_frame_path"] = str(
                        Path("frames") / session / filename
                    ).replace("\\", "/")
                    completed += 1
                    if completed % 250 == 0:
                        self.progress(
                            "MATERIALIZING",
                            min(36, 27 + int(9 * completed / max(1, len(samples)))),
                            f"학습 프레임 {completed}/{len(samples)}",
                        )
            finally:
                capture.release()

        split_counts = {"train": 0, "validation": 0, "test": 0}
        scenario_counts = {}
        session_counts = {}
        for sample in samples:
            split = sample["split"]
            split_counts[split] = split_counts.get(split, 0) + 1
            scenario = str(sample.get("scenario") or "unknown")
            scenario_counts[scenario] = scenario_counts.get(scenario, 0) + 1
            session = str(sample.get("session") or "")
            bucket = session_counts.setdefault(session, {"count": 0, "split": split})
            if bucket["split"] != split:
                raise ValueError(f"SESSION_SPLIT_LEAKAGE:{session}")
            bucket["count"] += 1

        with open(package / "samples.jsonl", "w", encoding="utf-8") as file:
            for sample in samples:
                file.write(json.dumps(sample, ensure_ascii=False, separators=(",", ":")) + "\n")

        document = dict(aligned)
        document.update(
            dataset_id=f"{model_id}-package",
            recordings_root=str(package),
            accepted_samples=len(samples),
            split_counts=split_counts,
            scenario_counts=scenario_counts,
            sessions=[
                {
                    "session": session,
                    "split": values["split"],
                    "accepted_samples": values["count"],
                    "rejected_samples": 0,
                    "rejected_reasons": {},
                    "scenario_counts": {},
                    "record_gps": None,
                }
                for session, values in sorted(session_counts.items())
            ],
        )
        document["training_package"] = {
            "schema": PACKAGE_SCHEMA,
            "image_width": spec.image_width,
            "image_height": spec.image_height,
            "jpeg_quality": 88,
            "maximum_sample_hz": float(target_hz),
            "source_recordings_root": str(self.recordings_root),
            "source_sessions": list(sessions),
            "materialized_frames": len(samples),
        }
        _json(package / "dataset.json", document)
        _json(package / "training_package.json", document["training_package"])
        return package, document

    def train(
        self,
        dataset_path,
        checkpoint_path,
        metrics_path,
        *,
        config,
        mode,
        base_checkpoint=None,
        new_sessions=None,
        base_model_id=None,
    ):
        _, _, torch, nn, DataLoader, _ = require_training_dependencies()
        spec = DrivingModelSpec()
        torch.manual_seed(config.seed)
        device = self._device(torch, config.device)
        train_dataset = ManifestDataset(
            str(dataset_path), "train", spec, recordings_root_override=str(dataset_path)
        )
        validation_dataset = ManifestDataset(
            str(dataset_path), "validation", spec, recordings_root_override=str(dataset_path)
        )
        if len(train_dataset) == 0:
            raise ValueError("Training split contains no samples")

        model = create_torch_model(spec)
        if base_checkpoint:
            base = torch.load(str(base_checkpoint), map_location="cpu", weights_only=True)
            base_spec = DrivingModelSpec(**base["model_spec"])
            if asdict(base_spec) != asdict(spec):
                raise ValueError("BASE_MODEL_INPUT_CONTRACT_MISMATCH")
            model.load_state_dict(base["model_state_dict"])
        if mode == "QUICK":
            for parameter in model.image_encoder.parameters():
                parameter.requires_grad = False
        model = model.to(device)
        parameters = [parameter for parameter in model.parameters() if parameter.requires_grad]
        optimizer = torch.optim.AdamW(
            parameters, lr=config.learning_rate, weight_decay=config.weight_decay
        )
        criterion = nn.MSELoss(reduction="mean")
        sampler = self._sampler(torch, train_dataset, mode, set(new_sessions or []), config)
        train_loader = DataLoader(
            train_dataset,
            batch_size=config.batch_size,
            shuffle=sampler is None,
            sampler=sampler,
            num_workers=0,
        )
        validation_loader = (
            DataLoader(validation_dataset, batch_size=config.batch_size, shuffle=False, num_workers=0)
            if len(validation_dataset)
            else None
        )

        history = []
        best_validation = math.inf
        best_state = None
        stale_epochs = 0
        patience = 2 if mode == "QUICK" else 4
        for epoch in range(1, config.epochs + 1):
            self.check_cancelled()
            model.train()
            total = 0.0
            batches = 0
            for image, lidar, auxiliary, target, _ in train_loader:
                self.check_cancelled()
                image, lidar, auxiliary, target = (
                    image.to(device), lidar.to(device), auxiliary.to(device), target.to(device)
                )
                optimizer.zero_grad(set_to_none=True)
                prediction = model(image, lidar, auxiliary)
                loss = (
                    config.steering_loss_weight * criterion(prediction[:, 0], target[:, 0])
                    + config.throttle_loss_weight * criterion(prediction[:, 1], target[:, 1])
                )
                loss.backward()
                optimizer.step()
                total += float(loss.detach().cpu())
                batches += 1
            train_loss = total / max(1, batches)
            validation_loss = None
            if validation_loader is not None:
                model.eval()
                val_total = 0.0
                val_batches = 0
                with torch.no_grad():
                    for image, lidar, auxiliary, target, _ in validation_loader:
                        prediction = model(
                            image.to(device), lidar.to(device), auxiliary.to(device)
                        )
                        target = target.to(device)
                        loss = (
                            config.steering_loss_weight * criterion(prediction[:, 0], target[:, 0])
                            + config.throttle_loss_weight * criterion(prediction[:, 1], target[:, 1])
                        )
                        val_total += float(loss.detach().cpu())
                        val_batches += 1
                validation_loss = val_total / max(1, val_batches)
                if validation_loss + 1e-8 < best_validation:
                    best_validation = validation_loss
                    stale_epochs = 0
                    best_state = {
                        key: value.detach().cpu().clone()
                        for key, value in model.state_dict().items()
                    }
                else:
                    stale_epochs += 1
            history.append(
                {"epoch": epoch, "train_loss": train_loss, "validation_loss": validation_loss}
            )
            percent = 38 + int(34 * epoch / max(1, config.epochs))
            self.progress(
                "TRAINING",
                min(72, percent),
                f"Epoch {epoch}/{config.epochs} · loss {train_loss:.4f}",
                epoch=epoch,
                epochs=config.epochs,
                train_loss=train_loss,
                validation_loss=validation_loss,
            )
            if validation_loader is not None and stale_epochs >= patience:
                break

        if best_state is not None:
            model.load_state_dict(best_state)
        checkpoint = {
            "schema": "autonomy_ai_checkpoint_v1",
            "model_spec": asdict(spec),
            "training_config": asdict(config),
            "model_state_dict": model.state_dict(),
            "created_at": datetime.now(timezone.utc).isoformat(),
            "training_mode": mode,
            "base_model_id": base_model_id,
        }
        Path(checkpoint_path).parent.mkdir(parents=True, exist_ok=True)
        torch.save(checkpoint, str(checkpoint_path))
        metrics = {
            "schema": "autonomy_ai_training_metrics_v1",
            "device": str(device),
            "mode": mode,
            "base_model_id": base_model_id,
            "train_samples": len(train_dataset),
            "validation_samples": len(validation_dataset),
            "best_validation_loss": None if best_validation == math.inf else best_validation,
            "early_stopping_patience": patience,
            "epochs_completed": len(history),
            "image_encoder_frozen": mode == "QUICK",
            "quick_new_data_target_fraction": 0.30 if mode == "QUICK" else None,
            "history": history,
            "checkpoint": Path(checkpoint_path).name,
        }
        _json(metrics_path, metrics)
        train_dataset.close()
        validation_dataset.close()
        return metrics

    @staticmethod
    def _device(torch, requested):
        if requested != "auto":
            return torch.device(requested)
        if torch.cuda.is_available():
            return torch.device("cuda")
        if getattr(torch.backends, "mps", None) and torch.backends.mps.is_available():
            return torch.device("mps")
        return torch.device("cpu")

    @staticmethod
    def _sampler(torch, dataset, mode, new_sessions, config):
        if mode == "QUICK" and new_sessions:
            new_indices = [
                index
                for index, sample in enumerate(dataset.samples)
                if str(sample.get("session") or "") in new_sessions
            ]
            old_indices = [index for index in range(len(dataset.samples)) if index not in set(new_indices)]
            if new_indices and old_indices:
                new_weight = 0.30 / len(new_indices)
                old_weight = 0.70 / len(old_indices)
                weights = [
                    new_weight if index in set(new_indices) else old_weight
                    for index in range(len(dataset.samples))
                ]
                generator = torch.Generator().manual_seed(config.seed)
                return torch.utils.data.WeightedRandomSampler(
                    weights, num_samples=len(weights), replacement=True, generator=generator
                )
        if config.balance_scenarios:
            counts = {}
            for sample in dataset.samples:
                scenario = str(sample.get("scenario") or "unknown")
                counts[scenario] = counts.get(scenario, 0) + 1
            if len(counts) > 1:
                maximum = max(counts.values())
                weights = []
                for sample in dataset.samples:
                    count = max(1, counts[str(sample.get("scenario") or "unknown")])
                    ratio = (maximum / count) ** config.scenario_balance_exponent
                    weights.append(min(config.maximum_scenario_weight_ratio, max(1.0, ratio)))
                generator = torch.Generator().manual_seed(config.seed)
                return torch.utils.data.WeightedRandomSampler(
                    weights, num_samples=len(weights), replacement=True, generator=generator
                )
        return None

    @staticmethod
    def compare_evaluations(base, candidate):
        base_steering = float(base.get("steering_mae_degrees") or 0.0)
        candidate_steering = float(candidate.get("steering_mae_degrees") or 0.0)
        base_throttle = float(base.get("throttle_mae") or 0.0)
        candidate_throttle = float(candidate.get("throttle_mae") or 0.0)
        scenario = {}
        names = sorted(
            set((base.get("scenario_metrics") or {}).keys())
            | set((candidate.get("scenario_metrics") or {}).keys())
        )
        for name in names:
            old = (base.get("scenario_metrics") or {}).get(name) or {}
            new = (candidate.get("scenario_metrics") or {}).get(name) or {}
            if old.get("steering_mae_degrees") is None or new.get("steering_mae_degrees") is None:
                continue
            scenario[name] = {
                "base_steering_mae_degrees": old["steering_mae_degrees"],
                "candidate_steering_mae_degrees": new["steering_mae_degrees"],
                "steering_delta_degrees": float(new["steering_mae_degrees"])
                - float(old["steering_mae_degrees"]),
            }
        # This is advisory only. Closed-area validation is still mandatory.
        regression_ok = (
            candidate_steering <= base_steering * 1.10 + 0.10
            and candidate_throttle <= base_throttle * 1.15 + 0.01
        )
        return {
            "schema": "swing_ai_regression_comparison_v1",
            "base": base,
            "candidate": candidate,
            "steering_delta_degrees": candidate_steering - base_steering,
            "throttle_delta": candidate_throttle - base_throttle,
            "scenario": scenario,
            "regression_guard_passed": regression_ok,
            "note": "Offline regression guard only; does not grant vehicle permission",
        }


__all__ = ["CONTEXT_SCHEMA", "PACKAGE_SCHEMA", "TrainingPipeline"]
