"""End-to-end AUTO_GPS training pipeline for the Windows Compute Worker."""

from __future__ import annotations

from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil

from autonomous_car.ai import (
    GpsDatasetBuilder,
    GpsDrivingModelSpec,
    GpsEvaluator,
    GpsOnnxExporter,
    GpsTrainer,
    TrainingConfig,
)


GPS_CONTEXT_SCHEMA = "swing_gps_training_context_v1"


def _write_json(path, value):
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


class GpsTrainingPipeline:
    """Build/train/evaluate/export one route-bound AUTO_GPS candidate.

    RECORD files are already mirrored into the Worker cache before this class is
    entered. New segmented-JPEG sessions are consumed directly through the
    dataset manifest's saved_frame_path field; legacy MP4 sessions remain
    compatible through ManifestDataset's video fallback.
    """

    def __init__(self, recordings_root, work_root, progress=None, cancelled=None):
        self.recordings_root = Path(recordings_root).resolve()
        self.work_root = Path(work_root).resolve()
        self.progress = progress or (lambda phase, percent, message=None, **extra: None)
        self.cancelled = cancelled or (lambda: False)
        self.work_root.mkdir(parents=True, exist_ok=True)

    def check_cancelled(self):
        if self.cancelled():
            raise RuntimeError("JOB_CANCELLED")

    def run(self, *, model_id, sessions, route_path, epochs=None):
        model_id = str(model_id or "").strip()
        sessions = list(dict.fromkeys(str(item or "").strip() for item in sessions or [] if str(item or "").strip()))
        if len(sessions) < 3:
            raise ValueError("GPS_BASE_TRAINING_REQUIRES_AT_LEAST_3_RECORD_SESSIONS")

        route_path = Path(route_path).resolve()
        if not route_path.is_file():
            raise FileNotFoundError(f"GPS_ROUTE_NOT_FOUND:{route_path}")
        route_document = _load_json(route_path)
        route_id = str(route_document.get("route_id") or "").strip()
        if not route_id:
            raise ValueError("GPS_ROUTE_ID_MISSING")

        dataset_root = self.work_root / "gps-dataset"
        if dataset_root.exists():
            shutil.rmtree(dataset_root)
        dataset_root.mkdir(parents=True, exist_ok=True)

        self.progress(
            "BUILDING_GPS_DATASET",
            18,
            "GPS 품질·IMU·카메라·LiDAR·조작 시점을 GPS Route에 맞추는 중",
        )
        dataset = GpsDatasetBuilder(
            str(self.recordings_root),
            str(dataset_root),
            str(route_path),
        ).build(sessions, f"{model_id}-gps")
        self.check_cancelled()

        gps_quality_summary = dict(dataset.get("gps_quality") or {})
        dataset_path = dataset_root / dataset["dataset_id"]
        split_counts = dict(dataset.get("split_counts") or {})
        evaluation_split = "test" if int(split_counts.get("test") or 0) > 0 else "validation"
        if int(split_counts.get("train") or 0) <= 0:
            raise ValueError("GPS_TRAINING_SPLIT_HAS_NO_SAMPLES")
        if int(split_counts.get(evaluation_split) or 0) <= 0:
            raise ValueError("GPS_HELD_OUT_SESSION_REQUIRED")

        output = self.work_root / "output"
        if output.exists():
            shutil.rmtree(output)
        output.mkdir(parents=True, exist_ok=True)

        configured_epochs = max(1, min(60, int(epochs or 30)))
        config = TrainingConfig(
            epochs=configured_epochs,
            batch_size=32,
            learning_rate=1e-3,
            weight_decay=1e-5,
            num_workers=0,
            device="auto",
            balance_scenarios=True,
            # Curve-aware loss now supplies an additional deterministic priority.
            # Keep sampler balancing moderate so rare curves are present in a
            # batch without multiplying them by the historical 8x ceiling.
            scenario_balance_exponent=0.50,
            maximum_scenario_weight_ratio=4.0,
        )

        self.progress("GPS_TRAINING", 38, f"AUTO_GPS 학습 시작 · 최대 {configured_epochs} epoch · 검증 개선 정지 적용")
        trainer = GpsTrainer(config=config)
        training = trainer.train(
            str(dataset_path),
            str(output),
            recordings_root_override=str(self.recordings_root),
        )
        self.check_cancelled()

        checkpoint = output / "checkpoint.pt"
        training_metrics = output / "training_metrics.json"
        evaluation_dir = output / "evaluation"
        self.progress("GPS_EVALUATING", 76, f"고정 {evaluation_split} 구간에서 직선·완만·급커브·전이 성능 평가 중")
        evaluation = GpsEvaluator().evaluate(
            str(dataset_path),
            str(checkpoint),
            split=evaluation_split,
            recordings_root_override=str(self.recordings_root),
            output_path=str(evaluation_dir),
            device="auto",
        )
        self.check_cancelled()

        export_dir = output / "export"
        self.progress("GPS_EXPORTING", 88, "GPS ONNX 변환과 PyTorch/ONNX 일치 검증 중")
        manifest = GpsOnnxExporter().export(
            str(checkpoint),
            str(export_dir),
            verify=True,
        )
        if str(manifest.get("route_id") or "") != route_id:
            raise ValueError("GPS_EXPORTED_ROUTE_ID_MISMATCH")

        session_splits = {
            str(item.get("session")): str(item.get("split"))
            for item in dataset.get("sessions") or []
            if item.get("session") and item.get("split")
        }
        context = {
            "schema": GPS_CONTEXT_SCHEMA,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "model_id": model_id,
            "mode": "BASE",
            "policy_type": "AUTO_GPS",
            "route_id": route_id,
            "source_sessions": sessions,
            "session_splits": session_splits,
            "evaluation_split": evaluation_split,
            "model_spec": asdict(GpsDrivingModelSpec()),
            "training_config": asdict(config),
            "dataset_training_policy": dict(dataset.get("gps_training_policy") or {}),
            "training_metrics": training,
            "candidate_evaluation": evaluation,
            "route_quality": dict((dataset.get("route") or {}).get("quality") or {}),
            "gps_quality": gps_quality_summary,
        }
        context_path = output / "training_context.json"
        _write_json(context_path, context)

        self.progress("READY_TO_INSTALL", 98, "AUTO_GPS 후보 모델 패키지 준비 완료")
        return {
            "model_id": model_id,
            "mode": "BASE",
            "policy_type": "AUTO_GPS",
            "route_id": route_id,
            "dataset": str(dataset_path),
            "checkpoint": str(checkpoint),
            "training_metrics": str(training_metrics),
            "model": str(export_dir / manifest["model_file"]),
            "manifest": str(export_dir / "model_manifest.json"),
            "evaluation": str(evaluation_dir / "evaluation_metrics.json"),
            "context": str(context_path),
            "comparison": None,
            "evaluation_summary": evaluation,
            "gps_quality_summary": gps_quality_summary,
            "regression_comparison": None,
        }


__all__ = ["GPS_CONTEXT_SCHEMA", "GpsTrainingPipeline"]
