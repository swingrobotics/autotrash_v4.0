"""Training-package materialization for JPEG-first and legacy MP4 RECORDs."""

from __future__ import annotations

import json
from pathlib import Path
import shutil

from autonomous_car.ai import DatasetBuilder, DrivingModelSpec
from autonomous_car.ai.training import require_training_dependencies
from . import training_pipeline as pipeline_module


_INSTALLED = False


def _build_package(
    self,
    *,
    model_id,
    sessions,
    split_override=None,
    target_hz=10.0,
):
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
    spec = DrivingModelSpec()
    captures = {}
    completed = 0
    source_counts = {"SEGMENTED_JPEG": 0, "LEGACY_MP4": 0}
    try:
        for sample in samples:
            self.check_cancelled()
            camera = sample.get("camera") or {}
            session = str(sample.get("session") or "session")
            frame = None
            source_kind = "SEGMENTED_JPEG"
            saved = str(camera.get("saved_frame_path") or "").strip()
            if saved:
                source = (self.recordings_root / saved).resolve()
                try:
                    source.relative_to(self.recordings_root)
                except ValueError as error:
                    raise ValueError(f"CAMERA_FRAME_PATH_OUTSIDE_RECORDINGS:{saved}") from error
                frame = cv2.imread(str(source), cv2.IMREAD_COLOR)
                if frame is None:
                    raise OSError(f"Unable to read training frame: {source}")
            else:
                source_kind = "LEGACY_MP4"
                video_rel = str(camera.get("video_path") or "").strip()
                if not video_rel:
                    raise OSError(f"{session}: camera sample has neither JPEG nor MP4 source")
                video = (self.recordings_root / video_rel).resolve()
                try:
                    video.relative_to(self.recordings_root)
                except ValueError as error:
                    raise ValueError(f"CAMERA_VIDEO_PATH_OUTSIDE_RECORDINGS:{video_rel}") from error
                capture = captures.get(str(video))
                if capture is None:
                    capture = cv2.VideoCapture(str(video))
                    if not capture.isOpened():
                        capture.release()
                        raise OSError(f"Unable to open training video: {video}")
                    captures[str(video)] = capture
                index = int(camera.get("video_frame_index") or 0)
                capture.set(cv2.CAP_PROP_POS_FRAMES, index)
                ok, frame = capture.read()
                if not ok or frame is None:
                    raise OSError(f"Unable to decode frame {index} from {video}")

            frame = cv2.resize(
                frame,
                (spec.image_width, spec.image_height),
                interpolation=cv2.INTER_AREA,
            )
            frame_dir = package / "frames" / session
            frame_dir.mkdir(parents=True, exist_ok=True)
            frame_number = int(camera.get("source_sequence") or completed + 1)
            filename = f"{completed:08d}_{frame_number:08d}.jpg"
            destination = frame_dir / filename
            if not cv2.imwrite(
                str(destination),
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), 88],
            ):
                raise OSError(f"Unable to write {destination}")
            camera["saved_frame_path"] = str(
                Path("frames") / session / filename
            ).replace("\\", "/")
            camera["materialized_source_kind"] = source_kind
            sample["camera"] = camera
            source_counts[source_kind] = source_counts.get(source_kind, 0) + 1
            completed += 1
            if completed % 250 == 0:
                self.progress(
                    "MATERIALIZING",
                    min(36, 27 + int(9 * completed / max(1, len(samples)))),
                    f"학습 프레임 {completed}/{len(samples)}",
                )
    finally:
        for capture in captures.values():
            try:
                capture.release()
            except Exception:
                pass

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
        "schema": pipeline_module.PACKAGE_SCHEMA,
        "image_width": spec.image_width,
        "image_height": spec.image_height,
        "jpeg_quality": 88,
        "maximum_sample_hz": float(target_hz),
        "source_recordings_root": str(self.recordings_root),
        "source_sessions": list(sessions),
        "materialized_frames": len(samples),
        "source_counts": source_counts,
        "source_policy": "segmented JPEG preferred; legacy MP4 fallback",
    }
    pipeline_module._json(package / "dataset.json", document)
    pipeline_module._json(
        package / "training_package.json", document["training_package"]
    )
    return package, document


def install_frame_training_compat():
    global _INSTALLED
    if _INSTALLED:
        return
    pipeline_module.TrainingPipeline.build_package = _build_package
    _INSTALLED = True


__all__ = ["install_frame_training_compat"]
