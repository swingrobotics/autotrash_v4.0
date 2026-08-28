#!/usr/bin/env python3
"""Static integration checks for the RECORD model preview dashboard flow.

Keep this validator ASCII-only so it is safe when users build from a GitHub ZIP
with Windows PowerShell 5.1, whose default text decoding can otherwise corrupt
UTF-8 Korean literals.
"""

from __future__ import annotations

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent

FILES = {
    "preview": ROOT / "autonomous_car" / "ai" / "record_preview.py",
    "bridge": ROOT / "compute_record_preview_bridge.py",
    "hmi": ROOT / "record_model_preview_hmi.py",
    "overlay": ROOT / "record_replay_candidate_overlay_hmi.py",
    "worker": ROOT / "swing_compute" / "record_preview_worker_extensions.py",
    "entry": ROOT / "scripts" / "run_compute_worker.py",
    "temporal": ROOT / "autonomous_car" / "ai" / "temporal_gps.py",
    "measured": ROOT / "autonomous_car" / "ai" / "measured_steering_gps.py",
}


def source(name):
    path = FILES[name]
    text = path.read_text(encoding="utf-8")
    ast.parse(text, filename=str(path))
    return text


def require(text, values, label):
    missing = [value for value in values if value not in text]
    if missing:
        raise SystemExit(f"{label} contract missing: {missing}")


def main():
    preview = source("preview")
    bridge = source("bridge")
    hmi = source("hmi")
    overlay = source("overlay")
    worker = source("worker")
    entry = source("entry")
    temporal = source("temporal")
    measured = source("measured")

    require(
        preview,
        [
            'RECORD_PREVIEW_AUTHORITY = "NONE"',
            "progress_callback=None",
            "cancelled=None",
            "progress_callback(int(done), int(total_camera_rows))",
            'raise RuntimeError("JOB_CANCELLED")',
            "route_features = route_extractor.extract(",
            "runtime.reset_temporal_state()",
            "measured_steering_degrees=actual_steering",
            '"actual_steering_degrees": actual_steering',
            "last_inference = None",
        ],
        "record preview core",
    )
    if "previous_route_index" in preview:
        raise SystemExit(
            "record preview must use the same unrestricted GPS route projection as training"
        )

    require(
        temporal,
        [
            "TEMPORAL_HISTORY_STEPS = 5",
            "TEMPORAL_AUXILIARY_SIZE = TEMPORAL_HISTORY_STEPS * TEMPORAL_AUXILIARY_VALUES_PER_STEP",
            '"previous_steering_history_degrees"',
            '"current_steering_excluded": True',
            '"route_recovery"',
            "GPS_TRAINING_CURVE_COVERAGE_MISSING",
            "2_train_1_validation_curve_coverage_aware",
        ],
        "temporal GPS training",
    )

    require(
        measured,
        [
            'STEERING_HISTORY_SOURCE = "MEASURED_ENCODER"',
            '"actual_steering_degrees"',
            '"current_measured_steering_excluded": True',
            '"requires_measured_steering_feedback": True',
            '"prediction_feedback_forbidden": True',
        ],
        "measured steering temporal GPS",
    )

    require(
        bridge,
        [
            '"/api/v2/compute/preview-models"',
            '"/api/v2/compute/preview-model-file"',
            '"/api/v2/compute/preview-transfer"',
            '"/api/v2/compute/preview-artifact"',
            "_vehicle_safe_for_training_transfer",
            "_worker_url_allowed",
            "PREVIEW_MODEL_NOT_AUTHORIZED",
            '"preview_video": ("video/mp4"',
            '"preview_csv": ("text/csv; charset=utf-8"',
            '"temporal_gps"',
            '"temporal_history_steps"',
            '"auxiliary_feature_size"',
        ],
        "rover preview bridge",
    )

    require(
        worker,
        [
            '_PREVIEW_KIND = "preview_record_model"',
            '"record_model_preview"] = True',
            '"record_model_preview_artifacts"] = True',
            '"record_model_preview_h264"] = True',
            '"record_model_preview_temporal_gps"] = True',
            "download_preview_model_file",
            "download_gps_route",
            "progress_callback=preview_progress",
            "cancelled=lambda: self._cancelled(job_id)",
            'name not in {"preview_video", "preview_csv"}',
            '"control_authority": "NONE"',
            "_ffmpeg_executable",
            '"libx264"',
            '"yuv420p"',
            '"+faststart"',
            'phase="ENCODING_PREVIEW"',
            '"video_codec": "H264_YUV420P_FASTSTART"',
            'effective_sample_every = 1 if temporal_gps else request["sample_every"]',
            '"temporal_gps": temporal_gps',
        ],
        "worker preview extension",
    )

    require(
        entry,
        [
            "install_record_preview_worker_extensions",
            "install_record_worker_extensions()",
            "install_gps_worker_extensions()",
            "install_record_preview_worker_extensions()",
        ],
        "worker entry point",
    )
    if not (
        entry.index("install_record_worker_extensions()")
        < entry.index("install_gps_worker_extensions()")
        < entry.index("install_record_preview_worker_extensions()")
    ):
        raise SystemExit("worker extension install order is invalid")

    require(
        hmi,
        [
            "root.id='record-model-preview'",
            'id="record-model-preview-model"',
            'id="record-model-preview-video"',
            'id="record-model-preview-csv"',
            "record_model_preview",
            "record_model_preview_artifacts",
            "record_model_preview_h264",
            "ENCODING_PREVIEW:",
            "H.264",
            "'/api/v2/compute/preview-transfer'",
            "'/api/v2/compute/preview-artifact'",
            "kind:'preview_record_model'",
            "record-model-preview-summary",
            "model.temporal_gps",
            "stepSelect.value='1'",
            "temporal_history_steps",
            "data.temporal_gps?' / temporal':''",
            "MODEL",
            "HUMAN",
        ],
        "dashboard preview HMI",
    )

    require(
        overlay,
        [
            "from record_model_preview_hmi import RECORD_MODEL_PREVIEW_HMI",
            "+ RECORD_MODEL_PREVIEW_HMI",
        ],
        "RECORD replay dashboard splice",
    )

    print("RECORD_PREVIEW_DASHBOARD_CONTRACT_PASS")


if __name__ == "__main__":
    main()
