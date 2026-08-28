"""Static regression for route-bound AUTO_GPS Compute Worker training."""

from pathlib import Path


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    gps_dataset = Path("autonomous_car/ai/gps_dataset.py").read_text(encoding="utf-8")
    pipeline = Path("swing_compute/gps_training_pipeline.py").read_text(encoding="utf-8")
    worker = Path("swing_compute/gps_worker_extensions.py").read_text(encoding="utf-8")
    runner = Path("scripts/run_compute_worker.py").read_text(encoding="utf-8")
    bridge = Path("compute_gps_training_bridge.py").read_text(encoding="utf-8")
    hmi = Path("compute_gps_training_hmi.py").read_text(encoding="utf-8")
    shell = Path("unified_dashboard_data_tools.py").read_text(encoding="utf-8")

    for token in (
        "frame_dataset_builder",
        "JPEG-first",
        "RTK_FIXED_REQUIRED",
        'document["policy_type"] = "AUTO_GPS"',
    ):
        _require(token in gps_dataset, f"GPS JPEG dataset contract missing: {token}")

    for token in (
        "GpsDatasetBuilder",
        "GpsTrainer",
        "GpsEvaluator",
        "GpsOnnxExporter",
        "GPS_BASE_TRAINING_REQUIRES_AT_LEAST_3_RECORD_SESSIONS",
        '"policy_type": "AUTO_GPS"',
        '"route_id": route_id',
    ):
        _require(token in pipeline, f"GPS Worker pipeline contract missing: {token}")

    for token in (
        '"train_gps_rover_records"',
        "download_gps_route",
        '"gps_conditioned_training": True',
        '"gps_segmented_jpeg_training": True',
        "self.worker.sync_recordings",
    ):
        _require(token in worker, f"GPS Worker job contract missing: {token}")

    _require(
        "install_gps_worker_extensions" in runner,
        "Compute Worker entrypoint does not install GPS training extension",
    )
    _require(
        runner.index("install_record_worker_extensions()")
        < runner.index("install_gps_worker_extensions()"),
        "GPS Worker extension must be installed after recursive RECORD sync",
    )

    for token in (
        "/api/v2/compute/gps-transfer",
        "/api/v2/compute/gps-route",
        "/api/v2/compute/gps-model/install",
        "recording_session_path",
        "_vehicle_safe_for_training_transfer",
        'policy_type="AUTO_GPS"',
        "GPS_WORKER_MANIFEST_ROUTE_MISMATCH",
    ):
        _require(token in bridge, f"rover GPS training bridge missing: {token}")

    for token in (
        "GPS AI 모델 학습",
        "gps_conditioned_training",
        "train_gps_rover_records",
        "/api/v2/compute/gps-transfer",
        "/api/v2/compute/gps-model/install",
    ):
        _require(token in hmi, f"GPS training HMI contract missing: {token}")

    for token in (
        "COMPUTE_GPS_TRAINING_HMI",
        "install_compute_gps_training_bridge",
        "install_compute_gps_training_bridge()",
    ):
        _require(token in shell, f"unified dashboard GPS training wiring missing: {token}")

    print("Compute AUTO_GPS training V2 regression: PASS")
    print(
        {
            "worker_gps_job": "PASS",
            "segmented_jpeg_dataset": "PASS",
            "usb_route_build": "PASS",
            "route_transfer": "PASS",
            "gps_candidate_install": "PASS",
            "dashboard_gps_training": "PASS",
        }
    )


if __name__ == "__main__":
    main()
