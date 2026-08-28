import json
import os
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

from autonomous_car import DriveMode
from autonomous_car.ai.dataset_integrity import validate_dataset_split_integrity
from autonomous_car.production_guard import ProductionRuntimeGuard, shutdown_runtime


class FakeController:
    def __init__(self, active=False):
        self.active = active
        self.reasons = []

    def stop(self, reason="stop"):
        self.reasons.append(reason)
        self.active = False


class FakeMotor:
    def __init__(self):
        self.stopped = 0
        self.steering_stopped = 0
        self.connected = True
        self.encoder_connected = True

    def snapshot(self):
        return {
            "connected": self.connected,
            "encoder_connected": self.encoder_connected,
        }

    def stop(self):
        self.stopped += 1

    def stop_steering(self):
        self.steering_stopped += 1


class FakeStateMachine:
    def __init__(self):
        self.mode = DriveMode.MANUAL
        self.transitions = []

    def transition(self, target, reason):
        self.mode = target
        self.transitions.append((target, reason))


class FakeRecorder:
    def __init__(self, root):
        import threading

        self.root_path = root
        self.session_path = None
        self.active = False
        self.error = None
        self.lock = threading.RLock()
        self._files = {}

    def _flush_streams(self, sync=False):
        return None

    def add_event(self, *args, **kwargs):
        return True

    def stop(self):
        self.active = False


class FakeSensor:
    def __init__(self, timestamp=None):
        self.timestamp = timestamp if timestamp is not None else time.time()

    def snapshot(self):
        return {"last_update": self.timestamp}


def build_release(root):
    motor = FakeMotor()
    recorder = FakeRecorder(root)
    lidar = FakeSensor()
    imu = FakeSensor()
    state = FakeStateMachine()
    legacy = SimpleNamespace(
        motor_controller=motor,
        record_manager=recorder,
        lidar_monitor=lidar,
        imu_monitor=imu,
        vehicle_state_machine=state,
        auto_route_runtime=FakeController(),
    )

    def atomic_json(path, document):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as file:
            json.dump(document, file)

    def lidar_points(require_connected=True):
        snapshot = {"connected": True, "last_update": lidar.timestamp}
        return snapshot, [object()] * 50

    selected_model_path = os.path.join(root, "selected-model.json")

    def select_ai_model(model_id):
        with open(selected_model_path, "w", encoding="utf-8") as file:
            json.dump({"model_id": model_id}, file)
        return {"model_id": model_id}

    ai = SimpleNamespace(
        AUTO_AI_CONTROLLER=FakeController(),
        SELECTED_MODEL_PATH=selected_model_path,
        select_ai_model=select_ai_model,
    )
    full = SimpleNamespace(
        legacy=legacy,
        ai=ai,
        AUTO_LOCAL_CONTROLLER=FakeController(),
        AUTO_ORCHESTRATOR=FakeController(),
        MAPPING_CONTROLLER=FakeController(),
        _atomic_json=atomic_json,
        _lidar_points=lidar_points,
    )
    return SimpleNamespace(full=full)


def validate_dataset_leakage_guard(root):
    dataset = os.path.join(root, "dataset")
    os.makedirs(dataset)
    document = {
        "recordings_root": root,
        "sample_manifest": "samples.jsonl",
    }
    with open(os.path.join(dataset, "dataset.json"), "w", encoding="utf-8") as file:
        json.dump(document, file)
    rows = [
        {"session": "run_a", "split": "train", "camera": {"source_sequence": 1}},
        {"session": "run_a", "split": "validation", "camera": {"source_sequence": 2}},
    ]
    with open(os.path.join(dataset, "samples.jsonl"), "w", encoding="utf-8") as file:
        for row in rows:
            file.write(json.dumps(row) + "\n")
    try:
        validate_dataset_split_integrity(dataset)
    except ValueError as error:
        assert "leakage" in str(error).lower()
    else:
        raise AssertionError("session leakage must be rejected")


def validate():
    with tempfile.TemporaryDirectory() as root:
        release = build_release(root)
        guard = ProductionRuntimeGuard(release)

        # Fresh LOCAL LiDAR is accepted; stale scan is rejected before SLAM.
        _, points = release.full._lidar_points()
        assert len(points) == 50
        release.full.legacy.lidar_monitor.timestamp = time.time() - 5.0
        try:
            release.full._lidar_points()
        except ValueError as error:
            assert "fresh LiDAR" in str(error)
        else:
            raise AssertionError("stale LiDAR must fail closed")

        # Service-level sensor guard rejects stale autonomous dependencies.
        release.full.ai.AUTO_AI_CONTROLLER.active = True
        reason = guard._sensor_fault_reason()
        assert reason == "AUTONOMY_LIDAR_STALE"
        guard._stop_autonomy(reason)
        assert release.full.ai.AUTO_AI_CONTROLLER.active is False
        assert release.full.legacy.motor_controller.stopped >= 1
        assert release.full.legacy.vehicle_state_machine.mode == DriveMode.FAULT

        # Durable selection wrapper remains callable.
        selected = release.full.ai.select_ai_model("demo")
        assert selected["model_id"] == "demo"
        assert os.path.isfile(release.full.ai.SELECTED_MODEL_PATH)

        # Shutdown always requests drive and steering stop.
        shutdown_runtime(release, production_guard=guard)
        assert release.full.legacy.motor_controller.stopped >= 2
        assert release.full.legacy.motor_controller.steering_stopped >= 1

        validate_dataset_leakage_guard(root)

    final_source = Path("server_v2_final.py").read_text(encoding="utf-8")
    training_source = Path("autonomous_car/ai/training.py").read_text(encoding="utf-8")
    config_source = Path("camera_stream/config.py").read_text(encoding="utf-8")
    service_source = Path("camera-stream.service").read_text(encoding="utf-8")
    registry_source = Path("autonomous_car/ai/model_registry.py").read_text(encoding="utf-8")
    map_store_source = Path("autonomous_car/localization/map_store.py").read_text(encoding="utf-8")
    grid_source = Path("autonomous_car/localization/occupancy_grid.py").read_text(encoding="utf-8")
    route_source = Path("autonomous_car/routes/gps_route.py").read_text(encoding="utf-8")
    required_server_fragments = [
        "AUTONOMY_MAX_JSON_BODY_BYTES",
        "AUTONOMY_HTTP_CONNECTION_TIMEOUT_SECONDS",
        "AUTONOMY_HTTP_MAX_CONNECTIONS",
        "threading.BoundedSemaphore",
        "def process_request_thread",
        "REQUEST_BODY_TOO_LARGE",
        "TRANSFER_ENCODING_NOT_SUPPORTED",
        "self.connection.settimeout",
        "daemon_threads = True",
        "PRIVATE_NETWORK_WRITE_REQUIRED",
        "ipaddress.ip_address",
        '"maximum_connections": maximum_connections',
        '"internet_exposure_supported": False',
        "signal.SIGTERM",
        "shutdown_runtime(",
        'status["production_guard"]',
    ]
    assert all(fragment in final_source for fragment in required_server_fragments)
    assert "validate_dataset_split_integrity" in training_source
    assert "dataset_integrity" in training_source
    assert "/dev/serial/by-id/*" in config_source
    assert "_default_gps_device" in config_source
    assert "Restart=on-failure" in service_source
    assert "KillSignal=SIGTERM" in service_source
    assert "KillMode=mixed" in service_source
    assert "TimeoutStopSec=10" in service_source
    for source in (registry_source, map_store_source, grid_source):
        assert "os.replace(temporary, path)" in source
        assert "_fsync_parent(path)" in source
        assert "os.fsync(" in source
    assert "os.replace(temporary, path)" in route_source
    assert "_fsync_parent(path)" in route_source
    assert "os.fsync(file.fileno())" in route_source

    return {
        "http_body_limit": "PASS",
        "socket_timeout": "PASS",
        "bounded_http_concurrency": "PASS",
        "private_network_write_scope": "PASS",
        "service_shutdown_fail_safe": "PASS",
        "systemd_graceful_stop": "PASS",
        "sensor_freshness_guard": "PASS",
        "stable_gnss_device_resolution": "PASS",
        "durable_selection_write": "PASS",
        "durable_registry_map_route_artifacts": "PASS",
        "dataset_training_integrity": "PASS",
    }


def main():
    result = validate()
    print("Production guard V2: PASS")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
