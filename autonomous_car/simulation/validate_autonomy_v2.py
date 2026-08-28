import json
import tempfile
from pathlib import Path

from autonomous_car.ai import ModelRegistry
from autonomous_car.localization import MapStore
from autonomous_car.mode_policy import policy_for
from autonomous_car.modes import AutoCapabilities, AutoModeSelector
from autonomous_car.safety import LocalAvoidancePlanner, SafetySupervisor
from autonomous_car.state import ControlRequest, DriveMode, SafetyContext, SensorStatus
from autonomous_car.state_machine import VehicleStateMachine
from lane_observer import SharedLaneObserver


def sensor(value, age=0.0, valid=True):
    return SensorStatus(value=value, timestamp=0.0, is_valid=valid, data_age=age)


def check_mode_policies():
    manual = policy_for(DriveMode.MANUAL)
    manual_assist = policy_for(DriveMode.MANUAL_ASSIST)
    record = policy_for(DriveMode.RECORD)
    ai = policy_for(DriveMode.AUTO_AI)
    gps = policy_for(DriveMode.AUTO_GPS)
    local = policy_for(DriveMode.AUTO_LOCAL)
    passed = (
        manual.driver_controlled
        and not manual.require_deadman
        and not manual.person_stop
        and not manual.obstacle_stop_fallback
        and not manual.require_lidar
        and manual_assist.driver_controlled
        and not manual_assist.require_deadman
        and record.driver_controlled
        and record.records_data
        and not record.require_deadman
        and not record.person_stop
        and ai.learned_driving
        and ai.person_stop
        and not ai.local_avoidance
        and not ai.obstacle_stop_fallback
        and gps.learned_driving
        and gps.gps_navigation
        and gps.person_stop
        and gps.require_lidar
        and not gps.lane_assist
        and not gps.local_avoidance
        and not gps.obstacle_stop_fallback
        and local.local_navigation
        and local.lane_assist
        and local.local_avoidance
        and local.person_stop
    )
    return passed, {
        "manual": manual.__dict__,
        "manual_assist": manual_assist.__dict__,
        "record": record.__dict__,
        "auto_ai": ai.__dict__,
        "auto_gps": gps.__dict__,
        "auto_local": local.__dict__,
    }


def check_state_machine():
    machine = VehicleStateMachine()
    snapshots = [
        machine.transition(DriveMode.MANUAL, "test_manual"),
        machine.transition(DriveMode.RECORD, "test_record"),
        machine.transition(DriveMode.MANUAL, "test_record_stop"),
        machine.transition(DriveMode.AUTO_AI, "test_ai"),
        machine.transition(DriveMode.DISARMED, "test_stop"),
        machine.transition(DriveMode.AUTO_GPS, "test_gps"),
        machine.transition(DriveMode.DISARMED, "test_stop2"),
        machine.transition(DriveMode.AUTO_LOCAL, "test_local"),
    ]
    passed = (
        snapshots[0]["canonical_mode"] == "MANUAL"
        and snapshots[1]["mode"] == "RECORD"
        and snapshots[3]["mode"] == "AUTO_AI"
        and snapshots[5]["mode"] == "AUTO_GPS"
        and snapshots[-1]["mode"] == "AUTO_LOCAL"
    )
    return passed, snapshots


def check_safety_policies():
    supervisor = SafetySupervisor(obstacle_restart_delay_seconds=0.0)
    request = ControlRequest(0.25, 0.0, True, True, "v2_validation")
    no_deadman_request = ControlRequest(0.25, 0.0, True, False, "v2_manual_no_deadman")
    obstacle = [{"bearing_degrees": 0.0, "distance_mm": 700}]
    manual = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL,
            arduino=sensor(True),
            lidar=sensor(obstacle),
            steering=sensor(0.0),
        ),
    )
    manual_no_deadman = supervisor.evaluate(
        no_deadman_request,
        SafetyContext(
            mode=DriveMode.MANUAL,
            arduino=sensor(True),
            lidar=sensor(obstacle),
            steering=sensor(0.0),
        ),
    )
    record_no_deadman = supervisor.evaluate(
        no_deadman_request,
        SafetyContext(
            mode=DriveMode.RECORD,
            arduino=sensor(True),
            lidar=sensor(obstacle),
            steering=sensor(0.0),
        ),
    )
    manual_stale = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL,
            arduino=sensor(True),
            lidar=sensor([], age=1.0),
            steering=sensor(0.0),
        ),
    )
    ai_obstacle = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.AUTO_AI,
            arduino=sensor(True),
            lidar=sensor(obstacle),
            steering=sensor(0.0),
        ),
    )
    ai_person = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.AUTO_AI,
            arduino=sensor(True),
            lidar=sensor(obstacle),
            steering=sensor(0.0),
            camera_hazard=True,
        ),
    )
    gps_obstacle = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.AUTO_GPS,
            arduino=sensor(True),
            lidar=sensor(obstacle),
            steering=sensor(0.0),
        ),
    )
    gps_person = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.AUTO_GPS,
            arduino=sensor(True),
            lidar=sensor(obstacle),
            steering=sensor(0.0),
            camera_hazard=True,
        ),
    )
    gps_stale = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.AUTO_GPS,
            arduino=sensor(True),
            lidar=sensor([], age=1.0),
            steering=sensor(0.0),
        ),
    )
    passed = (
        manual.allowed
        and manual.final_throttle == 0.25
        and manual_no_deadman.allowed
        and record_no_deadman.allowed
        and manual_stale.allowed
        and ai_obstacle.allowed
        and ai_person.stop_reason == "CAMERA_OBJECT_STOP"
        and gps_obstacle.allowed
        and gps_person.stop_reason == "CAMERA_OBJECT_STOP"
        and gps_stale.stop_reason == "LIDAR_TIMEOUT"
    )
    return passed, {
        "manual_obstacle": manual.as_dict(),
        "manual_no_deadman": manual_no_deadman.as_dict(),
        "record_no_deadman": record_no_deadman.as_dict(),
        "manual_stale_lidar": manual_stale.as_dict(),
        "auto_ai_obstacle": ai_obstacle.as_dict(),
        "auto_ai_person": ai_person.as_dict(),
        "auto_gps_obstacle": gps_obstacle.as_dict(),
        "auto_gps_person": gps_person.as_dict(),
        "auto_gps_stale_lidar": gps_stale.as_dict(),
    }


def check_auto_selector():
    selector = AutoModeSelector()
    gps = selector.select(
        AutoCapabilities(
            gps_ready=True,
            local_map_id="warehouse",
            local_localization_ready=True,
            ai_model_id="drive_v1",
            ai_model_validated=True,
            ai_environment_match=True,
        )
    )
    local = selector.select(
        AutoCapabilities(
            local_map_id="warehouse",
            local_localization_ready=True,
            ai_model_id="drive_v1",
            ai_model_validated=True,
            ai_environment_match=True,
        )
    )
    ai = selector.select(
        AutoCapabilities(
            ai_model_id="drive_v1",
            ai_model_validated=True,
            ai_environment_match=True,
        )
    )
    rejected = selector.select(
        AutoCapabilities(
            ai_model_id="drive_v1",
            ai_model_validated=True,
            ai_environment_match=False,
        )
    )
    passed = (
        gps.target_mode == DriveMode.AUTO_GPS
        and local.target_mode == DriveMode.AUTO_LOCAL
        and local.resource_id == "warehouse"
        and ai.target_mode == DriveMode.AUTO_AI
        and not rejected.ready
        and rejected.target_mode is None
    )
    return passed, {
        "gps": gps.__dict__,
        "local": local.__dict__,
        "ai": ai.__dict__,
        "rejected": rejected.__dict__,
    }


def check_map_and_model_registries():
    with tempfile.TemporaryDirectory() as directory:
        map_store = MapStore(f"{directory}/maps")
        first = map_store.create_map("Warehouse 1F")
        map_store.upsert_destination(
            first["map_id"], "charger", "Charger", 4.2, 1.5, 90.0
        )
        map_store.create_map("Office B1")
        maps = map_store.list_maps()
        warehouse = map_store.get_map(first["map_id"])
        registry = ModelRegistry(f"{directory}/models")
        registry.register(
            "drive_v1",
            "drive_v1.onnx",
            validated=True,
            auto_allowed=True,
            environments=["indoor", "warehouse"],
        )
        registry.register(
            "gps_v1",
            "gps_v1.onnx",
            metadata={"policy_type": "AUTO_GPS", "route_id": "route_a"},
            validated=True,
            auto_allowed=True,
        )
        compatible = registry.compatible_for_auto(["indoor", "warehouse"])
        incompatible = registry.compatible_for_auto(["outdoor"])
        gps_models = registry.compatible_for_gps_route("route_a", auto_only=True)
    passed = (
        len(maps) == 2
        and len(warehouse["destinations"]) == 1
        and warehouse["destinations"][0]["destination_id"] == "charger"
        and [model["model_id"] for model in compatible] == ["drive_v1"]
        and incompatible == []
        and [model["model_id"] for model in gps_models] == ["gps_v1"]
    )
    return passed, {
        "map_count": len(maps),
        "warehouse_destinations": warehouse["destinations"],
        "compatible_models": [model["model_id"] for model in compatible],
        "gps_models": [model["model_id"] for model in gps_models],
    }


def check_local_avoidance():
    planner = LocalAvoidancePlanner()
    avoid = planner.plan(
        [
            {"bearing_degrees": 0.0, "distance_mm": 1100},
            {"bearing_degrees": 35.0, "distance_mm": 2100},
            {"bearing_degrees": -35.0, "distance_mm": 800},
        ]
    )
    stop = planner.plan(
        [
            {"bearing_degrees": 0.0, "distance_mm": 700},
            {"bearing_degrees": 35.0, "distance_mm": 2100},
        ]
    )
    passed = (
        avoid.active
        and not avoid.stop_required
        and avoid.preferred_side == "left"
        and avoid.reason == "AVOID"
        and stop.stop_required
        and stop.reason == "TOO_CLOSE_STOP"
    )
    return passed, {"avoid": avoid.__dict__, "stop": stop.__dict__}


class _FakeLaneResult:
    def __init__(self, call):
        self.call = call

    def as_dict(self):
        return {
            "detected": True,
            "confidence": 0.91,
            "backend": "UFLD_ONNX",
            "left_line": {"points": [[100.0, 300.0], [140.0, 200.0]]},
            "right_line": {"points": [[500.0, 300.0], [460.0, 200.0]]},
            "center_line": {"points": [[300.0, 300.0], [300.0, 200.0]]},
            "fake_call": self.call,
        }


class _FakeHybrid:
    def __init__(self):
        self.calls = 0

    def analyze_neural_preview_jpeg(self, frame):
        self.calls += 1
        return _FakeLaneResult(self.calls)

    def preview_snapshot(self):
        return {
            "backend": "UFLD_ONNX",
            "control_authority": "NONE",
            "inference_ms": 72.0,
            "latency_allowed": True,
        }


def check_record_ufld_observer():
    observer = SharedLaneObserver()
    hybrid = _FakeHybrid()
    first = observer.observe(
        hybrid,
        b"frame-a",
        sequence=10,
        minimum_interval_seconds=0.50,
    )
    cached = observer.observe(
        hybrid,
        b"frame-b",
        sequence=11,
        minimum_interval_seconds=10.0,
    )
    refreshed = observer.observe(
        hybrid,
        b"frame-b",
        sequence=11,
        minimum_interval_seconds=0.0,
    )

    preview_source = Path("lane_neural_preview.py").read_text(encoding="utf-8")
    record_source = Path("lane_record_observer.py").read_text(encoding="utf-8")
    required_record_fields = [
        "lane_backend",
        "lane_control_authority",
        "lane_frame_sequence",
        "lane_inference_ms",
        "lane_left_json",
        "lane_right_json",
        "lane_center_json",
    ]
    passed = (
        first["frame_sequence"] == 10
        and cached["frame_sequence"] == 10
        and refreshed["frame_sequence"] == 11
        and hybrid.calls == 2
        and first["control_authority"] == "NONE"
        and "DriveMode.RECORD" in preview_source
        and "UFLD_LANE_OBSERVER.observe" in preview_source
        and "install_lane_record_observer()" in preview_source
        and "_record_mode_active" in record_source
        and "UFLD_LANE_OBSERVER.observe" in record_source
        and all(field in record_source for field in required_record_fields)
        and '"lane_control_authority": "NONE"' in record_source
        and "set_neural_enabled" not in record_source
        and "motor_controller" not in record_source
    )
    return passed, {
        "shared_inference_calls": hybrid.calls,
        "first_sequence": first["frame_sequence"],
        "cached_sequence": cached["frame_sequence"],
        "refreshed_sequence": refreshed["frame_sequence"],
        "control_authority": first["control_authority"],
        "record_fields": required_record_fields,
    }


def main():
    checks = {
        "mode_policies": check_mode_policies(),
        "state_machine": check_state_machine(),
        "safety_policies": check_safety_policies(),
        "auto_selector": check_auto_selector(),
        "map_and_model_registries": check_map_and_model_registries(),
        "local_avoidance": check_local_avoidance(),
        "record_ufld_observer": check_record_ufld_observer(),
    }
    result = {
        name: {"passed": passed, "details": details}
        for name, (passed, details) in checks.items()
    }
    result["passed"] = all(item["passed"] for item in result.values())
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
