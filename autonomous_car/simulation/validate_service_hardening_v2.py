import os
from pathlib import Path
import tempfile

from autonomous_car.ai import DatasetBuilder
from autonomous_car.full_runtime_hardening import install_full_runtime_hardening
from autonomous_car.recording import RecordManager
from autonomous_car.status_cache import install_gps_status_cache


class _FakeController:
    def __init__(self):
        self.preflight_calls = 0

    def preflight(self, auto_only=False):
        self.preflight_calls += 1
        return {
            "route_loaded": True,
            "ready": True,
            "details": {
                "route_id": "route-a",
                "model_id": "model-a",
                "model_stage": "CLOSED_AREA_VALIDATED",
            },
        }

    def snapshot(self):
        return {"active": False}


class _FakeRegistry:
    def list_models(self, policy_type=None):
        return [
            {
                "model_id": "model-a",
                "policy_type": policy_type or "AUTO_GPS",
                "validation_stage": "CLOSED_AREA_VALIDATED",
            }
        ]


class _FakeAi:
    def __init__(self):
        self.MODEL_REGISTRY = _FakeRegistry()


class _FakeFull:
    def __init__(self):
        self.ai = _FakeAi()


class _FakeIntegration:
    def __init__(self):
        self.controller = _FakeController()
        self.full = _FakeFull()
        self.select_calls = 0
        self.build_calls = 0

    def selected(self):
        return {"route_id": "route-a", "model_id": "model-a"}

    def list_routes(self):
        return [{"route_id": "route-a"}]

    def select(self, route_id, model_id):
        self.select_calls += 1
        return {"route_id": route_id, "model_id": model_id}

    def build_route(self, sessions, route_id):
        self.build_calls += 1
        return {"route_id": route_id, "sessions": list(sessions)}


def validate_status_cache():
    integration = _FakeIntegration()
    install_gps_status_cache(integration, ttl_seconds=60.0)

    first = integration.status()
    second = integration.status()
    assert first == second
    assert integration.controller.preflight_calls == 1
    assert first["manual_preflight"]["ready"] is True
    # AUTO strategy must still require AUTO_ALLOWED even when manual start is
    # permitted for a CLOSED_AREA_VALIDATED model.
    assert first["auto_preflight"]["ready"] is False
    assert first["auto_preflight"]["details"]["required_stage"] == "AUTO_ALLOWED"

    integration.select("route-a", "model-a")
    integration.status()
    assert integration.controller.preflight_calls == 2

    integration.build_route(["run-a"], "route-b")
    integration.status()
    assert integration.controller.preflight_calls == 3
    return "PASS"


def validate_label_alignment_guard():
    with tempfile.TemporaryDirectory() as root:
        recordings = os.path.join(root, "recordings")
        datasets = os.path.join(root, "datasets")
        os.makedirs(recordings)
        builder = DatasetBuilder(recordings, datasets)
        threshold = builder.MAXIMUM_LABEL_SKEW_SECONDS

        sample, reason = builder._sample_from_camera_row(
            "run",
            recordings,
            "train",
            {"steering_skew_seconds": threshold + 0.01},
            None,
            None,
            None,
            None,
        )
        assert sample is None
        assert reason == "STEERING_LABEL_NOT_SYNCHRONIZED"

        sample, reason = builder._sample_from_camera_row(
            "run",
            recordings,
            "train",
            {"control_skew_seconds": threshold + 0.01},
            None,
            None,
            None,
            None,
        )
        assert sample is None
        assert reason == "CONTROL_LABEL_NOT_SYNCHRONIZED"
    return "PASS"


def validate_recorder_timebase():
    assert abs(RecordManager._absolute_skew(10.0, 10.05) - 0.05) < 1e-9
    assert RecordManager._absolute_skew(None, 10.0) is None
    return "PASS"


def validate_final_controller_installation():
    import server_v2_full as full

    installed = install_full_runtime_hardening(full)
    assert installed["mapping"] is full.MAPPING_CONTROLLER
    assert installed["auto_local"] is full.AUTO_LOCAL_CONTROLLER
    assert installed["auto"] is full.AUTO_ORCHESTRATOR
    assert hasattr(full.MAPPING_CONTROLLER, "generation")
    assert hasattr(full.AUTO_LOCAL_CONTROLLER, "generation")
    assert hasattr(full.AUTO_ORCHESTRATOR, "generation")
    assert full.MAPPING_CONTROLLER.snapshot()["active"] is False
    assert full.AUTO_LOCAL_CONTROLLER.snapshot()["active"] is False
    assert full.AUTO_ORCHESTRATOR.snapshot()["active"] is False
    return "PASS"


def validate_final_entrypoint_contract():
    source = Path("server_v2_final.py").read_text(encoding="utf-8")
    required = (
        "install_full_runtime_hardening(release.full, gps_ai=gps_ai)",
        "install_gps_status_cache(gps_ai)",
        'self.path == "/api/v2/performance"',
        "AUTONOMY_API_TOKEN",
        "CROSS_ORIGIN_WRITE_REJECTED",
        "install_manual_takeover_guards(",
    )
    for marker in required:
        assert marker in source, marker
    assert source.index("install_full_runtime_hardening") < source.rindex(
        "install_manual_takeover_guards("
    )
    return "PASS"


def main():
    result = {
        "gps_status_cache": validate_status_cache(),
        "label_alignment_guard": validate_label_alignment_guard(),
        "recorder_source_timebase": validate_recorder_timebase(),
        "final_controller_installation": validate_final_controller_installation(),
        "final_entrypoint_contract": validate_final_entrypoint_contract(),
    }
    print("Final service hardening regression: PASS")
    for key, value in result.items():
        print(f"  {key}: {value}")


if __name__ == "__main__":
    main()
