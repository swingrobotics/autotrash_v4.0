import json

from autonomous_car.runtime_guard import install_manual_takeover_guards
from autonomous_car.state import DriveMode


class FakeStateMachine:
    def __init__(self, mode):
        self.mode = mode

    def transition(self, target, reason=None):
        self.mode = DriveMode(target)
        return self.snapshot()

    def snapshot(self):
        return {
            "mode": self.mode.value,
            "canonical_mode": self.mode.canonical.value,
        }


class FakeMotor:
    def snapshot(self):
        return {"enabled": False, "throttle": 0.0}

    def stop(self):
        return self.snapshot()


class FakeRoute:
    def __init__(self):
        self.active = False
        self.stop_count = 0

    def stop(self, reason):
        self.active = False
        self.stop_count += 1


class FakeController:
    def __init__(self, legacy, mode):
        self.legacy = legacy
        self.mode = mode
        self.active = False
        self.stop_count = 0

    def stop(self, reason):
        self.active = False
        self.stop_count += 1
        if self.legacy.vehicle_state_machine.mode == self.mode:
            self.legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)


class FakeOrchestrator:
    def __init__(self):
        self.active = False
        self.stop_count = 0

    def stop(self):
        self.active = False
        self.stop_count += 1


class FakeLegacy:
    def __init__(self):
        self.MANUAL_MAX_THROTTLE = 0.35
        self.vehicle_state_machine = FakeStateMachine(DriveMode.AUTO_AI)
        self.motor_controller = FakeMotor()
        self.auto_route_runtime = FakeRoute()
        self.manual_deadman_pressed = False
        self.drive_calls = 0
        self.steering_calls = 0

        def drive(throttle, enabled, deadman=False):
            self.drive_calls += 1
            self.manual_deadman_pressed = bool(enabled and deadman)
            return {"original_drive": True, "throttle": throttle}

        def steering(direction):
            self.steering_calls += 1
            return {"original_steering": True, "direction": direction}

        self.apply_safe_drive = drive
        self.apply_safe_steering = steering

    def safety_snapshot(self):
        return {"allowed": True}


def validate():
    legacy = FakeLegacy()
    ai = FakeController(legacy, DriveMode.AUTO_AI)
    local = FakeController(legacy, DriveMode.AUTO_LOCAL)
    auto = FakeOrchestrator()
    ai.active = True
    auto.active = True

    install_manual_takeover_guards(
        legacy,
        auto_ai_controller=ai,
        auto_local_controller=local,
        auto_orchestrator=auto,
    )

    ignored = legacy.apply_safe_drive(0.3, True, False)
    assert ignored["manual_command_ignored"] is True
    assert legacy.drive_calls == 0
    assert legacy.vehicle_state_machine.mode == DriveMode.AUTO_AI
    assert ai.active

    takeover = legacy.apply_safe_drive(0.2, True, True)
    assert takeover["original_drive"] is True
    assert abs(takeover["throttle"] - 0.07) < 1e-9
    assert legacy.drive_calls == 1
    assert ai.stop_count == 1
    assert not ai.active
    assert not auto.active
    assert legacy.vehicle_state_machine.mode == DriveMode.MANUAL_ASSIST

    # The full joystick range maps linearly into the configured manual output
    # range instead of saturating as soon as the raw stick passes 35%.
    full_forward = legacy.apply_safe_drive(1.0, True, False)
    half_forward = legacy.apply_safe_drive(0.5, True, False)
    half_reverse = legacy.apply_safe_drive(-0.5, True, False)
    assert abs(full_forward["throttle"] - 0.35) < 1e-9
    assert abs(half_forward["throttle"] - 0.175) < 1e-9
    assert abs(half_reverse["throttle"] + 0.175) < 1e-9

    legacy.vehicle_state_machine.mode = DriveMode.AUTO_LOCAL
    local.active = True
    legacy.manual_deadman_pressed = False
    ignored_steer = legacy.apply_safe_steering(0.5)
    assert ignored_steer["manual_command_ignored"] is True
    assert legacy.steering_calls == 0
    assert local.active

    legacy.manual_deadman_pressed = True
    steer_takeover = legacy.apply_safe_steering(-0.4)
    assert steer_takeover["original_steering"] is True
    assert legacy.steering_calls == 1
    assert local.stop_count == 1
    assert legacy.vehicle_state_machine.mode == DriveMode.MANUAL_ASSIST

    return {
        "autonomy_ignores_manual_without_deadman": "PASS",
        "drive_deadman_takeover": "PASS",
        "manual_throttle_uses_full_stick_range": "PASS",
        "steering_deadman_takeover": "PASS",
    }


def main():
    result = validate()
    print("Autonomy V2 runtime guard: PASS")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
