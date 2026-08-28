"""Regression checks for rover autonomous control-loop jitter handling."""

import json

from autonomous_car.safety import SafetySupervisor
from autonomous_car.state import ControlRequest, DriveMode, SafetyContext, SensorStatus


def _sensor(value):
    return SensorStatus(value=value, timestamp=0.0, is_valid=True, data_age=0.0)


def _context(mode, delay):
    return SafetyContext(
        mode=mode,
        arduino=_sensor(True),
        lidar=_sensor([]),
        steering=_sensor(0.0),
        loop_delay_seconds=delay,
    )


def validate():
    supervisor = SafetySupervisor(
        loop_delay_limit_seconds=0.20,
        loop_delay_hard_limit_seconds=0.40,
        autonomous_loop_delay_consecutive_limit=3,
        obstacle_restart_delay_seconds=0.0,
    )
    request = ControlRequest(
        throttle=0.20,
        steering=0.0,
        enabled=True,
        source="auto_ai",
    )

    first_soft = supervisor.evaluate(request, _context(DriveMode.AUTO_AI, 0.25))
    second_soft = supervisor.evaluate(request, _context(DriveMode.AUTO_AI, 0.24))
    third_soft = supervisor.evaluate(request, _context(DriveMode.AUTO_AI, 0.23))

    reset_supervisor = SafetySupervisor(
        loop_delay_limit_seconds=0.20,
        loop_delay_hard_limit_seconds=0.40,
        autonomous_loop_delay_consecutive_limit=3,
        obstacle_restart_delay_seconds=0.0,
    )
    reset_supervisor.evaluate(request, _context(DriveMode.AUTO_AI, 0.25))
    healthy = reset_supervisor.evaluate(request, _context(DriveMode.AUTO_AI, 0.10))
    after_reset = reset_supervisor.evaluate(request, _context(DriveMode.AUTO_AI, 0.25))
    hard = reset_supervisor.evaluate(request, _context(DriveMode.AUTO_AI, 0.41))

    manual_supervisor = SafetySupervisor(
        loop_delay_limit_seconds=0.20,
        loop_delay_hard_limit_seconds=0.40,
        autonomous_loop_delay_consecutive_limit=3,
        obstacle_restart_delay_seconds=0.0,
    )
    manual = manual_supervisor.evaluate(
        ControlRequest(
            throttle=0.20,
            steering=0.0,
            enabled=True,
            source="manual_timing_test",
        ),
        _context(DriveMode.MANUAL, 0.25),
    )

    snapshot = supervisor.snapshot()
    passed = (
        first_soft.allowed
        and second_soft.allowed
        and third_soft.stop_reason == "CONTROL_LOOP_DELAY"
        and healthy.allowed
        and after_reset.allowed
        and hard.stop_reason == "CONTROL_LOOP_DELAY"
        and manual.stop_reason == "CONTROL_LOOP_DELAY"
        and snapshot["loop_delay_violation_count"] == 3
        and snapshot["autonomous_loop_delay_consecutive_limit"] == 3
    )
    result = {
        "passed": passed,
        "first_soft_allowed": first_soft.allowed,
        "second_soft_allowed": second_soft.allowed,
        "third_soft_reason": third_soft.stop_reason,
        "healthy_reset_allowed": healthy.allowed,
        "after_reset_allowed": after_reset.allowed,
        "hard_delay_reason": hard.stop_reason,
        "manual_delay_reason": manual.stop_reason,
        "timing_snapshot": snapshot,
    }
    if not passed:
        raise AssertionError(json.dumps(result, indent=2))
    return result


def main():
    print(json.dumps(validate(), indent=2))
    print("Autonomous timing guard regression: PASS")


if __name__ == "__main__":
    main()
