#!/usr/bin/env python3
import argparse
import json
import time
import urllib.request


CONFIRMATION = "WHEELS_LIFTED"
STATIONARY_CONFIRMATION = "VEHICLE_STATIONARY_CLEAR"
SEQUENCE = (
    ("center", 0.0),
    ("left_mid", -0.5),
    ("left_max", -1.0),
    ("center_after_left", 0.0),
    ("right_mid", 0.5),
    ("right_max", 1.0),
    ("center_final", 0.0),
)


def request_json(base_url, path, payload=None, timeout=2.0):
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}{path}",
        data=body,
        headers={
            "Cache-Control": "no-cache",
            **({"Content-Type": "application/json"} if body is not None else {}),
        },
        method="POST" if body is not None else "GET",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.load(response)


def stop_vehicle(base_url):
    errors = []
    for path, payload in (
        ("/api/steering/stop", {}),
        ("/api/motor", {"throttle": 0.0, "enabled": False, "deadman": False}),
    ):
        try:
            request_json(base_url, path, payload)
        except Exception as error:
            errors.append(f"{path}: {error}")
    return errors


def verify_preconditions(base_url):
    steering = request_json(base_url, "/api/steering")
    safety = request_json(base_url, "/api/safety")
    lidar = request_json(base_url, "/api/lidar")
    failures = []
    if safety.get("state_machine", {}).get("mode") != "DISARMED":
        failures.append(
            f"mode={safety.get('state_machine', {}).get('mode')}"
        )
    if steering.get("enabled"):
        failures.append("motors already enabled")
    if int(steering.get("pwm") or 0) != 0:
        failures.append(f"drive pwm={steering.get('pwm')}")
    if int(steering.get("steering_pwm") or 0) != 0:
        failures.append(f"steering pwm={steering.get('steering_pwm')}")
    if not steering.get("connected"):
        failures.append("Arduino unavailable")
    if not steering.get("encoder_connected"):
        failures.append("AS5600 unavailable")
    if steering.get("encoder_error"):
        failures.append(f"AS5600 error={steering.get('encoder_error')}")
    if not lidar.get("connected"):
        failures.append("LD06 unavailable")
    if failures:
        raise RuntimeError("Precondition failed: " + ", ".join(failures))


def run_position(base_url, label, direction, duration, heartbeat_hz):
    interval = 1.0 / heartbeat_hz
    deadline = time.monotonic() + duration
    started = time.monotonic()
    samples = []
    settled_at = None
    settled_since = None
    while time.monotonic() < deadline:
        cycle_started = time.monotonic()
        drive = request_json(
            base_url,
            "/api/motor",
            {"throttle": 0.0, "enabled": True, "deadman": True},
        )
        steering = request_json(
            base_url,
            "/api/steering",
            {"direction": direction},
        )
        safety = steering.get("safety") or drive.get("safety") or {}
        target = steering.get("target_steering_angle_degrees")
        actual = steering.get("steering_angle_degrees")
        error = None
        if target is not None and actual is not None:
            error = float(target) - float(actual)
        sample = {
            "elapsed": round(time.monotonic() - started, 3),
            "target": target,
            "requested": steering.get("requested_steering_angle_degrees"),
            "actual": actual,
            "error": error,
            "raw": steering.get("encoder_raw"),
            "steering_pwm": steering.get("steering_pwm"),
            "rejection": steering.get("steering_rejection"),
            "stop_reason": safety.get("stop_reason"),
            "drive_pwm": drive.get("pwm"),
        }
        samples.append(sample)
        if sample["rejection"]:
            raise RuntimeError(f"{label}: {sample['rejection']}")
        if sample["drive_pwm"] != 0:
            raise RuntimeError(f"{label}: unexpected drive pwm={sample['drive_pwm']}")
        if error is not None and abs(error) <= 1.5:
            settled_since = settled_since or time.monotonic()
            if settled_at is None and time.monotonic() - settled_since >= 0.3:
                settled_at = time.monotonic() - started
        else:
            settled_since = None
        remaining = interval - (time.monotonic() - cycle_started)
        if remaining > 0:
            time.sleep(remaining)

    valid = [sample for sample in samples if sample["error"] is not None]
    final = valid[-1]
    requested = float(final["requested"])
    maximum_overshoot = max(
        (
            max(0.0, abs(float(sample["actual"])) - abs(requested))
            for sample in valid
            if sample["actual"] is not None
        ),
        default=0.0,
    )
    return {
        "position": label,
        "direction_command": direction,
        "requested_degrees": round(requested, 2),
        "final_target_degrees": round(float(final["target"]), 2),
        "final_actual_degrees": round(float(final["actual"]), 2),
        "final_error_degrees": round(float(final["error"]), 2),
        "absolute_error_degrees": round(abs(float(final["error"])), 2),
        "settle_time_seconds": None if settled_at is None else round(settled_at, 2),
        "maximum_overshoot_degrees": round(maximum_overshoot, 2),
        "final_encoder_raw": final["raw"],
        "maximum_steering_pwm": max(
            abs(int(sample["steering_pwm"] or 0)) for sample in samples
        ),
        "sample_count": len(samples),
    }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Test left, center, and right steering tracking without drive output."
    )
    parser.add_argument(
        "--base-url",
        default="http://192.168.137.2:8080",
        help="Dashboard base URL",
    )
    condition = parser.add_mutually_exclusive_group(required=True)
    condition.add_argument(
        "--confirm-wheels-lifted",
        help=f"Must be exactly {CONFIRMATION}",
    )
    condition.add_argument(
        "--confirm-vehicle-stationary",
        help=f"Must be exactly {STATIONARY_CONFIRMATION}",
    )
    parser.add_argument("--seconds", type=float, default=2.5)
    parser.add_argument("--pause", type=float, default=0.5)
    parser.add_argument("--heartbeat-hz", type=float, default=10.0)
    parser.add_argument(
        "--positions",
        default="all",
        help="Comma-separated position names from the built-in sequence, or all",
    )
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    lifted_confirmed = arguments.confirm_wheels_lifted == CONFIRMATION
    stationary_confirmed = (
        arguments.confirm_vehicle_stationary == STATIONARY_CONFIRMATION
    )
    if not lifted_confirmed and not stationary_confirmed:
        raise SystemExit(
            "Refusing to move steering: provide the exact required confirmation"
        )
    if arguments.seconds < 1.0 or arguments.pause < 0 or arguments.heartbeat_hz < 5:
        raise SystemExit("seconds >= 1, pause >= 0, heartbeat >= 5Hz required")

    sequence = SEQUENCE
    if arguments.positions != "all":
        requested_positions = {
            value.strip() for value in arguments.positions.split(",") if value.strip()
        }
        sequence = tuple(
            item for item in SEQUENCE if item[0] in requested_positions
        )
        if not sequence or len(sequence) != len(requested_positions):
            known = ", ".join(label for label, _ in SEQUENCE)
            raise SystemExit(f"Unknown position. Available positions: {known}")

    verify_preconditions(arguments.base_url)
    print("Preconditions passed. Drive PWM will remain zero.")
    results = []
    try:
        for label, direction in sequence:
            print(f"Testing {label} (direction={direction:+.1f})")
            results.append(
                run_position(
                    arguments.base_url,
                    label,
                    direction,
                    arguments.seconds,
                    arguments.heartbeat_hz,
                )
            )
            time.sleep(arguments.pause)
    finally:
        stop_errors = stop_vehicle(arguments.base_url)
        if stop_errors:
            print("WARNING: " + "; ".join(stop_errors))

    final_steering = request_json(arguments.base_url, "/api/steering")
    final_safety = request_json(arguments.base_url, "/api/safety")
    stopped = (
        not final_steering.get("enabled")
        and int(final_steering.get("pwm") or 0) == 0
        and int(final_steering.get("steering_pwm") or 0) == 0
        and final_safety.get("state_machine", {}).get("mode") == "DISARMED"
    )
    print(
        json.dumps(
            {
                "passed": stopped,
                "results": results,
                "final": {
                    "mode": final_safety.get("state_machine", {}).get("mode"),
                    "enabled": final_steering.get("enabled"),
                    "drive_pwm": final_steering.get("pwm"),
                    "steering_pwm": final_steering.get("steering_pwm"),
                    "angle_degrees": final_steering.get("steering_angle_degrees"),
                },
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    if not stopped:
        raise SystemExit("Final safe-stop verification failed")


if __name__ == "__main__":
    main()
