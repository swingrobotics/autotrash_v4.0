#!/usr/bin/env python3
import argparse
import json
import time
import urllib.request


CONFIRMATION = "WHEELS_LIFTED"


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
    last_error = None
    for _ in range(3):
        try:
            request_json(
                base_url,
                "/api/motor",
                {"throttle": 0.0, "enabled": False, "deadman": False},
            )
            last_error = None
        except Exception as error:
            last_error = error
        time.sleep(0.1)
    return last_error


def verify_preconditions(base_url):
    status = request_json(base_url, "/api/status")
    lidar = request_json(base_url, "/api/lidar")
    devices = status.get("devices", {})
    navigation = status.get("navigation", {})
    failures = []
    if navigation.get("mode") != "DISARMED":
        failures.append(f"mode={navigation.get('mode')}")
    if navigation.get("motors_enabled"):
        failures.append("motors already enabled")
    if abs(float(navigation.get("throttle") or 0.0)) > 0.001:
        failures.append(f"throttle={navigation.get('throttle')}")
    if not devices.get("arduino", {}).get("connected"):
        failures.append("Arduino unavailable")
    if not lidar.get("connected"):
        failures.append("LD06 unavailable")
    if failures:
        raise RuntimeError("Precondition failed: " + ", ".join(failures))


def run_level(base_url, throttle, duration, heartbeat_hz):
    interval = 1.0 / heartbeat_hz
    deadline = time.monotonic() + duration
    maximum_final_throttle = 0.0
    maximum_pwm = 0
    minimum_nonzero_pwm = None
    last_pwm = 0
    stop_reasons = set()
    while time.monotonic() < deadline:
        cycle_started = time.monotonic()
        result = request_json(
            base_url,
            "/api/motor",
            {
                "throttle": throttle,
                "enabled": True,
                "deadman": True,
            },
        )
        safety = result.get("safety") or {}
        maximum_final_throttle = max(
            maximum_final_throttle,
            abs(float(safety.get("final_throttle") or 0.0)),
        )
        current_pwm = abs(int(result.get("pwm") or 0))
        maximum_pwm = max(maximum_pwm, current_pwm)
        if current_pwm > 0:
            minimum_nonzero_pwm = (
                current_pwm
                if minimum_nonzero_pwm is None
                else min(minimum_nonzero_pwm, current_pwm)
            )
        last_pwm = current_pwm
        if safety.get("stop_reason"):
            stop_reasons.add(str(safety["stop_reason"]))
        remaining = interval - (time.monotonic() - cycle_started)
        if remaining > 0:
            time.sleep(remaining)
    return {
        "requested_throttle": throttle,
        "maximum_final_throttle": maximum_final_throttle,
        "maximum_pwm": maximum_pwm,
        "minimum_nonzero_pwm": minimum_nonzero_pwm,
        "last_pwm": last_pwm,
        "stop_reasons": sorted(stop_reasons),
    }


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Indoor wheel-spin bench test. The vehicle wheels must be fully lifted "
            "and secured before this command is allowed to run."
        )
    )
    parser.add_argument(
        "--base-url",
        default="http://192.168.137.2:8080",
        help="Dashboard base URL",
    )
    parser.add_argument(
        "--confirm-wheels-lifted",
        required=True,
        help=f"Must be exactly {CONFIRMATION}",
    )
    parser.add_argument(
        "--levels",
        default="0.20,0.30,0.35",
        help="Comma-separated absolute throttle levels",
    )
    parser.add_argument(
        "--direction",
        choices=("forward", "reverse"),
        default="forward",
    )
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--pause", type=float, default=2.0)
    parser.add_argument("--heartbeat-hz", type=float, default=10.0)
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.confirm_wheels_lifted != CONFIRMATION:
        raise SystemExit(
            f"Refusing to move motors: pass --confirm-wheels-lifted {CONFIRMATION}"
        )
    if arguments.seconds <= 0 or arguments.pause < 0 or arguments.heartbeat_hz < 5:
        raise SystemExit("seconds must be positive, pause non-negative, heartbeat at least 5Hz")
    try:
        levels = [abs(float(value)) for value in arguments.levels.split(",")]
    except ValueError as error:
        raise SystemExit(f"Invalid throttle level: {error}") from error
    if not levels or any(level < 0.08 or level > 0.35 for level in levels):
        raise SystemExit("Every throttle level must be between 0.08 and 0.35")
    sign = 1.0 if arguments.direction == "forward" else -1.0

    verify_preconditions(arguments.base_url)
    print("Preconditions passed. Keep hands, clothing, and cables away from wheels.")
    results = []
    try:
        for level in levels:
            throttle = sign * level
            print(
                f"Testing {arguments.direction} {level * 100:.0f}% "
                f"for {arguments.seconds:.1f}s"
            )
            results.append(
                run_level(
                    arguments.base_url,
                    throttle,
                    arguments.seconds,
                    arguments.heartbeat_hz,
                )
            )
            stop_error = stop_vehicle(arguments.base_url)
            if stop_error is not None:
                raise stop_error
            time.sleep(arguments.pause)
    finally:
        stop_error = stop_vehicle(arguments.base_url)
        if stop_error is not None:
            print(f"WARNING: final stop request failed: {stop_error}")

    print(json.dumps({"passed": True, "results": results}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
