import argparse
import json
import time
import urllib.request


def fetch_json(url, timeout):
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--base-url",
        default="http://192.168.137.2:8080",
    )
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--interval", type=float, default=0.02)
    args = parser.parse_args()

    deadline = time.monotonic() + args.timeout
    last_nonzero_evaluation = None
    maximum_observed_throttle = 0.0
    armed_mode = None

    while time.monotonic() < deadline:
        payload = fetch_json(f"{args.base_url}/api/safety", timeout=2.0)
        state_machine = payload.get("state_machine", {})
        safety = payload.get("safety", {})
        final_throttle = abs(float(safety.get("final_throttle") or 0.0))
        evaluation_time = safety.get("last_evaluated_at")
        if final_throttle > 0.0 and evaluation_time is not None:
            last_nonzero_evaluation = float(evaluation_time)
            maximum_observed_throttle = max(maximum_observed_throttle, final_throttle)
            armed_mode = state_machine.get("mode")
        elif (
            last_nonzero_evaluation is not None
            and safety.get("stop_reason") == "DEADMAN_RELEASED"
            and evaluation_time is not None
        ):
            motor = fetch_json(f"{args.base_url}/api/steering", timeout=2.0)
            latency_seconds = float(evaluation_time) - last_nonzero_evaluation
            result = {
                "passed": latency_seconds <= 0.3 and int(motor.get("pwm") or 0) == 0,
                "mode_during_drive": armed_mode,
                "maximum_observed_throttle": maximum_observed_throttle,
                "stop_reason": safety.get("stop_reason"),
                "command_evaluation_gap_ms": round(latency_seconds * 1000.0, 3),
                "motor_pwm_after_release": int(motor.get("pwm") or 0),
                "steering_pwm_after_release": int(motor.get("steering_pwm") or 0),
                "arduino_estop_latched": bool(motor.get("hardware_estop_active")),
            }
            print(json.dumps(result, ensure_ascii=False, indent=2))
            raise SystemExit(0 if result["passed"] else 1)
        time.sleep(args.interval)

    raise SystemExit("Timed out waiting for a drive command and deadman release")


if __name__ == "__main__":
    main()
