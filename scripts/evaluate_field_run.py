import argparse
import json
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from autonomous_car.recording import FieldRunReport


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate an AUTO_ROUTE or AUTO_HYBRID field recording",
    )
    parser.add_argument("session_path")
    parser.add_argument("--required-mode")
    parser.add_argument("--maximum-cross-track-error", type=float)
    parser.add_argument("--maximum-steering-error", type=float)
    parser.add_argument("--minimum-rtk-fixed-ratio", type=float)
    parser.add_argument("--require-completion", action="store_true")
    parser.add_argument("--required-stop-reason")
    parser.add_argument("--required-event")
    parser.add_argument("--minimum-sensor-valid-ratio", type=float)
    arguments = parser.parse_args()

    analyzer = FieldRunReport(arguments.session_path)
    report = analyzer.build()
    result = analyzer.evaluate(
        report,
        required_mode=arguments.required_mode,
        maximum_cross_track_error_m=arguments.maximum_cross_track_error,
        maximum_steering_error_degrees=arguments.maximum_steering_error,
        minimum_rtk_fixed_ratio=arguments.minimum_rtk_fixed_ratio,
        require_completion=arguments.require_completion,
        required_stop_reason=arguments.required_stop_reason,
        required_event=arguments.required_event,
        minimum_sensor_valid_ratio=arguments.minimum_sensor_valid_ratio,
    )
    document = {"evaluation": result, "report": report}
    print(json.dumps(document, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
