import csv
import os
from collections import Counter


def _rows(session_path, stream):
    path = os.path.join(session_path, f"{stream}.csv")
    if not os.path.isfile(path):
        return []
    with open(path, newline="", encoding="utf-8") as file:
        return list(csv.DictReader(file))


def _numbers(rows, field, absolute=False):
    values = []
    for row in rows:
        try:
            value = float(row.get(field, ""))
        except (TypeError, ValueError):
            continue
        values.append(abs(value) if absolute else value)
    return values


def _percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, round((len(ordered) - 1) * fraction)))
    return ordered[index]


def _sensor_health(rows, valid_field="is_valid", age_field="data_age", error_field="error_code"):
    valid_values = []
    errors = Counter()
    for row in rows:
        value = str(row.get(valid_field, "")).strip().lower()
        if value in {"true", "1", "yes"}:
            valid_values.append(True)
        elif value in {"false", "0", "no"}:
            valid_values.append(False)
        error = str(row.get(error_field, "")).strip()
        if error:
            errors[error] += 1
    ages = _numbers(rows, age_field)
    valid_count = sum(valid_values)
    return {
        "samples": len(valid_values),
        "valid_samples": valid_count,
        "invalid_samples": len(valid_values) - valid_count,
        "valid_ratio": (
            valid_count / len(valid_values) if valid_values else None
        ),
        "maximum_data_age_seconds": max(ages) if ages else None,
        "errors": dict(errors),
    }


class FieldRunReport:
    def __init__(self, session_path):
        self.session_path = os.path.abspath(session_path)

    def build(self):
        vehicle = _rows(self.session_path, "vehicle_state")
        gnss = _rows(self.session_path, "gnss")
        steering = _rows(self.session_path, "steering")
        arduino = _rows(self.session_path, "arduino")
        control = _rows(self.session_path, "control")
        lidar_summary = _rows(self.session_path, "lidar_summary")
        imu = _rows(self.session_path, "imu")
        perception = _rows(self.session_path, "perception")
        events = _rows(self.session_path, "events")

        timestamps = _numbers(vehicle, "monotonic")
        cross_track = _numbers(control, "cross_track_error_m", absolute=True)
        steering_error = _numbers(steering, "error_degrees", absolute=True)
        gnss_ages = _numbers(gnss, "data_age")
        modes = Counter(row.get("mode") for row in vehicle if row.get("mode"))
        stop_reasons = Counter(
            row.get("stop_reason") for row in control if row.get("stop_reason")
        )
        event_names = [row.get("event") for row in events if row.get("event")]
        fixed_samples = sum(
            1 for row in gnss if row.get("rtk_status") == "RTK FIXED"
        )
        curve_speeds = []
        straight_speeds = []
        for row in control:
            try:
                steering_value = abs(float(row.get("requested_steering", "")))
                speed_value = float(row.get("target_speed_mps", ""))
            except (TypeError, ValueError):
                continue
            if steering_value >= 0.45:
                curve_speeds.append(speed_value)
            elif steering_value <= 0.10:
                straight_speeds.append(speed_value)

        return {
            "session_path": self.session_path,
            "duration_seconds": (
                max(timestamps) - min(timestamps) if len(timestamps) >= 2 else 0.0
            ),
            "modes": dict(modes),
            "events": event_names,
            "stop_reasons": dict(stop_reasons),
            "gnss": {
                "samples": len(gnss),
                "rtk_fixed_samples": fixed_samples,
                "rtk_fixed_ratio": fixed_samples / len(gnss) if gnss else 0.0,
                "maximum_data_age_seconds": max(gnss_ages) if gnss_ages else None,
            },
            "sensor_health": {
                "gnss": _sensor_health(gnss),
                "imu": _sensor_health(imu),
                "steering": _sensor_health(steering),
                "lidar": _sensor_health(lidar_summary),
                "arduino": _sensor_health(arduino),
                "camera": _sensor_health(
                    perception,
                    "camera_is_valid",
                    "camera_data_age",
                    "camera_error_code",
                ),
            },
            "cross_track_error_m": {
                "samples": len(cross_track),
                "mean": sum(cross_track) / len(cross_track) if cross_track else None,
                "p95": _percentile(cross_track, 0.95),
                "maximum": max(cross_track) if cross_track else None,
            },
            "steering_error_degrees": {
                "samples": len(steering_error),
                "mean": (
                    sum(steering_error) / len(steering_error)
                    if steering_error
                    else None
                ),
                "p95": _percentile(steering_error, 0.95),
                "maximum": max(steering_error) if steering_error else None,
            },
            "curve_slowdown": {
                "straight_samples": len(straight_speeds),
                "curve_samples": len(curve_speeds),
                "straight_mean_target_speed_mps": (
                    sum(straight_speeds) / len(straight_speeds)
                    if straight_speeds
                    else None
                ),
                "curve_mean_target_speed_mps": (
                    sum(curve_speeds) / len(curve_speeds)
                    if curve_speeds
                    else None
                ),
                "observed": (
                    bool(straight_speeds and curve_speeds)
                    and sum(curve_speeds) / len(curve_speeds)
                    < sum(straight_speeds) / len(straight_speeds)
                ),
            },
        }

    @staticmethod
    def evaluate(
        report,
        required_mode=None,
        maximum_cross_track_error_m=None,
        maximum_steering_error_degrees=None,
        minimum_rtk_fixed_ratio=None,
        require_completion=False,
        required_stop_reason=None,
        required_event=None,
        minimum_sensor_valid_ratio=None,
    ):
        failures = []
        if required_mode and not report["modes"].get(required_mode):
            failures.append(f"MODE_NOT_OBSERVED:{required_mode}")
        maximum_cross_track = report["cross_track_error_m"]["maximum"]
        if maximum_cross_track_error_m is not None and (
            maximum_cross_track is None
            or maximum_cross_track > maximum_cross_track_error_m
        ):
            failures.append("CROSS_TRACK_ERROR_LIMIT")
        maximum_steering_error = report["steering_error_degrees"]["maximum"]
        if maximum_steering_error_degrees is not None and (
            maximum_steering_error is None
            or maximum_steering_error > maximum_steering_error_degrees
        ):
            failures.append("STEERING_ERROR_LIMIT")
        if (
            minimum_rtk_fixed_ratio is not None
            and report["gnss"]["rtk_fixed_ratio"] < minimum_rtk_fixed_ratio
        ):
            failures.append("RTK_FIXED_RATIO")
        if require_completion and "AUTO_ROUTE_COMPLETED" not in report["events"]:
            failures.append("AUTO_ROUTE_NOT_COMPLETED")
        if (
            required_stop_reason
            and not report["stop_reasons"].get(required_stop_reason)
        ):
            failures.append(f"STOP_REASON_NOT_OBSERVED:{required_stop_reason}")
        if required_event and required_event not in report["events"]:
            failures.append(f"EVENT_NOT_OBSERVED:{required_event}")
        if minimum_sensor_valid_ratio is not None:
            for sensor_name, health in report["sensor_health"].items():
                ratio = health["valid_ratio"]
                if ratio is not None and ratio < minimum_sensor_valid_ratio:
                    failures.append(f"SENSOR_VALID_RATIO:{sensor_name}")
        return {"passed": not failures, "failures": failures}
