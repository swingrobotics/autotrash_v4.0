from __future__ import annotations

import json
import math
import os
import threading

from autonomous_car.control.pure_pursuit import configure_vehicle_wheelbase_m
from autonomous_car.state import DriveMode

try:
    from camera_stream import config as vehicle_config
except ImportError:  # pragma: no cover - legacy standalone deployments
    vehicle_config = None


class VehicleSettingsError(ValueError):
    pass


class VehicleRuntimeSettings:
    """Persist and live-apply operator-tunable vehicle parameters.

    Machine-specific values are stored outside Git in vehicle-settings.json.
    Settings that affect motion can only be changed while the rover is stopped
    in DISARMED/MANUAL. Steering endpoint calibration remains owned by the
    existing steering calibration workflow and is intentionally not duplicated
    here.
    """

    DEFAULT_PATH = "/home/gnss/camera-stream/vehicle-settings.json"

    def __init__(self, legacy, path: str | None = None):
        self.legacy = legacy
        self.path = path or os.environ.get("VEHICLE_SETTINGS_PATH", self.DEFAULT_PATH)
        self._lock = threading.RLock()
        self._defaults = self._runtime_defaults()
        self._values = dict(self._defaults)
        self._load()
        self._apply(self._values)

    def _config_default(self, name, fallback):
        if vehicle_config is None:
            return fallback
        return getattr(vehicle_config, name, fallback)

    def _geometry_default(self, name, fallback=0.0):
        return float(
            getattr(
                self.legacy,
                name,
                self._config_default(name, fallback),
            )
        )

    def _runtime_defaults(self):
        return {
            "manual_max_throttle": float(
                getattr(
                    self.legacy,
                    "MANUAL_MAX_THROTTLE",
                    self._config_default("MANUAL_MAX_THROTTLE", 0.35),
                )
            ),
            # server.py historically did not import AUTO_MAX_THROTTLE. Fall
            # back to camera_stream.config so the environment variable remains
            # effective even before vehicle-settings.json exists.
            "auto_max_throttle": float(
                getattr(
                    self.legacy,
                    "AUTO_MAX_THROTTLE",
                    self._config_default("AUTO_MAX_THROTTLE", 0.35),
                )
            ),
            # Axle-center to axle-center distance. 0.53 m preserves the old
            # planner default until the operator measures the actual rover.
            "wheelbase_m": self._geometry_default("WHEELBASE_M", 0.53),
            # The remaining physical dimensions intentionally default to zero:
            # zero means "not measured yet" rather than pretending a guessed
            # number is a calibrated vehicle dimension.
            "front_track_width_m": self._geometry_default("FRONT_TRACK_WIDTH_M"),
            "rear_track_width_m": self._geometry_default("REAR_TRACK_WIDTH_M"),
            "wheel_diameter_m": self._geometry_default("WHEEL_DIAMETER_M"),
            "wheel_rolling_circumference_m": self._geometry_default(
                "WHEEL_ROLLING_CIRCUMFERENCE_M"
            ),
            "vehicle_width_m": self._geometry_default("VEHICLE_WIDTH_M"),
            "vehicle_length_m": self._geometry_default("VEHICLE_LENGTH_M"),
            "motor_min_pwm": int(getattr(self.legacy, "MOTOR_MIN_PWM", 80)),
            "motor_start_boost_seconds": float(getattr(self.legacy, "MOTOR_START_BOOST_SECONDS", 0.25)),
            "motor_timeout_seconds": float(getattr(self.legacy, "MOTOR_TIMEOUT_SECONDS", 0.30)),
            "steer_manual_pwm": int(getattr(self.legacy, "STEER_MANUAL_PWM", 255)),
            "steer_min_pwm": int(getattr(self.legacy, "STEER_MIN_PWM", 70)),
            "steer_control_kp": float(getattr(self.legacy, "STEER_CONTROL_KP", 4.0)),
            "steer_target_tolerance_degrees": float(
                getattr(self.legacy, "STEER_TARGET_TOLERANCE_DEGREES", 1.0)
            ),
            "steer_target_rate_dps": float(
                getattr(self.legacy, "STEER_TARGET_RATE_DEGREES_PER_SECOND", 360.0)
            ),
            "lidar_stop_distance_m": float(getattr(self.legacy, "LIDAR_STOP_DISTANCE_M", 0.60)),
            "lidar_crawl_distance_m": float(getattr(self.legacy, "LIDAR_CRAWL_DISTANCE_M", 0.80)),
            "lidar_slow_distance_m": float(getattr(self.legacy, "LIDAR_SLOW_DISTANCE_M", 1.50)),
            "lidar_safety_half_width_m": float(getattr(self.legacy, "LIDAR_SAFETY_HALF_WIDTH_M", 0.45)),
            "imu_heading_deadband_degrees": float(
                getattr(self.legacy, "IMU_HEADING_DEADBAND_DEGREES", 0.8)
            ),
            "imu_attitude_deadband_degrees": float(
                getattr(self.legacy, "IMU_ATTITUDE_DEADBAND_DEGREES", 0.15)
            ),
            "imu_turn_rate_threshold_dps": float(
                getattr(self.legacy, "IMU_TURN_RATE_THRESHOLD_DPS", 3.0)
            ),
        }

    @staticmethod
    def schema():
        return {
            "manual_max_throttle": {"min": 0.0, "max": 1.0, "step": 0.05, "unit": "%"},
            "auto_max_throttle": {"min": 0.0, "max": 1.0, "step": 0.05, "unit": "%"},
            "wheelbase_m": {"min": 0.20, "max": 2.00, "step": 0.001, "unit": "m"},
            "front_track_width_m": {"min": 0.0, "max": 2.00, "step": 0.001, "unit": "m"},
            "rear_track_width_m": {"min": 0.0, "max": 2.00, "step": 0.001, "unit": "m"},
            "wheel_diameter_m": {"min": 0.0, "max": 1.00, "step": 0.001, "unit": "m"},
            "wheel_rolling_circumference_m": {"min": 0.0, "max": 4.00, "step": 0.001, "unit": "m"},
            "vehicle_width_m": {"min": 0.0, "max": 3.00, "step": 0.001, "unit": "m"},
            "vehicle_length_m": {"min": 0.0, "max": 5.00, "step": 0.001, "unit": "m"},
            "motor_min_pwm": {"min": 0, "max": 255, "step": 1, "unit": "PWM"},
            "motor_start_boost_seconds": {"min": 0.0, "max": 2.0, "step": 0.05, "unit": "s"},
            "motor_timeout_seconds": {"min": 0.10, "max": 2.0, "step": 0.05, "unit": "s"},
            "steer_manual_pwm": {"min": 35, "max": 255, "step": 1, "unit": "PWM"},
            "steer_min_pwm": {"min": 0, "max": 255, "step": 1, "unit": "PWM"},
            "steer_control_kp": {"min": 0.5, "max": 20.0, "step": 0.5, "unit": "Kp"},
            "steer_target_tolerance_degrees": {"min": 0.2, "max": 5.0, "step": 0.1, "unit": "deg"},
            "steer_target_rate_dps": {"min": 10.0, "max": 720.0, "step": 10.0, "unit": "deg/s"},
            "lidar_stop_distance_m": {"min": 0.20, "max": 5.0, "step": 0.05, "unit": "m"},
            "lidar_crawl_distance_m": {"min": 0.25, "max": 6.0, "step": 0.05, "unit": "m"},
            "lidar_slow_distance_m": {"min": 0.30, "max": 10.0, "step": 0.05, "unit": "m"},
            "lidar_safety_half_width_m": {"min": 0.20, "max": 1.50, "step": 0.01, "unit": "m"},
            "imu_heading_deadband_degrees": {"min": 0.0, "max": 10.0, "step": 0.1, "unit": "deg"},
            "imu_attitude_deadband_degrees": {"min": 0.0, "max": 5.0, "step": 0.05, "unit": "deg"},
            "imu_turn_rate_threshold_dps": {"min": 0.1, "max": 30.0, "step": 0.1, "unit": "deg/s"},
        }

    @staticmethod
    def _geometry_keys():
        return (
            "wheelbase_m",
            "front_track_width_m",
            "rear_track_width_m",
            "wheel_diameter_m",
            "wheel_rolling_circumference_m",
            "vehicle_width_m",
            "vehicle_length_m",
        )

    def _normalize(self, payload, *, base=None):
        if not isinstance(payload, dict):
            raise VehicleSettingsError("settings must be a JSON object")
        result = dict(self._values if base is None else base)
        schema = self.schema()
        for key, raw in payload.items():
            if key not in schema:
                raise VehicleSettingsError(f"Unknown vehicle setting: {key}")
            spec = schema[key]
            try:
                value = float(raw)
            except (TypeError, ValueError) as error:
                raise VehicleSettingsError(f"{key} must be numeric") from error
            if not math.isfinite(value):
                raise VehicleSettingsError(f"{key} must be finite")
            if value < spec["min"] or value > spec["max"]:
                raise VehicleSettingsError(
                    f"{key} must be between {spec['min']} and {spec['max']}"
                )
            if key in {"motor_min_pwm", "steer_manual_pwm", "steer_min_pwm"}:
                value = int(round(value))
            result[key] = value

        if result["steer_min_pwm"] > result["steer_manual_pwm"]:
            raise VehicleSettingsError("steer_min_pwm cannot exceed steer_manual_pwm")
        if not (
            result["lidar_stop_distance_m"]
            < result["lidar_crawl_distance_m"]
            < result["lidar_slow_distance_m"]
        ):
            raise VehicleSettingsError(
                "LiDAR distances must satisfy STOP < CRAWL < SLOW"
            )
        return result

    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as file:
                stored = json.load(file)
        except FileNotFoundError:
            return
        except (OSError, ValueError, json.JSONDecodeError):
            return
        try:
            self._values = self._normalize(stored, base=self._defaults)
        except VehicleSettingsError:
            self._values = dict(self._defaults)

    def _persist(self, values):
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = f"{self.path}.tmp"
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(values, file, indent=2, sort_keys=True)
            file.flush()
            os.fsync(file.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, self.path)
        # fsync the parent directory as well so the rename itself survives a
        # sudden power loss on Linux/Raspberry Pi filesystems.
        if directory and hasattr(os, "O_DIRECTORY"):
            try:
                directory_fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except OSError:
                pass

    def _apply(self, values):
        legacy = self.legacy
        legacy.MANUAL_MAX_THROTTLE = float(values["manual_max_throttle"])
        legacy.AUTO_MAX_THROTTLE = float(values["auto_max_throttle"])

        geometry_bindings = {
            "wheelbase_m": "WHEELBASE_M",
            "front_track_width_m": "FRONT_TRACK_WIDTH_M",
            "rear_track_width_m": "REAR_TRACK_WIDTH_M",
            "wheel_diameter_m": "WHEEL_DIAMETER_M",
            "wheel_rolling_circumference_m": "WHEEL_ROLLING_CIRCUMFERENCE_M",
            "vehicle_width_m": "VEHICLE_WIDTH_M",
            "vehicle_length_m": "VEHICLE_LENGTH_M",
        }
        for setting_name, runtime_name in geometry_bindings.items():
            value = float(values[setting_name])
            setattr(legacy, runtime_name, value)
            if vehicle_config is not None:
                setattr(vehicle_config, runtime_name, value)

        # Wheelbase is already consumed by the bicycle/PurePursuit steering
        # geometry. Update existing instances and the default for future ones.
        configure_vehicle_wheelbase_m(values["wheelbase_m"])

        legacy.MOTOR_MIN_PWM = int(values["motor_min_pwm"])
        legacy.MOTOR_START_BOOST_SECONDS = float(values["motor_start_boost_seconds"])
        legacy.MOTOR_TIMEOUT_SECONDS = float(values["motor_timeout_seconds"])
        legacy.STEER_MANUAL_PWM = int(values["steer_manual_pwm"])
        legacy.STEER_MIN_PWM = int(values["steer_min_pwm"])
        legacy.STEER_CONTROL_KP = float(values["steer_control_kp"])
        legacy.STEER_TARGET_TOLERANCE_DEGREES = float(
            values["steer_target_tolerance_degrees"]
        )
        legacy.STEER_TARGET_RATE_DEGREES_PER_SECOND = float(
            values["steer_target_rate_dps"]
        )
        legacy.LIDAR_STOP_DISTANCE_M = float(values["lidar_stop_distance_m"])
        legacy.LIDAR_CRAWL_DISTANCE_M = float(values["lidar_crawl_distance_m"])
        legacy.LIDAR_SLOW_DISTANCE_M = float(values["lidar_slow_distance_m"])
        legacy.LIDAR_SAFETY_HALF_WIDTH_M = float(values["lidar_safety_half_width_m"])
        legacy.IMU_HEADING_DEADBAND_DEGREES = float(
            values["imu_heading_deadband_degrees"]
        )
        legacy.IMU_ATTITUDE_DEADBAND_DEGREES = float(
            values["imu_attitude_deadband_degrees"]
        )
        legacy.IMU_TURN_RATE_THRESHOLD_DPS = float(
            values["imu_turn_rate_threshold_dps"]
        )

        supervisor = legacy.safety_supervisor
        supervisor.maximum_throttle = float(values["auto_max_throttle"])
        supervisor.manual_maximum_throttle = float(values["manual_max_throttle"])
        supervisor.stop_distance_m = float(values["lidar_stop_distance_m"])
        supervisor.crawl_distance_m = float(values["lidar_crawl_distance_m"])
        supervisor.slow_distance_m = float(values["lidar_slow_distance_m"])
        supervisor.obstacle_checker.half_width_m = float(
            values["lidar_safety_half_width_m"]
        )

    @staticmethod
    def _numeric_snapshot_value(snapshot, *keys):
        for key in keys:
            if key not in snapshot:
                continue
            try:
                value = float(snapshot.get(key) or 0.0)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value):
                return value
        return 0.0

    def _editing_allowed(self):
        mode = self.legacy.vehicle_state_machine.mode
        try:
            canonical = mode.canonical
        except AttributeError:
            canonical = DriveMode(mode).canonical
        motor = self.legacy.motor_controller.snapshot()
        throttle = self._numeric_snapshot_value(motor, "throttle", "drive_throttle")
        steering_output = self._numeric_snapshot_value(
            motor,
            "steering_pwm",
            "steering_motor_command",
            "motor_command",
        )
        allowed_mode = canonical in {DriveMode.DISARMED, DriveMode.MANUAL}
        throttle_idle = abs(throttle) < 0.001
        steering_idle = abs(steering_output) < 0.001
        allowed = allowed_mode and throttle_idle and steering_idle
        reason = None
        if not allowed_mode:
            reason = "Vehicle settings can only be changed in DISARMED or MANUAL"
        elif not throttle_idle:
            reason = "Release throttle before changing vehicle settings"
        elif not steering_idle:
            reason = "Wait for steering output to stop before changing vehicle settings"
        return allowed, reason

    def snapshot(self):
        with self._lock:
            allowed, reason = self._editing_allowed()
            geometry = {key: float(self._values[key]) for key in self._geometry_keys()}
            measured = {
                key: (value > 0.0 if key != "wheelbase_m" else True)
                for key, value in geometry.items()
            }
            return {
                "settings": dict(self._values),
                "defaults": dict(self._defaults),
                "schema": self.schema(),
                "geometry": {
                    "values": geometry,
                    "measured": measured,
                    "complete": all(measured.values()),
                },
                "editable": allowed,
                "edit_block_reason": reason,
                "path": self.path,
            }

    def update(self, payload):
        with self._lock:
            allowed, reason = self._editing_allowed()
            if not allowed:
                raise VehicleSettingsError(reason or "Vehicle settings are locked")
            if payload.get("reset") is True:
                values = dict(self._defaults)
            else:
                incoming = payload.get("settings", payload)
                values = self._normalize(incoming)

            previous = dict(self._values)
            try:
                self._apply(values)
                self._persist(values)
            except Exception:
                # Do not leave a partially-applied live configuration if disk
                # persistence or an unexpected runtime assignment fails.
                self._apply(previous)
                raise
            self._values = dict(values)
            return self.snapshot()


__all__ = ["VehicleRuntimeSettings", "VehicleSettingsError"]
