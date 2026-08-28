import json
import tempfile
from types import SimpleNamespace

from autonomous_car.control.pure_pursuit import PurePursuit, configured_vehicle_wheelbase_m
from autonomous_car.safety import ObstacleChecker, SafetySupervisor
from autonomous_car.state import ControlRequest, DriveMode, SafetyContext, SensorStatus
from vehicle_runtime_settings import VehicleRuntimeSettings, VehicleSettingsError


class FakeMotor:
    def __init__(self):
        self.throttle = 0.0

    def snapshot(self):
        return {"throttle": self.throttle}


class FakeStateMachine:
    def __init__(self):
        self.mode = DriveMode.DISARMED


class FakeLegacy(SimpleNamespace):
    pass


def sensor(value=True):
    return SensorStatus(value=value, timestamp=0.0, is_valid=True, data_age=0.0)


def build_legacy():
    supervisor = SafetySupervisor(
        obstacle_checker=ObstacleChecker(half_width_m=0.45),
        maximum_throttle=0.35,
        manual_maximum_throttle=0.35,
    )
    return FakeLegacy(
        MANUAL_MAX_THROTTLE=0.35,
        AUTO_MAX_THROTTLE=0.35,
        WHEELBASE_M=0.53,
        FRONT_TRACK_WIDTH_M=0.0,
        REAR_TRACK_WIDTH_M=0.0,
        WHEEL_DIAMETER_M=0.0,
        WHEEL_ROLLING_CIRCUMFERENCE_M=0.0,
        VEHICLE_WIDTH_M=0.0,
        VEHICLE_LENGTH_M=0.0,
        MOTOR_MIN_PWM=80,
        MOTOR_START_BOOST_SECONDS=0.25,
        MOTOR_TIMEOUT_SECONDS=0.30,
        STEER_MANUAL_PWM=255,
        STEER_MIN_PWM=70,
        STEER_CONTROL_KP=4.0,
        STEER_TARGET_TOLERANCE_DEGREES=1.0,
        STEER_TARGET_RATE_DEGREES_PER_SECOND=360.0,
        LIDAR_STOP_DISTANCE_M=0.60,
        LIDAR_CRAWL_DISTANCE_M=0.80,
        LIDAR_SLOW_DISTANCE_M=1.50,
        LIDAR_SAFETY_HALF_WIDTH_M=0.45,
        IMU_HEADING_DEADBAND_DEGREES=0.8,
        IMU_ATTITUDE_DEADBAND_DEGREES=0.15,
        IMU_TURN_RATE_THRESHOLD_DPS=3.0,
        safety_supervisor=supervisor,
        vehicle_state_machine=FakeStateMachine(),
        motor_controller=FakeMotor(),
    )


def validate():
    with tempfile.TemporaryDirectory() as directory:
        path = f"{directory}/vehicle-settings.json"
        legacy = build_legacy()
        existing_pursuit = PurePursuit(wheelbase_m=0.53)
        manager = VehicleRuntimeSettings(legacy, path=path)
        initial = manager.snapshot()
        assert initial["editable"] is True
        assert initial["settings"]["manual_max_throttle"] == 0.35
        assert initial["settings"]["wheelbase_m"] == 0.53
        assert initial["settings"]["front_track_width_m"] == 0.0
        assert initial["geometry"]["complete"] is False
        assert abs(existing_pursuit.wheelbase_m - 0.53) < 1e-9

        updated = manager.update(
            {
                "settings": {
                    "manual_max_throttle": 0.70,
                    "auto_max_throttle": 0.30,
                    "wheelbase_m": 0.61,
                    "front_track_width_m": 0.48,
                    "rear_track_width_m": 0.47,
                    "wheel_diameter_m": 0.205,
                    "wheel_rolling_circumference_m": 0.638,
                    "vehicle_width_m": 0.57,
                    "vehicle_length_m": 0.91,
                    "motor_min_pwm": 92,
                    "motor_start_boost_seconds": 0.35,
                    "steer_manual_pwm": 250,
                    "steer_min_pwm": 85,
                    "steer_control_kp": 5.5,
                    "steer_target_rate_dps": 420,
                    "lidar_stop_distance_m": 0.65,
                    "lidar_crawl_distance_m": 0.90,
                    "lidar_slow_distance_m": 1.70,
                    "lidar_safety_half_width_m": 0.50,
                    "imu_heading_deadband_degrees": 1.0,
                }
            }
        )
        assert updated["settings"]["manual_max_throttle"] == 0.70
        assert updated["settings"]["wheelbase_m"] == 0.61
        assert updated["settings"]["front_track_width_m"] == 0.48
        assert updated["settings"]["wheel_rolling_circumference_m"] == 0.638
        assert updated["geometry"]["complete"] is True
        assert legacy.MANUAL_MAX_THROTTLE == 0.70
        assert legacy.AUTO_MAX_THROTTLE == 0.30
        assert legacy.WHEELBASE_M == 0.61
        assert legacy.FRONT_TRACK_WIDTH_M == 0.48
        assert legacy.REAR_TRACK_WIDTH_M == 0.47
        assert legacy.WHEEL_DIAMETER_M == 0.205
        assert legacy.WHEEL_ROLLING_CIRCUMFERENCE_M == 0.638
        assert legacy.VEHICLE_WIDTH_M == 0.57
        assert legacy.VEHICLE_LENGTH_M == 0.91
        assert abs(existing_pursuit.wheelbase_m - 0.61) < 1e-9
        assert abs(configured_vehicle_wheelbase_m() - 0.61) < 1e-9
        new_pursuit = PurePursuit(wheelbase_m=0.53)
        assert abs(new_pursuit.wheelbase_m - 0.61) < 1e-9
        assert legacy.MOTOR_MIN_PWM == 92
        assert legacy.STEER_TARGET_RATE_DEGREES_PER_SECOND == 420
        assert legacy.safety_supervisor.manual_maximum_throttle == 0.70
        assert legacy.safety_supervisor.maximum_throttle == 0.30
        assert legacy.safety_supervisor.obstacle_checker.half_width_m == 0.50

        manual = legacy.safety_supervisor.evaluate(
            ControlRequest(throttle=1.0, steering=0.0, enabled=True),
            SafetyContext(
                mode=DriveMode.MANUAL,
                arduino=sensor(),
                lidar=sensor([]),
                steering=sensor(0.0),
            ),
        )
        auto = legacy.safety_supervisor.evaluate(
            ControlRequest(throttle=1.0, steering=0.0, enabled=True),
            SafetyContext(
                mode=DriveMode.AUTO_AI,
                arduino=sensor(),
                lidar=sensor([]),
                steering=sensor(0.0),
            ),
        )
        assert manual.final_throttle == 0.70
        assert auto.final_throttle == 0.30

        with open(path, "r", encoding="utf-8") as file:
            persisted = json.load(file)
        assert persisted["motor_min_pwm"] == 92
        assert persisted["wheelbase_m"] == 0.61
        assert persisted["front_track_width_m"] == 0.48
        assert persisted["rear_track_width_m"] == 0.47
        assert persisted["wheel_diameter_m"] == 0.205
        assert persisted["wheel_rolling_circumference_m"] == 0.638
        assert persisted["vehicle_width_m"] == 0.57
        assert persisted["vehicle_length_m"] == 0.91

        reload_legacy = build_legacy()
        reloaded = VehicleRuntimeSettings(reload_legacy, path=path)
        reloaded_snapshot = reloaded.snapshot()
        assert reloaded_snapshot["settings"]["manual_max_throttle"] == 0.70
        assert reloaded_snapshot["settings"]["wheelbase_m"] == 0.61
        assert reloaded_snapshot["settings"]["wheel_diameter_m"] == 0.205
        assert reloaded_snapshot["geometry"]["complete"] is True
        assert reload_legacy.WHEELBASE_M == 0.61
        assert reload_legacy.FRONT_TRACK_WIDTH_M == 0.48
        assert reload_legacy.WHEEL_ROLLING_CIRCUMFERENCE_M == 0.638
        assert reload_legacy.VEHICLE_WIDTH_M == 0.57
        assert reload_legacy.safety_supervisor.maximum_throttle == 0.30

        try:
            manager.update(
                {
                    "settings": {
                        "lidar_stop_distance_m": 1.0,
                        "lidar_crawl_distance_m": 0.8,
                    }
                }
            )
            raise AssertionError("invalid LiDAR ordering should be rejected")
        except VehicleSettingsError:
            pass

        try:
            manager.update({"settings": {"wheelbase_m": 0.10}})
            raise AssertionError("invalid wheelbase should be rejected")
        except VehicleSettingsError:
            pass

        try:
            manager.update({"settings": {"wheel_diameter_m": 1.50}})
            raise AssertionError("invalid wheel diameter should be rejected")
        except VehicleSettingsError:
            pass

        legacy.vehicle_state_machine.mode = DriveMode.RECORD
        try:
            manager.update({"settings": {"manual_max_throttle": 0.5}})
            raise AssertionError("RECORD should lock vehicle settings")
        except VehicleSettingsError:
            pass

        legacy.vehicle_state_machine.mode = DriveMode.MANUAL
        legacy.motor_controller.throttle = 0.1
        try:
            manager.update({"settings": {"manual_max_throttle": 0.5}})
            raise AssertionError("non-zero throttle should lock vehicle settings")
        except VehicleSettingsError:
            pass

        panel_source = open("vehicle_settings_panel.py", encoding="utf-8").read()
        for control_id in (
            "vs-wheelbase-mm",
            "vs-front-track-mm",
            "vs-rear-track-mm",
            "vs-wheel-diameter-mm",
            "vs-wheel-circumference-mm",
            "vs-vehicle-width-mm",
            "vs-vehicle-length-mm",
        ):
            assert control_id in panel_source, f"missing geometry control: {control_id}"

        return {
            "persistence": "PASS",
            "live_apply": "PASS",
            "wheelbase_geometry": "PASS",
            "measured_vehicle_geometry": "PASS",
            "manual_auto_throttle_split": "PASS",
            "validation": "PASS",
            "motion_lock": "PASS",
            "dashboard_geometry_controls": "PASS",
        }


def main():
    result = validate()
    print("Vehicle settings V2: PASS")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
