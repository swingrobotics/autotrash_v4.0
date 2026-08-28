import csv
import json
import math
import os
import struct
import tempfile
import time
from types import SimpleNamespace
import zlib

from autonomous_car.control import LaneController, PathPoint, ThrottleCalibration
from autonomous_car.perception import (
    CameraCalibration,
    DetectedObject,
    ObjectDetector,
)
from autonomous_car.recording import FieldRunReport, LogReplay, RecordManager
from autonomous_car.modes import (
    AutoRoutePlanner,
    HybridFallbackGuard,
    LaneContinuityFilter,
)
from autonomous_car.safety import (
    RestartDelayGuard,
    SafetySupervisor,
    SteeringTrackingGuard,
)
from autonomous_car.simulation import RouteSimulator
from autonomous_car.state import ControlRequest, DriveMode, SafetyContext, SensorStatus
from autonomous_car.state_machine import InvalidStateTransition, VehicleStateMachine
from camera_stream.motor import drive_pwm_magnitude


def sensor(value):
    return SensorStatus(value=value, timestamp=0.0, is_valid=True, data_age=0.0)


def check_safety():
    supervisor = SafetySupervisor(obstacle_restart_delay_seconds=0.0)
    base_context = SafetyContext(
        mode=DriveMode.MANUAL_ASSIST,
        arduino=sensor(True),
        lidar=sensor([]),
        steering=sensor(0.0),
    )
    request = ControlRequest(0.35, 0.0, True, True, "validation")
    clear = supervisor.evaluate(request, base_context)
    stop_context = SafetyContext(
        mode=DriveMode.MANUAL_ASSIST,
        arduino=sensor(True),
        lidar=sensor([{"bearing_degrees": 0.0, "distance_mm": 700}]),
        steering=sensor(0.0),
    )
    stopped = supervisor.evaluate(request, stop_context)
    released = supervisor.evaluate(
        ControlRequest(0.35, 0.0, True, False, "validation"),
        base_context,
    )
    delayed = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=sensor(True),
            lidar=sensor([]),
            steering=sensor(0.0),
            loop_delay_seconds=0.25,
        ),
    )
    outside_corridor = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=sensor(True),
            lidar=sensor([{"bearing_degrees": 45.0, "distance_mm": 700}]),
            steering=sensor(0.0),
        ),
    )
    rear_obstacle = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=sensor(True),
            lidar=sensor([{"bearing_degrees": 180.0, "distance_mm": 300}]),
            steering=sensor(0.0),
        ),
    )
    reverse_with_front_obstacle = supervisor.evaluate(
        ControlRequest(-0.35, 0.0, True, True, "validation"),
        stop_context,
    )
    self_reflection = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=sensor(True),
            lidar=sensor(
                [{"bearing_degrees": -34.71, "distance_mm": 10, "confidence": 16}]
            ),
            steering=sensor(0.0),
        ),
    )
    alongside_vehicle = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=sensor(True),
            lidar=sensor(
                [{"bearing_degrees": 80.0, "distance_mm": 260, "confidence": 120}]
            ),
            steering=sensor(0.0),
        ),
    )
    restart_supervisor = SafetySupervisor(obstacle_restart_delay_seconds=1.5)
    restart_supervisor.evaluate(request, stop_context)
    restart_delayed = restart_supervisor.evaluate(request, base_context)
    stale_lidar = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=sensor(True),
            lidar=SensorStatus(value=[], timestamp=0.0, is_valid=True, data_age=0.31),
            steering=sensor(0.0),
        ),
    )
    stale_steering = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=sensor(True),
            lidar=sensor([]),
            steering=SensorStatus(value=0.0, timestamp=0.0, is_valid=True, data_age=0.36),
        ),
    )
    missing_arduino = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=SensorStatus(value=None, timestamp=None, is_valid=False),
            lidar=sensor([]),
            steering=sensor(0.0),
        ),
    )
    stale_arduino = supervisor.evaluate(
        request,
        SafetyContext(
            mode=DriveMode.MANUAL_ASSIST,
            arduino=SensorStatus(
                value=True,
                timestamp=0.0,
                is_valid=True,
                data_age=0.51,
            ),
            lidar=sensor([]),
            steering=sensor(0.0),
        ),
    )
    stale_command = supervisor.evaluate(
        ControlRequest(
            0.35,
            0.0,
            True,
            True,
            "validation",
            timestamp=time.monotonic() - 0.31,
        ),
        base_context,
    )
    safety_snapshot = supervisor.snapshot()
    passed = (
        clear.allowed
        and clear.final_throttle == 0.35
        and stopped.stop_reason == "OBSTACLE_STOP"
        and stopped.final_throttle == 0.0
        and released.stop_reason == "DEADMAN_RELEASED"
        and delayed.stop_reason == "CONTROL_LOOP_DELAY"
        and outside_corridor.allowed
        and rear_obstacle.allowed
        and reverse_with_front_obstacle.allowed
        and reverse_with_front_obstacle.final_throttle == -0.35
        and self_reflection.allowed
        and self_reflection.obstacle_distance_m is None
        and alongside_vehicle.allowed
        and alongside_vehicle.obstacle_distance_m is None
        and restart_delayed.stop_reason == "OBSTACLE_RESTART_DELAY"
        and restart_delayed.final_throttle == 0.0
        and stale_lidar.stop_reason == "LIDAR_TIMEOUT"
        and stale_steering.stop_reason == "STEERING_SENSOR_TIMEOUT"
        and missing_arduino.stop_reason == "ARDUINO_UNAVAILABLE"
        and stale_arduino.stop_reason == "ARDUINO_TIMEOUT"
        and stale_command.stop_reason == "COMMAND_TIMEOUT"
        and safety_snapshot["input_source"] == "validation"
    )
    return passed, {
        "clear_throttle": clear.final_throttle,
        "obstacle_reason": stopped.stop_reason,
        "deadman_reason": released.stop_reason,
        "loop_delay_reason": delayed.stop_reason,
        "outside_corridor_allowed": outside_corridor.allowed,
        "rear_obstacle_allowed": rear_obstacle.allowed,
        "reverse_throttle_with_front_obstacle": reverse_with_front_obstacle.final_throttle,
        "self_reflection_allowed": self_reflection.allowed,
        "obstacle_restart_reason": restart_delayed.stop_reason,
        "lidar_timeout_reason": stale_lidar.stop_reason,
        "steering_timeout_reason": stale_steering.stop_reason,
        "arduino_reason": missing_arduino.stop_reason,
        "arduino_timeout_reason": stale_arduino.stop_reason,
        "command_timeout_reason": stale_command.stop_reason,
        "input_source": safety_snapshot["input_source"],
    }


def check_steering_tracking():
    guard = SteeringTrackingGuard(maximum_error_degrees=7.0, timeout_seconds=1.0)
    initial = guard.evaluate(12.0, 0.0, active=True, now=10.0)
    waiting = guard.evaluate(12.0, 0.0, active=True, now=10.9)
    fault = guard.evaluate(12.0, 0.0, active=True, now=11.01)
    recovered = guard.evaluate(2.0, 1.0, active=True, now=11.1)
    snapshot = guard.snapshot(now=11.1)
    passed = (
        initial is None
        and waiting is None
        and fault == "STEERING_TRACKING_ERROR"
        and recovered is None
        and snapshot["exceeded_seconds"] == 0.0
    )
    return passed, {
        "fault": fault,
        "recovered": recovered,
        "snapshot": snapshot,
    }


def route_paths():
    straight = [PathPoint(index * 0.2, 0.0) for index in range(51)]
    arc = [
        PathPoint(5.0 * math.sin(angle), 5.0 * (1.0 - math.cos(angle)))
        for angle in [index * (math.pi / 2.0) / 60.0 for index in range(61)]
    ]
    s_curve = [
        PathPoint(index * 0.2, 0.55 * math.sin(index * 0.2 * math.pi / 4.0))
        for index in range(61)
    ]
    return {"straight": straight, "arc": arc, "s_curve": s_curve}


def check_routes():
    simulator = RouteSimulator()
    details = {}
    passed = True
    for name, path in route_paths().items():
        maximum_error = 0.0
        all_completed = True
        for offset_index in range(10):
            offset = (offset_index - 4.5) * 0.025
            result = simulator.run(path, initial_y=path[0].y + offset)
            maximum_error = max(maximum_error, result.maximum_cross_track_error_m)
            all_completed = all_completed and result.completed
        details[name] = {
            "completed_runs": 10 if all_completed else 0,
            "maximum_cross_track_error_m": round(maximum_error, 3),
        }
        passed = passed and all_completed and maximum_error <= 0.30
    return passed, details


def check_lane():
    controller = LaneController()
    if not controller.available:
        return False, {"error": "OPENCV_UNAVAILABLE"}
    import cv2
    import numpy as np

    image = np.zeros((360, 640, 3), dtype=np.uint8)
    cv2.line(image, (145, 345), (275, 190), (255, 255, 255), 8)
    cv2.line(image, (495, 345), (365, 190), (255, 255, 255), 8)
    encoded, jpeg = cv2.imencode(".jpg", image)
    result = controller.analyze_jpeg(jpeg.tobytes() if encoded else b"")
    passed = (
        result.detected
        and result.confidence >= 0.5
        and abs(result.lateral_error_m or 0.0) <= 0.10
        and abs(result.correction_angle_degrees) <= 5.0
    )
    return passed, result.as_dict()


def check_hybrid_fallback():
    guard = HybridFallbackGuard(camera_timeout_seconds=0.3, maximum_lane_failures=5)
    timeout = guard.evaluate(0.31)
    unchanged = guard.evaluate(0.10, new_frame=False)
    misses = [
        guard.evaluate(
            0.10,
            new_frame=True,
            lane_result={"detected": False},
        )
        for _ in range(5)
    ]
    guard.evaluate(
        0.10,
        new_frame=True,
        lane_result={"detected": False},
    )
    recovered = guard.evaluate(
        0.10,
        new_frame=True,
        lane_result={"detected": True, "confidence": 0.8},
    )
    low_confidence = [
        guard.evaluate(
            0.10,
            new_frame=True,
            lane_result={"detected": True, "confidence": 0.42},
        )
        for _ in range(5)
    ]
    passed = (
        timeout.fallback_reason == "CAMERA_TIMEOUT"
        and unchanged.fallback_reason is None
        and unchanged.lane_failure_count == 0
        and all(item.fallback_reason is None for item in misses[:4])
        and misses[4].fallback_reason == "LANE_NOT_DETECTED"
        and recovered.fallback_reason is None
        and recovered.lane_failure_count == 0
        and all(item.fallback_reason is None for item in low_confidence[:4])
        and low_confidence[4].fallback_reason == "LANE_CONFIDENCE_LOW"
    )
    return passed, {
        "camera_timeout_reason": timeout.fallback_reason,
        "failure_counts": [item.lane_failure_count for item in misses],
        "lane_fallback_reason": misses[4].fallback_reason,
        "low_confidence_fallback_reason": low_confidence[4].fallback_reason,
        "recovered_failure_count": recovered.lane_failure_count,
    }


def check_lane_continuity():
    continuity = LaneContinuityFilter(
        maximum_lateral_jump_m=0.35,
        maximum_heading_jump_degrees=12.0,
        correction_smoothing=0.5,
    )
    first = continuity.filter(
        {
            "detected": True,
            "confidence": 0.9,
            "lateral_error_m": 0.02,
            "heading_error_degrees": 1.0,
            "correction_angle_degrees": 2.0,
        }
    )
    stable = continuity.filter(
        {
            "detected": True,
            "confidence": 0.9,
            "lateral_error_m": 0.04,
            "heading_error_degrees": 2.0,
            "correction_angle_degrees": 4.0,
        }
    )
    jumped = continuity.filter(
        {
            "detected": True,
            "confidence": 0.95,
            "lateral_error_m": 0.60,
            "heading_error_degrees": 25.0,
            "correction_angle_degrees": -5.0,
        }
    )
    passed = (
        first["detected"]
        and stable["detected"]
        and 0.0 < stable["confidence"] < 0.9
        and stable["correction_angle_degrees"] == 3.0
        and not jumped["detected"]
        and jumped["error"] == "LANE_TEMPORAL_JUMP"
        and jumped["correction_angle_degrees"] == 0.0
    )
    return passed, {
        "stable_confidence": stable["confidence"],
        "smoothed_correction_degrees": stable["correction_angle_degrees"],
        "jump_error": jumped.get("error"),
    }


def check_camera_calibration():
    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "camera-calibration.json")
        calibration = CameraCalibration(path)
        snapshot = calibration.save(
            {
                "schema": "camera_calibration_v1",
                "image_size": [1280, 720],
                "camera_matrix": [
                    [640.0, 0.0, 640.0],
                    [0.0, 640.0, 360.0],
                    [0.0, 0.0, 1.0],
                ],
                "distortion_coefficients": [0.0, 0.0, 0.0, 0.0, 0.0],
                "rms_error": 0.25,
                "samples": 18,
            }
        )
        reloaded = CameraCalibration(path)
    passed = (
        snapshot["calibrated"]
        and reloaded.calibrated
        and abs(reloaded.horizontal_fov_degrees() - 90.0) < 1e-6
        and reloaded.snapshot()["samples"] == 18
    )
    return passed, reloaded.snapshot()


def check_object_fusion():
    detection = DetectedObject("person", 0.9, 280, 100, 80, 180, 0.0, 0.0)
    fused = ObjectDetector.fuse_lidar(
        [detection],
        [{"bearing_degrees": 1.0, "distance_mm": 1500}],
    )[0]
    return fused.in_vehicle_path, fused.as_dict()


def check_throttle():
    with tempfile.TemporaryDirectory() as directory:
        calibration = ThrottleCalibration(os.path.join(directory, "calibration.json"))
        calibration.set_points([(0.0, 0.0), (0.2, 0.25), (0.4, 0.45)])
        interpolated = calibration.throttle_for_speed(0.3)
    pwm = {
        "20_boost": drive_pwm_magnitude(0.20, True, 80),
        "20_settled": drive_pwm_magnitude(0.20, False, 80),
        "30_boost": drive_pwm_magnitude(0.30, True, 80),
        "30_settled": drive_pwm_magnitude(0.30, False, 80),
        "35_settled": drive_pwm_magnitude(0.35, False, 80),
    }
    passed = (
        abs(interpolated - 0.35) < 1e-9
        and pwm
        == {
            "20_boost": 80,
            "20_settled": 51,
            "30_boost": 80,
            "30_settled": 77,
            "35_settled": 89,
        }
    )
    return passed, {"throttle_at_0_3_mps": interpolated, "pwm": pwm}


def check_compressed_replay():
    with tempfile.TemporaryDirectory() as directory:
        with open(os.path.join(directory, "metadata.json"), "w", encoding="utf-8") as file:
            json.dump({"lidar_raw_encoding": "zlib_json_frames_v1"}, file)
        scan = [{"bearing_degrees": 0.0, "distance_mm": 1500, "confidence": 200}]
        payload = zlib.compress(json.dumps(scan).encode("utf-8"), level=1)
        with open(os.path.join(directory, "lidar_raw.bin"), "wb") as file:
            file.write(struct.pack("<dI", 123.5, len(payload)))
            file.write(payload)
        timestamp, restored = next(LogReplay(directory).iter_lidar_raw())
    passed = timestamp == 123.5 and restored == scan
    return passed, {"timestamp": timestamp, "point_count": len(restored)}


def check_state_replay():
    with tempfile.TemporaryDirectory() as directory:
        with open(os.path.join(directory, "metadata.json"), "w", encoding="utf-8") as file:
            json.dump({"timebase": "python_monotonic_seconds"}, file)
        with open(
            os.path.join(directory, "vehicle_state.csv"),
            "w",
            encoding="utf-8",
            newline="",
        ) as file:
            writer = csv.DictWriter(file, fieldnames=["monotonic", "mode"])
            writer.writeheader()
            writer.writerow({"monotonic": 10.0, "mode": "MANUAL_ASSIST"})
            writer.writerow({"monotonic": 12.0, "mode": "RECORD"})
        with open(
            os.path.join(directory, "control.csv"),
            "w",
            encoding="utf-8",
            newline="",
        ) as file:
            writer = csv.DictWriter(
                file,
                fieldnames=["monotonic", "requested_throttle", "final_throttle"],
            )
            writer.writeheader()
            writer.writerow(
                {
                    "monotonic": 10.5,
                    "requested_throttle": 0.30,
                    "final_throttle": 0.12,
                }
            )
        replay = LogReplay(directory)
        restored = replay.state_at(11.0)
    passed = (
        restored["vehicle_state"]["mode"] == "MANUAL_ASSIST"
        and restored["control"]["requested_throttle"] == "0.3"
        and restored["control"]["final_throttle"] == "0.12"
    )
    return passed, {
        "mode": restored["vehicle_state"]["mode"],
        "requested_throttle": restored["control"]["requested_throttle"],
        "final_throttle": restored["control"]["final_throttle"],
    }


def check_camera_state_sync():
    with tempfile.TemporaryDirectory() as directory:
        manager = RecordManager(
            directory,
            lambda: {},
            lambda: (b"jpeg", 7, 12.5, 1000.25),
        )
        manager.session_path = directory
        manager._open_streams()
        manager.active = True
        event_written = manager.add_event("MANUAL_OVERRIDE", "controller_deadman")
        manager._record_camera_frame(
            {
                "steering": {
                    "angle_degrees": 3.5,
                    "target_angle_degrees": 4.0,
                },
                "control": {
                    "requested_throttle": 0.30,
                    "final_throttle": 0.12,
                },
            }
        )
        manager.active = False
        manager._close_streams()
        with open(
            os.path.join(directory, "camera_timestamps.csv"),
            "r",
            encoding="utf-8",
        ) as file:
            row = next(csv.DictReader(file))
        with open(
            os.path.join(directory, "events.csv"),
            "r",
            encoding="utf-8",
        ) as file:
            event = next(csv.DictReader(file))
    vehicle_fields = set(RecordManager.STREAM_FIELDS["vehicle_state"])
    control_fields = set(RecordManager.STREAM_FIELDS["control"])
    gnss_fields = set(RecordManager.STREAM_FIELDS["gnss"])
    route_fields = set(RecordManager.STREAM_FIELDS["route"])
    arduino_fields = set(RecordManager.STREAM_FIELDS["arduino"])
    passed = (
        row["source_sequence"] == "7"
        and row["monotonic"] == "12.5"
        and row["steering_angle_degrees"] == "3.5"
        and row["target_steering_angle_degrees"] == "4.0"
        and row["requested_throttle"] == "0.3"
        and row["final_throttle"] == "0.12"
        and event_written
        and event["event"] == "MANUAL_OVERRIDE"
        and event["details"] == "controller_deadman"
        and {"system_state", "manual_override"}.issubset(vehicle_fields)
        and {"input_source", "gamepad_throttle"}.issubset(control_fields)
        and "gnss_timestamp" in gnss_fields
        and "gnss_timestamp" in route_fields
        and {"is_valid", "data_age", "error_code"}.issubset(gnss_fields)
        and {"is_valid", "data_age", "error_code"}.issubset(
            RecordManager.STREAM_FIELDS["imu"]
        )
        and {"is_valid", "data_age", "error_code"}.issubset(
            RecordManager.STREAM_FIELDS["steering"]
        )
        and {"is_valid", "data_age", "error_code"}.issubset(
            RecordManager.STREAM_FIELDS["lidar_summary"]
        )
        and {
            "last_response_at",
            "is_valid",
            "data_age",
            "error_code",
        }.issubset(arduino_fields)
        and {
            "lane_detected",
            "lane_confidence",
            "lane_lateral_error_m",
            "lane_heading_error_degrees",
            "lane_correction_angle_degrees",
            "lane_error",
        }.issubset(RecordManager.STREAM_FIELDS["perception"])
    )
    return passed, {
        "source_sequence": row["source_sequence"],
        "monotonic": row["monotonic"],
        "steering_angle_degrees": row["steering_angle_degrees"],
        "requested_throttle": row["requested_throttle"],
        "final_throttle": row["final_throttle"],
        "manual_override_event": event["event"],
        "record_state_fields_present": (
            {"system_state", "manual_override"}.issubset(vehicle_fields)
            and {"input_source", "gamepad_throttle"}.issubset(control_fields)
        ),
        "gnss_timestamp_fields_present": (
            "gnss_timestamp" in gnss_fields
            and "gnss_timestamp" in route_fields
        ),
        "sensor_validity_fields_present": all(
            {"is_valid", "data_age", "error_code"}.issubset(
                RecordManager.STREAM_FIELDS[stream]
            )
            for stream in ("gnss", "imu", "steering", "lidar_summary", "arduino")
        ),
        "lane_fields_present": all(
            field in RecordManager.STREAM_FIELDS["perception"]
            for field in (
                "lane_detected",
                "lane_confidence",
                "lane_lateral_error_m",
                "lane_heading_error_degrees",
                "lane_correction_angle_degrees",
                "lane_error",
            )
        ),
    }


def check_field_report():
    with tempfile.TemporaryDirectory() as directory:
        streams = {
            "vehicle_state": (
                ["monotonic", "mode"],
                [
                    {"monotonic": 10.0, "mode": "AUTO_ROUTE"},
                    {"monotonic": 20.0, "mode": "AUTO_ROUTE"},
                ],
            ),
            "gnss": (
                ["rtk_status", "is_valid", "data_age", "error_code"],
                [
                    {
                        "rtk_status": "RTK FIXED",
                        "is_valid": True,
                        "data_age": 0.05,
                        "error_code": "",
                    },
                    {
                        "rtk_status": "RTK FIXED",
                        "is_valid": True,
                        "data_age": 0.08,
                        "error_code": "",
                    },
                ],
            ),
            "steering": (
                ["error_degrees"],
                [{"error_degrees": 1.0}, {"error_degrees": -2.0}],
            ),
            "control": (
                [
                    "cross_track_error_m",
                    "requested_steering",
                    "target_speed_mps",
                    "stop_reason",
                ],
                [
                    {
                        "cross_track_error_m": 0.15,
                        "requested_steering": 0.0,
                        "target_speed_mps": 0.25,
                        "stop_reason": "",
                    },
                    {
                        "cross_track_error_m": -0.20,
                        "requested_steering": 0.6,
                        "target_speed_mps": 0.15,
                        "stop_reason": "OBSTACLE_STOP",
                    },
                ],
            ),
            "events": (
                ["event"],
                [{"event": "AUTO_ROUTE_COMPLETED"}],
            ),
        }
        for stream, (fields, rows) in streams.items():
            with open(
                os.path.join(directory, f"{stream}.csv"),
                "w",
                newline="",
                encoding="utf-8",
            ) as file:
                writer = csv.DictWriter(file, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
        analyzer = FieldRunReport(directory)
        report = analyzer.build()
        evaluation = analyzer.evaluate(
            report,
            required_mode="AUTO_ROUTE",
            maximum_cross_track_error_m=0.30,
            maximum_steering_error_degrees=3.0,
            minimum_rtk_fixed_ratio=1.0,
            minimum_sensor_valid_ratio=1.0,
            require_completion=True,
            required_stop_reason="OBSTACLE_STOP",
        )
    passed = (
        evaluation["passed"]
        and report["duration_seconds"] == 10.0
        and report["cross_track_error_m"]["maximum"] == 0.20
        and report["curve_slowdown"]["observed"]
    )
    return passed, {
        "evaluation": evaluation,
        "maximum_cross_track_error_m": report["cross_track_error_m"]["maximum"],
        "curve_slowdown_observed": report["curve_slowdown"]["observed"],
    }


def check_restart_delay():
    guard = RestartDelayGuard(1.5)
    guard.block("OBSTACLE_STOP")
    initial = guard.remaining(10.0)
    middle = guard.remaining(11.0)
    finished = guard.remaining(11.5)
    passed = initial == 1.5 and middle == 0.5 and finished == 0.0 and guard.reason is None
    return passed, {"initial": initial, "middle": middle, "finished": finished}


def check_route_sensor_timeouts():
    route = SimpleNamespace(
        origin={
            "origin_latitude": 35.0,
            "origin_longitude": 126.0,
            "origin_altitude": 0.0,
        },
        points=[PathPoint(0.0, 0.0), PathPoint(2.0, 0.0)],
    )
    planner = AutoRoutePlanner(route)
    gps = {
        "fix": "RTK FIXED",
        "latitude": 35.0,
        "longitude": 126.0,
        "altitude_m": 0.0,
        "received_at": 100.0,
    }
    imu = {"global_heading_degrees": 90.0, "last_update": 100.0}
    fresh = planner.update(gps, imu, now=100.05)
    stale_gnss = planner.update(gps, {**imu, "last_update": 100.31}, now=100.31)
    stale_imu = planner.update(
        {**gps, "received_at": 100.31},
        imu,
        now=100.31,
    )
    heading_mismatch = AutoRoutePlanner(route).update(
        gps,
        {**imu, "global_heading_degrees": 270.0},
        now=100.05,
    )
    deviation_latitude = 35.0 + math.degrees(
        2.0 / AutoRoutePlanner(route).converter.EARTH_RADIUS_M
    )
    route_deviation = AutoRoutePlanner(route).update(
        {**gps, "latitude": deviation_latitude},
        imu,
        now=100.05,
    )
    jump_planner = AutoRoutePlanner(route)
    jump_planner.update(gps, imu, now=100.05)
    jump_longitude = 126.0 + math.degrees(
        2.0
        / (
            jump_planner.converter.EARTH_RADIUS_M
            * math.cos(math.radians(35.0))
        )
    )
    position_jump = jump_planner.update(
        {
            **gps,
            "longitude": jump_longitude,
            "received_at": 100.10,
        },
        {**imu, "last_update": 100.10},
        now=100.10,
    )
    passed = (
        fresh.fault is None
        and stale_gnss.fault == "GNSS_TIMEOUT"
        and stale_imu.fault == "IMU_TIMEOUT"
        and heading_mismatch.fault == "ROUTE_HEADING_MISMATCH"
        and route_deviation.fault == "ROUTE_DEVIATION"
        and position_jump.fault == "GNSS_POSITION_JUMP"
    )
    return passed, {
        "fresh_fault": fresh.fault,
        "stale_gnss_fault": stale_gnss.fault,
        "stale_imu_fault": stale_imu.fault,
        "heading_fault": heading_mismatch.fault,
        "deviation_fault": route_deviation.fault,
        "position_jump_fault": position_jump.fault,
    }


def check_route_preflight():
    route = SimpleNamespace(
        origin={
            "origin_latitude": 35.0,
            "origin_longitude": 126.0,
            "origin_altitude": 0.0,
        },
        points=[PathPoint(0.0, 0.0), PathPoint(2.0, 0.0)],
    )
    planner = AutoRoutePlanner(route)
    gps = {
        "fix": "RTK FIXED",
        "latitude": 35.0,
        "longitude": 126.0,
        "altitude_m": 0.0,
        "received_at": 100.0,
    }
    imu = {"global_heading_degrees": 90.0, "last_update": 100.0}
    ready = planner.preflight(
        gps,
        imu,
        True,
        True,
        True,
        False,
        now=100.05,
    )
    emergency = planner.preflight(
        gps,
        imu,
        True,
        True,
        True,
        True,
        now=100.05,
    )
    invalid = planner.preflight(
        {
            **gps,
            "fix": "RTK FLOAT",
            "latitude": 35.00002,
            "received_at": 99.0,
        },
        {"global_heading_degrees": 270.0, "last_update": 99.0},
        False,
        False,
        False,
        False,
        now=100.05,
    )
    expected_invalid_errors = {
        "RTK_FIX_REQUIRED",
        "GNSS_TIMEOUT",
        "IMU_TIMEOUT",
        "LIDAR_UNAVAILABLE",
        "ARDUINO_UNAVAILABLE",
        "STEERING_UNAVAILABLE",
        "TOO_FAR_FROM_ROUTE_START",
        "START_HEADING_MISMATCH",
    }
    passed = (
        ready.ready
        and emergency.errors == ["EMERGENCY_STOP_ACTIVE"]
        and set(invalid.errors) == expected_invalid_errors
    )
    return passed, {
        "ready_errors": ready.errors,
        "emergency_errors": emergency.errors,
        "invalid_errors": invalid.errors,
    }


def check_state_machine():
    machine = VehicleStateMachine()
    repeated = machine.transition(DriveMode.DISARMED, "repeat")
    manual = machine.transition(DriveMode.MANUAL_ASSIST, "validation")
    emergency = machine.transition(DriveMode.EMERGENCY_STOP, "validation")
    invalid_blocked = False
    try:
        machine.transition(DriveMode.AUTO_ROUTE, "invalid")
    except InvalidStateTransition:
        invalid_blocked = True
    reset = machine.transition(DriveMode.DISARMED, "reset")
    passed = (
        repeated["mode"] == "DISARMED"
        and manual["mode"] == "MANUAL_ASSIST"
        and emergency["mode"] == "EMERGENCY_STOP"
        and invalid_blocked
        and reset["mode"] == "DISARMED"
    )
    return passed, {"repeated": repeated["mode"], "invalid_blocked": invalid_blocked}


def main():
    checks = {}
    for name, function in (
        ("safety", check_safety),
        ("routes", check_routes),
        ("lane", check_lane),
        ("hybrid_fallback", check_hybrid_fallback),
        ("lane_continuity", check_lane_continuity),
        ("camera_calibration", check_camera_calibration),
        ("object_fusion", check_object_fusion),
        ("throttle", check_throttle),
        ("compressed_replay", check_compressed_replay),
        ("state_replay", check_state_replay),
        ("camera_state_sync", check_camera_state_sync),
        ("field_report", check_field_report),
        ("restart_delay", check_restart_delay),
        ("route_sensor_timeouts", check_route_sensor_timeouts),
        ("route_preflight", check_route_preflight),
        ("steering_tracking", check_steering_tracking),
        ("state_machine", check_state_machine),
    ):
        passed, details = function()
        checks[name] = {"passed": bool(passed), "details": details}
    document = {"passed": all(item["passed"] for item in checks.values()), "checks": checks}
    print(json.dumps(document, ensure_ascii=False, indent=2))
    raise SystemExit(0 if document["passed"] else 1)


if __name__ == "__main__":
    main()
