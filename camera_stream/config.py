import glob
import os


def _default_gps_device():
    """Prefer one clearly identified persistent USB GNSS symlink.

    /dev/ttyACM* numbering depends on USB enumeration order. udev creates
    /dev/serial/by-id links for USB serial devices with a stable ID. Only use an
    automatically discovered link when exactly one candidate has an explicit
    GNSS/GPS vendor/product hint; otherwise preserve the historical fallback and
    require GPS_DEVICE to be configured explicitly.
    """

    tokens = (
        "gnss",
        "gps",
        "u-blox",
        "ublox",
        "zed-f9",
        "zed_f9",
        "septentrio",
        "unicore",
        "allystar",
    )
    matches = []
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        name = os.path.basename(path).lower()
        if any(token in name for token in tokens):
            matches.append(path)
    if len(matches) == 1:
        return matches[0]
    return "/dev/ttyACM1"


HOST = os.environ.get("CAMERA_HOST", "0.0.0.0")
PORT = int(os.environ.get("CAMERA_PORT", "8080"))
RECORDINGS_PATH = os.environ.get(
    "RECORDINGS_PATH",
    "/home/gnss/camera-stream/recordings",
)
RECORD_CAMERA_FPS = float(os.environ.get("RECORD_CAMERA_FPS", "10"))
THROTTLE_CALIBRATION_PATH = os.environ.get(
    "THROTTLE_CALIBRATION_PATH",
    "/home/gnss/camera-stream/throttle-calibration.json",
)

CAMERA_DEVICE = os.environ.get("CAMERA_DEVICE", "/dev/video0")
CAMERA_SIZE = os.environ.get("CAMERA_SIZE", "1280x720")
CAMERA_FRAMERATE = os.environ.get("CAMERA_FRAMERATE", "30")
CAMERA_CALIBRATION_PATH = os.environ.get(
    "CAMERA_CALIBRATION_PATH",
    "/home/gnss/camera-stream/camera-calibration.json",
)

IMU_CALIBRATION_PATH = os.environ.get(
    "IMU_CALIBRATION_PATH",
    "/home/gnss/camera-stream/imu-calibration.json",
)
IMU_HEADING_DEADBAND_DEGREES = float(
    os.environ.get("IMU_HEADING_DEADBAND_DEGREES", "0.8")
)
IMU_ATTITUDE_DEADBAND_DEGREES = float(
    os.environ.get("IMU_ATTITUDE_DEADBAND_DEGREES", "0.15")
)
IMU_MOUNTING_YAW_OFFSET_DEGREES = float(
    os.environ.get("IMU_MOUNTING_YAW_OFFSET_DEGREES", "0.0")
)
IMU_TURN_RATE_THRESHOLD_DPS = float(
    os.environ.get("IMU_TURN_RATE_THRESHOLD_DPS", "3.0")
)
IMU_YAW_DIRECTION_SIGN = (
    -1.0 if os.environ.get("IMU_YAW_DIRECTION_SIGN") == "-1" else 1.0
)

ARDUINO_DEVICE = os.environ.get("ARDUINO_DEVICE")
MOTOR_BAUD = int(os.environ.get("MOTOR_BAUD", "115200"))
MOTOR_TIMEOUT_SECONDS = float(os.environ.get("MOTOR_TIMEOUT_SECONDS", "0.30"))
MOTOR_MIN_PWM = int(os.environ.get("MOTOR_MIN_PWM", "80"))
MOTOR_START_BOOST_SECONDS = float(
    os.environ.get("MOTOR_START_BOOST_SECONDS", "0.25")
)
MANUAL_MAX_THROTTLE = float(os.environ.get("MANUAL_MAX_THROTTLE", "0.35"))
AUTO_MAX_THROTTLE = float(os.environ.get("AUTO_MAX_THROTTLE", "0.35"))
AUTO_OBSTACLE_RESTART_DELAY_SECONDS = float(
    os.environ.get("AUTO_OBSTACLE_RESTART_DELAY_SECONDS", "1.5")
)
AUTO_STEERING_MAX_ERROR_DEGREES = float(
    os.environ.get("AUTO_STEERING_MAX_ERROR_DEGREES", "7.0")
)
AUTO_STEERING_ERROR_TIMEOUT_SECONDS = float(
    os.environ.get("AUTO_STEERING_ERROR_TIMEOUT_SECONDS", "1.0")
)
AUTO_LANE_MIN_CONFIDENCE = float(
    os.environ.get("AUTO_LANE_MIN_CONFIDENCE", "0.55")
)
LIDAR_SAFETY_HALF_WIDTH_M = float(
    os.environ.get("LIDAR_SAFETY_HALF_WIDTH_M", "0.45")
)
LIDAR_TO_FRONT_BUMPER_M = float(
    os.environ.get("LIDAR_TO_FRONT_BUMPER_M", "0.254")
)
LIDAR_STOP_DISTANCE_M = float(os.environ.get("LIDAR_STOP_DISTANCE_M", "0.60"))
LIDAR_CRAWL_DISTANCE_M = float(os.environ.get("LIDAR_CRAWL_DISTANCE_M", "0.80"))
LIDAR_SLOW_DISTANCE_M = float(os.environ.get("LIDAR_SLOW_DISTANCE_M", "1.50"))
DRIVE_DIRECTION_SIGN = (
    -1 if os.environ.get("DRIVE_DIRECTION_SIGN", "-1") == "-1" else 1
)
STEER_MANUAL_PWM = max(
    35, min(255, int(os.environ.get("STEER_MANUAL_PWM", "255")))
)
STEER_MIN_PWM = max(
    35, min(STEER_MANUAL_PWM, int(os.environ.get("STEER_MIN_PWM", "70")))
)
STEER_CONTROL_KP = float(os.environ.get("STEER_CONTROL_KP", "4.0"))
STEER_TARGET_TOLERANCE_DEGREES = float(
    os.environ.get("STEER_TARGET_TOLERANCE_DEGREES", "1.0")
)
# 360 deg/s is intentionally above the rover's physical steering range, so the
# software target reaches the requested angle in a single control update. The
# actual steering speed remains bounded by motor PWM, closed-loop feedback and
# the configured mechanical steering limits.
STEER_TARGET_RATE_DEGREES_PER_SECOND = float(
    os.environ.get("STEER_TARGET_RATE_DEGREES_PER_SECOND", "360.0")
)
STEER_CENTER_TIMEOUT_SECONDS = float(
    os.environ.get("STEER_CENTER_TIMEOUT_SECONDS", "4.0")
)
STEER_LEFT_REFERENCE_RAW = int(os.environ.get("STEER_LEFT_REFERENCE_RAW", "3980"))
STEER_RIGHT_REFERENCE_RAW = int(os.environ.get("STEER_RIGHT_REFERENCE_RAW", "3503"))
STEER_LIMIT_ALLOWANCE_RAW = int(os.environ.get("STEER_LIMIT_ALLOWANCE_RAW", "50"))
STEER_CENTER_RAW = int(os.environ.get("STEER_CENTER_RAW", "3700"))

LIDAR_DEVICE = os.environ.get("LIDAR_DEVICE", "/dev/serial0")
LIDAR_PWM_GPIO = int(os.environ.get("LIDAR_PWM_GPIO", "18"))
LIDAR_CAMERA_YAW_DEGREES = float(
    os.environ.get("LIDAR_CAMERA_YAW_DEGREES", "0.0")
)
LIDAR_CAMERA_FOV_DEGREES = float(
    os.environ.get("LIDAR_CAMERA_FOV_DEGREES", "82.1")
)
LIDAR_MAX_OVERLAY_DISTANCE_MM = int(
    os.environ.get("LIDAR_MAX_OVERLAY_DISTANCE_MM", "4000")
)

GPS_DEVICE = os.environ.get("GPS_DEVICE") or _default_gps_device()
NTRIP_CONFIG_PATH = os.environ.get(
    "NTRIP_CONFIG_PATH",
    "/home/gnss/camera-stream/ntrip-config.json",
)
GPSD_CONTROL_SOCKET = os.environ.get(
    "GPSD_CONTROL_SOCKET",
    "/run/gpsd-control.sock",
)
