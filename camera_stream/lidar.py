import os
import subprocess
import threading
import time

try:
    import termios
except ImportError:
    termios = None

from .config import (
    LIDAR_CAMERA_FOV_DEGREES,
    LIDAR_CAMERA_YAW_DEGREES,
    LIDAR_DEVICE,
    LIDAR_MAX_OVERLAY_DISTANCE_MM,
    LIDAR_PWM_GPIO,
)


class LidarMonitor:
    FRAME_SIZE = 47
    HEADER = b"\x54\x2c"
    DATA_TIMEOUT_SECONDS = 2.0
    SAFETY_MIN_CONFIDENCE = 35
    SAFETY_ANGLE_TOLERANCE_DEGREES = 2.2

    def __init__(self):
        self.lock = threading.Lock()
        self.points = []
        self.safety_points = []
        self.previous_scan_points = []
        self.pending_points = []
        self.previous_point_angle = None
        self.scan_point_count = 0
        self.rotation_hz = None
        self.last_update = None
        self.error = None

    def start(self):
        threading.Thread(target=self._read_loop, daemon=True).start()

    @staticmethod
    def _crc8(data):
        checksum = 0
        for value in data:
            checksum ^= value
            for _ in range(8):
                checksum = (
                    ((checksum << 1) ^ 0x4D) & 0xFF
                    if checksum & 0x80
                    else (checksum << 1) & 0xFF
                )
        return checksum

    @staticmethod
    def _bearing(angle_degrees):
        bearing = (
            angle_degrees - LIDAR_CAMERA_YAW_DEGREES + 180.0
        ) % 360.0 - 180.0
        return round(bearing, 2)

    def snapshot(self):
        now = time.time()
        with self.lock:
            connected = self.last_update is not None and now - self.last_update < 1.0
            valid_points = [
                {
                    "bearing_degrees": self._bearing(angle),
                    "distance_mm": distance,
                    "confidence": confidence,
                }
                for angle, distance, confidence in self.points
                if distance > 0
            ]
            return {
                "connected": connected,
                "device": LIDAR_DEVICE,
                "rotation_hz": self.rotation_hz,
                "point_count": len(valid_points),
                "safety_point_count": len(self.safety_points),
                "scan_point_count": self.scan_point_count,
                "points": valid_points,
                "safety_points": [
                    {
                        "bearing_degrees": self._bearing(angle),
                        "distance_mm": distance,
                        "confidence": confidence,
                    }
                    for angle, distance, confidence in self.safety_points
                    if distance > 0
                ],
                "last_update": self.last_update,
                "error": None if connected else self.error,
                "camera_yaw_degrees": LIDAR_CAMERA_YAW_DEGREES,
                "camera_fov_degrees": LIDAR_CAMERA_FOV_DEGREES,
                "max_overlay_distance_mm": LIDAR_MAX_OVERLAY_DISTANCE_MM,
            }

    @classmethod
    def _points_match(cls, left, right, angle_tolerance=None):
        angle_tolerance = (
            cls.SAFETY_ANGLE_TOLERANCE_DEGREES
            if angle_tolerance is None
            else float(angle_tolerance)
        )
        distance_tolerance = max(
            30.0,
            min(160.0, min(left[1], right[1]) * 0.08),
        )
        angle_gap = abs(left[0] - right[0])
        angle_gap = min(angle_gap, 360.0 - angle_gap)
        return (
            angle_gap <= angle_tolerance
            and abs(left[1] - right[1]) <= distance_tolerance
        )

    @classmethod
    def _stable_safety_points(cls, current_points, previous_points):
        candidates = [
            point
            for point in current_points
            if point[1] > 0 and point[2] >= cls.SAFETY_MIN_CONFIDENCE
        ]
        previous = [
            point
            for point in previous_points
            if point[1] > 0 and point[2] >= cls.SAFETY_MIN_CONFIDENCE
        ]
        if not previous:
            return []

        previous_by_degree = {}
        for point in previous:
            degree = int(point[0]) % 360
            previous_by_degree.setdefault(degree, []).append(point)

        stable = []
        for index, point in enumerate(candidates):
            temporal_match = any(
                cls._points_match(point, old_point, 2.6)
                for offset in range(-3, 4)
                for old_point in previous_by_degree.get(
                    (int(point[0]) + offset) % 360,
                    (),
                )
            )
            if not temporal_match:
                continue
            adjacent = (
                (index > 0 and cls._points_match(point, candidates[index - 1]))
                or (
                    index + 1 < len(candidates)
                    and cls._points_match(point, candidates[index + 1])
                )
            )
            if adjacent:
                stable.append(point)
        return stable

    def _configure_port(self, file_descriptor):
        attributes = termios.tcgetattr(file_descriptor)
        attributes[0] = 0
        attributes[1] = 0
        attributes[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attributes[3] = 0
        attributes[4] = termios.B230400
        attributes[5] = termios.B230400
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 1
        termios.tcsetattr(file_descriptor, termios.TCSANOW, attributes)
        termios.tcflush(file_descriptor, termios.TCIFLUSH)
        result = subprocess.run(
            [
                "/usr/bin/pinctrl",
                "set",
                str(LIDAR_PWM_GPIO),
                "ip",
                "pn",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=2,
        )
        if result.returncode != 0:
            raise OSError(result.stderr.strip() or "failed to release lidar PWM pin")

    def _store_frame(self, frame):
        speed_degrees_per_second = int.from_bytes(frame[2:4], "little")
        start_angle = int.from_bytes(frame[4:6], "little") / 100.0
        end_angle = int.from_bytes(frame[42:44], "little") / 100.0
        angle_span = (end_angle - start_angle) % 360.0
        now = time.time()
        frame_points = []
        for index in range(12):
            offset = 6 + index * 3
            distance = int.from_bytes(frame[offset : offset + 2], "little")
            confidence = frame[offset + 2]
            angle = (start_angle + angle_span * index / 11.0) % 360.0
            frame_points.append((round(angle, 2), distance, confidence))

        with self.lock:
            for point in frame_points:
                angle = point[0]
                wrapped = (
                    self.previous_point_angle is not None
                    and angle < 20.0
                    and self.previous_point_angle > 340.0
                )
                if wrapped and self.pending_points:
                    self.points = self.pending_points
                    self.scan_point_count = len(self.pending_points)
                    self.safety_points = self._stable_safety_points(
                        self.pending_points,
                        self.previous_scan_points,
                    )
                    self.previous_scan_points = self.pending_points
                    self.pending_points = []
                self.pending_points.append(point)
                self.previous_point_angle = angle
            self.rotation_hz = speed_degrees_per_second / 360.0
            self.last_update = now
            self.error = None

    def _read_loop(self):
        if termios is None:
            with self.lock:
                self.error = "termios unavailable"
            return

        while True:
            file_descriptor = None
            try:
                file_descriptor = os.open(
                    LIDAR_DEVICE,
                    os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK,
                )
                self._configure_port(file_descriptor)
                buffer = bytearray()
                last_valid_frame = time.monotonic()
                while True:
                    try:
                        chunk = os.read(file_descriptor, 4096)
                    except BlockingIOError:
                        chunk = b""
                    if chunk:
                        buffer.extend(chunk)
                    else:
                        time.sleep(0.01)

                    while len(buffer) >= self.FRAME_SIZE:
                        header_index = buffer.find(self.HEADER)
                        if header_index < 0:
                            del buffer[:-1]
                            break
                        if header_index:
                            del buffer[:header_index]
                        if len(buffer) < self.FRAME_SIZE:
                            break
                        frame = bytes(buffer[: self.FRAME_SIZE])
                        if self._crc8(frame[:46]) == frame[46]:
                            self._store_frame(frame)
                            last_valid_frame = time.monotonic()
                            del buffer[: self.FRAME_SIZE]
                        else:
                            del buffer[0]
                    if time.monotonic() - last_valid_frame > self.DATA_TIMEOUT_SECONDS:
                        raise TimeoutError("lidar data timeout; reconnecting")
            except (OSError, ValueError, termios.error) as error:
                with self.lock:
                    self.error = str(error)
                    self.last_update = None
                    self.points = []
                    self.safety_points = []
                    self.previous_scan_points = []
                    self.pending_points = []
                    self.previous_point_angle = None
                time.sleep(2)
            finally:
                if file_descriptor is not None:
                    os.close(file_descriptor)
