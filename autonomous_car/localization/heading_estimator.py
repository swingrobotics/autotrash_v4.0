import math
import time


class HeadingEstimator:
    def __init__(self, imu_correction_gain=0.12, gnss_correction_gain=0.05, gnss_minimum_speed_mps=0.3):
        self.imu_correction_gain = float(imu_correction_gain)
        self.gnss_correction_gain = float(gnss_correction_gain)
        self.gnss_minimum_speed_mps = float(gnss_minimum_speed_mps)
        self.heading_degrees = None
        self.last_timestamp = None

    @staticmethod
    def _difference(target, current):
        return (float(target) - float(current) + 180.0) % 360.0 - 180.0

    def reset(self, heading_degrees=None, timestamp=None):
        self.heading_degrees = None if heading_degrees is None else float(heading_degrees) % 360.0
        self.last_timestamp = time.monotonic() if timestamp is None else float(timestamp)

    def update(
        self,
        imu_heading_degrees,
        yaw_rate_dps,
        gnss_course_degrees=None,
        speed_mps=0.0,
        timestamp=None,
    ):
        now = time.monotonic() if timestamp is None else float(timestamp)
        if imu_heading_degrees is None:
            return self.heading_degrees
        if self.heading_degrees is None:
            self.heading_degrees = float(imu_heading_degrees) % 360.0
            self.last_timestamp = now
            return self.heading_degrees
        elapsed = max(0.0, min(0.2, now - (self.last_timestamp or now)))
        self.last_timestamp = now
        if yaw_rate_dps is not None:
            self.heading_degrees = (self.heading_degrees + float(yaw_rate_dps) * elapsed) % 360.0
        self.heading_degrees = (
            self.heading_degrees
            + self.imu_correction_gain * self._difference(imu_heading_degrees, self.heading_degrees)
        ) % 360.0
        if (
            gnss_course_degrees is not None
            and speed_mps is not None
            and float(speed_mps) >= self.gnss_minimum_speed_mps
        ):
            self.heading_degrees = (
                self.heading_degrees
                + self.gnss_correction_gain * self._difference(gnss_course_degrees, self.heading_degrees)
            ) % 360.0
        return self.heading_degrees
