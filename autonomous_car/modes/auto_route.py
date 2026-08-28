from dataclasses import dataclass
import math
import time

from autonomous_car.control import PurePursuit
from autonomous_car.localization import LocalENUConverter


@dataclass(frozen=True)
class PreflightResult:
    ready: bool
    errors: list[str]
    start_distance_m: float | None = None
    heading_error_degrees: float | None = None


@dataclass(frozen=True)
class AutoRouteCommand:
    steering_angle_degrees: float
    throttle: float
    cross_track_error_m: float
    nearest_index: int
    target_index: int
    finished: bool = False
    fault: str | None = None


class AutoRoutePlanner:
    def __init__(
        self,
        route,
        wheelbase_m=0.53,
        lookahead_m=0.8,
        maximum_steering_degrees=20.0,
        base_throttle=0.25,
        maximum_cross_track_error_m=1.0,
        maximum_heading_error_degrees=60.0,
        maximum_position_jump_m=1.5,
    ):
        self.route = route
        self.converter = LocalENUConverter(
            route.origin["origin_latitude"],
            route.origin["origin_longitude"],
            route.origin.get("origin_altitude", 0.0),
        )
        self.pursuit = PurePursuit(wheelbase_m, lookahead_m, maximum_steering_degrees)
        self.base_throttle = float(base_throttle)
        self.maximum_cross_track_error_m = float(maximum_cross_track_error_m)
        self.maximum_heading_error_degrees = abs(float(maximum_heading_error_degrees))
        self.maximum_position_jump_m = abs(float(maximum_position_jump_m))
        self.previous_index = 0
        self.previous_position = None

    @staticmethod
    def compass_to_enu_heading(compass_degrees):
        return math.radians((90.0 - float(compass_degrees)) % 360.0)

    def preflight(
        self,
        gps,
        imu,
        lidar_connected,
        arduino_connected,
        steering_connected,
        emergency_stop_active=False,
        now=None,
    ):
        current_time = time.time() if now is None else now
        errors = []
        if gps.get("fix") != "RTK FIXED":
            errors.append("RTK_FIX_REQUIRED")
        received_at = gps.get("received_at")
        if received_at is None or current_time - received_at > 0.3:
            errors.append("GNSS_TIMEOUT")
        if imu.get("last_update") is None or current_time - imu["last_update"] > 0.1:
            errors.append("IMU_TIMEOUT")
        if not lidar_connected:
            errors.append("LIDAR_UNAVAILABLE")
        if not arduino_connected:
            errors.append("ARDUINO_UNAVAILABLE")
        if not steering_connected:
            errors.append("STEERING_UNAVAILABLE")
        if emergency_stop_active:
            errors.append("EMERGENCY_STOP_ACTIVE")
        start_distance = None
        heading_error = None
        if gps.get("latitude") is not None and gps.get("longitude") is not None:
            x, y, _ = self.converter.to_enu(gps["latitude"], gps["longitude"], gps.get("altitude_m"))
            start = self.route.points[0]
            start_distance = math.hypot(start.x - x, start.y - y)
            if start_distance > 1.0:
                errors.append("TOO_FAR_FROM_ROUTE_START")
            if len(self.route.points) > 1 and imu.get("global_heading_degrees") is not None:
                path_heading = math.atan2(
                    self.route.points[1].y - start.y,
                    self.route.points[1].x - start.x,
                )
                vehicle_heading = self.compass_to_enu_heading(imu["global_heading_degrees"])
                heading_error = abs(math.degrees((path_heading - vehicle_heading + math.pi) % (2 * math.pi) - math.pi))
                if heading_error > 30.0:
                    errors.append("START_HEADING_MISMATCH")
        else:
            errors.append("GNSS_POSITION_UNAVAILABLE")
        return PreflightResult(not errors, errors, start_distance, heading_error)

    def update(self, gps, imu, now=None):
        current_time = time.time() if now is None else float(now)
        if gps.get("fix") != "RTK FIXED":
            return self._fault("RTK_FIX_LOST")
        if gps.get("received_at") is None or current_time - gps["received_at"] > 0.3:
            return self._fault("GNSS_TIMEOUT")
        if imu.get("last_update") is None or current_time - imu["last_update"] > 0.1:
            return self._fault("IMU_TIMEOUT")
        if gps.get("latitude") is None or gps.get("longitude") is None:
            return self._fault("GNSS_POSITION_UNAVAILABLE")
        heading = imu.get("global_heading_degrees")
        if heading is None:
            return self._fault("IMU_HEADING_UNAVAILABLE")
        x, y, _ = self.converter.to_enu(gps["latitude"], gps["longitude"], gps.get("altitude_m"))
        if self.previous_position is not None:
            position_jump = math.hypot(
                x - self.previous_position[0],
                y - self.previous_position[1],
            )
            if position_jump > self.maximum_position_jump_m:
                self.previous_position = (x, y)
                return self._fault("GNSS_POSITION_JUMP")
        self.previous_position = (x, y)
        vehicle_heading = self.compass_to_enu_heading(heading)
        pursuit = self.pursuit.calculate(
            x,
            y,
            vehicle_heading,
            self.route.points,
            self.previous_index,
        )
        self.previous_index = pursuit.nearest_index
        heading_start_index = min(pursuit.nearest_index, len(self.route.points) - 2)
        heading_end_index = heading_start_index + 1
        path_heading = math.atan2(
            self.route.points[heading_end_index].y - self.route.points[heading_start_index].y,
            self.route.points[heading_end_index].x - self.route.points[heading_start_index].x,
        )
        heading_error = abs(
            math.degrees(
                (path_heading - vehicle_heading + math.pi) % (2.0 * math.pi) - math.pi
            )
        )
        if heading_error > self.maximum_heading_error_degrees:
            return self._fault("ROUTE_HEADING_MISMATCH")
        if pursuit.cross_track_error_m > self.maximum_cross_track_error_m:
            return AutoRouteCommand(
                0.0,
                0.0,
                pursuit.cross_track_error_m,
                pursuit.nearest_index,
                pursuit.target_index,
                fault="ROUTE_DEVIATION",
            )
        steering_ratio = min(1.0, abs(pursuit.steering_angle_degrees) / self.pursuit.maximum_steering_degrees)
        throttle = self.base_throttle * (1.0 - 0.6 * steering_ratio)
        if pursuit.finished:
            throttle = 0.0
        return AutoRouteCommand(
            pursuit.steering_angle_degrees,
            throttle,
            pursuit.cross_track_error_m,
            pursuit.nearest_index,
            pursuit.target_index,
            finished=pursuit.finished,
        )

    def _fault(self, reason):
        return AutoRouteCommand(0.0, 0.0, 0.0, self.previous_index, self.previous_index, fault=reason)
