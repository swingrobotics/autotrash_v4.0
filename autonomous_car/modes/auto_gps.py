from dataclasses import dataclass
import math

from autonomous_car.control import PathPoint, PurePursuit
from autonomous_car.safety import LocalAvoidancePlanner

from .auto_route import AutoRoutePlanner


@dataclass(frozen=True)
class AutoGpsCommand:
    steering_angle_degrees: float
    throttle: float
    cross_track_error_m: float
    nearest_index: int
    target_index: int
    finished: bool = False
    fault: str | None = None
    avoidance_active: bool = False
    avoidance_side: str | None = None
    avoidance_reason: str | None = None
    avoidance_target_index: int | None = None


class AutoGpsPlanner:
    """AUTO_GPS route follower with conservative temporary local avoidance.

    The GPS route remains the global path. When the LiDAR sector planner finds
    a safe side, this class latches a short ENU-space bypass path and follows it
    at low speed before rejoining a point ahead on the original route.

    A lower SafetySupervisor collision stop is still required. This planner is
    intentionally conservative and must be closed-area validated before moving
    field deployment.
    """

    def __init__(
        self,
        route,
        lidar_provider=None,
        route_planner=None,
        avoidance_planner=None,
        wheelbase_m=0.53,
        lookahead_m=0.65,
        maximum_steering_degrees=20.0,
        lateral_offset_m=0.60,
        entry_forward_m=0.65,
        pass_forward_m=1.45,
        rejoin_route_distance_m=2.60,
        avoidance_speed_scale=0.35,
    ):
        self.route = route
        self.route_planner = route_planner or AutoRoutePlanner(route)
        self.converter = self.route_planner.converter
        self.lidar_provider = lidar_provider
        self.avoidance_planner = avoidance_planner or LocalAvoidancePlanner(
            avoidance_speed_scale=avoidance_speed_scale,
        )
        self.local_pursuit = PurePursuit(
            wheelbase_m=wheelbase_m,
            lookahead_m=lookahead_m,
            maximum_steering_degrees=maximum_steering_degrees,
        )
        self.lateral_offset_m = abs(float(lateral_offset_m))
        self.entry_forward_m = max(0.2, float(entry_forward_m))
        self.pass_forward_m = max(self.entry_forward_m + 0.2, float(pass_forward_m))
        self.rejoin_route_distance_m = max(1.0, float(rejoin_route_distance_m))
        self.avoidance_speed_scale = max(0.05, min(1.0, float(avoidance_speed_scale)))
        self._temporary_path = None
        self._temporary_previous_index = 0
        self._avoidance_side = None
        self._avoidance_reason = None
        self._rejoin_index = None

    @property
    def avoidance_active(self):
        return self._temporary_path is not None

    def reset_avoidance(self):
        self._temporary_path = None
        self._temporary_previous_index = 0
        self._avoidance_side = None
        self._avoidance_reason = None
        self._rejoin_index = None

    def preflight(self, *args, **kwargs):
        return self.route_planner.preflight(*args, **kwargs)

    def update(self, gps, imu, lidar_points=None, now=None):
        base = self.route_planner.update(gps, imu, now=now)
        if base.fault or base.finished:
            self.reset_avoidance()
            return self._from_base(base)

        x, y, heading = self._pose(gps, imu)
        points = lidar_points
        if points is None and self.lidar_provider is not None:
            points = self.lidar_provider()
        decision = self.avoidance_planner.plan(points or [])

        if decision.stop_required:
            return AutoGpsCommand(
                steering_angle_degrees=0.0,
                throttle=0.0,
                cross_track_error_m=base.cross_track_error_m,
                nearest_index=base.nearest_index,
                target_index=base.target_index,
                fault=None,
                avoidance_active=True,
                avoidance_side=decision.preferred_side or self._avoidance_side,
                avoidance_reason=decision.reason,
                avoidance_target_index=self._rejoin_index,
            )

        if self._temporary_path is None and decision.active and decision.preferred_side:
            self._temporary_path, self._rejoin_index = self._build_temporary_path(
                x,
                y,
                heading,
                base.nearest_index,
                decision.preferred_side,
            )
            self._temporary_previous_index = 0
            self._avoidance_side = decision.preferred_side
            self._avoidance_reason = decision.reason

        if self._temporary_path is None:
            return self._from_base(base)

        local = self.local_pursuit.calculate(
            x,
            y,
            heading,
            self._temporary_path,
            self._temporary_previous_index,
        )
        self._temporary_previous_index = local.nearest_index

        if local.finished:
            side = self._avoidance_side
            rejoin = self._rejoin_index
            self.reset_avoidance()
            return AutoGpsCommand(
                steering_angle_degrees=base.steering_angle_degrees,
                throttle=base.throttle,
                cross_track_error_m=base.cross_track_error_m,
                nearest_index=base.nearest_index,
                target_index=base.target_index,
                finished=False,
                fault=None,
                avoidance_active=False,
                avoidance_side=side,
                avoidance_reason="REJOINED_GLOBAL_ROUTE",
                avoidance_target_index=rejoin,
            )

        throttle = min(
            max(0.0, base.throttle),
            self.route_planner.base_throttle * self.avoidance_speed_scale,
        )
        return AutoGpsCommand(
            steering_angle_degrees=local.steering_angle_degrees,
            throttle=throttle,
            cross_track_error_m=base.cross_track_error_m,
            nearest_index=base.nearest_index,
            target_index=base.target_index,
            finished=False,
            fault=None,
            avoidance_active=True,
            avoidance_side=self._avoidance_side,
            avoidance_reason=self._avoidance_reason or "AVOID",
            avoidance_target_index=self._rejoin_index,
        )

    def snapshot(self):
        return {
            "active": self.avoidance_active,
            "side": self._avoidance_side,
            "reason": self._avoidance_reason,
            "rejoin_index": self._rejoin_index,
            "temporary_path": [
                {"x": point.x, "y": point.y}
                for point in (self._temporary_path or [])
            ],
        }

    def _pose(self, gps, imu):
        x, y, _ = self.converter.to_enu(
            gps["latitude"],
            gps["longitude"],
            gps.get("altitude_m"),
        )
        heading = self.route_planner.compass_to_enu_heading(
            imu["global_heading_degrees"]
        )
        return x, y, heading

    def _build_temporary_path(self, x, y, heading, nearest_index, side):
        side_sign = 1.0 if side == "left" else -1.0
        forward_x = math.cos(heading)
        forward_y = math.sin(heading)
        left_x = -math.sin(heading)
        left_y = math.cos(heading)

        entry = PathPoint(
            x + forward_x * self.entry_forward_m + left_x * side_sign * self.lateral_offset_m * 0.55,
            y + forward_y * self.entry_forward_m + left_y * side_sign * self.lateral_offset_m * 0.55,
        )
        bypass = PathPoint(
            x + forward_x * self.pass_forward_m + left_x * side_sign * self.lateral_offset_m,
            y + forward_y * self.pass_forward_m + left_y * side_sign * self.lateral_offset_m,
        )
        rejoin_index = self._find_rejoin_index(nearest_index)
        rejoin = self.route.points[rejoin_index]
        return [entry, bypass, PathPoint(rejoin.x, rejoin.y)], rejoin_index

    def _find_rejoin_index(self, start_index):
        points = self.route.points
        start = max(0, min(int(start_index), len(points) - 1))
        distance = 0.0
        for index in range(start + 1, len(points)):
            distance += math.hypot(
                points[index].x - points[index - 1].x,
                points[index].y - points[index - 1].y,
            )
            if distance >= self.rejoin_route_distance_m:
                return index
        return len(points) - 1

    @staticmethod
    def _from_base(base):
        return AutoGpsCommand(
            steering_angle_degrees=base.steering_angle_degrees,
            throttle=base.throttle,
            cross_track_error_m=base.cross_track_error_m,
            nearest_index=base.nearest_index,
            target_index=base.target_index,
            finished=base.finished,
            fault=base.fault,
        )
