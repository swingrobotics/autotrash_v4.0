from __future__ import annotations

from dataclasses import dataclass
import math

from autonomous_car.control import PathPoint, PurePursuit
from autonomous_car.localization.grid_path_planner import GridPathPlanner
from autonomous_car.safety import LocalAvoidancePlanner


@dataclass(frozen=True)
class AutoLocalCommand:
    steering_angle_degrees: float
    throttle: float
    cross_track_error_m: float
    target_index: int
    finished: bool = False
    fault: str | None = None
    avoidance_active: bool = False
    avoidance_side: str | None = None
    avoidance_reason: str | None = None
    replanned: bool = False

    def as_dict(self):
        return self.__dict__.copy()


class AutoLocalPlanner:
    """Saved-map AUTO_LOCAL path following with temporary obstacle bypass."""

    def __init__(
        self,
        grid,
        destination,
        *,
        wheelbase_m=0.53,
        lookahead_m=0.65,
        maximum_steering_degrees=20.0,
        base_throttle=0.20,
        avoidance_speed_scale=0.35,
        maximum_cross_track_error_m=0.80,
    ):
        self.grid = grid
        self.destination = dict(destination)
        self.path_planner = GridPathPlanner(grid)
        self.pursuit = PurePursuit(wheelbase_m, lookahead_m, maximum_steering_degrees)
        self.bypass_pursuit = PurePursuit(wheelbase_m, 0.55, maximum_steering_degrees)
        self.avoidance = LocalAvoidancePlanner(avoidance_speed_scale=avoidance_speed_scale)
        self.base_throttle = float(base_throttle)
        self.avoidance_speed_scale = max(0.05, min(1.0, float(avoidance_speed_scale)))
        self.maximum_cross_track_error_m = abs(float(maximum_cross_track_error_m))
        self.path = []
        self.path_result = None
        self.previous_index = 0
        self.temporary_path = None
        self.temporary_index = 0
        self.avoidance_side = None
        self.avoidance_reason = None
        self.replan_count = 0

    def plan_from_pose(self, pose):
        self.path_result = self.path_planner.plan(
            pose.x,
            pose.y,
            float(self.destination["x"]),
            float(self.destination["y"]),
        )
        self.path = self.path_result.points
        if not self.path:
            raise ValueError("AUTO_LOCAL planner returned an empty path")
        self.previous_index = 0
        self.temporary_path = None
        self.temporary_index = 0
        self.replan_count += 1
        return self.path_result

    def update(self, pose, lidar_points):
        if not self.path:
            self.plan_from_pose(pose)

        goal_distance = math.hypot(
            float(self.destination["x"]) - pose.x,
            float(self.destination["y"]) - pose.y,
        )
        if goal_distance < 0.35:
            return AutoLocalCommand(0.0, 0.0, goal_distance, len(self.path) - 1, finished=True)

        decision = self.avoidance.plan(lidar_points or [])
        if decision.stop_required:
            return AutoLocalCommand(
                0.0,
                0.0,
                0.0,
                self.previous_index,
                avoidance_active=True,
                avoidance_side=decision.preferred_side or self.avoidance_side,
                avoidance_reason=decision.reason,
            )

        if self.temporary_path is None and decision.active and decision.preferred_side:
            self.temporary_path = self._build_bypass(pose, decision.preferred_side)
            self.temporary_index = 0
            self.avoidance_side = decision.preferred_side
            self.avoidance_reason = decision.reason

        if self.temporary_path is not None:
            result = self.bypass_pursuit.calculate(
                pose.x,
                pose.y,
                pose.yaw_radians,
                self.temporary_path,
                self.temporary_index,
            )
            self.temporary_index = result.nearest_index
            if result.finished:
                side = self.avoidance_side
                self.temporary_path = None
                self.temporary_index = 0
                self.avoidance_side = None
                self.avoidance_reason = None
                try:
                    self.plan_from_pose(pose)
                except ValueError as error:
                    return AutoLocalCommand(0.0, 0.0, 0.0, 0, fault=str(error))
                return AutoLocalCommand(
                    0.0,
                    0.0,
                    0.0,
                    0,
                    avoidance_active=False,
                    avoidance_side=side,
                    avoidance_reason="REJOINED_LOCAL_PATH",
                    replanned=True,
                )
            return AutoLocalCommand(
                result.steering_angle_degrees,
                self.base_throttle * self.avoidance_speed_scale,
                result.cross_track_error_m,
                result.target_index,
                avoidance_active=True,
                avoidance_side=self.avoidance_side,
                avoidance_reason=self.avoidance_reason or "AVOID",
            )

        result = self.pursuit.calculate(
            pose.x,
            pose.y,
            pose.yaw_radians,
            self.path,
            self.previous_index,
        )
        self.previous_index = result.nearest_index
        if result.cross_track_error_m > self.maximum_cross_track_error_m:
            try:
                self.plan_from_pose(pose)
                result = self.pursuit.calculate(
                    pose.x,
                    pose.y,
                    pose.yaw_radians,
                    self.path,
                    0,
                )
            except ValueError as error:
                return AutoLocalCommand(
                    0.0,
                    0.0,
                    result.cross_track_error_m,
                    result.target_index,
                    fault=f"LOCAL_PATH_LOST:{error}",
                )
            replanned = True
        else:
            replanned = False

        steering_ratio = min(
            1.0,
            abs(result.steering_angle_degrees) / self.pursuit.maximum_steering_degrees,
        )
        throttle = self.base_throttle * (1.0 - 0.55 * steering_ratio)
        return AutoLocalCommand(
            result.steering_angle_degrees,
            throttle,
            result.cross_track_error_m,
            result.target_index,
            finished=result.finished,
            replanned=replanned,
        )

    def snapshot(self):
        return {
            "destination": self.destination,
            "path": [{"x": p.x, "y": p.y} for p in self.path],
            "path_summary": None if self.path_result is None else self.path_result.as_dict(),
            "previous_index": self.previous_index,
            "temporary_path": [
                {"x": p.x, "y": p.y}
                for p in (self.temporary_path or [])
            ],
            "avoidance_side": self.avoidance_side,
            "avoidance_reason": self.avoidance_reason,
            "replan_count": self.replan_count,
        }

    @staticmethod
    def _build_bypass(pose, side):
        sign = 1.0 if side == "left" else -1.0
        fx = math.cos(pose.yaw_radians)
        fy = math.sin(pose.yaw_radians)
        lx = -math.sin(pose.yaw_radians)
        ly = math.cos(pose.yaw_radians)
        return [
            PathPoint(
                pose.x + fx * 0.55 + lx * sign * 0.38,
                pose.y + fy * 0.55 + ly * sign * 0.38,
            ),
            PathPoint(
                pose.x + fx * 1.20 + lx * sign * 0.62,
                pose.y + fy * 1.20 + ly * sign * 0.62,
            ),
            PathPoint(
                pose.x + fx * 1.75 + lx * sign * 0.32,
                pose.y + fy * 1.75 + ly * sign * 0.32,
            ),
        ]
