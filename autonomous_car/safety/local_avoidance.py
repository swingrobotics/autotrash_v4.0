from dataclasses import dataclass
import math


@dataclass(frozen=True)
class AvoidanceDecision:
    active: bool
    stop_required: bool
    preferred_side: str | None
    nearest_front_m: float | None
    left_clearance_m: float | None
    right_clearance_m: float | None
    speed_scale: float
    reason: str


class LocalAvoidancePlanner:
    """Conservative LiDAR sector planner for AUTO_GPS/AUTO_LOCAL.

    This module deliberately does not emit a raw steering command. It selects a
    safe side and speed envelope; the vehicle-specific path follower converts
    that decision into a temporary local path before returning to the global
    GPS/LOCAL path.
    """

    def __init__(
        self,
        trigger_distance_m=1.50,
        emergency_stop_distance_m=0.60,
        minimum_side_clearance_m=1.00,
        center_sector_degrees=20.0,
        side_sector_degrees=70.0,
        lidar_to_front_bumper_m=0.254,
        avoidance_speed_scale=0.35,
        bearing_sign_for_left=1,
    ):
        self.trigger_distance_m = float(trigger_distance_m)
        self.emergency_stop_distance_m = float(emergency_stop_distance_m)
        self.minimum_side_clearance_m = float(minimum_side_clearance_m)
        self.center_sector_degrees = abs(float(center_sector_degrees))
        self.side_sector_degrees = max(
            self.center_sector_degrees,
            abs(float(side_sector_degrees)),
        )
        self.lidar_to_front_bumper_m = max(0.0, float(lidar_to_front_bumper_m))
        self.avoidance_speed_scale = max(0.0, min(1.0, float(avoidance_speed_scale)))
        self.bearing_sign_for_left = 1 if float(bearing_sign_for_left) >= 0 else -1

    def plan(self, points) -> AvoidanceDecision:
        center = []
        left = []
        right = []

        for point in points or ():
            try:
                distance_m = float(point["distance_mm"]) / 1000.0
                bearing_degrees = self._normalize_bearing(
                    float(point["bearing_degrees"])
                )
            except (KeyError, TypeError, ValueError):
                continue

            if distance_m <= 0:
                continue

            bearing_radians = math.radians(bearing_degrees)
            forward = distance_m * math.cos(bearing_radians)
            if forward <= self.lidar_to_front_bumper_m:
                continue

            clearance = forward - self.lidar_to_front_bumper_m
            absolute_bearing = abs(bearing_degrees)

            if absolute_bearing <= self.center_sector_degrees:
                center.append(clearance)
                continue

            if absolute_bearing > self.side_sector_degrees:
                continue

            is_left = bearing_degrees * self.bearing_sign_for_left > 0
            (left if is_left else right).append(clearance)

        nearest_front = min(center) if center else None
        left_clearance = min(left) if left else None
        right_clearance = min(right) if right else None

        if nearest_front is None or nearest_front > self.trigger_distance_m:
            return AvoidanceDecision(
                active=False,
                stop_required=False,
                preferred_side=None,
                nearest_front_m=nearest_front,
                left_clearance_m=left_clearance,
                right_clearance_m=right_clearance,
                speed_scale=1.0,
                reason="CLEAR",
            )

        side = self._choose_side(left_clearance, right_clearance)
        if nearest_front <= self.emergency_stop_distance_m:
            return AvoidanceDecision(
                active=True,
                stop_required=True,
                preferred_side=side,
                nearest_front_m=nearest_front,
                left_clearance_m=left_clearance,
                right_clearance_m=right_clearance,
                speed_scale=0.0,
                reason="TOO_CLOSE_STOP",
            )

        if side is None:
            return AvoidanceDecision(
                active=True,
                stop_required=True,
                preferred_side=None,
                nearest_front_m=nearest_front,
                left_clearance_m=left_clearance,
                right_clearance_m=right_clearance,
                speed_scale=0.0,
                reason="NO_SAFE_SIDE",
            )

        return AvoidanceDecision(
            active=True,
            stop_required=False,
            preferred_side=side,
            nearest_front_m=nearest_front,
            left_clearance_m=left_clearance,
            right_clearance_m=right_clearance,
            speed_scale=self.avoidance_speed_scale,
            reason="AVOID",
        )

    def _choose_side(self, left_clearance, right_clearance):
        candidates = []
        # Unknown side sectors are not treated as free space. A side must have
        # observed clearance before the planner is allowed to select it.
        if left_clearance is not None and left_clearance >= self.minimum_side_clearance_m:
            candidates.append(("left", left_clearance))
        if right_clearance is not None and right_clearance >= self.minimum_side_clearance_m:
            candidates.append(("right", right_clearance))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[1], reverse=True)
        return candidates[0][0]

    @staticmethod
    def _normalize_bearing(value):
        return ((value + 180.0) % 360.0) - 180.0
