from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObstacleResult:
    distance_m: float | None
    point_count: int


class ObstacleChecker:
    def __init__(
        self,
        half_width_m=0.45,
        lidar_to_front_bumper_m=0.254,
        vehicle_half_width_m=0.2413,
    ):
        self.half_width_m = float(half_width_m)
        self.lidar_to_front_bumper_m = float(lidar_to_front_bumper_m)
        self.vehicle_half_width_m = float(vehicle_half_width_m)

    def check(self, points):
        nearest = None
        point_count = 0
        for point in points or ():
            try:
                distance_m = float(point["distance_mm"]) / 1000.0
                bearing = math.radians(float(point["bearing_degrees"]))
            except (KeyError, TypeError, ValueError):
                continue
            if distance_m <= 0:
                continue
            forward = distance_m * math.cos(bearing)
            lateral = distance_m * math.sin(bearing)
            if forward <= 0 or abs(lateral) > self.half_width_m:
                continue
            if forward <= self.lidar_to_front_bumper_m:
                continue
            point_count += 1
            clearance = max(0.0, forward - self.lidar_to_front_bumper_m)
            nearest = clearance if nearest is None else min(nearest, clearance)
        return ObstacleResult(nearest, point_count)
