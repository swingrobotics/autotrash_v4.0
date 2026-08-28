from dataclasses import dataclass
import math
import threading
import weakref


_RUNTIME_WHEELBASE_M = None
_RUNTIME_WHEELBASE_LOCK = threading.RLock()
_PURSUIT_INSTANCES = weakref.WeakSet()


def configure_vehicle_wheelbase_m(value):
    """Set the physical vehicle wheelbase for every PurePursuit instance.

    Wheelbase is a machine-wide physical dimension, not a planner tuning knob.
    VehicleRuntimeSettings calls this while the rover is stopped so existing
    planners and planners created later use the same measured axle-to-axle
    distance.
    """
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("wheelbase_m must be a positive finite number")
    global _RUNTIME_WHEELBASE_M
    with _RUNTIME_WHEELBASE_LOCK:
        _RUNTIME_WHEELBASE_M = value
        for pursuit in list(_PURSUIT_INSTANCES):
            pursuit.wheelbase_m = value
    return value


def configured_vehicle_wheelbase_m():
    with _RUNTIME_WHEELBASE_LOCK:
        return _RUNTIME_WHEELBASE_M


@dataclass(frozen=True)
class PathPoint:
    x: float
    y: float
    speed_mps: float | None = None


@dataclass(frozen=True)
class PursuitResult:
    steering_angle_degrees: float
    nearest_index: int
    target_index: int
    cross_track_error_m: float
    target_distance_m: float
    finished: bool


class PurePursuit:
    def __init__(self, wheelbase_m=0.53, lookahead_m=0.8, maximum_steering_degrees=20.0):
        requested_wheelbase = float(wheelbase_m)
        if not math.isfinite(requested_wheelbase) or requested_wheelbase <= 0.0:
            raise ValueError("wheelbase_m must be a positive finite number")
        with _RUNTIME_WHEELBASE_LOCK:
            self.wheelbase_m = float(
                _RUNTIME_WHEELBASE_M
                if _RUNTIME_WHEELBASE_M is not None
                else requested_wheelbase
            )
            _PURSUIT_INSTANCES.add(self)
        self.lookahead_m = float(lookahead_m)
        self.maximum_steering_degrees = abs(float(maximum_steering_degrees))

    def calculate(self, x, y, heading_radians, path, previous_index=0):
        if not path:
            raise ValueError("Path is empty")
        start = max(0, min(int(previous_index), len(path) - 1))
        search_end = min(len(path), start + 200)
        nearest_index = min(
            range(start, search_end),
            key=lambda index: math.hypot(path[index].x - x, path[index].y - y),
        )
        cross_track_error = math.hypot(path[nearest_index].x - x, path[nearest_index].y - y)
        target_index = nearest_index
        while target_index < len(path) - 1:
            distance = math.hypot(path[target_index].x - x, path[target_index].y - y)
            if distance >= self.lookahead_m:
                break
            target_index += 1
        target = path[target_index]
        target_distance = max(0.01, math.hypot(target.x - x, target.y - y))
        target_heading = math.atan2(target.y - y, target.x - x)
        alpha = (target_heading - heading_radians + math.pi) % (2 * math.pi) - math.pi
        curvature = 2.0 * math.sin(alpha) / target_distance
        steering = math.degrees(math.atan(self.wheelbase_m * curvature))
        steering = max(-self.maximum_steering_degrees, min(self.maximum_steering_degrees, steering))
        finish_distance = math.hypot(path[-1].x - x, path[-1].y - y)
        return PursuitResult(
            steering_angle_degrees=steering,
            nearest_index=nearest_index,
            target_index=target_index,
            cross_track_error_m=cross_track_error,
            target_distance_m=target_distance,
            finished=target_index == len(path) - 1 and finish_distance < 0.4,
        )


__all__ = [
    "PathPoint",
    "PurePursuit",
    "PursuitResult",
    "configure_vehicle_wheelbase_m",
    "configured_vehicle_wheelbase_m",
]
