from dataclasses import dataclass
import math

from autonomous_car.control import PurePursuit


@dataclass(frozen=True)
class RouteSimulationResult:
    completed: bool
    steps: int
    maximum_cross_track_error_m: float
    final_distance_m: float
    steering_reversals: int
    trajectory: list[tuple[float, float]]


class RouteSimulator:
    def __init__(self, wheelbase_m=0.53, lookahead_m=0.8, speed_mps=0.25, time_step=0.05):
        self.wheelbase_m = float(wheelbase_m)
        self.speed_mps = float(speed_mps)
        self.time_step = float(time_step)
        self.controller = PurePursuit(wheelbase_m, lookahead_m, maximum_steering_degrees=20.0)

    def run(self, path, initial_x=None, initial_y=None, initial_heading=None, maximum_seconds=120.0):
        if len(path) < 2:
            raise ValueError("Simulation path requires at least two points")
        x = path[0].x if initial_x is None else float(initial_x)
        y = path[0].y if initial_y is None else float(initial_y)
        heading = (
            math.atan2(path[1].y - path[0].y, path[1].x - path[0].x)
            if initial_heading is None
            else float(initial_heading)
        )
        previous_index = 0
        maximum_error = 0.0
        previous_sign = 0
        reversals = 0
        trajectory = [(x, y)]
        maximum_steps = int(maximum_seconds / self.time_step)
        completed = False
        for step in range(maximum_steps):
            result = self.controller.calculate(x, y, heading, path, previous_index)
            previous_index = result.nearest_index
            maximum_error = max(maximum_error, result.cross_track_error_m)
            steering_radians = math.radians(result.steering_angle_degrees)
            sign = 0 if abs(result.steering_angle_degrees) < 0.5 else (1 if steering_radians > 0 else -1)
            if sign and previous_sign and sign != previous_sign:
                reversals += 1
            if sign:
                previous_sign = sign
            x += self.speed_mps * math.cos(heading) * self.time_step
            y += self.speed_mps * math.sin(heading) * self.time_step
            heading += (
                self.speed_mps / self.wheelbase_m
                * math.tan(steering_radians)
                * self.time_step
            )
            trajectory.append((x, y))
            if result.finished:
                completed = True
                break
        final_distance = math.hypot(path[-1].x - x, path[-1].y - y)
        return RouteSimulationResult(
            completed,
            step + 1,
            maximum_error,
            final_distance,
            reversals,
            trajectory,
        )
