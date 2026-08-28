"""Offline regression checks for the V2 AUTO_GPS temporary bypass planner.

Run with:
    python3 -m autonomous_car.simulation.validate_auto_gps_v2

This is geometry/control validation only. It does not replace closed-area
vehicle validation of stopping distance, LiDAR coverage, or steering tracking.
"""

from dataclasses import dataclass
import math

from autonomous_car.control import PathPoint
from autonomous_car.modes import AutoGpsPlanner


@dataclass(frozen=True)
class _BaseCommand:
    steering_angle_degrees: float = 0.0
    throttle: float = 0.25
    cross_track_error_m: float = 0.0
    nearest_index: int = 0
    target_index: int = 4
    finished: bool = False
    fault: str | None = None


@dataclass(frozen=True)
class _Decision:
    active: bool
    stop_required: bool
    preferred_side: str | None
    speed_scale: float = 0.35
    reason: str = "CLEAR"


class _Route:
    def __init__(self):
        self.points = [PathPoint(index * 0.2, 0.0) for index in range(41)]


class _Converter:
    def to_enu(self, latitude, longitude, altitude=None):
        return float(latitude), float(longitude), 0.0


class _RoutePlanner:
    def __init__(self, route):
        self.route = route
        self.converter = _Converter()
        self.base_throttle = 0.25

    def preflight(self, *args, **kwargs):
        return True

    @staticmethod
    def compass_to_enu_heading(degrees):
        return math.radians(float(degrees))

    def update(self, gps, imu, now=None):
        index = max(
            0,
            min(
                len(self.route.points) - 1,
                round(float(gps["latitude"]) / 0.2),
            ),
        )
        return _BaseCommand(
            nearest_index=index,
            target_index=min(index + 4, len(self.route.points) - 1),
        )


class _SequenceAvoidance:
    def __init__(self, decisions):
        self.decisions = list(decisions)

    def plan(self, points):
        if self.decisions:
            return self.decisions.pop(0)
        return _Decision(False, False, None, 1.0, "CLEAR")


def validate():
    route = _Route()

    clear = AutoGpsPlanner(
        route,
        route_planner=_RoutePlanner(route),
        avoidance_planner=_SequenceAvoidance([
            _Decision(False, False, None, 1.0, "CLEAR")
        ]),
    )
    clear_command = clear.update(
        {"latitude": 0.0, "longitude": 0.0},
        {"global_heading_degrees": 0.0},
        [],
    )
    assert not clear_command.avoidance_active
    assert clear_command.throttle == 0.25

    left = AutoGpsPlanner(
        route,
        route_planner=_RoutePlanner(route),
        avoidance_planner=_SequenceAvoidance([
            _Decision(True, False, "left", 0.35, "AVOID"),
            _Decision(False, False, None, 1.0, "CLEAR"),
        ]),
    )
    avoid_command = left.update(
        {"latitude": 0.0, "longitude": 0.0},
        {"global_heading_degrees": 0.0},
        [],
    )
    assert avoid_command.avoidance_active
    assert avoid_command.avoidance_side == "left"
    assert avoid_command.throttle <= 0.25 * 0.35 + 1e-9
    assert avoid_command.steering_angle_degrees > 0.0

    snapshot = left.snapshot()
    assert len(snapshot["temporary_path"]) == 3
    assert snapshot["rejoin_index"] is not None
    last = snapshot["temporary_path"][-1]
    rejoined = left.update(
        {"latitude": last["x"], "longitude": last["y"]},
        {"global_heading_degrees": 0.0},
        [],
    )
    assert not rejoined.avoidance_active
    assert rejoined.avoidance_reason == "REJOINED_GLOBAL_ROUTE"

    stopped = AutoGpsPlanner(
        route,
        route_planner=_RoutePlanner(route),
        avoidance_planner=_SequenceAvoidance([
            _Decision(True, True, "right", 0.0, "TOO_CLOSE_STOP")
        ]),
    ).update(
        {"latitude": 0.0, "longitude": 0.0},
        {"global_heading_degrees": 0.0},
        [],
    )
    assert stopped.throttle == 0.0
    assert stopped.steering_angle_degrees == 0.0
    assert stopped.avoidance_reason == "TOO_CLOSE_STOP"

    return {
        "clear_route": "PASS",
        "left_bypass": "PASS",
        "route_rejoin": "PASS",
        "too_close_stop": "PASS",
    }


def main():
    result = validate()
    print("AUTO_GPS V2 validation: PASS")
    for name, status in result.items():
        print(f"- {name}: {status}")


if __name__ == "__main__":
    main()
