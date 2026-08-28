"""Regression for lane temporal outlier rejection and fast re-acquisition."""

from autonomous_car.control import LaneResult
from autonomous_car.control.lane_geometry_hardening import (
    _result_signature,
    _snapshot_lane_history,
    _temporal_guard,
)


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


class _Controller:
    def __init__(self):
        self._previous_width_profile = None
        self._previous_width_y = None
        self._previous_left_coefficients = None
        self._previous_right_coefficients = None
        self._frames_since_two_boundary = 1000
        self._lane_stable_result = None
        self._lane_stable_signature = None
        self._lane_stable_misses = 0
        self._lane_pending_signature = None
        self._lane_pending_count = 0


def _line(points):
    return {
        "top_x": float(points[0][0]),
        "top_y": float(points[0][1]),
        "bottom_x": float(points[-1][0]),
        "bottom_y": float(points[-1][1]),
        "points": [[float(x), float(y)] for x, y in points],
    }


def _lane(center_bottom=320.0, width_bottom=520.0, heading=0.0, confidence=0.92):
    top_y = 170.0
    bottom_y = 350.0
    top_width = width_bottom * 0.42
    center_top = center_bottom + heading * 1.5
    left = _line(
        [
            [center_top - top_width / 2.0, top_y],
            [center_bottom - width_bottom / 2.0, bottom_y],
        ]
    )
    right = _line(
        [
            [center_top + top_width / 2.0, top_y],
            [center_bottom + width_bottom / 2.0, bottom_y],
        ]
    )
    center = _line([[center_top, top_y], [center_bottom, bottom_y]])
    lateral = (320.0 - center_bottom) / max(width_bottom / 2.0, 1.0)
    return LaneResult(
        detected=True,
        confidence=confidence,
        lateral_error_normalized=lateral,
        lateral_error_m=lateral * 0.5,
        heading_error_degrees=heading,
        correction_angle_degrees=0.0,
        left_line_count=100,
        right_line_count=100,
        lane_width_pixels=width_bottom,
        lane_width_top_pixels=top_width,
        perspective_ratio=width_bottom / top_width,
        left_line=left,
        right_line=right,
        center_line=center,
        image_size=(640, 360),
        roi={"top": 150, "bottom": 354},
    )


def main():
    controller = _Controller()

    stable = _lane(center_bottom=320.0, width_bottom=520.0, heading=0.0)
    accepted = _temporal_guard(controller, stable, _snapshot_lane_history(controller))
    _require(accepted.detected, "initial lane should be accepted")
    _require(controller._lane_stable_signature is not None, "stable signature missing")

    # Simulate a one-frame false floor edge far to one side. The base detector
    # may already have mutated its internal lane-width history; guard must reject
    # the geometry and prevent it from becoming the new stable lane.
    spike = _lane(center_bottom=470.0, width_bottom=300.0, heading=22.0)
    before_spike = _snapshot_lane_history(controller)
    controller._previous_width_profile = [300.0] * 18
    controller._frames_since_two_boundary = 0
    rejected = _temporal_guard(controller, spike, before_spike)
    _require(not rejected.detected, "one-frame jump must not authorize control")
    _require(rejected.error == "TEMPORAL_OUTLIER_HELD", f"unexpected rejection: {rejected}")
    _require(
        abs(_result_signature(controller._lane_stable_result)["center"] - 320.0) < 1e-6,
        "spike replaced stable lane",
    )
    _require(
        controller._previous_width_profile is None,
        "rejected candidate poisoned previous width profile",
    )

    # The next normal frame must recover immediately instead of remaining stuck
    # on the rejected geometry.
    recovered = _lane(center_bottom=323.0, width_bottom=515.0, heading=1.0)
    recovered_result = _temporal_guard(
        controller,
        recovered,
        _snapshot_lane_history(controller),
    )
    _require(recovered_result.detected, "normal lane did not recover immediately")
    _require(
        abs(_result_signature(recovered_result)["center"] - 323.0) < 1e-6,
        "recovery did not select current normal lane",
    )
    _require(controller._lane_pending_count == 0, "pending outlier was not cleared")

    # A real lane change should not be blocked forever: two mutually consistent
    # observations of the new geometry trigger re-acquisition.
    new_lane_1 = _lane(center_bottom=445.0, width_bottom=390.0, heading=15.0)
    first_new = _temporal_guard(
        controller,
        new_lane_1,
        _snapshot_lane_history(controller),
    )
    _require(not first_new.detected, "first large lane change should be confirmed")
    _require(controller._lane_pending_count == 1, "pending confirmation count missing")

    new_lane_2 = _lane(center_bottom=442.0, width_bottom=395.0, heading=14.5)
    second_new = _temporal_guard(
        controller,
        new_lane_2,
        _snapshot_lane_history(controller),
    )
    _require(second_new.detected, "consistent new lane should re-acquire")
    _require(
        abs(_result_signature(controller._lane_stable_result)["center"] - 442.0) < 1e-6,
        "new lane was not made stable",
    )

    print("Lane temporal recovery regression: PASS")
    print(
        {
            "one_frame_spike": rejected.error,
            "recovered_center": _result_signature(recovered_result)["center"],
            "reacquired_center": _result_signature(second_new)["center"],
        }
    )


if __name__ == "__main__":
    main()
