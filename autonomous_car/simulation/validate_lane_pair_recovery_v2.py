"""Regression for pair-wise lane boundary rejection and recovery."""

from autonomous_car.control.lane_controller import LaneResult
from autonomous_car.control.lane_pair_hardening import (
    _intrinsic_pair_inconsistent,
    _pair_guard,
    _snapshot_state,
    _unilateral_jump,
)


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def _line(xs, top=150.0, bottom=354.0):
    count = len(xs)
    ys = [top + (bottom - top) * index / (count - 1) for index in range(count)]
    points = [[float(x), float(y)] for x, y in zip(xs, ys)]
    return {
        "top_x": points[0][0],
        "top_y": points[0][1],
        "bottom_x": points[-1][0],
        "bottom_y": points[-1][1],
        "points": points,
    }


def _lane(left_xs, right_xs, marking="BLACK", confidence=0.90):
    left = _line(left_xs)
    right = _line(right_xs)
    center_xs = [(left_x + right_x) * 0.5 for left_x, right_x in zip(left_xs, right_xs)]
    center = _line(center_xs)
    bottom_width = float(right_xs[-1] - left_xs[-1])
    top_width = float(right_xs[0] - left_xs[0])
    center_bottom = center_xs[-1]
    lateral = (320.0 - center_bottom) / max(bottom_width * 0.5, 1.0)
    return LaneResult(
        detected=True,
        confidence=confidence,
        lateral_error_normalized=lateral,
        lateral_error_m=lateral * 0.5,
        heading_error_degrees=0.0,
        correction_angle_degrees=0.0,
        left_line_count=100,
        right_line_count=100,
        lane_width_pixels=bottom_width,
        lane_width_top_pixels=top_width,
        perspective_ratio=bottom_width / max(top_width, 1.0),
        expected_lane_width_m=1.0,
        vehicle_width_m=0.4826,
        left_line=left,
        right_line=right,
        center_line=center,
        backend="CLASSICAL_CV",
        marking=marking,
        roi={"top": 150, "bottom": 354},
        image_size=(640, 360),
    )


class _Controller:
    pass


def _apply(controller, candidate):
    before = _snapshot_state(controller)
    return _pair_guard(controller, candidate, before)


def main():
    stable = _lane(
        [270, 245, 215, 185, 155, 125, 95],
        [370, 395, 425, 455, 485, 515, 545],
        marking="BLACK",
    )

    # A single unrelated edge on the right moves only that boundary. It must not
    # replace an already stable physical lane.
    wrong_right = _lane(
        [270, 245, 215, 185, 155, 125, 95],
        [465, 490, 520, 550, 580, 610, 635],
        marking="EDGE",
    )
    _require(_unilateral_jump(stable, wrong_right), "right-side jump was not detected")

    controller = _Controller()
    first = _apply(controller, stable)
    _require(first.detected, "initial stable lane was not accepted")

    rejected = _apply(controller, wrong_right)
    _require(not rejected.detected, "one-frame wrong boundary was accepted")
    _require(
        str(rejected.error or "").startswith("BOUNDARY_OUTLIER_REJECTED"),
        f"unexpected rejection state: {rejected.error}",
    )

    recovered = _apply(controller, stable)
    _require(recovered.detected, "stable lane did not recover immediately")
    _require(
        abs(recovered.right_line["bottom_x"] - stable.right_line["bottom_x"]) < 1e-6,
        "recovery did not return to the physical boundary",
    )

    # EDGE-only unilateral changes need five mutually consistent frames before
    # re-acquisition. This prevents persistent floor seams from winning after a
    # two-frame debounce.
    controller = _Controller()
    _apply(controller, stable)
    for index in range(4):
        result = _apply(controller, wrong_right)
        _require(
            not result.detected,
            f"EDGE-only unilateral boundary reacquired too early at frame {index + 1}",
        )
    fifth = _apply(controller, wrong_right)
    _require(fifth.detected, "consistent new EDGE geometry was never reacquired")

    # Strongly different curvature between left/right boundaries is invalid even
    # before a temporal baseline exists.
    bowed_right = _lane(
        [270, 245, 215, 185, 155, 125, 95],
        [370, 420, 485, 555, 590, 575, 545],
        marking="EDGE",
    )
    _require(
        _intrinsic_pair_inconsistent(bowed_right),
        "one-sided curvature mismatch was not rejected",
    )
    fresh = _Controller()
    inconsistent = _apply(fresh, bowed_right)
    _require(not inconsistent.detected, "inconsistent lane pair was accepted")
    _require(
        inconsistent.error == "BOUNDARY_PAIR_INCONSISTENT",
        f"wrong inconsistent-pair error: {inconsistent.error}",
    )

    print("Lane pair recovery regression: PASS")
    print(
        {
            "single_jump_error": rejected.error,
            "recovered_right_bottom": recovered.right_line["bottom_x"],
            "edge_reacquired_after": 5,
            "inconsistent_error": inconsistent.error,
        }
    )


if __name__ == "__main__":
    main()
