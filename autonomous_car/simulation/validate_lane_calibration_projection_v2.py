"""Regression for calibrated lane display geometry and adaptive curve fitting."""

import math

from autonomous_car.control import LaneController
from autonomous_car.control.lane_geometry_hardening import distort_undistorted_points


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


class _Calibration:
    calibrated = True
    data = {
        "image_size": [640, 360],
        "camera_matrix": [
            [420.0, 0.0, 320.0],
            [0.0, 420.0, 180.0],
            [0.0, 0.0, 1.0],
        ],
        "distortion_coefficients": [-0.20, 0.045, 0.0, 0.0, 0.0],
    }


def main():
    controller = LaneController()
    _require(controller.available, "OpenCV/NumPy are required")

    # Straight tape with small deterministic noise must remain a linear model.
    straight = []
    for y in range(0, 221, 4):
        x = 105.0 + 0.34 * y + math.sin(y * 0.11) * 0.7
        straight.append((x, float(y)))
    # A few unrelated edge candidates should be rejected rather than bending the
    # entire boundary into a quadratic.
    straight.extend([(195.0, 40.0), (65.0, 92.0), (230.0, 156.0)])
    straight_fit = controller._robust_fit(straight)
    _require(straight_fit is not None, "straight fit missing")
    _require(
        int(straight_fit.get("degree", 0)) == 1,
        f"straight lane was over-fit as a curve: {straight_fit}",
    )

    # A genuine smooth road curve should still select quadratic fitting.
    curved = []
    for y in range(0, 221, 4):
        x = 92.0 + 0.06 * y + 0.00225 * y * y + math.sin(y * 0.09) * 0.45
        curved.append((x, float(y)))
    curved_fit = controller._robust_fit(curved)
    _require(curved_fit is not None, "curved fit missing")
    _require(
        int(curved_fit.get("degree", 0)) == 2,
        f"real curve was flattened: {curved_fit}",
    )

    # Lane control runs in the undistorted plane, while the dashboard background
    # is raw MJPEG. Projection must therefore move edge points back into the raw
    # distorted image while leaving the optical center essentially unchanged.
    calibration = _Calibration()
    source = [[320.0, 180.0], [80.0, 280.0], [560.0, 280.0]]
    projected = distort_undistorted_points(calibration, source, (640, 360))
    _require(len(projected) == len(source), "projection point count changed")
    _require(
        abs(projected[0][0] - 320.0) < 0.5 and abs(projected[0][1] - 180.0) < 0.5,
        f"optical center should remain fixed: {projected[0]}",
    )
    _require(
        projected[1][0] > source[1][0] and projected[2][0] < source[2][0],
        f"barrel-distorted raw projection did not move edge points inward: {projected}",
    )

    print("Lane calibration/display projection regression: PASS")
    print({"straight": straight_fit, "curved": curved_fit, "projected": projected})


if __name__ == "__main__":
    main()
