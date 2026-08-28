"""Regression for prior-guided lane tracking at a near-field frame edge."""

from autonomous_car.control import LaneController


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def _mean_prior_error(points, coefficients):
    import numpy as np

    if not points:
        return 9999.0
    values = np.asarray(points, dtype=np.float64)
    expected = np.polyval(coefficients, values[:, 1])
    return float(np.mean(np.abs(values[:, 0] - expected)))


def _draw_boundary(binary, coefficients, y0, y1, half_width=2):
    import numpy as np

    h, w = binary.shape
    for y in range(max(0, int(y0)), min(h, int(y1))):
        x = float(np.polyval(coefficients, float(y)))
        if not (0.0 <= x < w):
            continue
        xi = int(round(x))
        binary[y, max(0, xi - half_width) : min(w, xi + half_width + 1)] = 255


def main():
    import numpy as np

    controller = LaneController(processing_width=640, processing_height=360)
    _require(controller.available, "OpenCV/NumPy are required")

    # This reproduces the recorded failure mode. The accepted right lane heads
    # out through the lower-right frame edge, while an unrelated vertical edge
    # remains visible through the full lower histogram. A fresh histogram search
    # tends to prefer that vertical edge, but prior-guided tracking must stay on
    # the real boundary that is still visible in the upper/middle ROI.
    h, w = 210, 640
    left_coefficients = np.asarray([-0.78, 265.0], dtype=np.float64)
    right_coefficients = np.asarray([1.36, 390.0], dtype=np.float64)
    binary = np.zeros((h, w), dtype=np.uint8)
    _draw_boundary(binary, left_coefficients, 0, h, half_width=2)
    _draw_boundary(binary, right_coefficients, 0, h, half_width=2)
    binary[:, 500:505] = 255  # table leg/floor edge distractor

    controller._previous_left_coefficients = left_coefficients.copy()
    controller._previous_right_coefficients = right_coefficients.copy()
    controller._frames_since_two_boundary = 0

    left, right = controller._sliding_window_points(binary)
    _require(len(left) >= 30, f"left prior tracking lost: {len(left)}")
    _require(len(right) >= 30, f"right prior tracking lost: {len(right)}")
    right_error = _mean_prior_error(right, right_coefficients)
    _require(
        right_error < 8.0,
        f"right boundary jumped from prior to distractor: mean error={right_error:.2f}",
    )

    # If the real right boundary is no longer visible at all while its accepted
    # prior predicts a near-field exit, do not promote the unrelated vertical
    # edge to a new lane. Returning no right points intentionally triggers the
    # bounded single-boundary/lost-lane path instead.
    missing = np.zeros((h, w), dtype=np.uint8)
    _draw_boundary(missing, left_coefficients, 0, h, half_width=2)
    missing[:, 500:505] = 255
    controller._frames_since_two_boundary = 1
    left2, right2 = controller._sliding_window_points(missing)
    _require(len(left2) >= 30, "left boundary unexpectedly disappeared")
    _require(
        len(right2) == 0,
        f"near-edge missing right lane was replaced by distractor: {len(right2)} points",
    )

    print("Lane prior tracking V2 regression: PASS")
    print(
        {
            "tracked_right_points": len(right),
            "tracked_right_mean_error_pixels": right_error,
            "missing_right_points": len(right2),
        }
    )


if __name__ == "__main__":
    main()
