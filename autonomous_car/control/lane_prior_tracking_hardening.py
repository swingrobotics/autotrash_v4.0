"""Prior-guided lane-boundary tracking.

The base sliding-window detector bootstraps every frame from a histogram of the
lower image. That is fragile when a real lane boundary exits the near-field
frame edge: the lower histogram can then lock onto a table leg, floor seam,
shadow, curb, or other strong edge even though the real boundary is still
visible higher in the image.

This layer follows the previously accepted polynomial first. It searches a
narrow corridor around each prior boundary over the full ROI and only falls
back to the global lower-histogram bootstrap when prior support is genuinely
lost. If a prior predicts that a boundary leaves the near-field image edge, an
unrelated lower-image edge is deliberately not used as a replacement; the base
controller can then use its bounded one-side inference/temporal fail-safe path.
"""

import math

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

from .lane_controller import LaneController


_INSTALLED = False
_ORIGINAL_SLIDING_WINDOW_POINTS = None


def _copy_points(points):
    return [(float(x), float(y)) for x, y in (points or [])]


def _collect_rows(nonzero_x, nonzero_y, indices):
    if indices is None or len(indices) == 0:
        return []
    rows = {}
    for index in indices:
        y = int(nonzero_y[index])
        rows.setdefault(y, []).append(int(nonzero_x[index]))
    points = []
    for y, values in rows.items():
        values.sort()
        points.append((float(values[len(values) // 2]), float(y)))
    points.sort(key=lambda point: point[1])
    return points


def _prior_corridor_points(binary, coefficients, margin):
    if np is None or binary is None or coefficients is None:
        return []
    active = binary > 0
    nonzero_y, nonzero_x = active.nonzero()
    if len(nonzero_x) == 0:
        return []
    try:
        predicted = np.polyval(coefficients, nonzero_y.astype(np.float64))
    except Exception:
        return []
    good = np.where(
        np.isfinite(predicted)
        & (np.abs(nonzero_x.astype(np.float64) - predicted) <= float(margin))
    )[0]
    return _collect_rows(nonzero_x, nonzero_y, good)


def _point_quality(points, roi_height):
    if not points:
        return {"rows": 0, "span": 0.0, "coverage": 0.0}
    ys = [point[1] for point in points]
    span = max(ys) - min(ys) if len(ys) > 1 else 0.0
    return {
        "rows": len(points),
        "span": float(span),
        "coverage": float(span / max(1.0, roi_height - 1.0)),
    }


def _mean_distance_to_prior(points, coefficients):
    if np is None or not points or coefficients is None:
        return math.inf
    values = np.asarray(points, dtype=np.float64)
    try:
        predicted = np.polyval(coefficients, values[:, 1])
    except Exception:
        return math.inf
    return float(np.mean(np.abs(values[:, 0] - predicted)))


def _select_side(
    controller,
    global_points,
    prior_points,
    coefficients,
    roi_height,
    roi_width,
):
    if coefficients is None or np is None:
        return _copy_points(global_points)

    prior_quality = _point_quality(prior_points, roi_height)
    strong_prior = (
        prior_quality["rows"] >= max(24, int(roi_height * 0.16))
        and prior_quality["coverage"] >= 0.30
    )
    usable_prior = (
        prior_quality["rows"] >= max(12, int(roi_height * 0.08))
        and prior_quality["coverage"] >= 0.17
    )

    try:
        bottom_prediction = float(np.polyval(coefficients, roi_height - 1.0))
    except Exception:
        bottom_prediction = roi_width * 0.5

    near_edge = (
        bottom_prediction <= roi_width * 0.055
        or bottom_prediction >= roi_width * 0.945
        or bottom_prediction < 0.0
        or bottom_prediction > roi_width - 1.0
    )
    age = int(getattr(controller, "_frames_since_two_boundary", 1000))

    # This is the normal locked-lane path. A corridor around the last accepted
    # polynomial is much less likely to jump to an unrelated parallel/vertical
    # edge than a fresh global histogram search.
    if strong_prior:
        return _copy_points(prior_points)

    # The video failure case: the true boundary is leaving the near-field frame
    # but remains visible higher up. Keep the partial prior-supported boundary,
    # or explicitly report it missing. Never replace it with an arbitrary lower
    # histogram edge while the accepted prior is still fresh.
    if near_edge and age <= 6:
        if usable_prior:
            return _copy_points(prior_points)
        return []

    # When support is marginal but still follows the accepted prior, prefer it
    # over a far-away global candidate for a few frames.
    if usable_prior and age <= 4:
        return _copy_points(prior_points)

    global_points = _copy_points(global_points)
    if global_points and age <= 3:
        distance = _mean_distance_to_prior(global_points, coefficients)
        # If the only global candidate moved dramatically away from the accepted
        # boundary, let the temporal/pair guards treat that side as missing
        # rather than feeding them a high-confidence false edge.
        if distance > max(44.0, roi_width * 0.095):
            return []

    return global_points


def _sliding_window_points_hardened(self, binary):
    global_left, global_right = _ORIGINAL_SLIDING_WINDOW_POINTS(self, binary)
    if np is None or binary is None:
        return global_left, global_right

    roi_height, roi_width = binary.shape[:2]
    margin = max(26, int(round(roi_width * 0.062)))
    left_coefficients = getattr(self, "_previous_left_coefficients", None)
    right_coefficients = getattr(self, "_previous_right_coefficients", None)

    left_prior = _prior_corridor_points(binary, left_coefficients, margin)
    right_prior = _prior_corridor_points(binary, right_coefficients, margin)

    left = _select_side(
        self,
        global_left,
        left_prior,
        left_coefficients,
        roi_height,
        roi_width,
    )
    right = _select_side(
        self,
        global_right,
        right_prior,
        right_coefficients,
        roi_height,
        roi_width,
    )
    return left, right


def install_lane_prior_tracking_hardening():
    global _INSTALLED, _ORIGINAL_SLIDING_WINDOW_POINTS
    if _INSTALLED:
        return
    _ORIGINAL_SLIDING_WINDOW_POINTS = LaneController._sliding_window_points
    LaneController._sliding_window_points = _sliding_window_points_hardened
    _INSTALLED = True


__all__ = ["install_lane_prior_tracking_hardening"]
