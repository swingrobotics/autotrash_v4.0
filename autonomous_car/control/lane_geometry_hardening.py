"""Runtime hardening for calibrated lane geometry.

This module keeps control geometry in the undistorted camera plane while
projecting dashboard polylines back into the raw/distorted MJPEG coordinate
space. It also adds:

* adaptive line-vs-quadratic model selection so straight tape is not over-fit,
* a temporal acceptance guard so a single bad frame cannot poison lane history,
* fast recovery to the last accepted lane, with deliberate re-acquisition when
  a genuinely new geometry is observed consistently.
"""

from dataclasses import replace
import math

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - handled by LaneController.available
    cv2 = None
    np = None

from .lane_controller import LaneController, LaneResult


_INSTALLED = False
_ORIGINAL_ANALYZE_IMAGE = None
_ORIGINAL_RESET = None


def _robust_polyfit(points, degree):
    if np is None or len(points) < max(18, degree + 3):
        return None
    values = np.asarray(points, dtype=np.float64)
    x = values[:, 0]
    y = values[:, 1]
    if np.ptp(y) < 45.0:
        return None

    mask = np.ones(len(values), dtype=bool)
    coefficients = None
    for _ in range(5):
        if int(mask.sum()) < max(14, degree + 4):
            return None
        try:
            coefficients = np.polyfit(y[mask], x[mask], degree)
        except (TypeError, ValueError, np.linalg.LinAlgError):
            return None
        predicted = np.polyval(coefficients, y)
        residual = np.abs(predicted - x)
        core = residual[mask]
        median_residual = float(np.median(core)) if len(core) else 0.0
        mad = float(np.median(np.abs(core - median_residual))) if len(core) else 0.0
        robust_sigma = 1.4826 * mad
        percentile = float(np.percentile(core, 78)) if len(core) else 0.0
        threshold = max(
            5.5,
            min(
                18.0,
                max(
                    percentile * 1.55 + 1.5,
                    median_residual + 3.0 * robust_sigma + 1.5,
                ),
            ),
        )
        next_mask = residual <= threshold
        if np.array_equal(next_mask, mask):
            break
        mask = next_mask

    if coefficients is None or int(mask.sum()) < 14:
        return None
    inlier_y = y[mask]
    inlier_x = x[mask]
    rows = len(set(int(value) for value in inlier_y))
    residuals = np.abs(np.polyval(coefficients, inlier_y) - inlier_x)
    residual_mean = float(np.mean(residuals)) if len(residuals) else math.inf
    residual_rms = float(np.sqrt(np.mean(np.square(residuals)))) if len(residuals) else math.inf
    y_min = float(np.min(inlier_y))
    y_max = float(np.max(inlier_y))
    coverage = min(1.0, np.ptp(inlier_y) / max(1.0, np.ptp(y)))
    return {
        "coefficients": coefficients,
        "inliers": int(mask.sum()),
        "rows": rows,
        "coverage": float(coverage),
        "residual": residual_mean,
        "residual_rms": residual_rms,
        "degree": int(degree),
        "y_min": y_min,
        "y_max": y_max,
    }


def _quadratic_bow(fit):
    if fit is None or int(fit.get("degree", 0)) != 2:
        return 0.0
    y0 = float(fit.get("y_min", 0.0))
    y1 = float(fit.get("y_max", y0))
    if y1 <= y0:
        return 0.0
    ym = (y0 + y1) * 0.5
    coeff = fit["coefficients"]
    x0 = float(np.polyval(coeff, y0))
    x1 = float(np.polyval(coeff, y1))
    xm = float(np.polyval(coeff, ym))
    chord_mid = (x0 + x1) * 0.5
    return xm - chord_mid


def _adaptive_robust_fit(points):
    """Prefer a straight boundary unless curvature is strongly supported."""
    linear = _robust_polyfit(points, 1)
    quadratic = _robust_polyfit(points, 2)
    if linear is None:
        if quadratic is not None:
            quadratic["curve_bow_pixels"] = float(_quadratic_bow(quadratic))
        return quadratic
    if quadratic is None:
        linear["curve_bow_pixels"] = 0.0
        return linear

    bow = float(_quadratic_bow(quadratic))
    y_span = max(1.0, float(quadratic["y_max"] - quadratic["y_min"]))
    a = float(quadratic["coefficients"][0])
    slope_change = abs(2.0 * a * y_span)
    line_residual = max(0.25, float(linear.get("residual", math.inf)))
    quad_residual = float(quadratic.get("residual", math.inf))
    improvement = line_residual - quad_residual
    improvement_ratio = quad_residual / line_residual

    use_quadratic = (
        abs(bow) >= 3.0
        and improvement >= 1.25
        and improvement_ratio <= 0.72
        and slope_change <= 1.15
        and abs(bow) <= max(28.0, y_span * 0.34)
    )

    selected = quadratic if use_quadratic else linear
    selected["curve_bow_pixels"] = bow if use_quadratic else 0.0
    selected["linear_residual"] = float(linear.get("residual", math.inf))
    selected["quadratic_residual"] = float(quadratic.get("residual", math.inf))
    selected["model_selection"] = "QUADRATIC" if use_quadratic else "LINEAR"
    return selected


def _calibration_vision_usable(calibration):
    if calibration is None:
        return False
    usable = getattr(calibration, "vision_usable", None)
    if usable is not None:
        return bool(usable)
    return bool(getattr(calibration, "calibrated", False))


def _scaled_camera_matrix(calibration, width, height):
    data = getattr(calibration, "data", None)
    if not data or np is None:
        return None, None
    try:
        original_width, original_height = [float(v) for v in data["image_size"]]
        matrix = np.asarray(data["camera_matrix"], dtype=np.float64).copy()
        distortion = np.asarray(data["distortion_coefficients"], dtype=np.float64)
    except (KeyError, TypeError, ValueError):
        return None, None
    if original_width <= 0 or original_height <= 0 or width <= 0 or height <= 0:
        return None, None
    matrix[0, 0] *= float(width) / original_width
    matrix[0, 2] *= float(width) / original_width
    matrix[1, 1] *= float(height) / original_height
    matrix[1, 2] *= float(height) / original_height
    return matrix, distortion


def distort_undistorted_points(calibration, points, image_size):
    """Map undistorted pixel coordinates back to the raw camera image plane."""
    if cv2 is None or np is None or not points:
        return [list(point) for point in (points or [])]
    if not _calibration_vision_usable(calibration):
        return [list(point) for point in points]
    width, height = int(image_size[0]), int(image_size[1])
    matrix, distortion = _scaled_camera_matrix(calibration, width, height)
    if matrix is None or distortion is None:
        return [list(point) for point in points]

    values = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    if fx <= 0 or fy <= 0:
        return values.tolist()
    normalized = np.column_stack(
        (
            (values[:, 0] - cx) / fx,
            (values[:, 1] - cy) / fy,
            np.ones(len(values), dtype=np.float64),
        )
    )
    projected, _ = cv2.projectPoints(
        normalized.reshape(-1, 1, 3),
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        matrix,
        distortion,
    )
    return projected.reshape(-1, 2).tolist()


def _project_line_document(calibration, document, image_size):
    if not document or not document.get("points"):
        return document
    projected = distort_undistorted_points(calibration, document["points"], image_size)
    if not projected:
        return document
    return {
        **document,
        "bottom_x": float(projected[-1][0]),
        "bottom_y": float(projected[-1][1]),
        "top_x": float(projected[0][0]),
        "top_y": float(projected[0][1]),
        "points": [[float(x), float(y)] for x, y in projected],
        "coordinate_space": "RAW_DISTORTED",
    }


def _copy_array(value):
    if value is None:
        return None
    try:
        return value.copy()
    except AttributeError:
        return value


def _snapshot_lane_history(controller):
    return {
        "width_profile": _copy_array(getattr(controller, "_previous_width_profile", None)),
        "width_y": _copy_array(getattr(controller, "_previous_width_y", None)),
        "left_coefficients": _copy_array(getattr(controller, "_previous_left_coefficients", None)),
        "right_coefficients": _copy_array(getattr(controller, "_previous_right_coefficients", None)),
        "frames_since_two_boundary": int(
            getattr(controller, "_frames_since_two_boundary", 1000)
        ),
    }


def _restore_rejected_history(controller, previous):
    controller._previous_width_profile = _copy_array(previous["width_profile"])
    controller._previous_width_y = _copy_array(previous["width_y"])
    controller._previous_left_coefficients = _copy_array(previous["left_coefficients"])
    controller._previous_right_coefficients = _copy_array(previous["right_coefficients"])
    # A frame still elapsed. Do not let a rejected two-line candidate reset the
    # one-side inference age to zero, otherwise a bad width profile can linger.
    controller._frames_since_two_boundary = min(
        1000,
        int(previous["frames_since_two_boundary"]) + 1,
    )


def _line_bottom_x(document):
    if not document:
        return None
    value = document.get("bottom_x")
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _result_signature(result):
    """Compact lane geometry signature in the undistorted processing plane."""
    if not isinstance(result, LaneResult) or not result.detected:
        return None
    width = float(result.lane_width_pixels or 0.0)
    if not math.isfinite(width) or width <= 8.0:
        return None
    center = _line_bottom_x(result.center_line)
    if center is None:
        left = _line_bottom_x(result.left_line)
        right = _line_bottom_x(result.right_line)
        if left is None or right is None:
            return None
        center = (left + right) * 0.5
    image_width = float((result.image_size or (640, 360))[0])
    lateral = result.lateral_error_normalized
    try:
        lateral = float(lateral)
    except (TypeError, ValueError):
        lateral = (image_width * 0.5 - center) / max(width * 0.5, 1.0)
    heading = float(result.heading_error_degrees or 0.0)
    top_width = float(result.lane_width_top_pixels or width)
    return {
        "lateral": lateral,
        "heading": heading,
        "width": width,
        "top_width": max(1.0, top_width),
        "center": center,
        "image_width": max(1.0, image_width),
    }


def _signature_delta(a, b):
    if a is None or b is None:
        return None
    width_ratio = max(a["width"], b["width"]) / max(1.0, min(a["width"], b["width"]))
    top_ratio = max(a["top_width"], b["top_width"]) / max(
        1.0, min(a["top_width"], b["top_width"])
    )
    center_delta = abs(a["center"] - b["center"]) / max(
        32.0, 0.5 * (a["width"] + b["width"])
    )
    return {
        "lateral": abs(a["lateral"] - b["lateral"]),
        "heading": abs(a["heading"] - b["heading"]),
        "width_ratio": width_ratio,
        "top_width_ratio": top_ratio,
        "center": center_delta,
    }


def _is_temporal_jump(stable, candidate):
    delta = _signature_delta(stable, candidate)
    if delta is None:
        return False
    extreme = (
        delta["lateral"] > 0.62
        or delta["heading"] > 24.0
        or delta["width_ratio"] > 1.70
        or delta["top_width_ratio"] > 1.90
        or delta["center"] > 0.62
    )
    moderate_count = sum(
        (
            delta["lateral"] > 0.34,
            delta["heading"] > 13.0,
            delta["width_ratio"] > 1.38,
            delta["top_width_ratio"] > 1.48,
            delta["center"] > 0.38,
        )
    )
    return bool(extreme or moderate_count >= 2)


def _pending_matches(previous_pending, candidate):
    delta = _signature_delta(previous_pending, candidate)
    if delta is None:
        return False
    return bool(
        delta["lateral"] <= 0.22
        and delta["heading"] <= 9.0
        and delta["width_ratio"] <= 1.26
        and delta["top_width_ratio"] <= 1.34
        and delta["center"] <= 0.24
    )


def _clear_pending(controller):
    controller._lane_pending_signature = None
    controller._lane_pending_count = 0


def _accept_temporal(controller, result, signature):
    controller._lane_stable_result = result
    controller._lane_stable_signature = signature
    controller._lane_stable_misses = 0
    _clear_pending(controller)
    return result


def _held_result(controller, reason):
    stable = getattr(controller, "_lane_stable_result", None)
    if not isinstance(stable, LaneResult):
        return None
    misses = int(getattr(controller, "_lane_stable_misses", 0)) + 1
    controller._lane_stable_misses = misses
    if misses > 2:
        return None
    return replace(
        stable,
        detected=False,
        confidence=min(float(stable.confidence), 0.49),
        correction_angle_degrees=0.0,
        error=reason,
        inferred_left=False,
        inferred_right=False,
    )


def _temporal_guard(controller, candidate, history_before):
    """Reject isolated lane jumps without preventing deliberate re-acquisition."""
    stable_signature = getattr(controller, "_lane_stable_signature", None)
    candidate_signature = _result_signature(candidate)

    if candidate_signature is None:
        _clear_pending(controller)
        held = _held_result(controller, "TEMPORAL_LANE_LOST_HELD")
        return held if held is not None else candidate

    if stable_signature is None:
        return _accept_temporal(controller, candidate, candidate_signature)

    if not _is_temporal_jump(stable_signature, candidate_signature):
        return _accept_temporal(controller, candidate, candidate_signature)

    # The base detector already updated its width/curve history before returning
    # this candidate. Roll that mutation back so one false frame cannot become
    # the prior used for subsequent one-boundary inference.
    _restore_rejected_history(controller, history_before)

    pending = getattr(controller, "_lane_pending_signature", None)
    if pending is not None and _pending_matches(pending, candidate_signature):
        controller._lane_pending_count = int(
            getattr(controller, "_lane_pending_count", 0)
        ) + 1
    else:
        controller._lane_pending_signature = candidate_signature
        controller._lane_pending_count = 1

    # Two mutually consistent frames are enough to re-acquire a genuinely new
    # lane. At ~3-8 FPS this adds only a short delay while blocking one-frame
    # floor-edge/lighting glitches.
    if int(getattr(controller, "_lane_pending_count", 0)) >= 2:
        _clear_pending(controller)
        controller._lane_stable_result = candidate
        controller._lane_stable_signature = candidate_signature
        controller._lane_stable_misses = 0
        # Candidate history was rolled back above. Rebuild the width profile from
        # accepted geometry so one-side inference is coherent after re-acquire.
        if candidate.left_line and candidate.right_line:
            left = candidate.left_line.get("points") or []
            right = candidate.right_line.get("points") or []
            if len(left) == len(right) and len(left) >= 2 and np is not None:
                widths = np.asarray(
                    [float(r[0]) - float(l[0]) for l, r in zip(left, right)],
                    dtype=np.float64,
                )
                ys = np.asarray([float(point[1]) for point in left], dtype=np.float64)
                if np.all(np.isfinite(widths)) and np.all(widths > 8.0):
                    controller._previous_width_profile = widths.copy()
                    controller._previous_width_y = ys.copy()
                    controller._frames_since_two_boundary = 0
        return candidate

    held = _held_result(controller, "TEMPORAL_OUTLIER_HELD")
    return held if held is not None else replace(
        candidate,
        detected=False,
        confidence=min(float(candidate.confidence), 0.49),
        correction_angle_degrees=0.0,
        error="TEMPORAL_OUTLIER_REJECTED",
    )


def _reset_with_temporal_state(self):
    result = _ORIGINAL_RESET(self)
    self._lane_stable_result = None
    self._lane_stable_signature = None
    self._lane_stable_misses = 0
    self._lane_pending_signature = None
    self._lane_pending_count = 0
    return result


def _analyze_image_hardened(self, image):
    history_before = _snapshot_lane_history(self)
    candidate = _ORIGINAL_ANALYZE_IMAGE(self, image)
    result = _temporal_guard(self, candidate, history_before)

    calibration = getattr(self, "camera_calibration", None)
    if (
        not isinstance(result, LaneResult)
        or not _calibration_vision_usable(calibration)
        or not result.image_size
    ):
        return result
    image_size = result.image_size
    return replace(
        result,
        left_line=_project_line_document(calibration, result.left_line, image_size),
        right_line=_project_line_document(calibration, result.right_line, image_size),
        center_line=_project_line_document(calibration, result.center_line, image_size),
    )


def install_lane_geometry_hardening():
    global _INSTALLED, _ORIGINAL_ANALYZE_IMAGE, _ORIGINAL_RESET
    if _INSTALLED:
        return
    _ORIGINAL_ANALYZE_IMAGE = LaneController.analyze_image
    _ORIGINAL_RESET = LaneController.reset
    LaneController._robust_fit = staticmethod(_adaptive_robust_fit)
    LaneController.analyze_image = _analyze_image_hardened
    LaneController.reset = _reset_with_temporal_state
    _INSTALLED = True


__all__ = [
    "distort_undistorted_points",
    "install_lane_geometry_hardening",
]
