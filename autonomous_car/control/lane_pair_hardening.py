"""Pair-wise lane-boundary hardening.

The base detector finds left/right boundaries independently. This layer rejects
the common failure mode where one side jumps to a floor seam, shadow, curb, or
other strong edge while the opposite side remains on the physical lane.

It runs after lane_geometry_hardening so it sees the exact result used by the
dashboard/control path. Rejected pair candidates roll back both the detector's
width history and the temporal guard state, preventing one bad boundary from
becoming the next frame's prior.
"""

from dataclasses import replace
import math

from .lane_controller import LaneController, LaneResult


_INSTALLED = False
_ORIGINAL_ANALYZE_IMAGE = None
_ORIGINAL_RESET = None

_STATE_ATTRS = (
    "_previous_width_profile",
    "_previous_width_y",
    "_previous_left_coefficients",
    "_previous_right_coefficients",
    "_frames_since_two_boundary",
    "_lane_stable_result",
    "_lane_stable_signature",
    "_lane_stable_misses",
    "_lane_pending_signature",
    "_lane_pending_count",
)


def _copy_value(value):
    if value is None:
        return None
    try:
        return value.copy()
    except AttributeError:
        return value


def _snapshot_state(controller):
    return {
        name: _copy_value(getattr(controller, name, None))
        for name in _STATE_ATTRS
    }


def _restore_state(controller, snapshot):
    for name, value in snapshot.items():
        if value is not None or hasattr(controller, name):
            setattr(controller, name, _copy_value(value))


def _line_points(document):
    if not isinstance(document, dict):
        return []
    values = document.get("points")
    if not isinstance(values, list):
        return []
    points = []
    for value in values:
        if not isinstance(value, (list, tuple)) or len(value) < 2:
            continue
        try:
            x, y = float(value[0]), float(value[1])
        except (TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    return points


def _sample_xs(document, count=7):
    points = _line_points(document)
    if len(points) < 2:
        return None
    count = max(3, int(count))
    result = []
    last = len(points) - 1
    for index in range(count):
        position = index * last / (count - 1)
        low = int(math.floor(position))
        high = min(last, low + 1)
        fraction = position - low
        x = points[low][0] * (1.0 - fraction) + points[high][0] * fraction
        result.append(float(x))
    return result


def _line_bow(document):
    xs = _sample_xs(document, 5)
    if xs is None:
        return None
    chord_mid = 0.5 * (xs[0] + xs[-1])
    return float(xs[2] - chord_mid)


def _pair_geometry(result):
    if not isinstance(result, LaneResult):
        return None
    left = _sample_xs(result.left_line)
    right = _sample_xs(result.right_line)
    if left is None or right is None or len(left) != len(right):
        return None
    widths = [r - l for l, r in zip(left, right)]
    if any((not math.isfinite(width) or width <= 8.0) for width in widths):
        return None
    bottom_width = widths[-1]
    left_bow = _line_bow(result.left_line)
    right_bow = _line_bow(result.right_line)
    if left_bow is None or right_bow is None:
        return None
    return {
        "left": left,
        "right": right,
        "widths": widths,
        "bottom_width": float(bottom_width),
        "top_width": float(widths[0]),
        "left_bow": float(left_bow),
        "right_bow": float(right_bow),
    }


def _intrinsic_pair_inconsistent(result):
    geometry = _pair_geometry(result)
    if geometry is None:
        return False
    scale = max(40.0, geometry["bottom_width"])
    left_bow = geometry["left_bow"] / scale
    right_bow = geometry["right_bow"] / scale
    bow_mismatch = abs(left_bow - right_bow)

    # A true road curve normally bends both physical boundaries coherently.
    # One large bow with an almost straight opposite side is characteristic of
    # an independently selected floor seam/shadow edge.
    unilateral_bow = (
        max(abs(left_bow), abs(right_bow)) > 0.075
        and min(abs(left_bow), abs(right_bow)) < 0.030
        and bow_mismatch > 0.075
    )
    opposite_bow = (
        abs(left_bow) > 0.050
        and abs(right_bow) > 0.050
        and left_bow * right_bow < 0.0
        and bow_mismatch > 0.12
    )

    # Lane width may change with perspective, but it should not bulge sharply
    # only in the middle of the image.
    widths = geometry["widths"]
    chord_widths = [
        widths[0] + (widths[-1] - widths[0]) * index / (len(widths) - 1)
        for index in range(len(widths))
    ]
    width_bulge = max(
        abs(width - expected) / scale
        for width, expected in zip(widths, chord_widths)
    )
    implausible_width_bulge = width_bulge > 0.12

    return bool(unilateral_bow or opposite_bow or implausible_width_bulge)


def _side_deltas(stable, candidate):
    stable_geometry = _pair_geometry(stable)
    candidate_geometry = _pair_geometry(candidate)
    if stable_geometry is None or candidate_geometry is None:
        return None
    scale = max(
        40.0,
        0.5
        * (
            stable_geometry["bottom_width"]
            + candidate_geometry["bottom_width"]
        ),
    )
    left = sum(
        abs(a - b)
        for a, b in zip(stable_geometry["left"], candidate_geometry["left"])
    ) / (len(stable_geometry["left"]) * scale)
    right = sum(
        abs(a - b)
        for a, b in zip(stable_geometry["right"], candidate_geometry["right"])
    ) / (len(stable_geometry["right"]) * scale)
    return float(left), float(right)


def _unilateral_jump(stable, candidate):
    deltas = _side_deltas(stable, candidate)
    if deltas is None:
        return False
    small = min(deltas)
    large = max(deltas)
    return bool(
        large > 0.135
        and small < 0.060
        and large / max(0.020, small) > 2.6
    )


def _pending_pair_matches(previous, candidate):
    if previous is None:
        return False
    deltas = _side_deltas(previous, candidate)
    if deltas is None:
        return False
    return max(deltas) <= 0.065


def _clear_pair_pending(controller):
    controller._lane_pair_pending_result = None
    controller._lane_pair_pending_count = 0


def _accept_pair(controller, result):
    controller._lane_pair_stable_result = result
    controller._lane_pair_reject_count = 0
    _clear_pair_pending(controller)
    return result


def _rejected_result(controller, candidate, reason):
    count = int(getattr(controller, "_lane_pair_reject_count", 0)) + 1
    controller._lane_pair_reject_count = count
    stable = getattr(controller, "_lane_pair_stable_result", None)
    if isinstance(stable, LaneResult) and count <= 2:
        return replace(
            stable,
            detected=False,
            confidence=min(float(stable.confidence), 0.49),
            correction_angle_degrees=0.0,
            error=reason + "_HELD",
            inferred_left=False,
            inferred_right=False,
        )
    return replace(
        candidate,
        detected=False,
        confidence=min(float(candidate.confidence), 0.35),
        correction_angle_degrees=0.0,
        error=reason,
        inferred_left=False,
        inferred_right=False,
    )


def _pair_guard(controller, candidate, state_before):
    if not isinstance(candidate, LaneResult):
        return candidate

    # Existing temporal/lost-lane states already failed safe. Do not reinterpret
    # them as a new pair candidate.
    if not candidate.detected:
        return candidate

    if _intrinsic_pair_inconsistent(candidate):
        _restore_state(controller, state_before)
        _clear_pair_pending(controller)
        return _rejected_result(
            controller,
            candidate,
            "BOUNDARY_PAIR_INCONSISTENT",
        )

    stable = getattr(controller, "_lane_pair_stable_result", None)
    if not isinstance(stable, LaneResult):
        return _accept_pair(controller, candidate)

    if not _unilateral_jump(stable, candidate):
        return _accept_pair(controller, candidate)

    # One boundary moved far while its partner stayed stable. Require several
    # mutually consistent observations before treating that as a real topology
    # change. EDGE-only candidates need even more evidence because floor seams,
    # shadows and furniture edges all live in that fallback channel.
    pending = getattr(controller, "_lane_pair_pending_result", None)
    if _pending_pair_matches(pending, candidate):
        pending_count = int(
            getattr(controller, "_lane_pair_pending_count", 0)
        ) + 1
    else:
        pending_count = 1
    controller._lane_pair_pending_result = candidate
    controller._lane_pair_pending_count = pending_count

    marking = str(candidate.marking or "").upper()
    required = 5 if marking == "EDGE" else 3
    if pending_count >= required:
        return _accept_pair(controller, candidate)

    # The wrapped detector may already have accepted this frame and mutated both
    # lane-width history and its own temporal state. Undo those mutations until
    # the pair-level re-acquisition requirement is satisfied.
    pending_result = controller._lane_pair_pending_result
    pending_count = controller._lane_pair_pending_count
    _restore_state(controller, state_before)
    controller._lane_pair_pending_result = pending_result
    controller._lane_pair_pending_count = pending_count
    return _rejected_result(controller, candidate, "BOUNDARY_OUTLIER_REJECTED")


def _analyze_image_pair_hardened(self, image):
    state_before = _snapshot_state(self)
    result = _ORIGINAL_ANALYZE_IMAGE(self, image)
    return _pair_guard(self, result, state_before)


def _reset_pair_state(self):
    result = _ORIGINAL_RESET(self)
    self._lane_pair_stable_result = None
    self._lane_pair_pending_result = None
    self._lane_pair_pending_count = 0
    self._lane_pair_reject_count = 0
    return result


def install_lane_pair_hardening():
    global _INSTALLED, _ORIGINAL_ANALYZE_IMAGE, _ORIGINAL_RESET
    if _INSTALLED:
        return
    _ORIGINAL_ANALYZE_IMAGE = LaneController.analyze_image
    _ORIGINAL_RESET = LaneController.reset
    LaneController.analyze_image = _analyze_image_pair_hardened
    LaneController.reset = _reset_pair_state
    _INSTALLED = True


__all__ = [
    "install_lane_pair_hardening",
]
