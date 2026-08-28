"""Prefer real colored lane markings over unrelated generic image edges.

BLACK/YELLOW/WHITE masks are semantic lane evidence. EDGE is intentionally only
an appearance fallback for faded/neutral markings. The base detector previously
ORed them with equal weight, allowing a long floor seam or furniture edge to win
the sliding-window histogram even when black tape was visible.

When a half-image has enough semantic color support, this layer keeps generic
edges only in a corridor around that colored marking. If color support is weak,
the original EDGE fallback remains unchanged for outdoor faded lanes.
"""

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None
    np = None

from .lane_controller import LaneController


_INSTALLED = False
_ORIGINAL_CANDIDATE_MASK = None


def _semantic_color_mask(source_masks, shape):
    if cv2 is None or np is None:
        return None
    color = np.zeros(shape, dtype=np.uint8)
    for key in ("BLACK", "YELLOW", "WHITE"):
        value = source_masks.get(key) if isinstance(source_masks, dict) else None
        if value is not None and getattr(value, "shape", None) == shape:
            color = cv2.bitwise_or(color, value)
    return color


def _prefer_semantic_boundaries(binary, source_masks):
    if cv2 is None or np is None or binary is None:
        return binary
    if getattr(binary, "ndim", 0) != 2:
        return binary
    h, w = binary.shape[:2]
    if h < 20 or w < 40:
        return binary

    color = _semantic_color_mask(source_masks, binary.shape)
    if color is None:
        return binary

    # Only semantic pixels already accepted by the base detector's ground ROI
    # count as seeds. This prevents bright/dark objects outside the road polygon
    # from suppressing useful EDGE fallback candidates.
    seeds = cv2.bitwise_and(color, binary)
    preferred = binary.copy()
    center = w // 2
    minimum_pixels = max(70, int(h * w * 0.00065))
    kernel_width = max(11, int(round(w * 0.035)))
    if kernel_width % 2 == 0:
        kernel_width += 1
    kernel_height = max(7, int(round(h * 0.045)))
    if kernel_height % 2 == 0:
        kernel_height += 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (kernel_width, kernel_height),
    )

    for x0, x1 in ((0, center), (center, w)):
        semantic = seeds[:, x0:x1]
        if int(np.count_nonzero(semantic)) < minimum_pixels:
            continue
        corridor = cv2.dilate(semantic, kernel, iterations=1)
        original_half = binary[:, x0:x1]
        supported = cv2.bitwise_and(original_half, corridor)
        # Slightly thicken the semantic seed so a narrow tape/paint stripe has
        # enough histogram weight without shifting its row-wise median center.
        semantic_boost = cv2.dilate(
            semantic,
            np.ones((3, 3), dtype=np.uint8),
            iterations=1,
        )
        preferred[:, x0:x1] = cv2.bitwise_or(supported, semantic_boost)

    return preferred


def _candidate_mask_hardened(self, roi):
    binary, source_masks = _ORIGINAL_CANDIDATE_MASK(self, roi)
    return _prefer_semantic_boundaries(binary, source_masks), source_masks


def install_lane_candidate_hardening():
    global _INSTALLED, _ORIGINAL_CANDIDATE_MASK
    if _INSTALLED:
        return
    _ORIGINAL_CANDIDATE_MASK = LaneController._candidate_mask
    LaneController._candidate_mask = _candidate_mask_hardened
    _INSTALLED = True


__all__ = [
    "install_lane_candidate_hardening",
]
