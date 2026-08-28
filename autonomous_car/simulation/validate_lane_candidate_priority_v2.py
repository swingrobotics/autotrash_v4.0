"""Regression for semantic lane-marking priority over unrelated EDGE fallback."""

import numpy as np

from autonomous_car.control.lane_candidate_hardening import (
    _prefer_semantic_boundaries,
)


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    height, width = 200, 640
    binary = np.zeros((height, width), dtype=np.uint8)
    black = np.zeros_like(binary)
    zero = np.zeros_like(binary)

    # Physical black tape on the left side: deliberately narrow.
    black[:, 148:153] = 255
    binary[:, 148:153] = 255

    # A stronger unrelated floor seam that would dominate a plain histogram.
    binary[:, 35:48] = 255

    # Right side has no semantic color support, so its EDGE fallback must remain.
    binary[:, 500:507] = 255

    preferred = _prefer_semantic_boundaries(
        binary,
        {
            "BLACK": black,
            "YELLOW": zero,
            "WHITE": zero,
            "EDGE": binary,
        },
    )

    physical_support = int(np.count_nonzero(preferred[:, 142:159]))
    false_left_support = int(np.count_nonzero(preferred[:, 30:55]))
    right_fallback_support = int(np.count_nonzero(preferred[:, 495:512]))

    _require(physical_support > 500, "semantic black tape was not preserved")
    _require(
        false_left_support < physical_support * 0.10,
        "unrelated left EDGE was not suppressed despite black-tape support",
    )
    _require(
        right_fallback_support >= height * 5,
        "EDGE fallback was incorrectly removed on a side without color support",
    )

    print("Lane semantic candidate priority regression: PASS")
    print(
        {
            "physical_support": physical_support,
            "false_left_support": false_left_support,
            "right_fallback_support": right_fallback_support,
        }
    )


if __name__ == "__main__":
    main()
