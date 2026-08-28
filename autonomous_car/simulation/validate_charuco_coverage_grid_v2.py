"""Regression for ChArUco 3x3 sensor coverage accounting."""

import tempfile

from autonomous_car.perception.camera_calibration import CameraCalibration
from autonomous_car.perception.camera_calibration_session import CameraCalibrationSession


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    with tempfile.TemporaryDirectory() as directory:
        session = CameraCalibrationSession(CameraCalibration(), directory)

        image_size = [1280, 720]
        # The board centroid is still in the middle third vertically, but its
        # detected corners extend into all three bottom cells. This reproduces
        # the old dashboard failure where centroid-only accounting could miss
        # the bottom row even though valid calibration observations exist there.
        #
        # y=350 is in the middle third (240..479), y=550 is in the bottom third
        # (480..719), and the mean y=450 remains in the middle third.
        corners = [
            [100, 350], [640, 350], [1180, 350],
            [100, 550], [640, 550], [1180, 550],
        ]
        cells = session._coverage_cells(corners, image_size)
        _require(6 in cells and 7 in cells and 8 in cells, f"bottom row missing: {cells}")

        centroid = [640 / 1280, sum(point[1] for point in corners) / len(corners) / 720]
        centroid_cell = session._coverage_cell(centroid)
        _require(centroid_cell == 4, f"test no longer reproduces centroid limitation: {centroid_cell}")

        grid = session._coverage_grid(
            [
                {
                    "charuco_corners": corners,
                    "image_size": image_size,
                    "centroid_normalized": centroid,
                }
            ]
        )
        _require(grid[6] == 1 and grid[7] == 1 and grid[8] == 1, f"bottom grid not counted: {grid}")
        _require(sum(1 for count in grid if count) >= 6, f"corner coverage unexpectedly narrow: {grid}")

    print("ChArUco 3x3 coverage-grid regression: PASS")
    print({"coverage_cells": cells, "centroid_cell": centroid_cell, "grid": grid})


if __name__ == "__main__":
    main()
