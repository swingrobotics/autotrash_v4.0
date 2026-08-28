"""Synthetic end-to-end regression for live ChArUco camera calibration."""

import os
import tempfile

from autonomous_car.perception.camera_calibration import (
    CameraCalibration,
    charuco_available,
    create_charuco_board,
    detect_charuco,
    normalize_charuco_config,
)
from autonomous_car.perception.camera_calibration_session import CameraCalibrationSession


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def _render_view(board_image, config, camera_matrix, rotation, translation, image_size):
    import cv2
    import numpy as np

    width, height = image_size
    board_width = config["squares_x"] * config["square_length_m"]
    board_height = config["squares_y"] * config["square_length_m"]
    object_corners = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [board_width, 0.0, 0.0],
            [board_width, board_height, 0.0],
            [0.0, board_height, 0.0],
        ],
        dtype=np.float32,
    )
    projected, _ = cv2.projectPoints(
        object_corners,
        np.asarray(rotation, dtype=np.float64),
        np.asarray(translation, dtype=np.float64),
        camera_matrix,
        np.zeros(5, dtype=np.float64),
    )
    destination = projected.reshape(-1, 2).astype(np.float32)
    source = np.asarray(
        [
            [0.0, 0.0],
            [board_image.shape[1] - 1.0, 0.0],
            [board_image.shape[1] - 1.0, board_image.shape[0] - 1.0],
            [0.0, board_image.shape[0] - 1.0],
        ],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(source, destination)
    canvas = np.full((height, width), 190, dtype=np.uint8)
    warped = cv2.warpPerspective(
        board_image,
        homography,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderValue=255,
    )
    mask = cv2.warpPerspective(
        np.full_like(board_image, 255),
        homography,
        (width, height),
        flags=cv2.INTER_NEAREST,
        borderValue=0,
    )
    canvas[mask > 0] = warped[mask > 0]
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def main():
    _require(charuco_available(), "cv2.aruco unavailable; install OpenCV contrib")

    import cv2
    import numpy as np

    config = normalize_charuco_config()
    board, _, config = create_charuco_board(config)
    _require(config["squares_x"] == 8 and config["squares_y"] == 8, "unexpected default board")
    _require(abs(config["square_length_m"] - 0.0254) < 1e-9, "unexpected square size")
    _require(abs(config["marker_length_m"] - 0.01905) < 1e-9, "unexpected marker size")
    _require(config["required_samples"] == 12, "PhotonVision minimum sample count changed")
    _require(config["recommended_samples"] >= 50, "PhotonVision recommended sample target missing")
    _require(config.get("legacy_pattern") is False, "current ChArUco pattern must default non-legacy")

    if hasattr(board, "generateImage"):
        board_image = board.generateImage((800, 800), marginSize=0, borderBits=1)
    else:
        board_image = board.draw((800, 800), marginSize=0, borderBits=1)

    image_size = (1280, 720)
    camera_matrix = np.asarray(
        [[820.0, 0.0, 640.0], [0.0, 815.0, 360.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    poses = []
    for index in range(16):
        rotation = [
            [-0.20, -0.12, -0.04, 0.05, 0.13, 0.20][index % 6],
            [-0.28, -0.16, 0.0, 0.17, 0.27][index % 5],
            [-0.25, -0.12, 0.0, 0.14, 0.24][(index * 2) % 5],
        ]
        translation = [
            [-0.14, -0.09, -0.04, 0.01, 0.06][index % 5],
            [-0.12, -0.08, -0.04, 0.0][(index * 3) % 4],
            [0.55, 0.62, 0.70, 0.78][index % 4],
        ]
        poses.append((rotation, translation))

    first = _render_view(
        board_image,
        config,
        camera_matrix,
        poses[0][0],
        poses[0][1],
        image_size,
    )
    detection = detect_charuco(first, config)
    _require(detection["valid"], f"synthetic ChArUco board not detected: {detection}")
    _require(detection["detected_corners"] >= 40, f"too few corners: {detection}")

    with tempfile.TemporaryDirectory() as directory:
        calibration = CameraCalibration(os.path.join(directory, "camera-calibration.json"))
        session = CameraCalibrationSession(
            calibration,
            os.path.join(directory, "charuco-samples"),
        )
        for sequence, (rotation, translation) in enumerate(poses[:14]):
            image = _render_view(
                board_image,
                config,
                camera_matrix,
                rotation,
                translation,
                image_size,
            )
            encoded, jpeg = cv2.imencode(
                ".jpg",
                image,
                [int(cv2.IMWRITE_JPEG_QUALITY), 95],
            )
            _require(encoded, "could not encode synthetic calibration image")
            session.capture(jpeg.tobytes(), sequence)

        before = session.snapshot()
        _require(before["sample_count"] == 14, f"sample capture failed: {before}")
        _require(before["ready_to_calibrate"], f"session not ready: {before}")
        _require(before["recommended_samples"] >= 50, f"recommended sample target lost: {before}")
        solved = session.solve()
        result = solved["calibration"]
        _require(result["calibrated"], f"calibration not saved: {solved}")
        _require(result["source"] == "CHARUCO", f"wrong calibration source: {result}")
        _require((result["rms_error"] or 99.0) < 0.75, f"RMS too high: {result}")
        _require(result["samples"] >= 12, f"too few samples used: {result}")
        _require(result["vision_usable"], f"good calibration rejected: {result}")
        _require(result["quality"] in {"GOOD", "ACCEPTABLE"}, f"bad quality label: {result}")
        _require(
            abs((result["horizontal_fov_degrees"] or 0.0) - 76.0) < 3.0,
            f"unexpected calibrated horizontal FOV: {result}",
        )
        undistorted = calibration.undistort(first)
        _require(undistorted.shape == first.shape, "undistort changed image dimensions")

        # A saved calibration with >1px RMS remains inspectable but must not be
        # allowed to distort lane/control geometry.
        rejected = CameraCalibration(os.path.join(directory, "rejected-calibration.json"))
        rejected.save(
            {
                "schema": "camera_calibration_v2",
                "source": "CHARUCO",
                "image_size": list(image_size),
                "camera_matrix": camera_matrix.tolist(),
                "distortion_coefficients": [-0.2, 0.04, 0.0, 0.0, 0.0],
                "rms_error": 1.25,
                "samples": 20,
            }
        )
        rejected_snapshot = rejected.snapshot()
        _require(rejected_snapshot["calibrated"], "rejected calibration should remain inspectable")
        _require(not rejected_snapshot["vision_usable"], "high-RMS calibration reached vision")
        _require(rejected_snapshot["quality"] == "REJECTED", f"wrong reject label: {rejected_snapshot}")
        _require(rejected.undistort(first) is first, "rejected calibration modified image")

    final_source = open("server_v2_final.py", encoding="utf-8").read()
    panel_source = open("camera_calibration_panel.py", encoding="utf-8").read()
    requirements = open("requirements-autonomy.txt", encoding="utf-8").read()
    _require("CAMERA_CALIBRATION_PANEL" in final_source, "calibration panel is not injected")
    _require('/api/camera/calibration/preview' in final_source, "preview endpoint missing")
    _require('/api/camera/calibration/capture' in final_source, "capture endpoint missing")
    _require('/api/camera/calibration/solve' in final_source, "solve endpoint missing")
    _require("CHARUCO LOCK" in panel_source, "live ChArUco overlay UI missing")
    _require(
        "opencv-contrib-python-headless==4.12.0.88" in requirements,
        "OpenCV contrib runtime dependency missing",
    )
    _require(
        "opencv-python-headless==4.12.0.88" not in requirements,
        "conflicting non-contrib OpenCV runtime dependency remains",
    )

    print("ChArUco camera calibration V2 regression: PASS")
    print(
        {
            "detected_corners": detection["detected_corners"],
            "sample_count": 14,
            "recommended_samples": config["recommended_samples"],
            "rms_error": result["rms_error"],
            "horizontal_fov_degrees": result["horizontal_fov_degrees"],
            "vision_usable": result["vision_usable"],
        }
    )


if __name__ == "__main__":
    main()
