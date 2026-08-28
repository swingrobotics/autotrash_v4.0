from .camera_calibration import CameraCalibration, calibrate_chessboard
from .camera_calibration_hardening import install_camera_calibration_hardening

# Apply the OpenCV/PhotonVision-aligned calibration contract before any runtime
# imports CameraCalibration or the live ChArUco session helpers.
install_camera_calibration_hardening()

from .object_detector import DetectedObject, ObjectDetector

__all__ = [
    "CameraCalibration",
    "DetectedObject",
    "ObjectDetector",
    "calibrate_chessboard",
    "install_camera_calibration_hardening",
]
