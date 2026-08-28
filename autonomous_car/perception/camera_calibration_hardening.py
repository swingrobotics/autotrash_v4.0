"""PhotonVision/OpenCV-aligned camera intrinsic calibration hardening.

The production path keeps SWING's existing ChArUco capture/session API, but
applies the quality rules that matter for real-time vision:

* 12 views are the bare minimum; 50+ diverse views are the quality target.
* Use OpenCV's canonical ChArUco board corner table, including explicit support
  for pre-4.6 legacy board patterns.
* Persist/report per-view and mean reprojection error and reject calibration
  from control geometry when the mean error exceeds one pixel.
* Treat calibration as resolution/aspect-ratio specific. Uniformly scaled
  resolutions are supported, but a different aspect ratio is not silently used.
* Cache OpenCV undistortion maps per resolution and use cv2.remap() so live UFLD
  does not rebuild the lens model for every frame.

PhotonVision is used as an engineering reference only; this module implements
SWING's own runtime contract around OpenCV.
"""

import math
import threading

try:
    import numpy as np
except ImportError:  # pragma: no cover - base module handles missing NumPy
    np = None

from . import camera_calibration as base


_INSTALLED = False
_ORIGINAL_NORMALIZE = base.normalize_charuco_config
_ORIGINAL_CALIBRATE = base.calibrate_charuco_samples
_ORIGINAL_SNAPSHOT = base.CameraCalibration.snapshot

MINIMUM_SAMPLES = 12
RECOMMENDED_SAMPLES = 50
MEAN_REPROJECTION_LIMIT_PX = 1.0
GOOD_REPROJECTION_LIMIT_PX = 0.5
MINIMUM_COVERAGE_CELLS = 4
GOOD_COVERAGE_CELLS = 6
ASPECT_RATIO_TOLERANCE = 0.01


def _normalize_charuco_config(config=None):
    raw = dict(config or {})
    normalized = _ORIGINAL_NORMALIZE(raw)
    # PhotonVision documents 12 images as the bare minimum and recommends more
    # than 50 for a high-quality calibration. Keep the solve minimum at 12 but
    # force older persisted sessions to surface the current quality target.
    normalized["required_samples"] = max(
        MINIMUM_SAMPLES,
        int(normalized.get("required_samples") or MINIMUM_SAMPLES),
    )
    normalized["recommended_samples"] = max(
        RECOMMENDED_SAMPLES,
        int(normalized.get("recommended_samples") or RECOMMENDED_SAMPLES),
    )
    # OpenCV 4.6 changed the generated ChArUco pattern for boards with an even
    # number of rows. Current PhotonVision boards use the new pattern; this flag
    # preserves an explicit compatibility path for older printed boards.
    normalized["legacy_pattern"] = bool(raw.get("legacy_pattern", False))
    return normalized


def _create_charuco_board(config=None):
    config = _normalize_charuco_config(config)
    dictionary = base._charuco_dictionary(config["dictionary"])
    size = (config["squares_x"], config["squares_y"])
    try:
        board = base.cv2.aruco.CharucoBoard(
            size,
            config["square_length_m"],
            config["marker_length_m"],
            dictionary,
        )
    except (AttributeError, TypeError):
        if not hasattr(base.cv2.aruco, "CharucoBoard_create"):
            raise RuntimeError("This OpenCV build does not provide ChArUco boards")
        board = base.cv2.aruco.CharucoBoard_create(
            size[0],
            size[1],
            config["square_length_m"],
            config["marker_length_m"],
            dictionary,
        )
    if config["legacy_pattern"]:
        if not hasattr(board, "setLegacyPattern"):
            raise RuntimeError(
                "This OpenCV build cannot use pre-4.6 ChArUco legacy patterns"
            )
        board.setLegacyPattern(True)
    return board, dictionary, config


def _charuco_object_points(ids, config):
    """Use the board's canonical corner table instead of recreating ID geometry."""
    if np is None:
        raise RuntimeError("NumPy is required")
    board, _, _ = _create_charuco_board(config)
    if not hasattr(board, "getChessboardCorners"):
        raise RuntimeError("OpenCV ChArUco board does not expose chessboard corners")
    board_points = np.asarray(board.getChessboardCorners(), dtype=np.float32).reshape(-1, 3)
    indices = np.asarray(ids, dtype=np.int32).reshape(-1)
    if len(indices) == 0:
        return np.empty((0, 3), dtype=np.float32)
    if np.any(indices < 0) or np.any(indices >= len(board_points)):
        raise ValueError("ChArUco corner id is outside the board corner table")
    return board_points[indices].copy()


def _calibrate_charuco_samples(samples, config=None, minimum_samples=None):
    """Add PhotonVision-style reprojection diagnostics to the OpenCV solve."""
    result = _ORIGINAL_CALIBRATE(samples, config, minimum_samples)
    per_view = [
        float(value)
        for value in (result.get("per_view_rms") or [])
        if math.isfinite(float(value))
    ]
    mean_error = (
        float(sum(per_view) / len(per_view))
        if per_view
        else float(result.get("rms_error") or float("inf"))
    )
    max_error = max(per_view) if per_view else mean_error
    sample_count = int(result.get("samples") or 0)
    result.update(
        calibration_method="OPENCV_PINHOLE_CHARUCO",
        mean_reprojection_error_px=float(mean_error),
        max_reprojection_error_px=float(max_error),
        per_view_reprojection_error_px=per_view,
        photonvision_quality={
            "minimum_samples": MINIMUM_SAMPLES,
            "recommended_samples": RECOMMENDED_SAMPLES,
            "mean_reprojection_limit_px": MEAN_REPROJECTION_LIMIT_PX,
            "minimum_samples_met": sample_count >= MINIMUM_SAMPLES,
            "recommended_samples_met": sample_count >= RECOMMENDED_SAMPLES,
            "reprojection_error_ok": bool(
                math.isfinite(mean_error)
                and mean_error <= MEAN_REPROJECTION_LIMIT_PX
            ),
        },
    )
    return result


def _error_metric(data):
    if not data:
        return None
    value = data.get("mean_reprojection_error_px")
    if value is None:
        value = data.get("rms_error")
    if value is None:
        return None
    try:
        value = float(value)
    except (TypeError, ValueError):
        return float("inf")
    return value


def _coverage_cells(data):
    if not data:
        return None
    session = data.get("session")
    if not isinstance(session, dict):
        return None
    grid = session.get("coverage_grid")
    if not isinstance(grid, (list, tuple)) or len(grid) != 9:
        return None
    try:
        return sum(1 for count in grid if int(count) > 0)
    except (TypeError, ValueError):
        return None


def _vision_usable(self):
    data = getattr(self, "data", None)
    if not data:
        return False
    try:
        matrix = data["camera_matrix"]
        fx = float(matrix[0][0])
        fy = float(matrix[1][1])
        cx = float(matrix[0][2])
        cy = float(matrix[1][2])
        width, height = [int(value) for value in data["image_size"]]
        distortion = [float(value) for value in data["distortion_coefficients"]]
    except (KeyError, TypeError, ValueError, IndexError):
        return False
    finite_intrinsics = all(math.isfinite(value) for value in (fx, fy, cx, cy))
    if width <= 0 or height <= 0 or not finite_intrinsics:
        return False
    if fx <= 0 or fy <= 0 or len(distortion) < 4:
        return False
    if not all(math.isfinite(value) for value in distortion):
        return False
    # Optical center should be on or reasonably close to the physical sensor.
    # Keep a small margin for real lenses/cropping without accepting nonsense.
    if not (-0.10 * width <= cx <= 1.10 * width):
        return False
    if not (-0.10 * height <= cy <= 1.10 * height):
        return False

    error = _error_metric(data)
    if error is not None and (
        not math.isfinite(error) or error > MEAN_REPROJECTION_LIMIT_PX
    ):
        return False

    if str(data.get("source") or "").upper() == "CHARUCO":
        sample_count = data.get("samples")
        if sample_count is not None:
            try:
                if int(sample_count) < MINIMUM_SAMPLES:
                    return False
            except (TypeError, ValueError):
                return False
        occupied = _coverage_cells(data)
        # Live sessions know their image-position distribution. Do not apply a
        # center-only calibration to lane/control geometry even if RMS is low.
        if occupied is not None and occupied < MINIMUM_COVERAGE_CELLS:
            return False
    return True


def _scaled_matrix(data, width, height):
    if np is None:
        raise RuntimeError("NumPy is required")
    original_width, original_height = [int(value) for value in data["image_size"]]
    source_ratio = original_width / max(1.0, float(original_height))
    target_ratio = width / max(1.0, float(height))
    relative_ratio_error = abs(target_ratio - source_ratio) / max(1e-9, source_ratio)
    if relative_ratio_error > ASPECT_RATIO_TOLERANCE:
        raise ValueError(
            "CAMERA_CALIBRATION_ASPECT_RATIO_MISMATCH:"
            f"calibrated={original_width}x{original_height},frame={width}x{height}"
        )
    matrix = np.asarray(data["camera_matrix"], dtype=np.float64).copy()
    scale_x = width / max(1.0, float(original_width))
    scale_y = height / max(1.0, float(original_height))
    matrix[0, 0] *= scale_x
    matrix[0, 2] *= scale_x
    matrix[1, 1] *= scale_y
    matrix[1, 2] *= scale_y
    return matrix


def _undistort_if_usable(self, image):
    """Fast undistortion using cached rectify maps instead of cv2.undistort()."""
    if not self.vision_usable or image is None:
        return image
    if base.cv2 is None or np is None:
        return image
    try:
        height, width = image.shape[:2]
        data = self.data
        matrix = _scaled_matrix(data, width, height)
        distortion = np.asarray(
            data["distortion_coefficients"], dtype=np.float64
        ).reshape(-1)
        key = (
            int(width),
            int(height),
            tuple(float(value) for value in matrix.reshape(-1)),
            tuple(float(value) for value in distortion),
        )
        lock = getattr(self, "_swing_undistort_lock", None)
        if lock is None:
            lock = threading.RLock()
            self._swing_undistort_lock = lock
        with lock:
            cache = getattr(self, "_swing_undistort_maps", None)
            if not isinstance(cache, dict):
                cache = {}
                self._swing_undistort_maps = cache
            maps = cache.get(key)
            if maps is None:
                map_x, map_y = base.cv2.initUndistortRectifyMap(
                    matrix,
                    distortion,
                    None,
                    matrix,
                    (int(width), int(height)),
                    base.cv2.CV_32FC1,
                )
                maps = (map_x, map_y)
                # A camera normally uses one capture size, but permit a few
                # uniformly scaled preview sizes without unbounded RAM growth.
                if len(cache) >= 4:
                    cache.pop(next(iter(cache)))
                cache[key] = maps
        self._swing_last_undistort_error = None
        return base.cv2.remap(
            image,
            maps[0],
            maps[1],
            interpolation=base.cv2.INTER_LINEAR,
            borderMode=base.cv2.BORDER_CONSTANT,
        )
    except Exception as error:
        # A bad/mismatched calibration must never corrupt the vision frame. The
        # caller receives the original frame and status exposes the reason.
        self._swing_last_undistort_error = f"{type(error).__name__}: {error}"
        return image


def _snapshot_with_quality(self):
    snapshot = _ORIGINAL_SNAPSHOT(self)
    data = getattr(self, "data", None) or {}
    error = _error_metric(data)
    occupied = _coverage_cells(data)
    sample_count = int(data.get("samples") or 0) if data else 0

    vertical_fov = None
    principal_point = None
    principal_offset = None
    try:
        width, height = [float(value) for value in data["image_size"]]
        matrix = data["camera_matrix"]
        fx = float(matrix[0][0])
        fy = float(matrix[1][1])
        cx = float(matrix[0][2])
        cy = float(matrix[1][2])
        vertical_fov = math.degrees(2.0 * math.atan(height / (2.0 * fy)))
        principal_point = [cx, cy]
        principal_offset = [
            (cx - width / 2.0) / max(1.0, width),
            (cy - height / 2.0) / max(1.0, height),
        ]
    except (KeyError, TypeError, ValueError, IndexError, ZeroDivisionError):
        pass

    if not snapshot.get("calibrated"):
        quality = "UNCALIBRATED"
    elif not self.vision_usable:
        quality = "REJECTED"
    else:
        good_error = error is None or error <= GOOD_REPROJECTION_LIMIT_PX
        good_samples = sample_count >= RECOMMENDED_SAMPLES
        good_coverage = occupied is None or occupied >= GOOD_COVERAGE_CELLS
        quality = "GOOD" if good_error and good_samples and good_coverage else "ACCEPTABLE"

    snapshot.update(
        vision_usable=bool(self.vision_usable),
        quality=quality,
        rms_limit_pixels=MEAN_REPROJECTION_LIMIT_PX,
        mean_reprojection_error_px=(None if error is None else float(error)),
        max_reprojection_error_px=data.get("max_reprojection_error_px"),
        recommended_charuco_samples=RECOMMENDED_SAMPLES,
        minimum_charuco_samples=MINIMUM_SAMPLES,
        recommended_samples_met=sample_count >= RECOMMENDED_SAMPLES,
        occupied_coverage_cells=occupied,
        minimum_coverage_cells=MINIMUM_COVERAGE_CELLS,
        vertical_fov_degrees=vertical_fov,
        principal_point_px=principal_point,
        principal_point_offset_ratio=principal_offset,
        undistort_runtime="CACHED_REMAP",
        undistort_error=getattr(self, "_swing_last_undistort_error", None),
    )
    return snapshot


def install_camera_calibration_hardening():
    global _INSTALLED
    if _INSTALLED:
        return
    base.normalize_charuco_config = _normalize_charuco_config
    base.create_charuco_board = _create_charuco_board
    base._charuco_object_points = _charuco_object_points
    base.calibrate_charuco_samples = _calibrate_charuco_samples
    base.CameraCalibration.vision_usable = property(_vision_usable)
    base.CameraCalibration.undistort = _undistort_if_usable
    base.CameraCalibration.snapshot = _snapshot_with_quality
    _INSTALLED = True


__all__ = [
    "ASPECT_RATIO_TOLERANCE",
    "GOOD_REPROJECTION_LIMIT_PX",
    "MEAN_REPROJECTION_LIMIT_PX",
    "MINIMUM_COVERAGE_CELLS",
    "MINIMUM_SAMPLES",
    "RECOMMENDED_SAMPLES",
    "install_camera_calibration_hardening",
]
