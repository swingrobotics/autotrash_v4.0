import json
import math
import os

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


CHARUCO_DICTIONARIES = (
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_4X4_1000",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_5X5_1000",
    "DICT_6X6_250",
    "DICT_6X6_1000",
    "DICT_APRILTAG_16h5",
    "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10",
    "DICT_APRILTAG_36h11",
)


class CameraCalibration:
    def __init__(self, path=None):
        self.path = path
        self.data = None
        self.error = None
        if path:
            self.load()

    @property
    def calibrated(self):
        return self.data is not None

    def load(self):
        self.data = None
        self.error = None
        if not self.path or not os.path.isfile(self.path):
            return False
        try:
            with open(self.path, encoding="utf-8") as file:
                document = json.load(file)
            image_size = [int(value) for value in document["image_size"]]
            camera_matrix = [
                [float(value) for value in row]
                for row in document["camera_matrix"]
            ]
            distortion = [
                float(value)
                for value in document["distortion_coefficients"]
            ]
            if (
                len(image_size) != 2
                or any(value <= 0 for value in image_size)
                or len(camera_matrix) != 3
                or any(len(row) != 3 for row in camera_matrix)
            ):
                raise ValueError("Invalid camera calibration dimensions")
            self.data = {
                **document,
                "image_size": image_size,
                "camera_matrix": camera_matrix,
                "distortion_coefficients": distortion,
            }
            return True
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            self.error = str(error)
            return False

    def save(self, document):
        if not self.path:
            raise ValueError("Camera calibration path is not configured")
        directory = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(directory, exist_ok=True)
        temporary = f"{self.path}.tmp"
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, self.path)
        if not self.load():
            raise ValueError(self.error or "Could not reload camera calibration")
        return self.snapshot()

    def horizontal_fov_degrees(self):
        if not self.data:
            return None
        width = float(self.data["image_size"][0])
        focal_x = float(self.data["camera_matrix"][0][0])
        if width <= 0 or focal_x <= 0:
            return None
        return math.degrees(2.0 * math.atan(width / (2.0 * focal_x)))

    def undistort(self, image):
        if not self.data or cv2 is None or np is None or image is None:
            return image
        height, width = image.shape[:2]
        original_width, original_height = self.data["image_size"]
        if original_width <= 0 or original_height <= 0:
            return image
        matrix = np.asarray(self.data["camera_matrix"], dtype=np.float64).copy()
        matrix[0, 0] *= width / original_width
        matrix[0, 2] *= width / original_width
        matrix[1, 1] *= height / original_height
        matrix[1, 2] *= height / original_height
        distortion = np.asarray(
            self.data["distortion_coefficients"],
            dtype=np.float64,
        )
        return cv2.undistort(image, matrix, distortion, None, matrix)

    def snapshot(self):
        source = self.data.get("source") if self.data else None
        charuco = self.data.get("charuco") if self.data else None
        return {
            "calibrated": self.calibrated,
            "path": self.path,
            "schema": self.data.get("schema") if self.data else None,
            "source": source,
            "horizontal_fov_degrees": self.horizontal_fov_degrees(),
            "rms_error": self.data.get("rms_error") if self.data else None,
            "samples": self.data.get("samples") if self.data else 0,
            "image_size": self.data.get("image_size") if self.data else None,
            "charuco": charuco,
            "rejected_samples": self.data.get("rejected_samples") if self.data else [],
            "error": self.error,
        }


def charuco_available():
    return (
        cv2 is not None
        and np is not None
        and hasattr(cv2, "aruco")
        and hasattr(cv2.aruco, "getPredefinedDictionary")
    )


def available_charuco_dictionaries():
    if not charuco_available():
        return []
    return [
        name
        for name in CHARUCO_DICTIONARIES
        if hasattr(cv2.aruco, name)
    ]


def normalize_charuco_config(config=None):
    config = dict(config or {})
    result = {
        "squares_x": int(config.get("squares_x", 8)),
        "squares_y": int(config.get("squares_y", 8)),
        "square_length_m": float(config.get("square_length_m", 0.0254)),
        "marker_length_m": float(config.get("marker_length_m", 0.01905)),
        "dictionary": str(config.get("dictionary") or "DICT_4X4_1000"),
        "minimum_corners": int(config.get("minimum_corners", 12)),
        "required_samples": int(config.get("required_samples", 12)),
        "recommended_samples": int(config.get("recommended_samples", 20)),
    }
    if result["squares_x"] < 4 or result["squares_x"] > 20:
        raise ValueError("squares_x must be between 4 and 20")
    if result["squares_y"] < 4 or result["squares_y"] > 20:
        raise ValueError("squares_y must be between 4 and 20")
    if not 0.005 <= result["square_length_m"] <= 0.20:
        raise ValueError("square_length_m must be between 0.005 and 0.20")
    if not 0.003 <= result["marker_length_m"] < result["square_length_m"]:
        raise ValueError("marker_length_m must be positive and smaller than square_length_m")
    available = available_charuco_dictionaries()
    if available and result["dictionary"] not in available:
        raise ValueError(
            f"Unsupported ChArUco dictionary {result['dictionary']}; "
            f"available: {', '.join(available)}"
        )
    maximum_corners = (result["squares_x"] - 1) * (result["squares_y"] - 1)
    result["minimum_corners"] = max(
        6, min(maximum_corners, result["minimum_corners"])
    )
    result["required_samples"] = max(8, min(100, result["required_samples"]))
    result["recommended_samples"] = max(
        result["required_samples"],
        min(150, result["recommended_samples"]),
    )
    return result


def _charuco_dictionary(name):
    if not charuco_available():
        raise RuntimeError(
            "OpenCV ArUco support is unavailable; install opencv-contrib-python-headless"
        )
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Unsupported ChArUco dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def create_charuco_board(config=None):
    config = normalize_charuco_config(config)
    dictionary = _charuco_dictionary(config["dictionary"])
    size = (config["squares_x"], config["squares_y"])
    try:
        board = cv2.aruco.CharucoBoard(
            size,
            config["square_length_m"],
            config["marker_length_m"],
            dictionary,
        )
    except (AttributeError, TypeError):
        if not hasattr(cv2.aruco, "CharucoBoard_create"):
            raise RuntimeError("This OpenCV build does not provide ChArUco boards")
        board = cv2.aruco.CharucoBoard_create(
            size[0],
            size[1],
            config["square_length_m"],
            config["marker_length_m"],
            dictionary,
        )
    return board, dictionary, config


def _polygon_area(points):
    if len(points) < 3:
        return 0.0
    array = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    hull = cv2.convexHull(array)
    return abs(float(cv2.contourArea(hull)))


def _charuco_pose_metrics(corners, image_size):
    points = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    width, height = image_size
    if len(points) == 0:
        return {
            "coverage_ratio": 0.0,
            "centroid_normalized": [0.5, 0.5],
            "orientation_degrees": 0.0,
            "span_ratio": 0.0,
        }
    centroid = points.mean(axis=0)
    area = _polygon_area(points)
    coverage = area / max(1.0, float(width * height))
    span_x = float(points[:, 0].max() - points[:, 0].min())
    span_y = float(points[:, 1].max() - points[:, 1].min())
    centered = points - centroid
    orientation = 0.0
    if len(points) >= 3:
        covariance = np.cov(centered.T)
        values, vectors = np.linalg.eigh(covariance)
        vector = vectors[:, int(np.argmax(values))]
        orientation = math.degrees(math.atan2(float(vector[1]), float(vector[0])))
    return {
        "coverage_ratio": float(coverage),
        "centroid_normalized": [
            float(centroid[0] / max(1.0, width)),
            float(centroid[1] / max(1.0, height)),
        ],
        "orientation_degrees": float(orientation),
        "span_ratio": float(max(span_x / max(1.0, width), span_y / max(1.0, height))),
    }


def detect_charuco(image, config=None):
    board, dictionary, config = create_charuco_board(config)
    if image is None:
        raise ValueError("Camera image is unavailable")
    gray = (
        image
        if getattr(image, "ndim", 0) == 2
        else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    )
    image_size = (int(gray.shape[1]), int(gray.shape[0]))
    marker_corners = ()
    marker_ids = None
    charuco_corners = None
    charuco_ids = None

    if hasattr(cv2.aruco, "CharucoDetector"):
        detector = cv2.aruco.CharucoDetector(board)
        (
            charuco_corners,
            charuco_ids,
            marker_corners,
            marker_ids,
        ) = detector.detectBoard(gray)
    else:
        if hasattr(cv2.aruco, "ArucoDetector"):
            marker_detector = cv2.aruco.ArucoDetector(dictionary)
            marker_corners, marker_ids, _ = marker_detector.detectMarkers(gray)
        else:
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
        if marker_ids is not None and len(marker_ids):
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                board,
            )

    if charuco_corners is None or charuco_ids is None:
        corners = []
        ids = []
    else:
        corners = (
            np.asarray(charuco_corners, dtype=np.float32)
            .reshape(-1, 2)
            .tolist()
        )
        ids = (
            np.asarray(charuco_ids, dtype=np.int32)
            .reshape(-1)
            .astype(int)
            .tolist()
        )

    marker_documents = []
    if marker_ids is not None:
        flat_ids = np.asarray(marker_ids, dtype=np.int32).reshape(-1).tolist()
        for marker_id, marker in zip(flat_ids, marker_corners or []):
            marker_documents.append(
                {
                    "id": int(marker_id),
                    "corners": np.asarray(marker, dtype=np.float32)
                    .reshape(-1, 2)
                    .tolist(),
                }
            )

    metrics = _charuco_pose_metrics(corners, image_size)
    expected_corners = (config["squares_x"] - 1) * (config["squares_y"] - 1)
    corner_score = min(1.0, len(corners) / max(1.0, expected_corners * 0.55))
    coverage_score = min(1.0, metrics["coverage_ratio"] / 0.12)
    span_score = min(1.0, metrics["span_ratio"] / 0.45)
    quality = 0.50 * corner_score + 0.30 * coverage_score + 0.20 * span_score
    valid = len(corners) >= config["minimum_corners"]
    if not valid:
        guidance = f"보드를 더 크게/선명하게 보여주세요 ({len(corners)}/{config['minimum_corners']} corners)"
    elif metrics["coverage_ratio"] < 0.025:
        guidance = "보드가 너무 작습니다. 카메라에 더 가깝게 보여주세요."
    elif metrics["span_ratio"] < 0.20:
        guidance = "보드를 화면에서 더 크게 보여주세요."
    else:
        guidance = "샘플로 사용할 수 있습니다. 위치와 각도를 바꿔 추가 촬영하세요."
    return {
        "valid": bool(valid),
        "config": config,
        "image_size": list(image_size),
        "charuco_corners": corners,
        "charuco_ids": ids,
        "marker_corners": marker_documents,
        "detected_corners": len(corners),
        "detected_markers": len(marker_documents),
        "expected_corners": expected_corners,
        "quality": float(max(0.0, min(1.0, quality))),
        "guidance": guidance,
        **metrics,
    }


def _charuco_object_points(ids, config):
    columns = config["squares_x"] - 1
    rows = config["squares_y"] - 1
    square = float(config["square_length_m"])
    points = []
    for raw_id in ids:
        corner_id = int(raw_id)
        if corner_id < 0 or corner_id >= columns * rows:
            raise ValueError(f"ChArUco corner id out of range: {corner_id}")
        row = corner_id // columns
        column = corner_id % columns
        points.append(((column + 1) * square, (row + 1) * square, 0.0))
    return np.asarray(points, dtype=np.float32)


def _calibrate_charuco_views(samples, image_size, config):
    object_points = []
    image_points = []
    for sample in samples:
        ids = sample["charuco_ids"]
        corners = sample["charuco_corners"]
        if len(ids) != len(corners) or len(ids) < 6:
            raise ValueError("Invalid ChArUco sample correspondence")
        object_points.append(_charuco_object_points(ids, config))
        image_points.append(
            np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
        )
    rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        tuple(int(value) for value in image_size),
        None,
        None,
    )
    per_view = []
    for object_view, image_view, rvec, tvec in zip(
        object_points, image_points, rvecs, tvecs
    ):
        projected, _ = cv2.projectPoints(
            object_view,
            rvec,
            tvec,
            matrix,
            distortion,
        )
        difference = (
            np.asarray(projected, dtype=np.float64).reshape(-1, 2)
            - np.asarray(image_view, dtype=np.float64).reshape(-1, 2)
        )
        per_view.append(
            float(np.sqrt(np.mean(np.sum(difference * difference, axis=1))))
        )
    return (
        float(rms),
        matrix,
        distortion,
        rvecs,
        tvecs,
        per_view,
    )


def calibrate_charuco_samples(samples, config=None, minimum_samples=None):
    if cv2 is None or np is None:
        raise RuntimeError("OpenCV and NumPy are required")
    config = normalize_charuco_config(config)
    minimum_samples = int(
        minimum_samples
        if minimum_samples is not None
        else config["required_samples"]
    )
    if len(samples) < minimum_samples:
        raise ValueError(
            f"At least {minimum_samples} valid ChArUco samples are required; "
            f"found {len(samples)}"
        )
    image_size = None
    normalized = []
    for index, sample in enumerate(samples):
        current_size = [int(value) for value in sample["image_size"]]
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            raise ValueError("All ChArUco samples must use the same image resolution")
        if len(sample.get("charuco_ids") or []) < config["minimum_corners"]:
            continue
        normalized.append({**sample, "_source_index": index})
    if len(normalized) < minimum_samples:
        raise ValueError(
            f"Only {len(normalized)} samples meet the corner threshold; "
            f"{minimum_samples} required"
        )

    first = _calibrate_charuco_views(normalized, image_size, config)
    kept = list(normalized)
    rejected = []
    if len(normalized) >= max(minimum_samples + 2, 14):
        errors = np.asarray(first[-1], dtype=np.float64)
        median_error = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median_error)))
        robust_sigma = max(0.05, 1.4826 * mad)
        threshold = max(1.25, median_error + 2.75 * robust_sigma)
        candidate_kept = [
            sample
            for sample, error in zip(normalized, errors)
            if float(error) <= threshold
        ]
        candidate_rejected = [
            sample
            for sample, error in zip(normalized, errors)
            if float(error) > threshold
        ]
        if candidate_rejected and len(candidate_kept) >= minimum_samples:
            kept = candidate_kept
            rejected = candidate_rejected

    result = (
        _calibrate_charuco_views(kept, image_size, config)
        if rejected
        else first
    )
    rms, matrix, distortion, _, _, per_view = result
    accepted_names = [
        str(sample.get("sample_id") or sample.get("filename") or sample["_source_index"])
        for sample in kept
    ]
    rejected_names = [
        str(sample.get("sample_id") or sample.get("filename") or sample["_source_index"])
        for sample in rejected
    ]
    return {
        "schema": "camera_calibration_v2",
        "source": "CHARUCO",
        "image_size": list(image_size),
        "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "rms_error": float(rms),
        "samples": len(kept),
        "per_view_rms": [float(value) for value in per_view],
        "accepted_images": accepted_names,
        "rejected_samples": rejected_names,
        "charuco": {
            "squares_x": config["squares_x"],
            "squares_y": config["squares_y"],
            "square_length_m": config["square_length_m"],
            "marker_length_m": config["marker_length_m"],
            "dictionary": config["dictionary"],
            "minimum_corners": config["minimum_corners"],
        },
    }


def calibrate_chessboard(
    image_paths,
    board_columns=9,
    board_rows=6,
    square_size_m=0.025,
):
    if cv2 is None or np is None:
        raise RuntimeError("OpenCV and NumPy are required")
    board_size = (int(board_columns), int(board_rows))
    object_template = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    object_template[:, :2] = (
        np.mgrid[0 : board_size[0], 0 : board_size[1]]
        .T.reshape(-1, 2)
        * float(square_size_m)
    )
    object_points = []
    image_points = []
    accepted = []
    image_size = None
    for path in image_paths:
        image = cv2.imread(path)
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            continue
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(
                gray,
                board_size,
                flags=cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
        else:
            found, corners = cv2.findChessboardCorners(gray, board_size)
            if found:
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (11, 11),
                    (-1, -1),
                    (
                        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                        30,
                        0.001,
                    ),
                )
        if not found:
            continue
        object_points.append(object_template.copy())
        image_points.append(corners)
        accepted.append(path)
    if image_size is None or len(object_points) < 8:
        raise ValueError(
            f"At least 8 valid chessboard images are required; found {len(object_points)}"
        )
    rms_error, matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    return {
        "schema": "camera_calibration_v1",
        "source": "CHESSBOARD",
        "image_size": list(image_size),
        "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "rms_error": float(rms_error),
        "samples": len(accepted),
        "board_columns": board_size[0],
        "board_rows": board_size[1],
        "square_size_m": float(square_size_m),
        "accepted_images": [os.path.basename(path) for path in accepted],
    }
