import json
import math
import os
import threading
import time

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

from .camera_calibration import (
    available_charuco_dictionaries,
    calibrate_charuco_samples,
    charuco_available,
    detect_charuco,
    normalize_charuco_config,
)


class CameraCalibrationSessionError(ValueError):
    pass


class CameraCalibrationSession:
    """Persistent ChArUco capture/calibration session for the live rover camera."""

    CONFIG_FILE = "session.json"

    def __init__(self, calibration, sample_directory):
        self.calibration = calibration
        self.sample_directory = os.path.abspath(sample_directory)
        self.lock = threading.RLock()
        self.config = normalize_charuco_config()
        self._preview_cache_key = None
        self._preview_cache = None
        self.last_result = None
        self.error = None
        self._load_config()

    def _config_path(self):
        return os.path.join(self.sample_directory, self.CONFIG_FILE)

    def _load_config(self):
        try:
            with open(self._config_path(), encoding="utf-8") as file:
                document = json.load(file)
            self.config = normalize_charuco_config(document.get("config") or document)
        except FileNotFoundError:
            return
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            self.error = f"Could not load ChArUco session config: {error}"

    def _persist_config(self):
        os.makedirs(self.sample_directory, exist_ok=True)
        path = self._config_path()
        temporary = f"{path}.tmp"
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(
                {
                    "schema": "charuco_capture_session_v1",
                    "config": self.config,
                },
                file,
                ensure_ascii=False,
                indent=2,
            )
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)

    def _metadata_paths(self):
        if not os.path.isdir(self.sample_directory):
            return []
        return sorted(
            os.path.join(self.sample_directory, name)
            for name in os.listdir(self.sample_directory)
            if name.startswith("sample-") and name.endswith(".json")
        )

    def _load_samples(self, include_points=False):
        samples = []
        for path in self._metadata_paths():
            try:
                with open(path, encoding="utf-8") as file:
                    item = json.load(file)
            except (OSError, ValueError, json.JSONDecodeError):
                continue
            if include_points:
                samples.append(item)
                continue
            metrics = item.get("metrics") or {}
            coverage_cells = metrics.get("coverage_cells")
            if coverage_cells is None:
                coverage_cells = self._coverage_cells(
                    item.get("charuco_corners"),
                    item.get("image_size"),
                )
            samples.append(
                {
                    "sample_id": item.get("sample_id"),
                    "captured_at": item.get("captured_at"),
                    "frame_sequence": item.get("frame_sequence"),
                    "image_size": item.get("image_size"),
                    "detected_corners": len(item.get("charuco_ids") or []),
                    "coverage_ratio": metrics.get("coverage_ratio"),
                    "centroid_normalized": metrics.get("centroid_normalized"),
                    "coverage_cells": coverage_cells,
                    "orientation_degrees": metrics.get("orientation_degrees"),
                    "quality": metrics.get("quality"),
                    "novelty": metrics.get("novelty"),
                    "image_file": item.get("image_file"),
                }
            )
        return samples

    @staticmethod
    def _coverage_cell(centroid):
        """Return the legacy centroid cell used for novelty/backward compatibility."""
        if not centroid or len(centroid) != 2:
            return None
        x = max(0.0, min(0.999999, float(centroid[0])))
        y = max(0.0, min(0.999999, float(centroid[1])))
        return int(y * 3) * 3 + int(x * 3)

    @staticmethod
    def _coverage_cells(corners, image_size):
        """Return every 3x3 sensor region actually occupied by detected corners.

        Coverage must describe where calibration observations exist on the image
        sensor, not where the board centroid happens to be. A large ChArUco board
        can cover the bottom third while its centroid remains in the middle row;
        centroid-only accounting made the dashboard's bottom three cells nearly
        impossible to fill.
        """
        if not corners or not image_size or len(image_size) != 2:
            return []
        try:
            width = float(image_size[0])
            height = float(image_size[1])
        except (TypeError, ValueError, IndexError):
            return []
        if width <= 0.0 or height <= 0.0:
            return []
        cells = set()
        for point in corners:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                continue
            try:
                x = float(point[0])
                y = float(point[1])
            except (TypeError, ValueError):
                continue
            if not math.isfinite(x) or not math.isfinite(y):
                continue
            nx = max(0.0, min(0.999999, x / width))
            ny = max(0.0, min(0.999999, y / height))
            cells.add(int(ny * 3) * 3 + int(nx * 3))
        return sorted(cells)

    def _coverage_grid(self, samples):
        grid = [0] * 9
        for sample in samples:
            cells = sample.get("coverage_cells")
            if cells is None:
                cells = self._coverage_cells(
                    sample.get("charuco_corners"),
                    sample.get("image_size"),
                )
            if cells:
                for raw_cell in cells:
                    try:
                        cell = int(raw_cell)
                    except (TypeError, ValueError):
                        continue
                    if 0 <= cell < 9:
                        grid[cell] += 1
                continue
            # Old sample metadata that predates corner-region tracking remains
            # usable via the previous centroid-only cell.
            cell = self._coverage_cell(sample.get("centroid_normalized"))
            if cell is not None:
                grid[cell] += 1
        return grid

    @staticmethod
    def _angle_distance(a, b):
        difference = abs(float(a) - float(b)) % 180.0
        return min(difference, 180.0 - difference)

    def _novelty(self, detection, samples):
        centroid = detection.get("centroid_normalized") or [0.5, 0.5]
        area = max(1e-6, float(detection.get("coverage_ratio") or 0.0))
        angle = float(detection.get("orientation_degrees") or 0.0)
        if not samples:
            return 1.0
        distances = []
        for sample in samples:
            other_centroid = sample.get("centroid_normalized") or [0.5, 0.5]
            dx = float(centroid[0]) - float(other_centroid[0])
            dy = float(centroid[1]) - float(other_centroid[1])
            center_distance = math.sqrt(dx * dx + dy * dy) / 0.50
            other_area = max(1e-6, float(sample.get("coverage_ratio") or 0.0))
            scale_distance = min(1.0, abs(math.log(area / other_area)) / 0.90)
            angle_distance = min(
                1.0,
                self._angle_distance(
                    angle,
                    sample.get("orientation_degrees") or 0.0,
                )
                / 45.0,
            )
            distances.append(
                min(
                    1.0,
                    0.55 * center_distance
                    + 0.25 * scale_distance
                    + 0.20 * angle_distance,
                )
            )
        return float(min(distances))

    def _snapshot_locked(self):
        samples = self._load_samples(include_points=False)
        grid = self._coverage_grid(samples)
        sample_count = len(samples)
        warnings = []
        occupied = sum(1 for count in grid if count)
        if sample_count >= self.config["required_samples"] and occupied < 4:
            warnings.append(
                "샘플 수는 충분하지만 화면 위치 분포가 좁습니다. "
                "보드를 좌/우/상/하로 옮긴 샘플을 추가하면 보정 정확도가 좋아집니다."
            )
        if sample_count >= self.config["required_samples"]:
            angles = [
                float(sample.get("orientation_degrees") or 0.0)
                for sample in samples
            ]
            if angles and max(angles) - min(angles) < 12.0:
                warnings.append(
                    "보드 기울기 다양성이 적습니다. 좌우/상하로 기울인 샘플을 추가하세요."
                )
        return {
            "available": bool(charuco_available()),
            "opencv_version": getattr(cv2, "__version__", None) if cv2 is not None else None,
            "aruco_available": bool(charuco_available()),
            "install_hint": (
                None
                if charuco_available()
                else "Install opencv-contrib-python-headless; cv2.aruco is required"
            ),
            "config": dict(self.config),
            "dictionaries": available_charuco_dictionaries(),
            "sample_count": sample_count,
            "required_samples": self.config["required_samples"],
            "recommended_samples": self.config["recommended_samples"],
            "ready_to_calibrate": (
                charuco_available()
                and sample_count >= self.config["required_samples"]
            ),
            "coverage_grid": grid,
            "occupied_coverage_cells": occupied,
            "samples": samples[-30:],
            "calibration": self.calibration.snapshot(),
            "last_result": self.last_result,
            "warnings": warnings,
            "sample_directory": self.sample_directory,
            "error": self.error,
        }

    def snapshot(self):
        with self.lock:
            return self._snapshot_locked()

    def configure(self, payload):
        with self.lock:
            incoming = payload.get("config", payload)
            proposed = normalize_charuco_config(incoming)
            existing = self._load_samples(include_points=False)
            if existing and proposed != self.config:
                raise CameraCalibrationSessionError(
                    "Reset captured samples before changing ChArUco board settings"
                )
            self.config = proposed
            self._preview_cache_key = None
            self._preview_cache = None
            self.error = None
            self._persist_config()
            return self._snapshot_locked()

    @staticmethod
    def _decode_jpeg(jpeg):
        if cv2 is None or np is None:
            raise CameraCalibrationSessionError("OpenCV/NumPy unavailable")
        if not jpeg:
            raise CameraCalibrationSessionError("Camera frame unavailable")
        image = cv2.imdecode(
            np.frombuffer(jpeg, dtype=np.uint8),
            cv2.IMREAD_COLOR,
        )
        if image is None:
            raise CameraCalibrationSessionError("Could not decode camera JPEG")
        return image

    def preview(self, jpeg, sequence=None):
        with self.lock:
            key = (
                sequence,
                len(jpeg or b""),
                tuple(sorted(self.config.items())),
            )
            if key == self._preview_cache_key and self._preview_cache is not None:
                return dict(self._preview_cache)
            if not charuco_available():
                result = {
                    "valid": False,
                    "error": "ARUCO_UNAVAILABLE",
                    "guidance": (
                        "opencv-contrib-python-headless가 필요합니다. "
                        "Pi 의존성을 업데이트하세요."
                    ),
                    "image_size": None,
                    "charuco_corners": [],
                    "charuco_ids": [],
                    "marker_corners": [],
                    "detected_corners": 0,
                    "detected_markers": 0,
                    "quality": 0.0,
                }
            else:
                try:
                    image = self._decode_jpeg(jpeg)
                    result = detect_charuco(image, self.config)
                    samples = self._load_samples(include_points=False)
                    result["novelty"] = self._novelty(result, samples)
                    result["coverage_cell"] = self._coverage_cell(
                        result.get("centroid_normalized")
                    )
                    result["coverage_cells"] = self._coverage_cells(
                        result.get("charuco_corners"),
                        result.get("image_size"),
                    )
                    if result["valid"] and result["novelty"] < 0.12:
                        result["guidance"] = (
                            "이전 샘플과 매우 비슷합니다. 보드를 다른 위치/거리/"
                            "각도로 옮긴 뒤 촬영하는 것을 권장합니다."
                        )
                    result["error"] = None
                except Exception as error:
                    result = {
                        "valid": False,
                        "error": f"{type(error).__name__}: {error}",
                        "guidance": "ChArUco 보드를 카메라에 보여주세요.",
                        "image_size": None,
                        "charuco_corners": [],
                        "charuco_ids": [],
                        "marker_corners": [],
                        "detected_corners": 0,
                        "detected_markers": 0,
                        "quality": 0.0,
                        "coverage_cells": [],
                    }
            result["frame_sequence"] = sequence
            self._preview_cache_key = key
            self._preview_cache = dict(result)
            return result

    def capture(self, jpeg, sequence=None):
        with self.lock:
            preview = self.preview(jpeg, sequence)
            if not preview.get("valid"):
                raise CameraCalibrationSessionError(
                    preview.get("guidance")
                    or preview.get("error")
                    or "ChArUco board not detected"
                )
            os.makedirs(self.sample_directory, exist_ok=True)
            timestamp = time.time()
            stamp = int(timestamp * 1000)
            sequence_text = "na" if sequence is None else str(sequence)
            sample_id = f"sample-{stamp}-{sequence_text}"
            image_name = f"{sample_id}.jpg"
            image_path = os.path.join(self.sample_directory, image_name)
            metadata_path = os.path.join(self.sample_directory, f"{sample_id}.json")
            with open(image_path, "wb") as file:
                file.write(jpeg)
                file.flush()
                os.fsync(file.fileno())
            metadata = {
                "schema": "charuco_sample_v1",
                "sample_id": sample_id,
                "captured_at": timestamp,
                "frame_sequence": sequence,
                "image_size": preview["image_size"],
                "charuco_corners": preview["charuco_corners"],
                "charuco_ids": preview["charuco_ids"],
                "config": dict(self.config),
                "image_file": image_name,
                "metrics": {
                    "detected_corners": preview.get("detected_corners"),
                    "detected_markers": preview.get("detected_markers"),
                    "coverage_ratio": preview.get("coverage_ratio"),
                    "centroid_normalized": preview.get("centroid_normalized"),
                    "coverage_cells": preview.get("coverage_cells") or [],
                    "orientation_degrees": preview.get("orientation_degrees"),
                    "span_ratio": preview.get("span_ratio"),
                    "quality": preview.get("quality"),
                    "novelty": preview.get("novelty"),
                },
            }
            temporary = f"{metadata_path}.tmp"
            with open(temporary, "w", encoding="utf-8") as file:
                json.dump(metadata, file, ensure_ascii=False, indent=2)
                file.flush()
                os.fsync(file.fileno())
            os.replace(temporary, metadata_path)
            self._preview_cache_key = None
            return {
                "captured": {
                    "sample_id": sample_id,
                    "detected_corners": preview.get("detected_corners"),
                    "quality": preview.get("quality"),
                    "novelty": preview.get("novelty"),
                    "coverage_cells": preview.get("coverage_cells") or [],
                },
                **self._snapshot_locked(),
            }

    def remove_last(self):
        with self.lock:
            paths = self._metadata_paths()
            if not paths:
                raise CameraCalibrationSessionError("No captured samples to remove")
            path = paths[-1]
            try:
                with open(path, encoding="utf-8") as file:
                    metadata = json.load(file)
            except (OSError, ValueError, json.JSONDecodeError):
                metadata = {}
            image_file = metadata.get("image_file")
            if image_file:
                try:
                    os.remove(os.path.join(self.sample_directory, image_file))
                except FileNotFoundError:
                    pass
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
            self._preview_cache_key = None
            return self._snapshot_locked()

    def reset_samples(self):
        with self.lock:
            for metadata_path in self._metadata_paths():
                try:
                    with open(metadata_path, encoding="utf-8") as file:
                        metadata = json.load(file)
                except (OSError, ValueError, json.JSONDecodeError):
                    metadata = {}
                image_file = metadata.get("image_file")
                if image_file:
                    try:
                        os.remove(os.path.join(self.sample_directory, image_file))
                    except FileNotFoundError:
                        pass
                try:
                    os.remove(metadata_path)
                except FileNotFoundError:
                    pass
            self._preview_cache_key = None
            self._preview_cache = None
            self.last_result = None
            return self._snapshot_locked()

    def solve(self):
        with self.lock:
            if not charuco_available():
                raise CameraCalibrationSessionError(
                    "cv2.aruco unavailable; install opencv-contrib-python-headless"
                )
            samples = self._load_samples(include_points=True)
            if len(samples) < self.config["required_samples"]:
                raise CameraCalibrationSessionError(
                    f"{self.config['required_samples']} samples required; "
                    f"currently {len(samples)}"
                )
            for sample in samples:
                stored_config = normalize_charuco_config(sample.get("config") or {})
                if stored_config != self.config:
                    raise CameraCalibrationSessionError(
                        "Captured samples use mixed ChArUco board settings; reset and recapture"
                    )
            result = calibrate_charuco_samples(
                samples,
                self.config,
                minimum_samples=self.config["required_samples"],
            )
            result["session"] = {
                "sample_directory": self.sample_directory,
                "captured_samples": len(samples),
                "coverage_grid": self._coverage_grid(samples),
            }
            calibration_snapshot = self.calibration.save(result)
            self.last_result = {
                "rms_error": result["rms_error"],
                "samples": result["samples"],
                "rejected_samples": result.get("rejected_samples") or [],
                "horizontal_fov_degrees": calibration_snapshot.get(
                    "horizontal_fov_degrees"
                ),
            }
            self.error = None
            return self._snapshot_locked()
