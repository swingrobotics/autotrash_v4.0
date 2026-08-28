"""External UFLD-primary lane controller with classical fallback.

MANUAL and AUTO_AI keep the existing classical control path. No-training AUTO
and AUTO_LOCAL may enable the open-source Ultra-Fast-Lane-Detection (UFLD)
TuSimple model as the primary lane detector. A separate neural preview path is
available for diagnostics in MANUAL/DISARMED without changing autonomous-control
enable state. SWING only converts the external lane tracks to its common
LaneResult/calibration/safety geometry.
"""

from __future__ import annotations

from dataclasses import replace
import math
import threading

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None
    np = None

from .lane_controller import LaneController, LaneResult
from .lane_geometry_hardening import distort_undistorted_points


class HybridLaneController:
    BACKEND = "HYBRID_ROAD"
    NEURAL_BACKEND = "UFLD_ONNX"

    def __init__(
        self,
        pretrained,
        camera_calibration=None,
        expected_lane_width_m=1.0,
        vehicle_width_m=0.4826,
        processing_width=640,
        processing_height=360,
        maximum_neural_inference_ms=160.0,
    ):
        self.pretrained = pretrained
        self.camera_calibration = camera_calibration
        self.expected_lane_width_m = float(expected_lane_width_m)
        self.vehicle_width_m = float(vehicle_width_m)
        self.processing_width = max(320, int(processing_width))
        self.processing_height = max(180, int(processing_height))
        self.maximum_neural_inference_ms = max(
            1.0, float(maximum_neural_inference_ms)
        )
        self.classical = LaneController(
            expected_lane_width_m=self.expected_lane_width_m,
            vehicle_width_m=self.vehicle_width_m,
            camera_calibration=camera_calibration,
            processing_width=self.processing_width,
            processing_height=self.processing_height,
        )
        # Neural inference receives an already-undistorted image, therefore the
        # geometry stages must not apply calibration a second time.
        self._neural_geometry = self._new_neural_geometry()
        # Preview has independent history so it cannot seed AUTO/AUTO_LOCAL.
        self._preview_geometry = self._new_neural_geometry()
        self._lock = threading.RLock()
        self._neural_enabled = False
        self._last_backend = "CLASSICAL_CV"
        self._fallback_reason = "NEURAL_DISABLED"
        self._last_neural = None
        self._last_preview = None
        self._neural_suspended_reason = None
        self._cached_jpeg = None
        self._cached_result = None

    def _new_neural_geometry(self):
        return LaneController(
            expected_lane_width_m=self.expected_lane_width_m,
            vehicle_width_m=self.vehicle_width_m,
            camera_calibration=None,
            processing_width=self.processing_width,
            processing_height=self.processing_height,
        )

    @property
    def available(self):
        return self.classical.available

    @property
    def neural_enabled(self):
        return self._neural_enabled

    def set_neural_enabled(self, enabled):
        with self._lock:
            enabled = bool(enabled)
            if enabled != self._neural_enabled:
                self._cached_jpeg = None
                self._cached_result = None
                if enabled:
                    self._neural_suspended_reason = None
            self._neural_enabled = enabled
            if not enabled:
                self._fallback_reason = "NEURAL_DISABLED"
            return self.snapshot()

    def probe_neural_latency_jpeg(self, jpeg, attempts=2):
        """Warm/benchmark the external neural model without accepting geometry."""
        with self._lock:
            result = {
                "ready": False,
                "allowed": False,
                "attempts": [],
                "maximum_neural_inference_ms": self.maximum_neural_inference_ms,
                "error": None,
            }
            try:
                if not jpeg:
                    raise ValueError("CAMERA_FRAME_UNAVAILABLE")
                if cv2 is None or np is None:
                    raise RuntimeError("OpenCV/NumPy unavailable")
                image = cv2.imdecode(
                    np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if image is None:
                    raise RuntimeError("JPEG_DECODE_FAILED")
                source = image
                if self._calibration_usable(self.camera_calibration):
                    source = self.camera_calibration.undistort(source)
                if not self.pretrained.ensure_loaded():
                    raise RuntimeError(
                        self.pretrained.snapshot().get("error")
                        or "PRETRAINED_MODEL_UNAVAILABLE"
                    )
                last = None
                for _ in range(max(1, min(4, int(attempts)))):
                    neural = self.pretrained.infer(source)
                    last = neural
                    inference_ms = float(neural.get("inference_ms") or 0.0)
                    result["attempts"].append(inference_ms)
                warm_ms = float(result["attempts"][-1])
                result["warm_inference_ms"] = warm_ms
                result["model"] = None if last is None else last.get("model")
                result["ready"] = True
                result["allowed"] = (
                    math.isfinite(warm_ms)
                    and warm_ms <= self.maximum_neural_inference_ms
                )
                if not result["allowed"]:
                    self._neural_suspended_reason = (
                        "NEURAL_INFERENCE_TOO_SLOW:"
                        f"{warm_ms:.2f}ms>{self.maximum_neural_inference_ms:.2f}ms"
                    )
                    result["error"] = self._neural_suspended_reason
                else:
                    self._neural_suspended_reason = None
                return result
            except Exception as error:
                result["error"] = f"{type(error).__name__}: {error}"
                return result

    def set_expected_lane_width_m(self, value):
        value = float(value)
        if not math.isfinite(value) or value <= 0:
            return
        with self._lock:
            self.expected_lane_width_m = value
            self.classical.set_expected_lane_width_m(value)
            self._neural_geometry.set_expected_lane_width_m(value)
            self._preview_geometry.set_expected_lane_width_m(value)

    def reset(self):
        with self._lock:
            self.classical.reset()
            self._neural_geometry.reset()
            self._preview_geometry.reset()
            self._last_backend = "CLASSICAL_CV"
            self._fallback_reason = "RESET"
            self._last_neural = None
            self._last_preview = None
            self._neural_suspended_reason = None
            self._cached_jpeg = None
            self._cached_result = None

    @staticmethod
    def _calibration_usable(calibration):
        if calibration is None:
            return False
        usable = getattr(calibration, "vision_usable", None)
        if usable is not None:
            return bool(usable)
        return bool(getattr(calibration, "calibrated", False))

    @staticmethod
    def _fit_x(fit, y_value):
        if np is None:
            raise RuntimeError("NumPy unavailable")
        coefficients = fit.get("coefficients") if isinstance(fit, dict) else fit
        if coefficients is None:
            raise RuntimeError("NEURAL_FIT_COEFFICIENTS_MISSING")
        values = np.asarray(coefficients, dtype=np.float64).reshape(-1)
        if values.size not in {2, 3} or not np.all(np.isfinite(values)):
            raise RuntimeError(
                f"NEURAL_FIT_COEFFICIENTS_INVALID:{values.shape}"
            )
        result = float(np.polyval(values, float(y_value)))
        if not math.isfinite(result):
            raise RuntimeError("NEURAL_FIT_EVALUATION_NONFINITE")
        return result

    def _candidate_fits(
        self,
        neural,
        source_shape,
        roi_top,
        roi_bottom,
        geometry,
    ):
        source_height, source_width = source_shape[:2]
        width = self.processing_width
        height = self.processing_height
        scale_x = width / max(1.0, float(source_width))
        scale_y = height / max(1.0, float(source_height))
        roi_height = roi_bottom - roi_top
        candidates = []
        for lane in neural.get("lanes") or []:
            points = []
            for raw in lane.get("points") or []:
                if not isinstance(raw, (list, tuple)) or len(raw) != 2:
                    continue
                try:
                    x = float(raw[0]) * scale_x
                    y_global = float(raw[1]) * scale_y
                except (TypeError, ValueError):
                    continue
                if not math.isfinite(x) or not math.isfinite(y_global):
                    continue
                if y_global < roi_top or y_global >= roi_bottom:
                    continue
                if x < -0.20 * width or x > 1.20 * width:
                    continue
                points.append((x, y_global - roi_top))
            fit = geometry._robust_fit(points)
            if fit is None:
                continue
            values = np.asarray(points, dtype=np.float64)
            observed_bottom_y = float(np.max(values[:, 1]))
            observed_bottom_x = self._fit_x(fit, observed_bottom_y)
            bottom_x = self._fit_x(fit, max(0.0, roi_height - 1.0))
            top_x = self._fit_x(fit, 0.0)
            try:
                confidence = float(lane.get("confidence") or 0.0)
            except (TypeError, ValueError):
                confidence = 0.0
            lane_id = lane.get("lane_id", lane.get("query_id", -1))
            try:
                lane_id = int(lane_id)
            except (TypeError, ValueError):
                lane_id = -1
            candidates.append(
                {
                    "lane_id": lane_id,
                    "query_id": lane_id,
                    "confidence": max(0.0, min(1.0, confidence)),
                    "points": points,
                    "fit": fit,
                    "observed_bottom_x": observed_bottom_x,
                    "bottom_x": bottom_x,
                    "top_x": top_x,
                }
            )
        return candidates

    @staticmethod
    def _pair_score(left, right, center, width):
        # UFLD/TuSimple defines the two ego lanes as lane indices 1 and 2.
        # Prefer those external semantic tracks strongly, but retain geometric
        # validation so a pathological prediction cannot bypass lane-width checks.
        external_lane_bonus = 0.0
        if left["lane_id"] == 1:
            external_lane_bonus += 0.35
        if right["lane_id"] == 2:
            external_lane_bonus += 0.35
        confidence = left["confidence"] + right["confidence"]
        left_distance = abs(center - left["observed_bottom_x"]) / max(1.0, width)
        right_distance = abs(right["observed_bottom_x"] - center) / max(1.0, width)
        center_preference = -0.18 * (left_distance + right_distance)
        return confidence + external_lane_bonus + center_preference

    def _select_pair(self, candidates):
        width = self.processing_width
        center = width / 2.0
        guard = max(8.0, width * 0.015)
        lefts = [
            item
            for item in candidates
            if item["observed_bottom_x"] < center - guard
        ]
        rights = [
            item
            for item in candidates
            if item["observed_bottom_x"] > center + guard
        ]
        pairs = []
        for left in lefts:
            for right in rights:
                if left is right:
                    continue
                bottom_width = right["bottom_x"] - left["bottom_x"]
                top_width = right["top_x"] - left["top_x"]
                if bottom_width <= 20.0 or top_width <= 8.0:
                    continue
                ratio = bottom_width / max(1.0, top_width)
                if ratio < 1.01 or ratio > 12.0:
                    continue
                center_bottom = (left["bottom_x"] + right["bottom_x"]) / 2.0
                if center_bottom < -0.35 * width or center_bottom > 1.35 * width:
                    continue
                score = self._pair_score(left, right, center, width)
                pairs.append((score, left, right))
        if not pairs:
            return None, None
        pairs.sort(key=lambda item: item[0], reverse=True)
        _, left, right = pairs[0]
        return left, right

    def _project_line_to_raw(self, document, image_size):
        if not document or not document.get("points"):
            return document
        projected = distort_undistorted_points(
            self.camera_calibration,
            document["points"],
            image_size,
        )
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

    def _project_result_to_raw(self, result, calibrated):
        if not calibrated or not result.image_size:
            return result
        image_size = result.image_size
        return replace(
            result,
            left_line=self._project_line_to_raw(result.left_line, image_size),
            right_line=self._project_line_to_raw(result.right_line, image_size),
            center_line=self._project_line_to_raw(result.center_line, image_size),
        )

    def _neural_result(
        self,
        image,
        *,
        geometry=None,
        enforce_latency=True,
        preview=False,
    ):
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV/NumPy unavailable")
        geometry = geometry or self._neural_geometry
        source = image
        calibrated = self._calibration_usable(self.camera_calibration)
        if calibrated:
            source = self.camera_calibration.undistort(source)

        neural = self.pretrained.infer(source)
        lanes = neural.get("lanes") or []
        diagnostics = {
            key: value for key, value in neural.items() if key != "lanes"
        }
        diagnostics["coordinate_space"] = (
            "UNDISTORTED" if calibrated else "RAW"
        )
        diagnostics["candidate_lane_ids"] = [
            int(lane.get("lane_id", lane.get("query_id", -1))) for lane in lanes
        ]
        try:
            inference_ms = float(neural.get("inference_ms") or 0.0)
        except (TypeError, ValueError):
            inference_ms = float("inf")
        diagnostics["latency_allowed"] = bool(
            math.isfinite(inference_ms)
            and inference_ms <= self.maximum_neural_inference_ms
        )
        diagnostics["maximum_neural_inference_ms"] = self.maximum_neural_inference_ms
        if preview:
            self._last_preview = diagnostics
        else:
            self._last_neural = diagnostics

        if enforce_latency and not diagnostics["latency_allowed"]:
            self._neural_suspended_reason = (
                "NEURAL_INFERENCE_TOO_SLOW:"
                f"{inference_ms:.2f}ms>{self.maximum_neural_inference_ms:.2f}ms"
            )
            raise RuntimeError(self._neural_suspended_reason)
        if len(lanes) < 2:
            raise RuntimeError("NEURAL_TWO_BOUNDARIES_REQUIRED")

        width = self.processing_width
        height = self.processing_height
        roi_top = int(height * 0.42)
        roi_bottom = int(height * 0.985)
        roi_height = roi_bottom - roi_top
        candidates = self._candidate_fits(
            neural,
            source.shape,
            roi_top,
            roi_bottom,
            geometry,
        )
        left, right = self._select_pair(candidates)
        if left is None or right is None:
            raise RuntimeError("NEURAL_EGO_LANE_PAIR_REQUIRED")

        result = geometry._calculate(
            width,
            height,
            roi_height,
            roi_top,
            left["fit"],
            right["fit"],
            len(left["points"]),
            len(right["points"]),
            "NEURAL_LANE",
        )
        if not result.detected:
            raise RuntimeError(result.error or "NEURAL_GEOMETRY_INVALID")

        pair_confidence = min(left["confidence"], right["confidence"])
        confidence = max(
            0.0,
            min(
                float(result.confidence),
                0.58 + 0.42 * pair_confidence,
            ),
        )
        diagnostics.update(
            {
                "selected_left_lane_id": left["lane_id"],
                "selected_right_lane_id": right["lane_id"],
                "selected_left_confidence": left["confidence"],
                "selected_right_confidence": right["confidence"],
                "error": None,
            }
        )
        result = replace(
            result,
            confidence=confidence,
            backend=self.NEURAL_BACKEND,
            marking="UFLD_LANE",
        )
        return self._project_result_to_raw(result, calibrated)

    def _classical_result(self, image, fallback=False):
        result = self.classical.analyze_image(image)
        if fallback:
            return replace(result, backend="CLASSICAL_CV_FALLBACK")
        return result

    def analyze_neural_preview_jpeg(self, jpeg):
        """Run UFLD for display only without changing autonomous-control state."""
        with self._lock:
            if not jpeg:
                return LaneResult(
                    False,
                    0.0,
                    error="CAMERA_FRAME_UNAVAILABLE",
                    backend=self.NEURAL_BACKEND,
                )
            if cv2 is None or np is None:
                return LaneResult(
                    False,
                    0.0,
                    error="OPENCV_UNAVAILABLE",
                    backend=self.NEURAL_BACKEND,
                )
            image = cv2.imdecode(
                np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if image is None:
                return LaneResult(
                    False,
                    0.0,
                    error="JPEG_DECODE_FAILED",
                    backend=self.NEURAL_BACKEND,
                )
            suspended_before = self._neural_suspended_reason
            try:
                return self._neural_result(
                    image,
                    geometry=self._preview_geometry,
                    enforce_latency=False,
                    preview=True,
                )
            except Exception as error:
                if self._last_preview is None:
                    self._last_preview = {}
                self._last_preview["error"] = f"{type(error).__name__}: {error}"
                return LaneResult(
                    False,
                    0.0,
                    error=str(error),
                    backend=self.NEURAL_BACKEND,
                    marking="NEURAL_PREVIEW",
                    image_size=(self.processing_width, self.processing_height),
                )
            finally:
                # Diagnostic preview cannot modify the AUTO latency breaker.
                self._neural_suspended_reason = suspended_before

    def preview_snapshot(self):
        with self._lock:
            return {
                "backend": self.NEURAL_BACKEND,
                "control_authority": "NONE",
                "neural_enabled_for_control": self._neural_enabled,
                "maximum_neural_inference_ms": self.maximum_neural_inference_ms,
                **dict(self._last_preview or {}),
            }

    def analyze_image(self, image):
        if image is None:
            return LaneResult(
                False,
                0.0,
                error="CAMERA_FRAME_UNAVAILABLE",
                backend=self.BACKEND,
            )
        with self._lock:
            if self._neural_enabled:
                if self._neural_suspended_reason is not None:
                    self._fallback_reason = self._neural_suspended_reason
                    result = self._classical_result(image, fallback=True)
                    self._last_backend = result.backend
                    return result
                try:
                    result = self._neural_result(image)
                    self._last_backend = result.backend
                    self._fallback_reason = None
                    return result
                except Exception as error:
                    self._fallback_reason = f"{type(error).__name__}: {error}"
                    result = self._classical_result(image, fallback=True)
                    self._last_backend = result.backend
                    return result
            result = self._classical_result(image, fallback=False)
            self._last_backend = result.backend
            return result

    def analyze_jpeg(self, jpeg):
        if not jpeg:
            return LaneResult(
                False,
                0.0,
                error="CAMERA_FRAME_UNAVAILABLE",
                backend=self.BACKEND,
            )
        with self._lock:
            if jpeg is self._cached_jpeg and self._cached_result is not None:
                return self._cached_result
            if cv2 is None or np is None:
                result = LaneResult(
                    False,
                    0.0,
                    error="OPENCV_UNAVAILABLE",
                    backend=self.BACKEND,
                )
            else:
                image = cv2.imdecode(
                    np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                result = (
                    LaneResult(
                        False,
                        0.0,
                        error="JPEG_DECODE_FAILED",
                        backend=self.BACKEND,
                    )
                    if image is None
                    else self.analyze_image(image)
                )
            self._cached_jpeg = jpeg
            self._cached_result = result
            return result

    def snapshot(self):
        with self._lock:
            return {
                "available": self.available,
                "neural_enabled": self._neural_enabled,
                "backend": self._last_backend,
                "neural_backend": self.NEURAL_BACKEND,
                "fallback_reason": self._fallback_reason,
                "maximum_neural_inference_ms": self.maximum_neural_inference_ms,
                "neural_suspended_reason": self._neural_suspended_reason,
                "expected_lane_width_m": self.expected_lane_width_m,
                "vehicle_width_m": self.vehicle_width_m,
                "last_neural": self._last_neural,
                "last_preview": self._last_preview,
                "pretrained": self.pretrained.snapshot(),
            }


__all__ = ["HybridLaneController"]
