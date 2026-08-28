from dataclasses import dataclass
import json
import math
import os
import time

from .features import LidarSectorizer, SECTOR_DEFINITIONS
from .ort_session import build_cpu_session_options


class InferenceDependencyError(ValueError):
    """Vehicle inference dependency problem that can be returned as HTTP 409."""


@dataclass(frozen=True)
class AutoAiInference:
    steering_degrees: float
    throttle: float
    normalized_steering: float
    person_stop: bool
    inference_seconds: float
    lidar_observed_sectors: int

    def as_dict(self):
        return {
            "steering_degrees": self.steering_degrees,
            "throttle": self.throttle,
            "normalized_steering": self.normalized_steering,
            "person_stop": self.person_stop,
            "inference_seconds": self.inference_seconds,
            "lidar_observed_sectors": self.lidar_observed_sectors,
        }


class AutoAiRuntime:
    """ONNX Runtime inference for learned driving on the vehicle.

    General obstacle behavior is intentionally not overridden here. The model
    sees the LiDAR sector features and owns ordinary driving/avoidance. A person
    hazard is the explicit external STOP exception agreed for AUTO_AI.
    """

    EXPECTED_INPUTS = {"image", "lidar", "auxiliary"}
    EXPECTED_OUTPUTS = {"control"}

    def __init__(self, model_path, manifest_path=None, providers=None):
        try:
            import cv2
            import numpy as np
            import onnxruntime as ort
        except ImportError as error:
            raise InferenceDependencyError(
                "AUTO_AI_DEPENDENCY_MISSING: OpenCV, NumPy and ONNX Runtime are required"
            ) from error

        self.cv2 = cv2
        self.np = np
        self.ort = ort
        self.model_path = os.path.abspath(model_path)
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(self.model_path)
        self.manifest_path = os.path.abspath(
            manifest_path or os.path.join(os.path.dirname(self.model_path), "model_manifest.json")
        )
        if not os.path.isfile(self.manifest_path):
            raise FileNotFoundError(self.manifest_path)
        try:
            with open(self.manifest_path, "r", encoding="utf-8") as file:
                self.manifest = json.load(file)
            spec = self.manifest["model_spec"]
            self.image_width = int(spec["image_width"])
            self.image_height = int(spec["image_height"])
            self.maximum_steering_degrees = abs(float(spec["maximum_steering_degrees"]))
            self.maximum_abs_yaw_rate_dps = abs(float(spec["maximum_abs_yaw_rate_dps"]))
            self.lidar_maximum_distance_m = abs(float(spec["lidar_maximum_distance_m"]))
            if self.image_width <= 0 or self.image_height <= 0:
                raise ValueError("image dimensions must be positive")
            if self.maximum_steering_degrees <= 0.0:
                raise ValueError("maximum steering must be positive")
            if self.lidar_maximum_distance_m <= 0.0:
                raise ValueError("LiDAR maximum distance must be positive")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise ValueError(f"AUTO_AI_MANIFEST_INVALID: {error}") from error

        self.sectorizer = LidarSectorizer(
            maximum_distance_m=self.lidar_maximum_distance_m,
            minimum_confidence=35,
        )

        session_options, self.threading_config = build_cpu_session_options(ort)
        available = set(ort.get_available_providers())
        if providers is None:
            requested = ["CPUExecutionProvider"]
        else:
            requested = [provider for provider in providers if provider in available]
            if not requested:
                requested = ["CPUExecutionProvider"]
        try:
            self.session = ort.InferenceSession(
                self.model_path,
                sess_options=session_options,
                providers=requested,
            )
        except Exception as error:
            message = str(error).strip()
            if "external data" in message.lower() or ".onnx.data" in message.lower():
                code = "AUTO_AI_EXTERNAL_DATA_MISSING"
            else:
                code = "AUTO_AI_MODEL_LOAD_FAILED"
            raise ValueError(f"{code}: {message or type(error).__name__}") from error
        self.providers = self.session.get_providers()
        self._validate_onnx_contract()

    def _validate_onnx_contract(self):
        inputs = {item.name for item in self.session.get_inputs()}
        outputs = {item.name for item in self.session.get_outputs()}
        if inputs != self.EXPECTED_INPUTS or not self.EXPECTED_OUTPUTS.issubset(outputs):
            raise ValueError(
                "AUTO_AI_ONNX_CONTRACT_MISMATCH: "
                f"inputs={sorted(inputs)}, outputs={sorted(outputs)}"
            )

    def infer_jpeg(
        self,
        jpeg_bytes,
        lidar_points,
        imu_yaw_rate_dps=None,
        *,
        person_hazard=False,
    ):
        if person_hazard:
            return AutoAiInference(
                steering_degrees=0.0,
                throttle=0.0,
                normalized_steering=0.0,
                person_stop=True,
                inference_seconds=0.0,
                lidar_observed_sectors=0,
            )

        image = self._image_tensor(jpeg_bytes)
        lidar_features = self.sectorizer.transform(lidar_points or [])
        lidar = self._lidar_tensor(lidar_features)
        auxiliary = self._auxiliary_tensor(imu_yaw_rate_dps)

        started = time.perf_counter()
        outputs = self.session.run(
            ["control"],
            {
                "image": image,
                "lidar": lidar,
                "auxiliary": auxiliary,
            },
        )
        elapsed = time.perf_counter() - started
        if not outputs:
            raise RuntimeError("AUTO_AI model returned no outputs")
        control = self.np.asarray(outputs[0], dtype=self.np.float32).reshape(-1)
        if control.size < 2:
            raise RuntimeError("AUTO_AI model control output must contain steering and throttle")
        normalized_steering = self._clamp(float(control[0]), -1.0, 1.0)
        throttle = self._clamp(float(control[1]), -1.0, 1.0)
        if not math.isfinite(normalized_steering) or not math.isfinite(throttle):
            raise RuntimeError("AUTO_AI model produced a non-finite control value")
        return AutoAiInference(
            steering_degrees=normalized_steering * self.maximum_steering_degrees,
            throttle=throttle,
            normalized_steering=normalized_steering,
            person_stop=False,
            inference_seconds=elapsed,
            lidar_observed_sectors=sum(lidar_features.observed.values()),
        )

    def snapshot(self):
        return {
            "model_path": self.model_path,
            "manifest_path": self.manifest_path,
            "providers": list(self.providers),
            "image_size": [self.image_width, self.image_height],
            "maximum_steering_degrees": self.maximum_steering_degrees,
            "lidar_maximum_distance_m": self.lidar_maximum_distance_m,
            "onnx_threading": dict(self.threading_config),
        }

    def _image_tensor(self, jpeg_bytes):
        if not jpeg_bytes:
            raise ValueError("Camera JPEG is required for AUTO_AI inference")
        encoded = self.np.frombuffer(jpeg_bytes, dtype=self.np.uint8)
        frame = self.cv2.imdecode(encoded, self.cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("Unable to decode camera JPEG")
        frame = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
        frame = self.cv2.resize(
            frame,
            (self.image_width, self.image_height),
            interpolation=self.cv2.INTER_AREA,
        )
        frame = frame.astype(self.np.float32) / 255.0
        return self.np.transpose(frame, (2, 0, 1))[None, ...]

    def _lidar_tensor(self, features):
        maximum = max(0.01, self.lidar_maximum_distance_m)
        vector = [
            self._clamp(float(features.distances_m[name]) / maximum, 0.0, 1.0)
            for name, _ in SECTOR_DEFINITIONS
        ]
        vector.extend(
            1.0 if features.observed[name] else 0.0
            for name, _ in SECTOR_DEFINITIONS
        )
        return self.np.asarray([vector], dtype=self.np.float32)

    def _auxiliary_tensor(self, imu_yaw_rate_dps):
        if imu_yaw_rate_dps is None:
            return self.np.asarray([[0.0, 0.0]], dtype=self.np.float32)
        try:
            value = float(imu_yaw_rate_dps)
        except (TypeError, ValueError):
            return self.np.asarray([[0.0, 0.0]], dtype=self.np.float32)
        if not math.isfinite(value):
            return self.np.asarray([[0.0, 0.0]], dtype=self.np.float32)
        maximum = max(1e-6, self.maximum_abs_yaw_rate_dps)
        return self.np.asarray(
            [[self._clamp(value / maximum, -1.0, 1.0), 1.0]],
            dtype=self.np.float32,
        )

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))
