from collections import deque
from dataclasses import dataclass
import json
import math
import os
import sys
import time

from .features import LidarSectorizer, SECTOR_DEFINITIONS
from .ort_session import build_cpu_session_options
from autonomous_car.routes.gps_route import ROUTE_FEATURE_ORDER


@dataclass(frozen=True)
class GpsAiInference:
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


class GpsAiRuntime:
    EXPECTED_INPUTS = {"image", "lidar", "auxiliary", "route"}
    EXPECTED_OUTPUTS = {"control"}
    MEASURED_STEERING_SOURCE = "MEASURED_ENCODER"
    LEGACY_STEERING_SOURCE = "MODEL_PREDICTION"

    def __init__(self, model_path, manifest_path=None, providers=None):
        try:
            import cv2
            import numpy as np
            import onnxruntime as ort
        except ImportError as error:
            raise ValueError(
                "AUTO_GPS_DEPENDENCY_MISSING: OpenCV, NumPy and ONNX Runtime are required"
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
            if self.manifest.get("policy_type") != "AUTO_GPS":
                raise ValueError("Selected model is not an AUTO_GPS model")
            spec = self.manifest["model_spec"]
            self.image_width = int(spec["image_width"])
            self.image_height = int(spec["image_height"])
            self.maximum_steering_degrees = abs(float(spec["maximum_steering_degrees"]))
            self.maximum_abs_yaw_rate_dps = abs(float(spec["maximum_abs_yaw_rate_dps"]))
            self.lidar_maximum_distance_m = abs(float(spec["lidar_maximum_distance_m"]))
            self.route_feature_size = int(
                spec.get("route_feature_size", len(ROUTE_FEATURE_ORDER))
            )
            self.temporal_history_steps = max(1, int(spec.get("temporal_history_steps") or 1))
            self.auxiliary_feature_size = int(spec.get("auxiliary_feature_size") or 2)
            self.temporal_auxiliary = self.auxiliary_feature_size > 2
            auxiliary_contract = (
                ((self.manifest.get("inputs") or {}).get("auxiliary") or {})
                if isinstance(self.manifest.get("inputs") or {}, dict)
                else {}
            )
            if self.temporal_auxiliary:
                if self.temporal_history_steps < 2:
                    raise ValueError("AUTO_GPS temporal history must contain multiple steps")
                if self.auxiliary_feature_size != self.temporal_history_steps * 4:
                    raise ValueError("AUTO_GPS temporal auxiliary size mismatch")
                # Temporal v2 models created before the measured-feedback contract
                # fed their own previous prediction back into the next frame. Keep
                # those already-installed models loadable, but new v3 manifests
                # explicitly request encoder-measured steering instead.
                source = str(
                    auxiliary_contract.get("steering_history_source")
                    or self.LEGACY_STEERING_SOURCE
                ).strip().upper()
                if source not in {
                    self.MEASURED_STEERING_SOURCE,
                    self.LEGACY_STEERING_SOURCE,
                }:
                    raise ValueError(
                        f"AUTO_GPS temporal steering history source unsupported: {source}"
                    )
                self.steering_history_source = source
            elif self.auxiliary_feature_size != 2:
                raise ValueError("AUTO_GPS legacy auxiliary size must be 2")
            else:
                self.steering_history_source = "NONE"
            self.requires_measured_steering = (
                self.temporal_auxiliary
                and self.steering_history_source == self.MEASURED_STEERING_SOURCE
            )
            order = (
                ((self.manifest.get("inputs") or {}).get("route") or {}).get("feature_order")
                or list(ROUTE_FEATURE_ORDER)
            )
            if (
                list(order) != list(ROUTE_FEATURE_ORDER)
                or self.route_feature_size != len(ROUTE_FEATURE_ORDER)
            ):
                raise ValueError("AUTO_GPS route feature contract mismatch")
            if self.image_width <= 0 or self.image_height <= 0:
                raise ValueError("image dimensions must be positive")
            if self.maximum_steering_degrees <= 0.0:
                raise ValueError("maximum steering must be positive")
            if self.lidar_maximum_distance_m <= 0.0:
                raise ValueError("LiDAR maximum distance must be positive")
            self.route_id = self.manifest.get("route_id")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise ValueError(f"AUTO_GPS_MANIFEST_INVALID: {error}") from error

        self._yaw_history = deque(maxlen=self.temporal_history_steps)
        self._steering_history = deque(maxlen=self.temporal_history_steps)
        self.sectorizer = LidarSectorizer(
            maximum_distance_m=self.lidar_maximum_distance_m,
            minimum_confidence=35,
        )

        options, self.threading_config = build_cpu_session_options(ort)
        available = set(ort.get_available_providers())
        requested = (
            ["CPUExecutionProvider"]
            if providers is None
            else [provider for provider in providers if provider in available]
        )
        if not requested:
            requested = ["CPUExecutionProvider"]
        try:
            self.session = ort.InferenceSession(
                self.model_path,
                sess_options=options,
                providers=requested,
            )
        except Exception as error:
            message = str(error).strip()
            if "external data" in message.lower() or ".onnx.data" in message.lower():
                code = "AUTO_GPS_EXTERNAL_DATA_MISSING"
            else:
                code = "AUTO_GPS_MODEL_LOAD_FAILED"
            raise ValueError(f"{code}: {message or type(error).__name__}") from error
        self.providers = self.session.get_providers()
        self._validate_onnx_contract()

    def _validate_onnx_contract(self):
        inputs = {item.name for item in self.session.get_inputs()}
        outputs = {item.name for item in self.session.get_outputs()}
        if inputs != self.EXPECTED_INPUTS or not self.EXPECTED_OUTPUTS.issubset(outputs):
            raise ValueError(
                "AUTO_GPS_ONNX_CONTRACT_MISMATCH: "
                f"inputs={sorted(inputs)}, outputs={sorted(outputs)}"
            )
        auxiliary = next((item for item in self.session.get_inputs() if item.name == "auxiliary"), None)
        if auxiliary is not None:
            shape = list(getattr(auxiliary, "shape", ()) or ())
            if len(shape) >= 2 and isinstance(shape[-1], int) and shape[-1] != self.auxiliary_feature_size:
                raise ValueError(
                    "AUTO_GPS_AUXILIARY_SHAPE_MISMATCH: "
                    f"onnx={shape[-1]}, manifest={self.auxiliary_feature_size}"
                )

    def reset_temporal_state(self):
        self._yaw_history.clear()
        self._steering_history.clear()

    def infer_jpeg(
        self,
        jpeg_bytes,
        lidar_points,
        imu_yaw_rate_dps,
        route_features,
        person_hazard=False,
        measured_steering_degrees=None,
    ):
        if person_hazard:
            self.reset_temporal_state()
            return GpsAiInference(0.0, 0.0, 0.0, True, 0.0, 0)
        image = self._image_tensor(jpeg_bytes)
        features = self.sectorizer.transform(lidar_points or [])
        lidar = self._lidar_tensor(features)
        auxiliary = self._auxiliary_tensor(imu_yaw_rate_dps)
        route = self._route_tensor(route_features)
        started = time.perf_counter()
        output = self.session.run(
            ["control"],
            {"image": image, "lidar": lidar, "auxiliary": auxiliary, "route": route},
        )
        elapsed = time.perf_counter() - started
        if not output:
            raise RuntimeError("AUTO_GPS model returned no output")
        control = self.np.asarray(output[0], dtype=self.np.float32).reshape(-1)
        if control.size < 2:
            raise RuntimeError("AUTO_GPS output requires steering and throttle")
        steering = self._clamp(float(control[0]), -1.0, 1.0)
        throttle = self._clamp(float(control[1]), -1.0, 1.0)
        if not math.isfinite(steering) or not math.isfinite(throttle):
            raise RuntimeError("AUTO_GPS produced non-finite output")
        steering_degrees = steering * self.maximum_steering_degrees
        if self.temporal_auxiliary:
            if self.requires_measured_steering:
                measured = self._resolve_measured_steering(measured_steering_degrees)
                self._steering_history.append(measured)
            else:
                # Compatibility only for temporal v2 manifests. New models must
                # never feed a model prediction back as if it were measured state.
                self._steering_history.append(steering_degrees)
        return GpsAiInference(
            steering_degrees,
            throttle,
            steering,
            False,
            elapsed,
            sum(features.observed.values()),
        )

    def snapshot(self):
        return {
            "model_path": self.model_path,
            "manifest_path": self.manifest_path,
            "providers": list(self.providers),
            "route_id": self.route_id,
            "image_size": [self.image_width, self.image_height],
            "temporal_auxiliary": self.temporal_auxiliary,
            "temporal_history_steps": self.temporal_history_steps,
            "auxiliary_feature_size": self.auxiliary_feature_size,
            "steering_history_source": self.steering_history_source,
            "requires_measured_steering": self.requires_measured_steering,
            "temporal_history_filled": {
                "yaw": len(self._yaw_history),
                "steering": len(self._steering_history),
            },
            "onnx_threading": dict(self.threading_config),
        }

    def _image_tensor(self, jpeg_bytes):
        if not jpeg_bytes:
            raise ValueError("Camera JPEG required")
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

    @staticmethod
    def _finite(value):
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return value if math.isfinite(value) else None

    @staticmethod
    def _left_pad(values, size):
        values = list(values)[-size:]
        return [None] * max(0, size - len(values)) + values

    def _resolve_measured_steering(self, explicit_value):
        value = self._finite(explicit_value)
        if value is not None:
            return value

        # The production rover already has the hardware controller loaded as
        # module ``server``. Read it lazily instead of importing server here, so
        # the same runtime stays usable on the Windows preview/training machine.
        # Callers such as RECORD preview should pass the recorded encoder angle
        # explicitly and never depend on this live fallback.
        server_module = sys.modules.get("server")
        controller = getattr(server_module, "motor_controller", None)
        if controller is None:
            return None
        try:
            snapshot = controller.snapshot()
        except Exception:
            return None
        if snapshot.get("encoder_connected") is False:
            return None
        return self._finite(snapshot.get("steering_angle_degrees"))

    def _auxiliary_tensor(self, value):
        if not self.temporal_auxiliary:
            value = self._finite(value)
            if value is None:
                return self.np.asarray([[0.0, 0.0]], dtype=self.np.float32)
            maximum = max(1e-6, self.maximum_abs_yaw_rate_dps)
            return self.np.asarray(
                [[self._clamp(value / maximum, -1.0, 1.0), 1.0]],
                dtype=self.np.float32,
            )

        self._yaw_history.append(self._finite(value))
        yaw = self._left_pad(self._yaw_history, self.temporal_history_steps)
        steering = self._left_pad(self._steering_history, self.temporal_history_steps)
        maximum_yaw = max(1e-6, self.maximum_abs_yaw_rate_dps)
        maximum_steering = max(1e-6, self.maximum_steering_degrees)
        vector = []
        for yaw_value, steering_value in zip(yaw, steering):
            vector.extend(
                [
                    0.0 if yaw_value is None else self._clamp(yaw_value / maximum_yaw, -1.0, 1.0),
                    0.0 if yaw_value is None else 1.0,
                    0.0 if steering_value is None else self._clamp(steering_value / maximum_steering, -1.0, 1.0),
                    0.0 if steering_value is None else 1.0,
                ]
            )
        if len(vector) != self.auxiliary_feature_size:
            raise RuntimeError("AUTO_GPS temporal auxiliary construction failed")
        return self.np.asarray([vector], dtype=self.np.float32)

    def _route_tensor(self, route_features):
        values = getattr(route_features, "normalized", None)
        if values is None and isinstance(route_features, dict):
            values = route_features.get("normalized")
        if values is None or len(values) != self.route_feature_size:
            raise ValueError("AUTO_GPS route features missing")
        return self.np.asarray([[float(value) for value in values]], dtype=self.np.float32)

    @staticmethod
    def _clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))