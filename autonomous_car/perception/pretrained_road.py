"""External Ultra-Fast-Lane-Detection ONNX perception for no-training AUTO.

The detector/model contract is the open-source Ultra-Fast-Lane-Detection (UFLD)
TuSimple ResNet18 pipeline, not a SWING-trained network. The preprocessing and
decoding contract is owned by the vendored ``third_party.ufld`` adapter so the
runtime, regression tests and attribution all use one implementation.

SWING adapts the external lane points into its common geometry, calibration and
safety interfaces. Model artifact installation is handled by
``scripts/install_pretrained_road_model.sh``.
"""

from __future__ import annotations

import os
import threading
import time

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - surfaced through available/snapshot
    cv2 = None
    np = None

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover
    ort = None

try:
    from third_party.ufld import (
        ANCHOR_COUNT,
        GRIDING_NUM,
        INPUT_HEIGHT,
        INPUT_WIDTH,
        LANE_COUNT,
        decode_tusimple_output,
        prepare_tusimple_input,
    )
except ImportError:  # pragma: no cover - dependency status is reported cleanly
    ANCHOR_COUNT = 56
    GRIDING_NUM = 100
    INPUT_HEIGHT = 288
    INPUT_WIDTH = 800
    LANE_COUNT = 4
    decode_tusimple_output = None
    prepare_tusimple_input = None


DEFAULT_MODEL_NAME = "UFLD_TUSIMPLE_RES18_288X800"
DEFAULT_MODEL_FILENAME = "ultrafast_lane_tusimple_288x800.onnx"
DEFAULT_MODEL_INPUT = (INPUT_WIDTH, INPUT_HEIGHT)
DEFAULT_SOURCE_ARCHIVE_URL = (
    "https://s3.ap-northeast-2.wasabisys.com/"
    "pinto-model-zoo/140_Ultra-Fast-Lane-Detection/resources_tusimple.tar.gz"
)
DEFAULT_SOURCE_MEMBER = ""


class PretrainedRoadPerception:
    """Lazy ONNX Runtime wrapper for external UFLD TuSimple ResNet18."""

    def __init__(
        self,
        model_path,
        input_size=DEFAULT_MODEL_INPUT,
        threads=2,
        lane_probability_threshold=0.55,
    ):
        self.model_path = os.path.abspath(str(model_path))
        self.input_width = int(input_size[0])
        self.input_height = int(input_size[1])
        if (self.input_width, self.input_height) != (INPUT_WIDTH, INPUT_HEIGHT):
            raise ValueError(
                "UFLD_TUSIMPLE_INPUT_SIZE_FIXED:"
                f"{self.input_width}x{self.input_height};"
                f"expected {INPUT_WIDTH}x{INPUT_HEIGHT}"
            )
        self.threads = max(1, int(threads))
        # Kept under the historical option name for API compatibility. For UFLD
        # it gates mean per-anchor lane confidence after the external decoder.
        self.lane_probability_threshold = max(
            0.05, min(0.99, float(lane_probability_threshold))
        )
        self._lock = threading.RLock()
        self._session = None
        self._input_name = None
        self._output_name = None
        self._error = None
        self._last_inference_ms = None
        self._last_lane_count = 0
        self._last_max_lane_probability = None
        self._runs = 0

    @property
    def available(self):
        return bool(
            cv2 is not None
            and np is not None
            and ort is not None
            and prepare_tusimple_input is not None
            and decode_tusimple_output is not None
            and os.path.isfile(self.model_path)
        )

    @property
    def loaded(self):
        return self._session is not None

    @staticmethod
    def _static_dimension(value):
        return value if isinstance(value, int) and value > 0 else None

    def ensure_loaded(self):
        with self._lock:
            if self._session is not None:
                return True
            self._error = None
            if cv2 is None or np is None:
                self._error = "OpenCV/NumPy unavailable"
                return False
            if ort is None:
                self._error = "onnxruntime unavailable"
                return False
            if prepare_tusimple_input is None or decode_tusimple_output is None:
                self._error = "vendored UFLD decoder unavailable"
                return False
            if not os.path.isfile(self.model_path):
                self._error = f"model missing: {self.model_path}"
                return False
            try:
                options = ort.SessionOptions()
                options.intra_op_num_threads = self.threads
                options.inter_op_num_threads = 1
                options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
                options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                try:
                    options.add_session_config_entry("session.intra_op.allow_spinning", "0")
                    options.add_session_config_entry("session.inter_op.allow_spinning", "0")
                except Exception:
                    pass
                session = ort.InferenceSession(
                    self.model_path,
                    sess_options=options,
                    providers=["CPUExecutionProvider"],
                )
                inputs = session.get_inputs()
                outputs = session.get_outputs()
                if len(inputs) != 1 or len(outputs) != 1:
                    raise RuntimeError(
                        f"UFLD expects 1 input/1 output, got {len(inputs)}/{len(outputs)}"
                    )
                image_input = inputs[0]
                output = outputs[0]
                input_shape = list(image_input.shape or ())
                if len(input_shape) != 4:
                    raise RuntimeError(f"unexpected UFLD input shape: {input_shape}")
                channels = self._static_dimension(input_shape[1])
                height = self._static_dimension(input_shape[2])
                width = self._static_dimension(input_shape[3])
                if channels not in {None, 3}:
                    raise RuntimeError(f"unexpected UFLD input channels: {input_shape}")
                if height is not None and height != INPUT_HEIGHT:
                    raise RuntimeError(
                        f"unexpected UFLD input height {height}; expected {INPUT_HEIGHT}"
                    )
                if width is not None and width != INPUT_WIDTH:
                    raise RuntimeError(
                        f"unexpected UFLD input width {width}; expected {INPUT_WIDTH}"
                    )
                output_shape = list(output.shape or ())
                static_tail = [self._static_dimension(value) for value in output_shape[-3:]]
                expected_tail = [GRIDING_NUM + 1, ANCHOR_COUNT, LANE_COUNT]
                if (
                    len(output_shape) != 4
                    or all(value is not None for value in static_tail)
                    and static_tail != expected_tail
                ):
                    raise RuntimeError(
                        f"unexpected UFLD output shape {output_shape}; "
                        f"expected [1,{GRIDING_NUM + 1},{ANCHOR_COUNT},{LANE_COUNT}]"
                    )
                self._session = session
                self._input_name = str(image_input.name)
                self._output_name = str(output.name)
                return True
            except Exception as error:
                self._session = None
                self._error = f"{type(error).__name__}: {error}"
                return False

    def unload(self):
        with self._lock:
            self._session = None
            self._input_name = None
            self._output_name = None

    def _prepare_input(self, image):
        tensor = prepare_tusimple_input(image, cv2)
        expected = (1, 3, INPUT_HEIGHT, INPUT_WIDTH)
        if tuple(tensor.shape) != expected:
            raise RuntimeError(
                f"vendored UFLD preprocessing returned {tuple(tensor.shape)}; "
                f"expected {expected}"
            )
        return tensor

    def _decode(self, output, image_size):
        return decode_tusimple_output(
            output,
            image_size,
            confidence_threshold=self.lane_probability_threshold,
        )

    def infer(self, image):
        if image is None:
            raise ValueError("camera image unavailable")
        with self._lock:
            if not self.ensure_loaded():
                raise RuntimeError(self._error or "pretrained lane model unavailable")
            image_tensor = self._prepare_input(image)
            started = time.perf_counter()
            output = self._session.run(
                [self._output_name],
                {self._input_name: image_tensor},
            )[0]
            inference_ms = (time.perf_counter() - started) * 1000.0
            height, width = image.shape[:2]
            lanes, confidences = self._decode(output, (width, height))
            maximum = max(confidences) if confidences else None
            self._last_inference_ms = float(inference_ms)
            self._last_lane_count = len(lanes)
            self._last_max_lane_probability = maximum
            self._runs += 1
            return {
                "lanes": lanes,
                "inference_ms": float(inference_ms),
                "lane_count": len(lanes),
                "max_lane_probability": maximum,
                "model": DEFAULT_MODEL_NAME,
                "decoder": "EXTERNAL_UFLD_TUSIMPLE",
                "decoder_adapter": "third_party.ufld",
            }

    def snapshot(self):
        with self._lock:
            return {
                "model": DEFAULT_MODEL_NAME,
                "model_path": self.model_path,
                "model_present": os.path.isfile(self.model_path),
                "available": self.available,
                "loaded": self.loaded,
                "input_size": [self.input_width, self.input_height],
                "expected_output": [1, GRIDING_NUM + 1, ANCHOR_COUNT, LANE_COUNT],
                "threads": self.threads,
                "lane_probability_threshold": self.lane_probability_threshold,
                "decoder_adapter": "third_party.ufld",
                "input": self._input_name,
                "output": self._output_name,
                "last_inference_ms": self._last_inference_ms,
                "last_lane_count": self._last_lane_count,
                "last_max_lane_probability": self._last_max_lane_probability,
                "runs": self._runs,
                "error": self._error,
            }


__all__ = [
    "DEFAULT_MODEL_FILENAME",
    "DEFAULT_MODEL_INPUT",
    "DEFAULT_MODEL_NAME",
    "DEFAULT_SOURCE_ARCHIVE_URL",
    "DEFAULT_SOURCE_MEMBER",
    "PretrainedRoadPerception",
]
