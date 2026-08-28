"""Ultra-Fast-Lane-Detection TuSimple ONNX preprocessing/decoder.

This module is an adapted, dependency-light form of the MIT-licensed public ONNX
inference implementation from:
https://github.com/ibaiGorordo/onnx-Ultra-Fast-Lane-Detection-Inference
which implements the MIT-licensed detector from:
https://github.com/cfzd/Ultra-Fast-Lane-Detection

Changes are limited to removing visualization/debug/scipy dependencies, accepting
arbitrary source image dimensions, and returning dictionaries for the host
application. The UFLD griding/row-anchor decoding procedure is preserved.
See LICENSE and NOTICE.md in this directory.
"""

from __future__ import annotations

import numpy as np


TUSIMPLE_ROW_ANCHORS = np.asarray(list(range(64, 285, 4)), dtype=np.float32)
GRIDING_NUM = 100
LANE_COUNT = 4
ANCHOR_COUNT = 56
INPUT_WIDTH = 800
INPUT_HEIGHT = 288
INPUT_MEAN = np.asarray((0.485, 0.456, 0.406), dtype=np.float32)
INPUT_STD = np.asarray((0.229, 0.224, 0.225), dtype=np.float32)


def prepare_tusimple_input(image, cv2_module):
    """Apply the public UFLD ONNX input preprocessing to an OpenCV BGR frame."""
    rgb = cv2_module.cvtColor(image, cv2_module.COLOR_BGR2RGB)
    resized = cv2_module.resize(
        rgb,
        (INPUT_WIDTH, INPUT_HEIGHT),
        interpolation=cv2_module.INTER_AREA,
    ).astype(np.float32)
    normalized = (resized / 255.0 - INPUT_MEAN) / INPUT_STD
    return np.ascontiguousarray(
        normalized.transpose(2, 0, 1)[None, ...],
        dtype=np.float32,
    )


def _softmax(values, axis=0):
    values = np.asarray(values, dtype=np.float32)
    shifted = values - np.max(values, axis=axis, keepdims=True)
    exponential = np.exp(shifted)
    return exponential / np.maximum(
        np.sum(exponential, axis=axis, keepdims=True),
        1e-12,
    )


def decode_tusimple_output(output, image_size, confidence_threshold=0.55):
    """Decode UFLD [1,101,56,4] output using the published TuSimple anchors."""
    values = np.asarray(output, dtype=np.float32)
    if values.ndim == 4 and values.shape[0] == 1:
        values = values[0]
    expected = (GRIDING_NUM + 1, ANCHOR_COUNT, LANE_COUNT)
    if values.shape != expected:
        raise RuntimeError(f"unsupported UFLD output shape: {values.shape}; expected {expected}")

    # Public decoder reverses the anchor dimension before recovering locations.
    values = values[:, ::-1, :]
    spatial_probability = _softmax(values[:-1, :, :], axis=0)
    grid_index = np.arange(1, GRIDING_NUM + 1, dtype=np.float32).reshape(-1, 1, 1)
    locations = np.sum(spatial_probability * grid_index, axis=0)
    classes = np.argmax(values, axis=0)
    valid = classes != GRIDING_NUM
    locations[~valid] = 0.0

    # The no-lane class is useful only as a diagnostic confidence gate; lane
    # coordinates themselves follow the external expectation/argmax decoder.
    all_probability = _softmax(values, axis=0)
    width, height = int(image_size[0]), int(image_size[1])
    column_sample = np.linspace(0.0, INPUT_WIDTH - 1.0, GRIDING_NUM, dtype=np.float32)
    column_step = float(column_sample[1] - column_sample[0])

    lanes = []
    confidences = []
    for lane_id in range(LANE_COUNT):
        lane_valid = valid[:, lane_id]
        support = int(np.count_nonzero(lane_valid))
        if support <= 2:
            continue
        anchor_confidence = 1.0 - all_probability[GRIDING_NUM, :, lane_id]
        confidence = float(np.mean(anchor_confidence[lane_valid]))
        if confidence < float(confidence_threshold):
            continue

        points = []
        for point_index in range(ANCHOR_COUNT):
            location = float(locations[point_index, lane_id])
            if location <= 0.0:
                continue
            x = location * column_step * width / float(INPUT_WIDTH) - 1.0
            source_anchor = ANCHOR_COUNT - 1 - point_index
            y = height * float(TUSIMPLE_ROW_ANCHORS[source_anchor]) / float(INPUT_HEIGHT) - 1.0
            if -0.05 * width <= x <= 1.05 * width and 0.0 <= y <= height * 1.02:
                points.append([float(x), float(y)])
        if len(points) < 3:
            continue
        lanes.append(
            {
                "lane_id": int(lane_id),
                "query_id": int(lane_id),
                "confidence": max(0.0, min(1.0, confidence)),
                "support": support,
                "points": points,
            }
        )
        confidences.append(confidence)

    return lanes, confidences


__all__ = [
    "ANCHOR_COUNT",
    "GRIDING_NUM",
    "INPUT_HEIGHT",
    "INPUT_WIDTH",
    "LANE_COUNT",
    "TUSIMPLE_ROW_ANCHORS",
    "decode_tusimple_output",
    "prepare_tusimple_input",
]
