"""Regression for UFLD-primary geometry, latency breaker and classical fallback."""

from autonomous_car.control.hybrid_lane_controller import HybridLaneController


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


class _FakePretrained:
    def __init__(self):
        self.fail = False
        self.calls = 0
        self.inference_ms = 8.0

    def infer(self, image):
        self.calls += 1
        if self.fail:
            raise RuntimeError("synthetic neural failure")
        h, w = image.shape[:2]
        ys = [h * (0.42 + index * (0.56 / 55.0)) for index in range(56)]
        left = [[w * (0.44 - 0.38 * (y / h)), y] for y in ys]
        right = [[w * (0.56 + 0.38 * (y / h)), y] for y in ys]
        return {
            "lanes": [
                {"lane_id": 1, "query_id": 1, "confidence": 0.96, "points": left},
                {"lane_id": 2, "query_id": 2, "confidence": 0.95, "points": right},
            ],
            "inference_ms": self.inference_ms,
            "lane_count": 2,
            "max_lane_probability": 0.96,
            "model": "FAKE_UFLD",
            "decoder": "EXTERNAL_UFLD_TUSIMPLE",
        }

    def snapshot(self):
        return {
            "available": True,
            "loaded": True,
            "calls": self.calls,
            "last_inference_ms": self.inference_ms,
            "error": None,
        }

    def ensure_loaded(self):
        return True


def _black_tape_image():
    import cv2
    import numpy as np

    image = np.full((360, 640, 3), 185, dtype=np.uint8)
    cv2.line(image, (14, 354), (250, 154), (18, 18, 18), 10)
    cv2.line(image, (626, 354), (390, 154), (18, 18, 18), 10)
    return image


def _validate_fit_candidates(candidates, label):
    _require(len(candidates) == 2, f"expected two {label} UFLD fits: {candidates}")
    _require({item["lane_id"] for item in candidates} == {1, 2}, candidates)
    for candidate in candidates:
        fit = candidate["fit"]
        _require(isinstance(fit, dict), f"{label} fit metadata contract changed: {fit!r}")
        _require("coefficients" in fit, f"{label} fit coefficients missing: {fit!r}")
        for key in ("observed_bottom_x", "bottom_x", "top_x"):
            _require(
                isinstance(candidate[key], float),
                f"{label} fit evaluation did not return scalar {key}: {candidate}",
            )


def main():
    import cv2

    fake = _FakePretrained()
    controller = HybridLaneController(
        fake,
        processing_width=640,
        processing_height=360,
        maximum_neural_inference_ms=160.0,
    )
    _require(controller.available, "OpenCV/NumPy are required")
    image = _black_tape_image()

    # UFLD tracks are external detector output. SWING fits them only into the
    # common LaneResult geometry, with separate histories for control/preview.
    raw = fake.infer(image)
    roi_top = int(controller.processing_height * 0.42)
    roi_bottom = int(controller.processing_height * 0.985)
    control_candidates = controller._candidate_fits(
        raw, image.shape, roi_top, roi_bottom, controller._neural_geometry
    )
    preview_candidates = controller._candidate_fits(
        raw, image.shape, roi_top, roi_bottom, controller._preview_geometry
    )
    _validate_fit_candidates(control_candidates, "control")
    _validate_fit_candidates(preview_candidates, "preview")
    _require(
        controller._neural_geometry is not controller._preview_geometry,
        "preview and control geometry histories must be isolated",
    )

    controller.set_neural_enabled(True)
    neural = controller.analyze_image(image)
    _require(neural.detected, f"neural geometry did not lock: {neural}")
    _require(neural.backend == "UFLD_ONNX", f"wrong primary backend: {neural.backend}")
    _require(neural.marking == "UFLD_LANE", f"wrong neural marking: {neural.marking}")
    snapshot = controller.snapshot()
    last_neural = snapshot.get("last_neural") or {}
    _require(last_neural.get("selected_left_lane_id") == 1, last_neural)
    _require(last_neural.get("selected_right_lane_id") == 2, last_neural)

    fake.fail = True
    fallback = controller.analyze_image(image)
    _require(
        fallback.backend == "CLASSICAL_CV_FALLBACK",
        f"neural failure did not select classical fallback: {fallback}",
    )
    _require(fallback.detected, f"classical fallback failed on black tape: {fallback}")

    # Slow external inference costs one attempt, then the control path latches it
    # off. This 160 ms gate remains independent of diagnostic preview behavior.
    fake.fail = False
    fake.inference_ms = 250.0
    controller.set_neural_enabled(False)
    controller.set_neural_enabled(True)
    before = fake.calls
    slow = controller.analyze_image(image)
    after_first = fake.calls
    slow_again = controller.analyze_image(image)
    after_second = fake.calls
    _require(slow.backend == "CLASSICAL_CV_FALLBACK", slow)
    _require(slow_again.backend == "CLASSICAL_CV_FALLBACK", slow_again)
    _require(after_first == before + 1, "slow inference was not attempted once")
    _require(after_second == after_first, "latency breaker did not suspend neural retries")
    snapshot = controller.snapshot()
    _require(
        str(snapshot["neural_suspended_reason"]).startswith("NEURAL_INFERENCE_TOO_SLOW:"),
        snapshot,
    )

    ok, jpeg = cv2.imencode(".jpg", image)
    _require(ok, "JPEG encode failed")
    probe = controller.probe_neural_latency_jpeg(jpeg.tobytes(), attempts=2)
    _require(probe["ready"] and not probe["allowed"], probe)
    _require(len(probe["attempts"]) == 2, probe)

    controller.set_neural_enabled(False)
    classical = controller.analyze_image(image)
    _require(
        classical.backend == "CLASSICAL_CV",
        f"manual classical path changed: {classical}",
    )

    print("Hybrid lane controller V2 regression: PASS")
    print(
        {
            "external_backend": "UFLD_ONNX",
            "ego_lane_ids": [1, 2],
            "control_preview_geometry_isolated": True,
            "neural_confidence": neural.confidence,
            "fallback_confidence": fallback.confidence,
            "latency_probe": probe,
            "pretrained_calls": fake.calls,
        }
    )


if __name__ == "__main__":
    main()
