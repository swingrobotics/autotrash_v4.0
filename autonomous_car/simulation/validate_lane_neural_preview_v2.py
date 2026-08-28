"""Regression for diagnostic-only UFLD preview and control isolation."""

from autonomous_car.control.hybrid_lane_controller import HybridLaneController


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


class _FakePretrained:
    def __init__(self, inference_ms=250.0):
        self.inference_ms = float(inference_ms)
        self.calls = 0

    def infer(self, image):
        self.calls += 1
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


def _jpeg():
    import cv2
    import numpy as np

    image = np.full((360, 640, 3), 150, dtype=np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    _require(ok, "JPEG encode failed")
    return encoded.tobytes()


def main():
    fake = _FakePretrained(inference_ms=250.0)
    controller = HybridLaneController(
        fake,
        processing_width=640,
        processing_height=360,
        maximum_neural_inference_ms=160.0,
    )

    _require(not controller.neural_enabled, "preview test must start with neural control disabled")
    preview = controller.analyze_neural_preview_jpeg(_jpeg())
    _require(preview.detected, f"preview failed to decode valid UFLD lanes: {preview}")
    _require(preview.backend == "UFLD_ONNX", f"wrong preview backend: {preview.backend}")
    _require(not controller.neural_enabled, "preview changed autonomous neural enable state")

    snapshot = controller.snapshot()
    _require(snapshot["neural_suspended_reason"] is None, snapshot)
    last_preview = snapshot.get("last_preview") or {}
    _require(last_preview.get("latency_allowed") is False, last_preview)
    _require(float(last_preview.get("inference_ms")) == 250.0, last_preview)
    _require(last_preview.get("selected_left_lane_id") == 1, last_preview)
    _require(last_preview.get("selected_right_lane_id") == 2, last_preview)
    _require(controller.preview_snapshot().get("control_authority") == "NONE", controller.preview_snapshot())

    # The same slow model must still trip the real control-path latency breaker.
    controller.set_neural_enabled(True)
    controlled = controller.analyze_image(__import__("numpy").full((360, 640, 3), 150, dtype="uint8"))
    _require(controlled.backend == "CLASSICAL_CV_FALLBACK", controlled)
    _require(
        str(controller.snapshot()["neural_suspended_reason"]).startswith(
            "NEURAL_INFERENCE_TOO_SLOW:"
        ),
        controller.snapshot(),
    )

    endpoint_source = open("lane_neural_preview.py", encoding="utf-8").read()
    overlay_source = open("lane_dashboard_overlay.py", encoding="utf-8").read()
    _require("/api/lane/neural-preview" in endpoint_source, "preview endpoint missing")
    _require("control_authority\": \"NONE" in endpoint_source, "preview control isolation missing")
    _require("UFLD 미리보기" in overlay_source, "preview dashboard toggle missing")
    _require("swing.lane.neuralPreview" in overlay_source, "preview preference persistence missing")

    print("Lane neural preview V2 regression: PASS")
    print(
        {
            "preview_backend": preview.backend,
            "preview_inference_ms": last_preview.get("inference_ms"),
            "preview_latency_allowed": last_preview.get("latency_allowed"),
            "control_backend_after_slow_inference": controlled.backend,
        }
    )


if __name__ == "__main__":
    main()
