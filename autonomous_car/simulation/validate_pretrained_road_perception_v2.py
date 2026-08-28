"""Regression for external UFLD ONNX preprocessing/decoder without model binary."""

import os
import tempfile

from autonomous_car.perception.pretrained_road import PretrainedRoadPerception


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


class _FakeSession:
    def __init__(self):
        self.last_names = None
        self.last_feed = None

    def run(self, names, feed):
        import numpy as np

        self.last_names = list(names)
        self.last_feed = feed
        output = np.full((1, 101, 56, 4), -5.0, dtype=np.float32)
        # Outer lanes absent.
        output[0, 100, :, 0] = 8.0
        output[0, 100, :, 3] = 8.0
        # Ego lanes 1/2 present. Decoder reverses the 56-anchor dimension; make
        # the lanes converge toward image center as they rise in the image.
        for decoded_index in range(56):
            source_anchor = 55 - decoded_index
            fraction = decoded_index / 55.0
            left_grid = int(round(33 + 12 * fraction))
            right_grid = int(round(68 - 12 * fraction))
            output[0, 100, source_anchor, 1] = -8.0
            output[0, 100, source_anchor, 2] = -8.0
            output[0, left_grid, source_anchor, 1] = 8.0
            output[0, right_grid, source_anchor, 2] = 8.0
        return [output]


def main():
    import numpy as np

    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "ultrafast_lane_tusimple_288x800.onnx")
        open(path, "wb").close()
        runtime = PretrainedRoadPerception(path)
        session = _FakeSession()
        runtime._session = session
        runtime._input_name = "input"
        runtime._output_name = "output"

        image = np.full((360, 640, 3), (30, 80, 160), dtype=np.uint8)
        result = runtime.infer(image)
        _require(result["lane_count"] == 2, f"expected two ego lanes: {result}")
        _require(
            {lane["lane_id"] for lane in result["lanes"]} == {1, 2},
            f"wrong external UFLD lane IDs: {result}",
        )
        _require(result["decoder"] == "EXTERNAL_UFLD_TUSIMPLE", result)
        _require(
            result["decoder_adapter"] == "third_party.ufld",
            f"runtime is not using vendored UFLD adapter: {result}",
        )
        _require(session.last_names == ["output"], session.last_names)
        _require(set(session.last_feed) == {"input"}, session.last_feed.keys())
        tensor = session.last_feed["input"]
        _require(tensor.shape == (1, 3, 288, 800), f"bad UFLD tensor: {tensor.shape}")
        _require(tensor.dtype == np.float32, "bad UFLD input dtype")
        for lane in result["lanes"]:
            _require(len(lane["points"]) >= 50, f"UFLD lane decode too short: {lane}")
            ys = [point[1] for point in lane["points"]]
            _require(min(ys) >= 0 and max(ys) <= 360, f"lane y remap failed: {lane}")
            _require(lane["confidence"] >= 0.9, f"lane confidence unexpectedly low: {lane}")

        snapshot = runtime.snapshot()
        _require(snapshot["loaded"], f"fake runtime did not stay loaded: {snapshot}")
        _require(snapshot["runs"] == 1, f"run count mismatch: {snapshot}")
        _require(snapshot["input_size"] == [800, 288], f"wrong input size: {snapshot}")
        _require(snapshot["expected_output"] == [1, 101, 56, 4], snapshot)
        _require(snapshot["decoder_adapter"] == "third_party.ufld", snapshot)

        try:
            PretrainedRoadPerception(path, input_size=(640, 360))
        except ValueError as error:
            _require("UFLD_TUSIMPLE_INPUT_SIZE_FIXED" in str(error), error)
        else:
            raise AssertionError("non-UFLD input size was accepted")

    print("Pretrained road perception V2 regression: PASS")
    print(
        {
            "model": result["model"],
            "decoder": result["decoder"],
            "decoder_adapter": result["decoder_adapter"],
            "lane_ids": [lane["lane_id"] for lane in result["lanes"]],
            "lane_count": result["lane_count"],
            "max_lane_probability": result["max_lane_probability"],
            "inference_ms": result["inference_ms"],
        }
    )


if __name__ == "__main__":
    main()
