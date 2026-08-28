# Ultra-Fast-Lane-Detection attribution

SWING_CAR's default no-training road-lane backend uses the external open-source
Ultra-Fast-Lane-Detection (UFLD) inference contract.

Sources used:

- Detector/model architecture: https://github.com/cfzd/Ultra-Fast-Lane-Detection
- ONNX inference/decoder reference: https://github.com/ibaiGorordo/onnx-Ultra-Fast-Lane-Detection-Inference
- Converted TuSimple ONNX artifact distribution: https://github.com/PINTO0309/PINTO_model_zoo/tree/main/140_Ultra-Fast-Lane-Detection

The UFLD detector/artifact is MIT licensed. See `LICENSE` in this directory.
SWING_CAR does not retrain or claim authorship of the pretrained detector; its
project code only adapts external lane tracks to vehicle calibration, common
lane geometry and safety/control interfaces.
