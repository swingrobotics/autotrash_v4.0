# Third-party pretrained models

## Ultra-Fast-Lane-Detection (default no-training AUTO perception)

SWING_CAR now uses the published open-source **Ultra-Fast-Lane-Detection (UFLD)**
TuSimple ResNet18 detector as the default neural lane backend for no-training
road autonomy. The detector and decoding contract come from the external UFLD
projects; SWING_CAR only adapts the returned lane tracks to its common calibrated
`LaneResult`, temporal geometry and SafetySupervisor interfaces.

- Original detector: `cfzd/Ultra-Fast-Lane-Detection`
- Original detector license: MIT
- ONNX inference reference: `ibaiGorordo/onnx-Ultra-Fast-Lane-Detection-Inference`
- ONNX inference reference license: MIT
- Converted artifact source: `PINTO0309/PINTO_model_zoo`, entry `140_Ultra-Fast-Lane-Detection`
- Artifact entry license: MIT (copyright attribution to cfzd retained upstream)
- Training domain: TuSimple outdoor road-lane scenes
- Backbone: ResNet18
- Runtime: ONNX Runtime `CPUExecutionProvider`
- Input: `1 x 3 x 288 x 800`
- Output: `1 x 101 x 56 x 4`
- Four external lane tracks; TuSimple ego-lane indices 1 and 2 are preferred
- SWING_CAR artifact: `ultrafast_lane_tusimple_288x800.onnx`
- Install with: `bash scripts/install_pretrained_road_model.sh`

The model binary is not committed to this repository. The installer downloads
PINTO's TuSimple archive, finds the 288x800 UFLD ONNX member, validates the exact
input/output contract with ONNX Runtime, performs a finite-value smoke inference,
and records the installed artifact SHA-256 beside the model.

### External decoder contract

The runtime follows the public UFLD ONNX inference pipeline rather than inventing
a SWING-specific lane network:

1. OpenCV BGR camera frame -> RGB.
2. Resize to 800x288.
3. ImageNet mean/std normalization.
4. ONNX inference yielding 101 grid classes x 56 TuSimple row anchors x 4 lanes.
5. Reverse the anchor axis, softmax the spatial grid classes and recover lane
   positions using the published TuSimple row anchors `[64, 68, ..., 284]`.
6. Prefer external lane IDs 1/2 for the ego left/right boundaries.
7. Convert only those external points into SWING_CAR's calibrated geometry and
   control representation.

The classical BLACK/YELLOW/WHITE/EDGE detector remains a fallback for indoor tape
and other out-of-domain scenes; it is no longer the intended outdoor primary.

### Runtime policy

UFLD is lazy and is not run in AUTO_AI. In MANUAL/DISARMED the operator can use
`UFLD 미리보기`, which has `control_authority=NONE` and separate geometry history.
Before PRETRAINED_ROAD AUTO can move the rover, the installed UFLD model is warmed
and its actual inference latency must pass the default **160 ms** budget. The
existing 200 ms soft / 400 ms hard autonomous timing protection is not relaxed.
If inference exceeds the neural budget during control, the circuit breaker stops
further neural retries and falls back to classical perception.

AUTO_LOCAL may use UFLD only after the same stopped latency preflight. LiDAR
remains the independent obstacle-safety source; UFLD is only a lane detector.

## Rejected/retired target-hardware trials

### LSTR 180x320

LSTR was integrated as a lightweight candidate after YOLOP. Although its ONNX
runtime and calibration adapter worked, field testing did not produce reliable
lane acquisition for the low-mounted SWING_CAR camera. It is no longer the
default detector. The project switched to the external UFLD inference pipeline
instead of continuing to tune a project-specific LSTR adapter.

### YOLOP 320

YOLOP 320 was integrated and validated functionally, but target-Pi measurement
showed approximately 0.85-1.01 seconds per inference (about 0.91 seconds warm
average), well beyond the existing 400 ms hard timing limit. It is not a
supported driving backend; Safety timing thresholds were not weakened for it.
