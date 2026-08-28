# AUTO_GPS

`AUTO_GPS` is the normalized-route-conditioned learned-driving mode. It is not the legacy deterministic Pure Pursuit `AUTO_ROUTE` controller.

## 1. Collect repeated human demonstrations

Drive the same intended outdoor route multiple times in `RECORD` with GPS/RTK enabled.

Recommended source characteristics:

- human-controlled driving
- high RTK FIXED coverage
- camera/LiDAR/IMU/steering/throttle recorded normally
- similar start/end geometry and travel direction
- realistic curve, obstacle-avoidance and recovery demonstrations

The route normalizer requires multiple compatible source runs; do not create a route from a single duplicated trajectory and treat that as diversity.

## 2. Build the normalized route

```bash
python3 scripts/normalize_gps_route.py \
  outdoor_loop_v1 \
  run_001 run_002 run_003 run_004 \
  --recordings-root /home/gnss/camera-stream/recordings \
  --routes-root /home/gnss/camera-stream/gps-routes
```

The normalized reference route is built from RTK FIXED geometry using jump filtering, ENU conversion, run-direction alignment, progress resampling, outlier rejection, median fusion, smoothing and fixed-distance resampling.

RTK FLOAT/DGPS may be conditionally accepted for **training samples** under the configured quality/deviation policy, but they do not redefine the normalized route centerline and they do not relax live AUTO_GPS RTK FIXED safety.

## 3. Route-relative learned features

Each accepted frame receives eight route-relative values:

```text
cross_track_error
heading_error
near_bearing_error
near_distance
far_bearing_error
far_distance
remaining_distance
route_progress
```

Raw latitude/longitude are retained for diagnostics/evaluation and are not direct neural-network inputs.

## 4. Steering label vs measured steering state

Current RECORD camera timing data separates two concepts:

```text
target_steering_angle_degrees  desired/requested human command
steering_angle_degrees         actual encoder-measured steering angle
```

The supervised steering target prefers the human target command. The actual encoder angle is preserved as vehicle state.

This separation is mandatory for the current temporal AUTO_GPS contract.

## 5. Temporal v3 auxiliary contract

New AUTO_GPS training/export uses a five-step temporal context. Each step contributes four auxiliary values:

```text
IMU yaw rate normalized
IMU yaw-rate presence
previous measured steering normalized
previous measured-steering presence
```

Total temporal auxiliary size: `5 x 4 = 20`.

For a prediction at frame `t`:

- yaw history includes recent yaw-rate observations through the current frame according to the synchronized temporal builder
- steering history contains **previous encoder-measured angles**, not previous human targets and not previous model predictions
- the current frame's target steering is never part of its own temporal input
- the current frame's measured steering is also excluded from the current prediction and is appended after inference so it becomes prior vehicle state for the next frame
- missing measured steering is represented by zero value with presence bit `0`

The exported manifest identifies the steering-history source as `MEASURED_ENCODER` and requires measured steering feedback.

### Legacy v2 compatibility

Older temporal v2 models did not declare the measured-steering source and historically fed previous model prediction back into temporal steering history. They remain loadable for compatibility, but that legacy semantic is intentionally not silently changed underneath an existing model.

To use the corrected contract, retrain/export a new temporal v3 model and select that new model.

## 6. Build the AUTO_GPS dataset

```bash
python3 scripts/build_gps_ai_dataset.py \
  /home/gnss/camera-stream/gps-routes/outdoor_loop_v1.json \
  run_001 run_002 run_003 run_004 \
  --recordings-root /home/gnss/camera-stream/recordings \
  --output-root /home/gnss/camera-stream/datasets \
  --dataset-id outdoor_loop_v1_dataset
```

The builder preserves whole-session split isolation and scenario balancing. Meaningful curve/recovery examples are protected from silently disappearing from the training split.

## 7. Train

```bash
python3 scripts/train_gps_ai_model.py \
  /data/datasets/outdoor_loop_v1_dataset \
  /data/training/outdoor_loop_model_v001 \
  --recordings-root /data/recordings \
  --epochs 30 \
  --batch-size 32 \
  --device auto
```

The temporal trainer uses validation-based early stopping and restores the best validation checkpoint. Increasing the maximum epoch count is an experiment, not a substitute for checking `best_epoch`, curve coverage and held-out behavior.

## 8. Evaluate and export

```bash
python3 scripts/evaluate_gps_ai_model.py \
  /data/datasets/outdoor_loop_v1_dataset \
  /data/training/outdoor_loop_model_v001/checkpoint.pt \
  --recordings-root /data/recordings \
  --split test \
  --output-path /data/training/outdoor_loop_model_v001/evaluation
```

```bash
python3 scripts/export_gps_ai_onnx.py \
  /data/training/outdoor_loop_model_v001/checkpoint.pt \
  /data/training/outdoor_loop_model_v001/export
```

Current learned inputs are represented as:

```text
image
lidar
auxiliary
route
```

The manifest records `policy_type=AUTO_GPS`, route binding and the temporal steering-history contract.

## 9. RECORD model preview

The synchronized RECORD preview replays the recorded camera/GPS/IMU data and passes the recorded `steering_angle_degrees` into measured-steering temporal state.

Preview comparison distinguishes:

- MODEL predicted command
- HUMAN target command
- ACTUAL encoder-measured steering

For measured-steering temporal GPS preview, temporal cadence is preserved at every relevant frame and temporal state is reset across sensor synchronization gaps instead of carrying stale history forward.

## 10. Register and lifecycle

```bash
python3 scripts/register_gps_ai_model.py \
  outdoor_loop_model_v001 \
  /tmp/gps_drive_model.onnx \
  --manifest /tmp/model_manifest.json \
  --evaluation /tmp/evaluation_metrics.json \
  --stage OFFLINE_VALIDATED
```

Lifecycle:

```text
TRAINED
  -> OFFLINE_VALIDATED
  -> CLOSED_AREA_VALIDATED
  -> AUTO_ALLOWED
```

AUTO_GPS models are route-bound. Runtime rejects a route/model mismatch.

## 11. Live runtime safety

The relaxed conditional training policy does not relax live localization safety. Current runtime safety includes:

- RTK FIXED requirement
- fresh GNSS/IMU/camera/LiDAR required by the policy
- route/model compatibility
- route deviation limits
- measured steering feedback for temporal v3 models
- person STOP
- E-STOP/watchdogs
- Arduino/steering health and output limits

If RTK FIXED or another required runtime input becomes invalid, the rover remains responsible for fail-closed behavior.
