# AUTO_AI

`AUTO_AI` is the route-independent learned-driving mode. It learns human steering/throttle behavior from `RECORD` sessions and does not normally use GPS/RTK as a neural-network input.

## Data source

Collect `RECORD` sessions while a human drives. The recording pipeline keeps synchronized camera, LiDAR, IMU, steering and throttle state.

The learned-label policy is intentionally separated from hard safety. Autonomous/FAULT/E-STOP contaminated sessions and frames where hard safety forced the vehicle to stop are rejected from human imitation labels.

For steering, target steering is preferred as the desired command label; actual encoder steering is retained separately as measured state.

## Inputs and outputs

Current route-independent learned input concept:

```text
camera frame
+ LiDAR sector distances / observed masks
+ IMU auxiliary values such as yaw rate
        -> model
        -> steering + throttle
```

GPS may be recorded for diagnostics/evaluation but is not a normal AUTO_AI learned feature.

LiDAR is represented as compact sectors rather than raw point clouds so the deployment contract remains practical on rover hardware.

## Build a dataset

Example:

```bash
python3 scripts/build_ai_dataset.py \
  --recordings-root /home/gnss/camera-stream/recordings \
  --output-root /home/gnss/camera-stream/ai-datasets \
  --dataset-id warehouse_drive_v1 \
  run_2026-08-22_10-00-00 \
  run_2026-08-22_10-20-00 \
  run_2026-08-22_10-40-00
```

The builder aligns streams by monotonic time, rejects unusable samples, builds compact LiDAR features and splits by whole RECORD session to reduce adjacent-frame leakage.

## Train

Training is normally performed on the Windows Compute Worker or another suitable PC/GPU.

CLI example:

```bash
python3 scripts/train_ai_model.py \
  /data/ai-datasets/warehouse_drive_v1 \
  /data/training/drive_v001 \
  --recordings-root /data/recordings \
  --epochs 30 \
  --batch-size 32 \
  --device auto
```

Training output includes a PyTorch checkpoint and metrics. Training completion alone does not authorize vehicle use.

## Held-out evaluation

```bash
python3 scripts/evaluate_ai_model.py \
  /data/ai-datasets/warehouse_drive_v1 \
  /data/training/drive_v001/checkpoint.pt \
  --recordings-root /data/recordings \
  --split test \
  --output-path /data/training/drive_v001/evaluation
```

Review overall and scenario-specific steering/throttle error, especially straight, left/right curves, obstacle behavior and recovery examples. Offline metrics are not a substitute for closed-area validation.

## Export ONNX

```bash
python3 scripts/export_ai_onnx.py \
  /data/training/drive_v001/checkpoint.pt \
  /data/training/drive_v001/export
```

The exporter produces the deployment ONNX artifact and manifest and normally verifies ONNX Runtime numerical parity.

## Register and lifecycle

Example:

```bash
python3 scripts/register_ai_model.py \
  drive_v001 \
  /tmp/drive_model.onnx \
  --manifest /tmp/model_manifest.json \
  --evaluation /tmp/evaluation_metrics.json \
  --environment indoor \
  --environment warehouse \
  --stage OFFLINE_VALIDATED
```

Lifecycle:

```text
TRAINED
  -> OFFLINE_VALIDATED
  -> CLOSED_AREA_VALIDATED
  -> AUTO_ALLOWED
```

`CLOSED_AREA_VALIDATED` allows explicit/manual AUTO_AI use according to the runtime policy. `AUTO_ALLOWED` is the stronger stage required before top-level `AUTO` may select a compatible route-independent model.

## Runtime safety contract

AUTO_AI ordinary steering/throttle behavior comes from the learned model. Hard safety remains external:

- E-STOP
- command/watchdogs
- Arduino health
- steering health/limits/tracking
- required camera/LiDAR freshness
- output limits
- person danger-path STOP

A missing/stale learned input is a runtime safety problem; it is not silently replaced by an unrelated planner.

## Windows Worker workflow

The packaged Worker can transfer selected rover RECORD sessions, train/evaluate/export candidates and return/install model artifacts without requiring the user to run the CLI manually.

See [../worker.md](../worker.md) for Worker installation and release details.
