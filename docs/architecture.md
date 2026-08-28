# SWING_CAR architecture

This document is the current high-level architecture for `main`.

## Runtime ownership

The Raspberry Pi is the vehicle authority. It owns:

- motor and steering command delivery
- E-STOP and watchdog handling
- Arduino/steering health checks
- required sensor freshness checks
- `SafetySupervisor` evaluation
- final stop/fault behavior

The Windows Compute Worker may train models, preview RECORD sessions and provide optional remote perception, but it does not become the final actuator safety controller.

## Canonical modes

User-facing modes:

1. `MANUAL`
2. `RECORD`
3. `AUTO_AI`
4. `AUTO_GPS`
5. `AUTO_LOCAL`
6. `AUTO`

System states:

- `DISARMED`
- `EMERGENCY_STOP`
- `FAULT`

`PRETRAINED_ROAD` is an internal `AUTO` strategy. Legacy `AUTO_ROUTE` and `AUTO_HYBRID` implementations remain for migration/regression and do not define the current user-facing AUTO_GPS policy.

## Final rover service layering

The server files are a composition stack, not six independent production servers:

```text
server_v2_final.py
  imports server_v2_release.py
    imports server_v2_full.py
      imports server_v2_ai.py
        imports server_v2.py
          imports server.py
```

Responsibilities are accumulated through that chain:

- `server.py` — proven hardware/telemetry backend and original operator UI.
- `server_v2.py` — canonical V2 mode/API layer over the legacy backend.
- `server_v2_ai.py` — AUTO_AI model registry/runtime integration.
- `server_v2_full.py` — AUTO_LOCAL/map/full mode integration.
- `server_v2_release.py` — final dataset/record/release integration layer.
- `server_v2_gps_ai.py` — route-bound AUTO_GPS controller installed into the final service.
- `server_v2_final.py` — production entrypoint, request hardening, status/runtime guards and final HMI composition.

`camera-stream.service` launches `server_v2_final.py`.

Refactoring these modules into a package is reasonable future work, but deleting intermediate files without replacing their imports would break the production service.

## RECORD data contract

`RECORD` remains human-controlled driving. It stores synchronized camera, LiDAR, IMU, steering, throttle and optional GPS information.

For steering, the frame timeline distinguishes:

- actual encoder-measured steering: `steering_angle_degrees`
- requested/target steering: `target_steering_angle_degrees`

This distinction is important for temporal learned driving: desired steering is a label, while measured steering is vehicle state.

Autonomous/FAULT/E-STOP contaminated sessions and hard-safety-forced stop frames are excluded from imitation-learning labels by the dataset pipeline.

## AUTO_AI

AUTO_AI is route-independent imitation learning:

```text
camera + LiDAR sectors + IMU
            -> learned model
            -> steering + throttle
```

GPS is not a normal learned input. Person STOP and rover hard-safety checks remain outside the model.

## AUTO_GPS

AUTO_GPS adds normalized route-relative features:

```text
camera + LiDAR + temporal auxiliary + route features
                    -> learned model
                    -> steering + throttle
```

The route vector contains signed cross-track/heading information, near/far lookahead bearing and distance, remaining distance and route progress. Raw latitude/longitude are diagnostic data, not direct neural-network inputs.

New temporal v3 models use a five-step history with:

- IMU yaw-rate value + presence
- previous encoder-measured steering value + presence

The current-frame measured steering and current target are excluded from the current prediction. After inference, the current measured encoder angle becomes prior state for the next frame.

Old temporal v2 manifests remain loadable with their legacy model-prediction feedback semantics; they should be replaced by retrained v3 candidates rather than silently changing their input contract.

## AUTO_LOCAL

AUTO_LOCAL uses persistent local mapping/navigation:

```text
LiDAR + IMU
  -> localization against saved occupancy map
  -> inflated-grid A*
  -> local path following / avoidance
  -> SafetySupervisor
```

Persistent localization loss stops/faults. Lane information may assist where eligible but is not required for saved-map navigation.

## Top-level AUTO

AUTO selects conservatively at start. Current intent is:

1. matching route-bound `AUTO_ALLOWED` GPS model when GPS/RTK/route preflight is ready
2. AUTO_LOCAL when a map/destination can localize and plan
3. compatible route-independent `AUTO_ALLOWED` AI
4. eligible `PRETRAINED_ROAD` lane strategy when its own model/camera/LiDAR/lane/latency gates pass
5. otherwise refuse autonomous motion

A safety fault does not silently reset and hot-switch to another strategy.

## Windows Compute Worker boundary

The Worker provides compute/training/preview capabilities over the private LAN and local loopback. It uses a managed data root under `%LOCALAPPDATA%\SWING Robotics\Compute Worker`.

The packaged Windows Manager explicitly starts/stops the Worker. Installation removes old Startup shortcuts; reboot does not automatically start the Worker.

## Runtime data

These are generated state and are intentionally ignored by Git:

- `recordings/`
- `models/`
- `datasets/`
- `gps-routes/`
- `maps/`
- calibration/config runtime JSON files
- PyInstaller/Inno outputs

Historical design and completion documents are retained under `docs/archive/`; they are evidence/context, not the current contract.
