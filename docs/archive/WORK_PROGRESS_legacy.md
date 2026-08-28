> Archived historical document. This file describes an earlier feature-branch state and is not the current `main` contract. See `README.md` and `docs/architecture.md`.

# WORK PROGRESS

## Current target branch

`feature/autonomy-v2-architecture`

## Current user-facing architecture

The rover keeps six V2 drive modes:

- `MANUAL`
- `RECORD`
- `AUTO_AI`
- `AUTO_GPS`
- `AUTO_LOCAL`
- `AUTO`

`DISARMED`, `EMERGENCY_STOP`, and `FAULT` are system states.

`PRETRAINED_ROAD` is an internal `AUTO` strategy, not a seventh user-selectable drive mode. It uses the externally sourced UFLD TuSimple lane model when GPS, LOCAL and eligible user-trained AI strategies are unavailable and the lane-specific preflight succeeds.

## Dashboard decision

The original V1 dashboard is again the primary `/` operator screen because it already contains the proven live camera, LiDAR, GNSS/RTK, IMU, steering, Raspberry Pi/network status, manual drive, gamepad, calibration, and maintenance workflows.

V2 is presented as an option rather than replacing the V1 dashboard:

- `/` -> original V1 dashboard + compact V2 drive-mode chooser and server-authoritative lane overlay.
- `/v2` -> advanced V2 data/model/GPS-AI/local-map management screen.
- `/legacy` -> redirects to `/`.
- `/ai-data` -> redirects to `/v2#data`.
- `/gps-ai` -> redirects to `/v2#gps`.

The lane overlay can show a diagnostic UFLD preview in `DISARMED`/`MANUAL`/`MANUAL_ASSIST`/`RECORD`. The preview has no control authority. During `RECORD`, the same throttled UFLD observation cache is shared with the telemetry writer so the dashboard does not cause duplicate neural inference.

Deprecated V1 `AUTO_ROUTE`, `AUTO_HYBRID`, old route process/load, and old RECORD start/stop controls are hidden in the primary UI so they do not compete with canonical V2 mode handling. Their backend remains for compatibility/regression only.

## Fixed: empty DriveMode error

Observed operator error:

`'' is not a valid DriveMode`

Root cause: the legacy `_read_json()` caches a decoded JSON payload on the HTTP handler instance. `BaseHTTPRequestHandler` can reuse the same handler instance for HTTP/1.1 keep-alive requests. V2 subclass handlers could inspect request bodies before the legacy `do_POST()` reset logic ran, allowing the previous request body (for example `{}` from safety reset/E-stop) to be reused by a later `/api/v2/mode` request.

Fix:

1. `server_v2_final.py` clears `_cached_json_payload` at the beginning of every `handle_one_request()`.
2. `/api/v2/mode` is validated before any `DriveMode(...)` construction.
3. Empty, whitespace, malformed and deprecated mode names return a controlled HTTP 400 error.
4. Valid mode requests are normalized to the canonical uppercase form and the normalized payload is reused by lower V2 handlers without reading the socket body twice.
5. The V2 option drawer uses a hard allow-list and never submits an empty mode.

Accepted mode endpoint values:

`MANUAL`, `RECORD`, `AUTO_AI`, `AUTO_GPS`, `AUTO_LOCAL`, `AUTO`, `DISARMED`.

Emergency stop remains a dedicated endpoint.

## AUTO_AI

GPS-independent learned driving:

Camera + stabilized LiDAR sectors + IMU yaw rate -> steering + throttle.

Human RECORD labels are target steering and requested throttle. Autonomous/fault/E-stop sessions and hard-safety-forced stop frames are excluded. Training/evaluation/export/ONNX runtime and model lifecycle gating are implemented.

## RECORD UFLD observation

Human `RECORD` remains fully driver-controlled. In addition to the existing camera, steering, throttle, LiDAR, IMU and optional GPS data, the final V2 stack now runs the external UFLD lane detector as a read-only observer at a throttled interval (default 0.50 s).

The UFLD observer:

- has `control_authority=NONE` and never calls motor control or `set_neural_enabled()`;
- shares one inference cache with the dashboard UFLD preview to avoid duplicate Pi inference;
- resets its cache for each new RECORD session so a preceding MANUAL preview cannot become the first recorded lane sample;
- does not abort RECORD if the UFLD model/dependency/inference fails; the failure is stored as lane telemetry instead;
- writes lane detection/confidence, lateral/heading/correction error, backend/marking, inference latency, latency status, left/right/center lane geometry, image/ROI geometry, source camera sequence, source camera monotonic time and observation monotonic time into `perception.csv`;
- retains the original `camera_timestamps.csv` steering/throttle timestamps so UFLD geometry can be aligned back to the human driving labels.

`SWING_UFLD_OBSERVER_PERIOD_SECONDS` controls the shared observer period and is clamped to 0.20-2.0 seconds.

## AUTO_GPS

GPS-conditioned learned driving:

Repeated GPS ON RECORD sessions -> normalized RTK route -> route-relative features + Camera/LiDAR/IMU -> steering + throttle.

Raw latitude/longitude are not neural network inputs. GPS models are route-bound and route/model mismatch is rejected.

## AUTO_LOCAL

Persistent local occupancy map + LiDAR/IMU localization + A* + local path following/avoidance. Persistent localization loss stops/faults.

Lane assistance uses the same production `HybridLaneController` as the operator overlay. On AUTO_LOCAL start, the UFLD model is warmed and latency-qualified while propulsion is stopped. If the neural backend is unavailable or exceeds its latency budget, AUTO_LOCAL keeps the classical lane detector as its secondary lane correction instead of blocking LOCAL navigation.

## UFLD pretrained lane tracking

The external lane path is now connected end to end:

1. `scripts/install_pretrained_road_model.sh` installs and validates the TuSimple UFLD ONNX artifact outside Git.
2. `third_party/ufld` contains the attributed MIT preprocessing/decoder adapter.
3. `PretrainedRoadPerception` imports that vendored adapter instead of maintaining a duplicate decoder.
4. `HybridLaneController` converts UFLD lane tracks to calibrated SWING lane geometry and retains the classical detector as fallback where permitted.
5. MANUAL/DISARMED can run UFLD as a display-only diagnostic; RECORD uses the same read-only observer cache and persists its lane geometry/diagnostics.
6. `AUTO_LOCAL` may use UFLD as a secondary steering correction after pre-motion latency qualification.
7. `AUTO` may select internal strategy `PRETRAINED_ROAD` only after GPS, LOCAL and compatible user-trained AI are exhausted and its own camera/LiDAR/model/lane/latency preflight succeeds.
8. `PRETRAINED_ROAD` commands still pass through `SafetySupervisor`, steering tracking checks, control-loop age checks, E-STOP/manual takeover and production shutdown guards.
9. The primary operator UI recognizes `PRETRAINED_ROAD` as an active AUTO strategy and presents the main action as stop, not start, while that strategy is running.

The no-training lane AUTO remains deliberately low-speed and must not be treated as field-approved until the target Raspberry Pi physical gates below pass.

## AUTO

Conservative selection order:

1. matching `AUTO_ALLOWED` GPS AI when RTK/sensors/route are ready;
2. LOCAL when map/destination/localization are ready;
3. environment-compatible route-independent `AUTO_ALLOWED` AI;
4. `PRETRAINED_ROAD` UFLD lane tracking when its model, camera, LiDAR, lane confidence and neural latency preflight are acceptable;
5. otherwise refuse motion.

## Validation status

Automated checks cover:

- Python compile of V1/V2 final service and UI modules;
- V1-primary + V2-option dashboard structure;
- valid/invalid V2 drive-mode request normalization;
- V2 policy/state/model registry;
- GPS normalized route/model/features and GPS ON RECORD dataset path;
- AUTO_LOCAL mapping/navigation;
- AUTO_AI dataset/environment gating;
- external UFLD preprocessing/decoder contract and vendored adapter usage;
- shared UFLD MANUAL/RECORD observer cache, RECORD-mode allowance and no-control-authority policy;
- recorded UFLD lane geometry/diagnostics and camera timestamp alignment fields;
- hybrid UFLD/classical lane control contract and diagnostic preview isolation;
- `PRETRAINED_ROAD` AUTO priority, Safety path, lane-loss handling, latency gate and operator running-state UX;
- manual takeover isolation;
- AUTO_AI and AUTO_GPS train/evaluate/ONNX Runtime smoke.

Target Raspberry Pi and closed-area hardware validation are still required before the Draft PR should be merged to `main`. For UFLD this specifically includes model installation, warmed inference latency, live lane overlay alignment in MANUAL and RECORD, confirmation that `perception.csv` receives UFLD rows without affecting human steering, wheels-off steering direction/limit checks for autonomous use, lane-loss stop, E-STOP/manual takeover, and only then very-low-speed closed-area motion.
