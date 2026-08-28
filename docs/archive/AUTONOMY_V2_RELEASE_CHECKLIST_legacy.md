> Archived historical release checklist. It predates the current `main` state and still contains feature-branch/Draft-PR merge gates. Use `docs/validation/FIELD_TEST.md` for current physical validation.

# Autonomy V2 Release Checklist

This checklist separates software-complete gates from target-vehicle validation. CI success alone does not authorize unattended or normal-speed autonomous driving.

## User-facing architecture

- [x] Keep the six canonical drive modes: `MANUAL`, `RECORD`, `AUTO_AI`, `AUTO_GPS`, `AUTO_LOCAL`, `AUTO`.
- [x] Keep `DISARMED`, `EMERGENCY_STOP`, and `FAULT` as system states, not user drive modes.
- [x] Keep the proven V1 dashboard as the primary `/` operator screen.
- [x] Reuse the existing V1 `주행 모드` modal as a V2 chooser with readiness/status while keeping actual start/stop on the main dashboard.
- [x] Keep advanced V2 data/model/map/hardware/system management at `/v2`; do not expose a second operator DRIVE screen there.
- [x] Hide deprecated V1 `AUTO_ROUTE` / `AUTO_HYBRID` start controls so they cannot compete with the V2 runtime.
- [x] Preserve the legacy hardware backend and V1 sensor/manual-control UI.

## HTTP / operator request safety

- [x] Clear cached JSON request bodies at the start of every HTTP/1.1 request in the final service handler.
- [x] Validate `/api/v2/mode` before constructing `DriveMode` so empty/malformed requests produce a controlled 400 response.
- [x] Accept only `MANUAL`, `RECORD`, `AUTO_AI`, `AUTO_GPS`, `AUTO_LOCAL`, `AUTO`, and `DISARMED` on the mode endpoint.
- [x] Keep emergency stop on its dedicated endpoint.
- [x] Reject browser cross-origin writes.
- [x] Bound JSON request bodies with `AUTONOMY_MAX_JSON_BODY_BYTES` and reject unsupported transfer encodings.
- [x] Apply a per-connection socket timeout with `AUTONOMY_HTTP_CONNECTION_TIMEOUT_SECONDS`.
- [x] Use daemon request threads and a bounded listen backlog in the final HTTP server.
- [x] When `AUTONOMY_API_TOKEN` is unset, accept write requests only from loopback/private/link-local client addresses.
- [x] When `AUTONOMY_API_TOKEN` is configured, require the matching bearer/header token for writes.
- [ ] Do not expose the Python `http.server` service directly to the public Internet. If remote operation is ever required, put it behind an authenticated VPN/reverse proxy or replace the serving layer with a production server.

## Production runtime fail-safe / persistence

- [x] Independent runtime guard stops autonomy on stale LiDAR/IMU, Arduino loss, or steering-feedback loss.
- [x] AUTO_LOCAL rejects stale LiDAR before localization/SLAM processing.
- [x] RECORD has a configurable minimum free-space floor (`RECORD_MIN_FREE_BYTES`) and stops on low storage.
- [x] Periodic RECORD flush failures are surfaced as recorder errors and terminate the affected recording generation.
- [x] Final model/route/config selection persistence fsyncs the parent directory after replacement/write in the final service.
- [x] SIGTERM/SIGINT unwind through a final actuator shutdown path that stops autonomous controllers, drive output, steering output, RECORD and active mapping.
- [x] `camera-stream.service` uses `Restart=on-failure`, `KillSignal=SIGTERM`, `KillMode=mixed`, and `TimeoutStopSec=10` so normal service shutdown gets a graceful actuator-stop window before remaining processes are killed.

## MANUAL / RECORD

- [x] MANUAL remains human-only driving with hard safety.
- [x] RECORD remains human driving plus synchronized sensor/control recording.
- [x] RECORD supports GPS/RTK ON/OFF.
- [x] New RECORD sessions store raw LiDAR plus stabilized `safety_points` used by learned inference.
- [x] Camera-frame/control label timing skew is recorded and excessive skew can be rejected during dataset building.
- [x] Manual takeover guard prevents legacy input from silently competing with active autonomy.
- [x] Explicit takeover stops active autonomous controllers and returns control to MANUAL.

## AUTO_AI

- [x] Human RECORD-only DatasetBuilder with whole-session train/validation/test split.
- [x] Training entrypoint re-validates dataset split integrity and rejects session/source leakage even if a manifest is manually modified.
- [x] Camera + LiDAR + IMU learned input contract; GPS excluded.
- [x] Human target steering and requested throttle labels.
- [x] Autonomous/FAULT/E-STOP contaminated sessions rejected.
- [x] Safety-supervisor forced-stop frames rejected as imitation labels.
- [x] Scenario-balanced PyTorch training.
- [x] Held-out evaluation.
- [x] ONNX export is verified against ONNX Runtime numerical output parity.
- [x] Pi ONNX Runtime inference implementation and runtime thread/spinning tuning hooks.
- [x] Person STOP remains external to the learned driving policy.
- [x] Model lifecycle gating through `AUTO_ALLOWED`.

## AUTO_GPS

- [x] Repeated GPS ON human RECORD sessions can be fused into a normalized route.
- [x] RTK FIXED filtering, jump rejection, ENU conversion, direction alignment, progress resampling, outlier rejection, median fusion, smoothing/resampling.
- [x] GPS-conditioned DatasetBuilder attaches route-relative features to normal AI inputs.
- [x] Raw latitude/longitude are not neural-network inputs.
- [x] Route feature vector includes cross-track error, heading error, near/far lookahead bearing+distance, remaining distance and progress.
- [x] GPS-conditioned PyTorch train/evaluate/export pipeline.
- [x] Four-input ONNX runtime: image + LiDAR + IMU + route features.
- [x] Route-bound model registration; route/model mismatch rejected.
- [x] Runtime requires RTK FIXED and fresh GNSS/IMU, checks route deviation, and retains external hard safety.

## AUTO_LOCAL

- [x] Persistent multi-map store and named destinations.
- [x] Sparse occupancy grid mapping/refinement.
- [x] LiDAR/IMU scan matching and global localization.
- [x] Inflated-grid A* planning and local path following.
- [x] Local obstacle avoidance/replan.
- [x] Persistent localization loss stops/faults.
- [x] Final runtime adds a service-level stale-LiDAR backstop before localization.

## AUTO

- [x] Conservative strategy selection: ready `AUTO_ALLOWED` GPS AI -> LOCAL -> environment-compatible route-independent `AUTO_ALLOWED` AI -> refuse motion.
- [x] No unconditional AI fallback.
- [x] No silent hot failover after a selected strategy safety fault.
- [x] Run-specific event/generation ownership prevents a stopped prior strategy worker from issuing stale output into a new run.

## USB / device identity

- [x] Arduino discovery prefers `/dev/serial/by-id` when an Arduino persistent USB identity is available.
- [x] GNSS configuration prefers a single clearly identified `/dev/serial/by-id` GNSS/GPS device and otherwise falls back conservatively to the historical device path; `GPS_DEVICE` always overrides discovery.
- [ ] Confirm the real Pi's `/dev/serial/by-id` names and explicitly set `GPS_DEVICE`/`ARDUINO_DEVICE` if more than one matching device exists.

## Automated software validation

- [x] Python compile of V1/V2 final service and dashboard modules.
- [x] V2 policy/state/model-registry regression.
- [x] V1-primary + V2-option dashboard regression.
- [x] Vehicle-settings persistence/live-apply regression.
- [x] Runtime stop/generation/I/O hardening regression.
- [x] Final-service/status-cache regression.
- [x] Production runtime fail-safe regression for stale sensors, shutdown, HTTP hardening, dataset leakage, stable GNSS selection and service policy.
- [x] Dashboard JavaScript syntax validation.
- [x] GPS normalized-route/model/feature regression.
- [x] GPS ON RECORD -> conditioned dataset regression.
- [x] AUTO_LOCAL mapping/navigation regression.
- [x] AUTO_AI dataset/environment regression.
- [x] Manual takeover isolation regression.
- [x] AUTO_AI train -> held-out eval -> ONNX export/parity -> ONNX Runtime smoke.
- [x] AUTO_GPS train -> held-out eval -> four-input ONNX export/parity -> ONNX Runtime smoke.

## Raspberry Pi / physical rover gates

- [ ] Pull the current feature branch onto the target Raspberry Pi.
- [ ] Install/confirm `requirements-pi-ai.txt` in the service virtual environment.
- [ ] Run `scripts/validate_autonomy_v2.sh` successfully on the Pi.
- [ ] Install/copy the current `camera-stream.service`, run `systemctl daemon-reload`, and confirm `systemctl cat camera-stream` matches the repository unit.
- [ ] Start `server_v2_final.py` through the real service and verify `/` loads the V1 dashboard plus the V2 mode chooser.
- [ ] Verify `/api/v2/status` reports `production_guard` and the expected `security` limits/scope.
- [ ] Verify all V1 sensor cards, camera stream, LiDAR, GNSS/RTK, IMU, steering and network controls against real devices.
- [ ] Run `ls -l /dev/serial/by-id/` and confirm GNSS/Arduino resolve to the intended hardware after unplug/replug and reboot.
- [ ] Verify repeated HTTP/1.1 V2 mode changes do not reuse a prior JSON body and oversized/slow POSTs are rejected without affecting control responsiveness.
- [ ] With wheels off the ground, stop/restart `camera-stream.service` and verify SIGTERM de-energizes drive and steering before `TimeoutStopSec` expires.
- [ ] With wheels off the ground, unplug LiDAR, disconnect/disable IMU, interrupt Arduino serial, and invalidate steering feedback during each AUTO path; confirm fail-closed stop/FAULT behavior.
- [ ] Exercise RECORD with a temporary low free-space threshold and verify low-storage/flush failure ends recording safely without corrupting the service.
- [ ] Verify MANUAL command/watchdog and manual takeover behavior with wheels off the ground first.
- [ ] Verify E-STOP and safety reset on real hardware.
- [ ] Benchmark sustained AUTO_AI/AUTO_GPS inference latency/FPS, CPU/RAM/temperature and watchdog margin on the Pi.
- [ ] Inspect the installed service identity with `systemctl show camera-stream -p User -p Group -p SupplementaryGroups` and `id gnss`. The repository unit currently has no `User=` directive; do not switch it to `gnss` until serial/video/I2C/GPIO/gpsd and NetworkManager permissions are proven on the actual Pi.
- [ ] After those permissions are proven, migrate the service away from root if feasible and repeat every hardware/API regression above.
- [ ] Collect real GPS OFF RECORD data and train/evaluate a real AUTO_AI model.
- [ ] Collect repeated GPS ON RECORD runs and train/evaluate a route-bound AUTO_GPS model.
- [ ] Validate AUTO_AI at low speed in a closed area.
- [ ] Validate AUTO_GPS normalized-route following, obstacle behavior/recovery, RTK loss and route deviation at low speed in a closed area.
- [ ] Validate AUTO_LOCAL mapping, relocalization, path planning, avoidance and localization-loss stop at low speed in a closed area.
- [ ] Measure stopping distance and steering tracking margins.
- [ ] Promote model lifecycle only when the matching evidence exists.

## Merge gate

- [ ] Keep PR Draft until target-Pi and closed-area checks above are evidenced.
- [ ] Do not merge to `main` solely because CI is green.
