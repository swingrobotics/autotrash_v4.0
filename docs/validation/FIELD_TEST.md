# Closed-area vehicle validation

This is the current physical validation checklist for the canonical SWING_CAR modes on `main`.

Do not use a person as a test obstacle. Perform low-speed testing in a flat closed area with a clear E-STOP plan. Software/CI success is not evidence that the target vehicle passed these checks.

## 1. Pre-motion hardware checks

With propulsion disabled and wheels lifted where appropriate:

- [ ] Confirm `DISARMED` and zero drive/steering PWM.
- [ ] Confirm controller/dashboard E-STOP latches the rover and Arduino path.
- [ ] Confirm watchdog/communication loss stops command output.
- [ ] Confirm Arduino responses remain fresh.
- [ ] Confirm steering encoder is connected and its reported angle changes in the correct direction when the wheels physically turn.
- [ ] Command center/left/right positions repeatedly and measure target-versus-actual steering error.
- [ ] Confirm steering mechanical limits are never exceeded.
- [ ] Interrupt steering feedback and confirm autonomous paths fail closed.
- [ ] Interrupt LiDAR/IMU/Arduino one at a time and confirm required modes stop/fault.
- [ ] Stop/restart `camera-stream.service` and verify graceful shutdown de-energizes drive/steering within the configured service stop window.

## 2. MANUAL

- [ ] Verify the driver has sole steering/throttle authority except hard safety.
- [ ] Verify deadman/watchdog behavior.
- [ ] Verify E-STOP and reset workflow.
- [ ] Verify steering target/actual feedback at center, left and right.
- [ ] Measure low-speed throttle response and stopping distance before autonomous motion.

## 3. RECORD

Create representative manual RECORD sessions.

- [ ] Confirm camera video and `camera_timestamps.csv` are generated.
- [ ] Confirm actual steering `steering_angle_degrees` is present.
- [ ] Confirm target steering `target_steering_angle_degrees` is present where expected.
- [ ] Confirm LiDAR, IMU, control and optional GNSS streams remain synchronized and valid.
- [ ] Confirm recording stops safely on low-storage/error conditions.
- [ ] Replay/inspect the log without affecting live vehicle control.

For learned driving, collect left/right curves, transitions, recoveries and ordinary obstacle behavior intentionally rather than mostly straight driving.

## 4. AUTO_AI

Before moving the vehicle:

- [ ] Use a model whose lifecycle stage matches the intended explicit test.
- [ ] Verify camera/LiDAR/IMU input freshness gates.
- [ ] Verify steering/throttle output bounds.
- [ ] Verify person hazard remains an external STOP.

Closed-area progression:

- [ ] Straight low-speed run.
- [ ] Gentle left and right turns.
- [ ] Sharper left and right examples represented by the training set.
- [ ] Ordinary obstacle behavior represented by the training demonstrations.
- [ ] Recovery after avoidance/curve transition.
- [ ] Sensor-loss and E-STOP tests.

Do not promote a model lifecycle stage beyond the evidence actually collected.

## 5. AUTO_GPS

Use a newly retrained temporal v3 model when validating the measured-steering contract.

Preflight:

- [ ] Selected normalized route matches the selected GPS model.
- [ ] RTK FIXED is available for live runtime.
- [ ] GNSS/IMU/camera/LiDAR freshness checks pass.
- [ ] Steering encoder feedback is valid.
- [ ] Runtime/model status indicates measured steering is required for a v3 temporal model.

Stationary/wheels-off checks:

- [ ] Turn the physical steering and verify the encoder angle changes.
- [ ] Verify actual encoder feedback is not replaced by the model's previous predicted steering.
- [ ] Verify stale/missing measured steering causes the intended fail-closed behavior for the deployed v3 contract.

Closed-area progression:

- [ ] Start near the expected route start and heading.
- [ ] Run a short straight segment at low speed.
- [ ] Run left and right curve sections.
- [ ] Observe route recovery after demonstrated deviations/avoidance.
- [ ] Confirm excessive route deviation faults/stops.
- [ ] Drop RTK FIXED and confirm fail-closed behavior.
- [ ] Confirm destination completion stop.
- [ ] Repeat representative sections to check consistency, not only one successful pass.

Use synchronized RECORD model preview before field motion to compare MODEL command, HUMAN target and ACTUAL encoder steering on the same source recording.

## 6. AUTO_LOCAL

- [ ] Build/refine a map manually.
- [ ] Reboot/restart and verify global relocalization repeatability.
- [ ] Save and select a destination.
- [ ] Verify A* path generation before propulsion.
- [ ] Run a short low-speed destination path.
- [ ] Verify local obstacle avoidance/replan.
- [ ] Verify persistent localization loss stops/faults.
- [ ] Verify E-STOP/person STOP/common hard safety.

## 7. AUTO / PRETRAINED_ROAD

Top-level `AUTO` is a selector. Verify readiness logic before testing motion.

- [ ] Confirm AUTO refuses motion when no eligible strategy is ready.
- [ ] Confirm the selected strategy matches the current route/map/model/lane conditions.
- [ ] Confirm a strategy safety fault does not silently reset and hot-switch to another driving strategy.

If `PRETRAINED_ROAD` is selected internally:

- [ ] Validate camera calibration/overlay alignment first.
- [ ] Validate lane confidence/loss behavior.
- [ ] Validate inference latency on the actual compute path.
- [ ] Test lane loss and manual takeover before low-speed lane-following motion.

## 8. Windows Worker validation

When the Windows Worker participates:

- [ ] Confirm the Manager starts/stops the Worker explicitly.
- [ ] Confirm reboot does not unexpectedly auto-start the Worker.
- [ ] Confirm the rover remains safe when Ethernet/Worker is disconnected.
- [ ] Measure actual end-to-end remote perception latency if using remote UFLD.
- [ ] Confirm RECORD preview/training jobs have no actuator control authority.

## 9. End-of-test shutdown

- [ ] Return to `DISARMED`.
- [ ] Confirm drive PWM is zero.
- [ ] Confirm steering output is de-energized/stopped as intended.
- [ ] Stop active RECORD/mapping jobs cleanly.
- [ ] Preserve the session IDs, model IDs, route/map IDs and measured results used as evidence.

Historical AUTO_ROUTE/AUTO_HYBRID field checklists are retained only under `docs/archive/` and should not be used as the current mode contract.
