import threading
import time

from autonomous_car import ControlRequest, DriveMode


def install_full_runtime_hardening(full, gps_ai=None):
    """Install final-service hardened LOCAL/mapping/AUTO controllers.

    The base module remains import-compatible for simulations and older entry
    points. The final service replaces its fresh singleton controllers before
    any driving mode is started, so heavy SLAM/lane work is not performed while
    holding lifecycle locks and every run owns an independent cancellation
    Event/generation token.
    """
    legacy = full.legacy

    class HardenedMappingController(full.MappingController):
        def __init__(self):
            super().__init__()
            self.compute_lock = threading.RLock()
            self.generation = 0
            self.timing = None

        def start(self, map_id=None, name=None, refine=False):
            with self.lock:
                if self.active:
                    raise ValueError("A mapping session is already active")
                if self.thread is not None and self.thread.is_alive():
                    raise RuntimeError("Previous mapping worker is still stopping")
                if (
                    full.AUTO_LOCAL_CONTROLLER.active
                    or full.ai.AUTO_AI_CONTROLLER.active
                    or legacy.auto_route_runtime.active
                    or (gps_ai is not None and gps_ai.controller.active)
                ):
                    raise ValueError("Stop autonomous driving before mapping")
                full.v2._ensure_manual_runtime("mapping_prepare")
                if legacy.vehicle_state_machine.snapshot().get("canonical_mode") != "MANUAL":
                    raise ValueError("Mapping requires MANUAL mode")

                refine = bool(refine)
                if refine:
                    document = full.MAP_STORE.get_map(map_id)
                    asset = full.MAP_STORE.asset_path(map_id)
                    grid = full.SparseOccupancyGrid.load(asset)
                    origin = document.get("origin") or {}
                    reference = origin.get("imu_reference_heading_degrees")
                    refine_localized = False
                else:
                    if map_id:
                        document = full.MAP_STORE.get_map(map_id)
                        if document.get("map_file"):
                            raise ValueError("Map already has an asset; use refine=true")
                    else:
                        document = full.MAP_STORE.create_map(name or "Local Map")
                    map_id = document["map_id"]
                    grid = full.SparseOccupancyGrid()
                    imu = full._fresh_imu_snapshot()
                    reference = float(imu["global_heading_degrees"])
                    refine_localized = True

                engine = full.LidarImuSlam(grid)
                if not refine:
                    engine.reset_mapping()
                self.map_id = map_id
                self.refine = refine
                self.engine = engine
                self.imu_reference_heading_degrees = reference
                self.refine_localized = refine_localized
                self.error = None
                self.started_at = time.time()
                self.session_id = f"map_{int(self.started_at)}"
                self.score_total = 0.0
                self.score_count = 0
                self.timing = None
                self.active = True
                self.generation += 1
                generation = self.generation
                stop_event = threading.Event()
                self.stop_event = stop_event
                self.thread = threading.Thread(
                    target=self._run_hardened,
                    args=(generation, stop_event, engine),
                    daemon=True,
                )
                self.thread.start()
                return self.snapshot()

        def stop(self, save=True):
            stop_event = self.stop_event
            stop_event.set()
            with self.lock:
                if not self.active and self.engine is None:
                    return self.snapshot()
                self.active = False
                self.generation += 1
                thread = self.thread
                engine = self.engine
                map_id = self.map_id
                session_id = self.session_id
                reference = self.imu_reference_heading_degrees
                score_total = self.score_total
                score_count = self.score_count
            if thread and thread is not threading.current_thread():
                thread.join(timeout=4.0)
            if thread and thread.is_alive():
                raise RuntimeError("Mapping worker did not stop before save")

            if save and engine is not None and map_id:
                with self.compute_lock:
                    if engine.grid.scan_count < 3:
                        raise ValueError("Mapping session has too few valid scans to save")
                    filename = "occupancy.json.gz"
                    path = full.MAP_STORE.map_path(map_id) / filename
                    quality = engine.grid.save(path)
                    quality.update(
                        mean_localization_score=(
                            score_total / score_count if score_count else None
                        ),
                        trajectory_points=len(engine.trajectory),
                        mapping_session_id=session_id,
                    )
                    full.MAP_STORE.register_map_asset(
                        map_id,
                        filename,
                        full.SparseOccupancyGrid.FORMAT,
                        origin={
                            "x": 0.0,
                            "y": 0.0,
                            "heading_degrees": 0.0,
                            "imu_reference_heading_degrees": reference,
                        },
                        quality=quality,
                    )
                    full.MAP_STORE.add_mapping_session(map_id, session_id)
            return self.snapshot()

        def snapshot(self):
            with self.lock:
                state = {
                    "active": self.active,
                    "map_id": self.map_id,
                    "refine": self.refine,
                    "session_id": self.session_id,
                    "started_at": self.started_at,
                    "error": self.error,
                    "refine_localized": self.refine_localized,
                    "imu_reference_heading_degrees": self.imu_reference_heading_degrees,
                    "mean_localization_score": (
                        self.score_total / self.score_count if self.score_count else None
                    ),
                    "generation": self.generation,
                    "timing": self.timing,
                    "engine": self.engine,
                }
            engine = state.pop("engine")
            with self.compute_lock:
                state["slam"] = None if engine is None else engine.snapshot()
            return state

        def current_pose(self):
            with self.lock:
                engine = self.engine
            if engine is None:
                return None
            with self.compute_lock:
                return engine.pose

        def _current(self, generation, stop_event, engine):
            with self.lock:
                return (
                    self.active
                    and self.generation == generation
                    and self.stop_event is stop_event
                    and not stop_event.is_set()
                    and self.engine is engine
                )

        def _run_hardened(self, generation, stop_event, engine):
            while not stop_event.wait(0.10):
                started = time.perf_counter()
                try:
                    lidar_started = time.perf_counter()
                    _, points = full._lidar_points()
                    imu = full._fresh_imu_snapshot()
                    sensor_seconds = time.perf_counter() - lidar_started
                    with self.lock:
                        if not self._current(generation, stop_event, engine):
                            return
                        reference = self.imu_reference_heading_degrees
                        refine = self.refine
                        refine_localized = self.refine_localized
                    if reference is None:
                        reference = float(imu["global_heading_degrees"])
                    heading = full._relative_map_heading(
                        imu.get("global_heading_degrees"), reference
                    )
                    compute_started = time.perf_counter()
                    with self.compute_lock:
                        if refine and not refine_localized:
                            localization = engine.global_localize(points, heading)
                            if not localization.localized:
                                with self.lock:
                                    if self._current(generation, stop_event, engine):
                                        self.imu_reference_heading_degrees = reference
                                        self.timing = {
                                            "sensor_snapshot_seconds": sensor_seconds,
                                            "slam_seconds": time.perf_counter() - compute_started,
                                            "loop_seconds": time.perf_counter() - started,
                                        }
                                continue
                            refine_localized = True
                        result = engine.process_mapping_scan(points, heading)
                    compute_seconds = time.perf_counter() - compute_started
                    with self.lock:
                        if not self._current(generation, stop_event, engine):
                            return
                        self.imu_reference_heading_degrees = reference
                        self.refine_localized = refine_localized
                        if result.localized:
                            self.score_total += float(result.score)
                            self.score_count += 1
                        self.timing = {
                            "sensor_snapshot_seconds": sensor_seconds,
                            "slam_seconds": compute_seconds,
                            "loop_seconds": time.perf_counter() - started,
                        }
                except Exception as error:
                    with self.lock:
                        if self.generation == generation and self.stop_event is stop_event:
                            self.error = f"{type(error).__name__}: {error}"
                    stop_event.wait(0.20)

    class HardenedAutoLocalController(full.AutoLocalController):
        def __init__(self):
            super().__init__()
            self.compute_lock = threading.RLock()
            self.generation = 0

        def start(self, map_id=None, destination_id=None):
            with self.lock:
                if self.active:
                    return self.snapshot()
                if self.thread is not None and self.thread.is_alive():
                    raise RuntimeError("Previous AUTO_LOCAL worker is still stopping")
                if full.MAPPING_CONTROLLER.active:
                    raise ValueError("Stop mapping before AUTO_LOCAL")
                selected = full._selected_local()
                map_id = map_id or selected.get("map_id")
                destination_id = destination_id or selected.get("destination_id")
                if not map_id or not destination_id:
                    raise ValueError("Select a local map and destination first")

            # Potentially expensive map load/global localization occurs before
            # the vehicle enters AUTO_LOCAL and while the motor remains stopped.
            document = full.MAP_STORE.get_map(map_id)
            destination = full.MAP_STORE.get_destination(map_id, destination_id)
            grid = full.SparseOccupancyGrid.load(full.MAP_STORE.asset_path(map_id))
            imu = full._fresh_imu_snapshot()
            _, points = full._lidar_points()
            reference = (document.get("origin") or {}).get("imu_reference_heading_degrees")
            heading = full._relative_map_heading(imu.get("global_heading_degrees"), reference)
            engine = full.LidarImuSlam(grid)
            localization = engine.global_localize(points, heading)
            if not localization.localized:
                raise ValueError(
                    f"Unable to localize on map (score={localization.score:.3f})"
                )
            planner = full.AutoLocalPlanner(grid, destination)
            planner.plan_from_pose(localization.pose)

            with self.lock:
                if self.active:
                    return self.snapshot()
                if full.ai.AUTO_AI_CONTROLLER.active:
                    full.ai.AUTO_AI_CONTROLLER.stop("auto_local_takeover")
                if gps_ai is not None and gps_ai.controller.active:
                    gps_ai.controller.stop("auto_local_takeover")
                if legacy.auto_route_runtime.active:
                    legacy.auto_route_runtime.stop("auto_local_takeover")
                if legacy.record_manager.active:
                    legacy.record_manager.stop()
                mode = legacy.vehicle_state_machine.mode
                if mode in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
                    raise ValueError("Safety reset is required before AUTO_LOCAL")
                if mode not in {
                    DriveMode.DISARMED,
                    DriveMode.MANUAL,
                    DriveMode.MANUAL_ASSIST,
                }:
                    legacy.vehicle_state_machine.transition(
                        DriveMode.DISARMED, "auto_local_prepare"
                    )
                legacy.motor_controller.stop()
                legacy.vehicle_state_machine.transition(
                    DriveMode.AUTO_LOCAL, "auto_local_started"
                )
                metadata = dict(legacy.recording_metadata())
                metadata.update(
                    purpose="AUTO_LOCAL",
                    record_gps=False,
                    map_id=map_id,
                    destination_id=destination_id,
                    autonomy_schema="v2",
                )
                legacy.record_manager.start(metadata)
                self.owns_recording = True
                self.map_id = map_id
                self.destination_id = destination_id
                self.destination = destination
                self.engine = engine
                self.planner = planner
                self.imu_reference_heading_degrees = reference
                self.last_command = None
                self.last_lane = None
                self.error = None
                self.started_at = time.time()
                self.localization_lost_since = None
                self.steering_tracking_guard.reset()
                self.active = True
                self.generation += 1
                generation = self.generation
                stop_event = threading.Event()
                self.stop_event = stop_event
                self.thread = threading.Thread(
                    target=self._run_hardened,
                    args=(generation, stop_event, engine, planner),
                    daemon=True,
                )
                self.thread.start()
                return self.snapshot()

        def stop(self, reason="operator_stop"):
            stop_event = self.stop_event
            stop_event.set()
            with self.lock:
                self.active = False
                self.generation += 1
                legacy.motor_controller.stop()
                self.steering_tracking_guard.reset()
                if self.owns_recording:
                    self.owns_recording = False
                    threading.Thread(target=legacy.record_manager.stop, daemon=True).start()
                if legacy.vehicle_state_machine.mode == DriveMode.AUTO_LOCAL:
                    legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)
                return self.snapshot()

        def _current(self, generation, stop_event, engine, planner):
            return (
                self.active
                and self.generation == generation
                and self.stop_event is stop_event
                and not stop_event.is_set()
                and self.engine is engine
                and self.planner is planner
            )

        def snapshot(self):
            with self.lock:
                state = {
                    "active": self.active,
                    "map_id": self.map_id,
                    "destination_id": self.destination_id,
                    "destination": self.destination,
                    "error": self.error,
                    "started_at": self.started_at,
                    "last_command": self.last_command,
                    "lane": self.last_lane,
                    "steering_tracking": self.steering_tracking_guard.snapshot(),
                    "generation": self.generation,
                    "engine": self.engine,
                    "planner_object": self.planner,
                }
            engine = state.pop("engine")
            planner = state.pop("planner_object")
            with self.compute_lock:
                state["slam"] = None if engine is None else engine.snapshot()
                state["planner"] = None if planner is None else planner.snapshot()
            return state

        def current_pose(self):
            with self.lock:
                engine = self.engine
            if engine is None:
                return None
            with self.compute_lock:
                if not engine.localized:
                    return None
                return engine.pose

        def _run_hardened(self, generation, stop_event, engine, planner):
            previous_loop = None
            while not stop_event.is_set():
                loop_started = time.perf_counter()
                monotonic_started = time.monotonic()
                loop_delay = (
                    0.0
                    if previous_loop is None
                    else max(0.0, monotonic_started - previous_loop)
                )
                previous_loop = monotonic_started
                try:
                    sensor_started = time.perf_counter()
                    lidar, points = full._lidar_points()
                    imu = full._fresh_imu_snapshot()
                    heading = full._relative_map_heading(
                        imu.get("global_heading_degrees"),
                        self.imu_reference_heading_degrees,
                    )
                    sensor_seconds = time.perf_counter() - sensor_started
                    if stop_event.is_set():
                        return

                    compute_started = time.perf_counter()
                    with self.compute_lock:
                        localization = engine.update_localization(points, heading)
                        command = None
                        if localization.localized:
                            command = planner.update(
                                engine.pose,
                                lidar.get("safety_points") or [],
                            )
                    local_seconds = time.perf_counter() - compute_started

                    with self.lock:
                        if not self._current(generation, stop_event, engine, planner):
                            return
                        if not localization.localized:
                            legacy.motor_controller.stop()
                            if self.localization_lost_since is None:
                                self.localization_lost_since = time.monotonic()
                            if (
                                time.monotonic() - self.localization_lost_since
                                > self.LOCALIZATION_HOLD_SECONDS
                            ):
                                self._fault_locked("LOCALIZATION_LOST")
                                return
                            self.last_command = {
                                "localization": localization.as_dict(),
                                "timing": {
                                    "sensor_snapshot_seconds": sensor_seconds,
                                    "local_planning_seconds": local_seconds,
                                    "control_loop_seconds": time.perf_counter() - loop_started,
                                },
                            }
                            stop_event.wait(0.10)
                            continue
                        self.localization_lost_since = None

                        motor = legacy.motor_controller.snapshot()
                        tracking_fault = self.steering_tracking_guard.evaluate(
                            motor.get("target_steering_angle_degrees"),
                            motor.get("steering_angle_degrees"),
                            active=motor.get("steering_control_mode") == "ANGLE",
                        )
                        if tracking_fault:
                            self._fault_locked(tracking_fault)
                            return
                        if command is None:
                            self._fault_locked("AUTO_LOCAL_PLANNER_UNAVAILABLE")
                            return
                        if command.fault:
                            self._fault_locked(command.fault)
                            return
                        if command.finished:
                            stop_event.set()
                            legacy.record_manager.add_event(
                                "AUTO_LOCAL_COMPLETED", self.destination_id or ""
                            )
                            self.active = False
                            self.generation += 1
                            legacy.motor_controller.stop()
                            if self.owns_recording:
                                self.owns_recording = False
                                threading.Thread(
                                    target=legacy.record_manager.stop, daemon=True
                                ).start()
                            legacy.vehicle_state_machine.transition(
                                DriveMode.MANUAL_ASSIST,
                                "local_destination_reached",
                            )
                            return

                    lane_started = time.perf_counter()
                    lane = self._lane_assist()
                    lane_seconds = time.perf_counter() - lane_started
                    steering_angle = command.steering_angle_degrees
                    if lane and lane.get("detected"):
                        confidence = float(lane.get("confidence") or 0.0)
                        if confidence >= legacy.AUTO_LANE_MIN_CONFIDENCE:
                            steering_angle += confidence * float(
                                lane.get("correction_angle_degrees") or 0.0
                            )
                    steering_angle = max(-20.0, min(20.0, steering_angle))
                    calibrated_throttle = legacy.throttle_calibration.throttle_for_speed(
                        command.throttle
                    )
                    request = ControlRequest(
                        throttle=calibrated_throttle,
                        steering=legacy.normalized_steering_for_angle(steering_angle),
                        enabled=True,
                        source="auto_local",
                    )
                    safety_started = time.perf_counter()
                    decision = legacy.safety_supervisor.evaluate(
                        request, legacy.safety_context(loop_delay)
                    )
                    safety_seconds = time.perf_counter() - safety_started
                    command_data = {
                        **command.as_dict(),
                        "steering_angle_degrees": steering_angle,
                        "calibrated_throttle": calibrated_throttle,
                        "localization": localization.as_dict(),
                        "safety": decision.as_dict(),
                        "timing": {
                            "sensor_snapshot_seconds": sensor_seconds,
                            "local_planning_seconds": local_seconds,
                            "lane_seconds": lane_seconds,
                            "safety_seconds": safety_seconds,
                            "loop_period_seconds": loop_delay,
                        },
                    }

                    with self.lock:
                        if not self._current(generation, stop_event, engine, planner):
                            return
                        self.last_lane = lane
                        actuation_started = time.perf_counter()
                        if not decision.allowed:
                            legacy.motor_controller.set_drive(0.0, True)
                            legacy.motor_controller.stop_steering()
                            if decision.stop_reason not in {
                                "CAMERA_OBJECT_STOP",
                                "OBSTACLE_STOP",
                                "OBSTACLE_RESTART_DELAY",
                            }:
                                self.last_command = command_data
                                self._fault_locked(
                                    decision.stop_reason or "AUTO_LOCAL_SAFETY_STOP"
                                )
                                return
                        else:
                            legacy.motor_controller.set_drive(0.0, True)
                            steering_result = legacy.motor_controller.set_steering(
                                decision.final_steering
                            )
                            if steering_result.get("steering_rejection"):
                                self.last_command = command_data
                                self._fault_locked("STEERING_COMMAND_REJECTED")
                                return
                            legacy.motor_controller.set_drive(
                                decision.final_throttle, True
                            )
                        command_data["timing"]["actuation_seconds"] = (
                            time.perf_counter() - actuation_started
                        )
                        command_data["timing"]["control_loop_seconds"] = (
                            time.perf_counter() - loop_started
                        )
                        self.last_command = command_data
                except Exception as error:
                    self._fault(
                        generation,
                        stop_event,
                        f"AUTO_LOCAL_RUNTIME_ERROR:{type(error).__name__}:{error}",
                    )
                    return
                elapsed = time.perf_counter() - loop_started
                stop_event.wait(max(0.0, 0.10 - elapsed))

        def _fault(self, generation, stop_event, reason):
            stop_event.set()
            with self.lock:
                if generation != self.generation or self.stop_event is not stop_event:
                    return
                self._fault_locked(reason)

        def _fault_locked(self, reason):
            self.active = False
            self.stop_event.set()
            self.generation += 1
            self.error = reason
            legacy.motor_controller.stop()
            if self.owns_recording:
                self.owns_recording = False
                threading.Thread(target=legacy.record_manager.stop, daemon=True).start()
            if legacy.vehicle_state_machine.mode != DriveMode.FAULT:
                legacy.vehicle_state_machine.transition(DriveMode.FAULT, reason)

    class HardenedAutoOrchestrator(full.AutoOrchestrator):
        def __init__(self):
            super().__init__()
            self.generation = 0

        def _current(self, generation):
            with self.lock:
                return self.active and self.generation == generation

        def _append_attempt(self, generation, document):
            with self.lock:
                if not self.active or self.generation != generation:
                    return False
                self.last_attempts.append(document)
                return True

        def start(self):
            with self.lock:
                self.generation += 1
                generation = self.generation
                self.active = True
                self.strategy = None
                self.reason = None
                self.resource_id = None
                self.last_attempts = []
                self.started_at = time.time()

            preflight = full.v2._route_preflight()
            if not self._append_attempt(
                generation,
                {
                    "mode": "AUTO_GPS",
                    "ready": preflight.get("ready"),
                    "details": preflight,
                },
            ):
                raise ValueError("AUTO selection cancelled")
            if preflight.get("ready") and self._current(generation):
                result = full.ai.enhanced_select_mode("AUTO_GPS")
                if result.get("accepted"):
                    return self._selected_guarded(
                        generation,
                        "AUTO_GPS",
                        "gps_preflight_ready",
                        None,
                        result,
                    )

            local_selection = full._selected_local()
            if (
                self._current(generation)
                and local_selection.get("map_id")
                and local_selection.get("destination_id")
            ):
                check = full.AUTO_LOCAL_CONTROLLER.preflight()
                if not self._append_attempt(
                    generation, {"mode": "AUTO_LOCAL", **check}
                ):
                    raise ValueError("AUTO selection cancelled")
                if check.get("ready") and self._current(generation):
                    result = full.AUTO_LOCAL_CONTROLLER.start()
                    return self._selected_guarded(
                        generation,
                        "AUTO_LOCAL",
                        "saved_map_localized",
                        local_selection.get("map_id"),
                        result,
                    )

            if not self._current(generation):
                raise ValueError("AUTO selection cancelled")
            tags = full._auto_config().get("environment_tags") or []
            compatible = full.ai.MODEL_REGISTRY.compatible_for_auto(tags)
            if not self._append_attempt(
                generation,
                {
                    "mode": "AUTO_AI",
                    "environment_tags": tags,
                    "compatible_models": [
                        model.get("model_id") for model in compatible
                    ],
                },
            ):
                raise ValueError("AUTO selection cancelled")
            if compatible and self._current(generation):
                selected_id = full.ai.ai_status().get("selected_model_id")
                model = next(
                    (
                        item
                        for item in compatible
                        if item.get("model_id") == selected_id
                    ),
                    compatible[0],
                )
                full.ai.select_ai_model(model["model_id"])
                if not self._current(generation):
                    raise ValueError("AUTO selection cancelled")
                result = full.ai.AUTO_AI_CONTROLLER.start(model["model_id"])
                return self._selected_guarded(
                    generation,
                    "AUTO_AI",
                    "auto_allowed_ai_environment_match",
                    model["model_id"],
                    result,
                )

            with self.lock:
                if self.generation != generation:
                    raise ValueError("AUTO selection cancelled")
                self.active = False
                self.reason = "no_safe_autonomous_strategy"
            legacy.motor_controller.stop()
            raise ValueError(
                "AUTO found no ready GPS, LOCAL, or AUTO_ALLOWED AI strategy"
            )

        def stop(self):
            with self.lock:
                strategy = self.strategy
                self.generation += 1
                self.active = False
                self.strategy = None
                self.reason = "operator_stop"
                self.resource_id = None
            # Cancel the strategy selected on behalf of AUTO. This also closes
            # the race where STOP lands just after a strategy's start call.
            if strategy == "AUTO_GPS" and gps_ai is not None and gps_ai.controller.active:
                gps_ai.controller.stop("auto_selector_stop")
            elif strategy == "AUTO_LOCAL" and full.AUTO_LOCAL_CONTROLLER.active:
                full.AUTO_LOCAL_CONTROLLER.stop("auto_selector_stop")
            elif strategy == "AUTO_AI" and full.ai.AUTO_AI_CONTROLLER.active:
                full.ai.AUTO_AI_CONTROLLER.stop("auto_selector_stop")
            legacy.motor_controller.stop()

        def snapshot(self):
            with self.lock:
                return {
                    "active": self.active,
                    "strategy": self.strategy,
                    "reason": self.reason,
                    "resource_id": self.resource_id,
                    "started_at": self.started_at,
                    "last_attempts": list(self.last_attempts),
                    "generation": self.generation,
                }

        def _cancel_started_strategy(self, strategy):
            if strategy == "AUTO_GPS" and gps_ai is not None and gps_ai.controller.active:
                gps_ai.controller.stop("auto_selector_cancelled")
            elif strategy == "AUTO_LOCAL" and full.AUTO_LOCAL_CONTROLLER.active:
                full.AUTO_LOCAL_CONTROLLER.stop("auto_selector_cancelled")
            elif strategy == "AUTO_AI" and full.ai.AUTO_AI_CONTROLLER.active:
                full.ai.AUTO_AI_CONTROLLER.stop("auto_selector_cancelled")
            legacy.motor_controller.stop()

        def _selected_guarded(
            self, generation, strategy, reason, resource_id, result
        ):
            with self.lock:
                if not self.active or self.generation != generation:
                    cancelled = True
                else:
                    cancelled = False
                    self.strategy = strategy
                    self.reason = reason
                    self.resource_id = resource_id
                    response = {
                        "accepted": True,
                        "target": "AUTO",
                        "selected_strategy": strategy,
                        "reason": reason,
                        "resource_id": resource_id,
                        "runtime": result,
                    }
            if cancelled:
                self._cancel_started_strategy(strategy)
                raise ValueError("AUTO selection cancelled")
            return response

    # Final service installs this before any user mode can start. Replacing the
    # module globals is safe because server_v2_full functions resolve them at
    # call time rather than capturing the original objects.
    full.MAPPING_CONTROLLER = HardenedMappingController()
    full.AUTO_LOCAL_CONTROLLER = HardenedAutoLocalController()
    full.AUTO_ORCHESTRATOR = HardenedAutoOrchestrator()
    return {
        "mapping": full.MAPPING_CONTROLLER,
        "auto_local": full.AUTO_LOCAL_CONTROLLER,
        "auto": full.AUTO_ORCHESTRATOR,
    }


__all__ = ["install_full_runtime_hardening"]
