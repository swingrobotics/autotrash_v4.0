#!/usr/bin/env python3
"""Full Autonomy V2 integration server.

This is the feature-branch service entrypoint that joins the proven hardware
backend with MANUAL/RECORD/AUTO_AI/AUTO_GPS/AUTO_LOCAL/AUTO, map management,
AI model management, and conservative automatic strategy selection.
"""

import json
import math
import os
import threading
import time

import server_v2_ai as ai
from autonomous_car import ControlRequest, DriveMode
from autonomous_car.ai import MODEL_LIFECYCLE, ModelRegistryError
from autonomous_car.localization import (
    LidarImuSlam,
    MapStore,
    MapStoreError,
    SparseOccupancyGrid,
)
from autonomous_car.modes import AutoLocalPlanner
from autonomous_car.safety import SteeringTrackingGuard


legacy = ai.legacy
v2 = ai.v2
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
MAPS_ROOT = os.environ.get("AUTONOMY_MAPS_PATH", os.path.join(PROJECT_ROOT, "maps"))
MAP_STORE = MapStore(MAPS_ROOT)
LOCAL_SELECTION_PATH = os.path.join(MAPS_ROOT, "selected-local.json")
AUTO_CONFIG_PATH = os.path.join(PROJECT_ROOT, "autonomy-auto.json")


def _atomic_json(path, document):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump(document, file, ensure_ascii=False, indent=2, sort_keys=True)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)


def _read_json(path, default):
    try:
        with open(path, "r", encoding="utf-8") as file:
            document = json.load(file)
        return document if isinstance(document, dict) else dict(default)
    except (OSError, ValueError, json.JSONDecodeError):
        return dict(default)


def _relative_map_heading(current_heading_degrees, reference_heading_degrees):
    """Convert compass-style IMU heading into CCW map-frame yaw degrees."""
    if current_heading_degrees is None or reference_heading_degrees is None:
        return None
    current = float(current_heading_degrees)
    reference = float(reference_heading_degrees)
    return (-(current - reference) + 180.0) % 360.0 - 180.0


def _fresh_imu_snapshot(maximum_age_seconds=0.25):
    snapshot = legacy.imu_monitor.snapshot()
    last_update = snapshot.get("last_update")
    if last_update is None or time.time() - float(last_update) > maximum_age_seconds:
        raise ValueError("AUTO_LOCAL requires a fresh IMU")
    if snapshot.get("global_heading_degrees") is None:
        raise ValueError("AUTO_LOCAL requires IMU global heading")
    return snapshot


def _lidar_points(require_connected=True):
    snapshot = legacy.lidar_monitor.snapshot()
    if require_connected and not snapshot.get("connected"):
        raise ValueError("AUTO_LOCAL requires connected LiDAR")
    points = snapshot.get("points") or []
    if require_connected and len(points) < 40:
        raise ValueError("AUTO_LOCAL requires a populated LiDAR scan")
    return snapshot, points


def _selected_local():
    return _read_json(LOCAL_SELECTION_PATH, {"map_id": None, "destination_id": None})


def select_local_target(map_id, destination_id):
    document = MAP_STORE.get_map(map_id)
    MAP_STORE.get_destination(map_id, destination_id)
    if not document.get("map_file"):
        raise MapStoreError("Selected map has no saved occupancy asset")
    selected = {
        "map_id": document["map_id"],
        "destination_id": str(destination_id),
    }
    _atomic_json(LOCAL_SELECTION_PATH, selected)
    return selected


def _auto_config():
    document = _read_json(AUTO_CONFIG_PATH, {"environment_tags": []})
    document["environment_tags"] = [
        str(tag).strip()
        for tag in document.get("environment_tags") or []
        if str(tag).strip()
    ]
    return document


def set_environment_tags(tags):
    if isinstance(tags, str):
        tags = [part.strip() for part in tags.split(",")]
    normalized = sorted({str(tag).strip().lower() for tag in (tags or []) if str(tag).strip()})
    document = {"environment_tags": normalized, "updated_at": time.time()}
    _atomic_json(AUTO_CONFIG_PATH, document)
    return document


class MappingController:
    """Background mapping/refinement while the operator remains in MANUAL."""

    def __init__(self):
        self.lock = threading.RLock()
        self.active = False
        self.map_id = None
        self.refine = False
        self.engine = None
        self.imu_reference_heading_degrees = None
        self.error = None
        self.started_at = None
        self.thread = None
        self.stop_event = threading.Event()
        self.score_total = 0.0
        self.score_count = 0
        self.session_id = None
        self.refine_localized = False

    def start(self, map_id=None, name=None, refine=False):
        with self.lock:
            if self.active:
                raise ValueError("A mapping session is already active")
            if AUTO_LOCAL_CONTROLLER.active or ai.AUTO_AI_CONTROLLER.active or legacy.auto_route_runtime.active:
                raise ValueError("Stop autonomous driving before mapping")
            v2._ensure_manual_runtime("mapping_prepare")
            if legacy.vehicle_state_machine.snapshot().get("canonical_mode") != "MANUAL":
                raise ValueError("Mapping requires MANUAL mode")

            refine = bool(refine)
            if refine:
                document = MAP_STORE.get_map(map_id)
                asset = MAP_STORE.asset_path(map_id)
                grid = SparseOccupancyGrid.load(asset)
                origin = document.get("origin") or {}
                self.imu_reference_heading_degrees = origin.get("imu_reference_heading_degrees")
                self.refine_localized = False
            else:
                if map_id:
                    document = MAP_STORE.get_map(map_id)
                    if document.get("map_file"):
                        raise ValueError("Map already has an asset; use refine=true")
                else:
                    document = MAP_STORE.create_map(name or "Local Map")
                map_id = document["map_id"]
                grid = SparseOccupancyGrid()
                imu = _fresh_imu_snapshot()
                self.imu_reference_heading_degrees = float(imu["global_heading_degrees"])
                self.refine_localized = True

            self.map_id = map_id
            self.refine = refine
            self.engine = LidarImuSlam(grid)
            if not refine:
                self.engine.reset_mapping()
            self.error = None
            self.started_at = time.time()
            self.session_id = f"map_{int(self.started_at)}"
            self.score_total = 0.0
            self.score_count = 0
            self.active = True
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return self.snapshot()

    def stop(self, save=True):
        with self.lock:
            if not self.active and self.engine is None:
                return self.snapshot()
            self.active = False
            self.stop_event.set()
            thread = self.thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=4.0)
        with self.lock:
            if save and self.engine is not None and self.map_id:
                if self.engine.grid.scan_count < 3:
                    raise ValueError("Mapping session has too few valid scans to save")
                filename = "occupancy.json.gz"
                path = MAP_STORE.map_path(self.map_id) / filename
                quality = self.engine.grid.save(path)
                quality.update(
                    mean_localization_score=(
                        self.score_total / self.score_count if self.score_count else None
                    ),
                    trajectory_points=len(self.engine.trajectory),
                    mapping_session_id=self.session_id,
                )
                MAP_STORE.register_map_asset(
                    self.map_id,
                    filename,
                    SparseOccupancyGrid.FORMAT,
                    origin={
                        "x": 0.0,
                        "y": 0.0,
                        "heading_degrees": 0.0,
                        "imu_reference_heading_degrees": self.imu_reference_heading_degrees,
                    },
                    quality=quality,
                )
                MAP_STORE.add_mapping_session(self.map_id, self.session_id)
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {
                "active": self.active,
                "map_id": self.map_id,
                "refine": self.refine,
                "session_id": self.session_id,
                "started_at": self.started_at,
                "error": self.error,
                "refine_localized": self.refine_localized,
                "imu_reference_heading_degrees": self.imu_reference_heading_degrees,
                "slam": None if self.engine is None else self.engine.snapshot(),
                "mean_localization_score": (
                    self.score_total / self.score_count if self.score_count else None
                ),
            }

    def current_pose(self):
        with self.lock:
            if self.engine is None:
                return None
            return self.engine.pose

    def _run(self):
        while not self.stop_event.wait(0.10):
            try:
                lidar, points = _lidar_points()
                imu = _fresh_imu_snapshot()
                if self.imu_reference_heading_degrees is None:
                    self.imu_reference_heading_degrees = float(imu["global_heading_degrees"])
                heading = _relative_map_heading(
                    imu.get("global_heading_degrees"),
                    self.imu_reference_heading_degrees,
                )
                with self.lock:
                    if not self.active or self.engine is None:
                        return
                    if self.refine and not self.refine_localized:
                        result = self.engine.global_localize(points, heading)
                        if not result.localized:
                            continue
                        self.refine_localized = True
                    result = self.engine.process_mapping_scan(points, heading)
                    if result.localized:
                        self.score_total += float(result.score)
                        self.score_count += 1
            except Exception as error:
                with self.lock:
                    self.error = f"{type(error).__name__}: {error}"
                self.stop_event.wait(0.20)


class AutoLocalController:
    LOCALIZATION_HOLD_SECONDS = 3.0

    def __init__(self):
        self.lock = threading.RLock()
        self.active = False
        self.map_id = None
        self.destination_id = None
        self.destination = None
        self.engine = None
        self.planner = None
        self.imu_reference_heading_degrees = None
        self.last_command = None
        self.last_lane = None
        self.error = None
        self.started_at = None
        self.thread = None
        self.stop_event = threading.Event()
        self.localization_lost_since = None
        self.owns_recording = False
        self.steering_tracking_guard = SteeringTrackingGuard(
            legacy.AUTO_STEERING_MAX_ERROR_DEGREES,
            legacy.AUTO_STEERING_ERROR_TIMEOUT_SECONDS,
        )

    def preflight(self, map_id=None, destination_id=None):
        selected = _selected_local()
        map_id = map_id or selected.get("map_id")
        destination_id = destination_id or selected.get("destination_id")
        if not map_id or not destination_id:
            return {"ready": False, "error": "LOCAL_MAP_AND_DESTINATION_REQUIRED"}
        try:
            document = MAP_STORE.get_map(map_id)
            destination = MAP_STORE.get_destination(map_id, destination_id)
            grid = SparseOccupancyGrid.load(MAP_STORE.asset_path(map_id))
            imu = _fresh_imu_snapshot()
            _, points = _lidar_points()
            reference = (document.get("origin") or {}).get("imu_reference_heading_degrees")
            heading = _relative_map_heading(imu.get("global_heading_degrees"), reference)
            engine = LidarImuSlam(grid)
            result = engine.global_localize(points, heading)
            if not result.localized:
                return {
                    "ready": False,
                    "error": "LOCALIZATION_FAILED",
                    "localization": result.as_dict(),
                }
            planner = AutoLocalPlanner(grid, destination)
            planned = planner.plan_from_pose(result.pose)
            return {
                "ready": True,
                "map_id": map_id,
                "destination_id": destination_id,
                "localization": result.as_dict(),
                "path": planned.as_dict(),
            }
        except Exception as error:
            return {"ready": False, "error": f"{type(error).__name__}: {error}"}

    def start(self, map_id=None, destination_id=None):
        with self.lock:
            if self.active:
                return self.snapshot()
            if MAPPING_CONTROLLER.active:
                raise ValueError("Stop mapping before AUTO_LOCAL")
            selected = _selected_local()
            map_id = map_id or selected.get("map_id")
            destination_id = destination_id or selected.get("destination_id")
            if not map_id or not destination_id:
                raise ValueError("Select a local map and destination first")

            document = MAP_STORE.get_map(map_id)
            destination = MAP_STORE.get_destination(map_id, destination_id)
            grid = SparseOccupancyGrid.load(MAP_STORE.asset_path(map_id))
            imu = _fresh_imu_snapshot()
            _, points = _lidar_points()
            reference = (document.get("origin") or {}).get("imu_reference_heading_degrees")
            heading = _relative_map_heading(imu.get("global_heading_degrees"), reference)
            engine = LidarImuSlam(grid)
            localization = engine.global_localize(points, heading)
            if not localization.localized:
                raise ValueError(
                    f"Unable to localize on map (score={localization.score:.3f})"
                )
            planner = AutoLocalPlanner(grid, destination)
            planner.plan_from_pose(localization.pose)

            if ai.AUTO_AI_CONTROLLER.active:
                ai.AUTO_AI_CONTROLLER.stop("auto_local_takeover")
            if legacy.auto_route_runtime.active:
                legacy.auto_route_runtime.stop("auto_local_takeover")
            if legacy.record_manager.active:
                legacy.record_manager.stop()
            mode = legacy.vehicle_state_machine.mode
            if mode in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
                raise ValueError("Safety reset is required before AUTO_LOCAL")
            if mode not in {DriveMode.DISARMED, DriveMode.MANUAL, DriveMode.MANUAL_ASSIST}:
                legacy.vehicle_state_machine.transition(DriveMode.DISARMED, "auto_local_prepare")
            legacy.motor_controller.stop()
            legacy.vehicle_state_machine.transition(DriveMode.AUTO_LOCAL, "auto_local_started")

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
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return self.snapshot()

    def stop(self, reason="operator_stop"):
        with self.lock:
            self.active = False
            self.stop_event.set()
            legacy.motor_controller.stop()
            self.steering_tracking_guard.reset()
            if self.owns_recording:
                self.owns_recording = False
                threading.Thread(target=legacy.record_manager.stop, daemon=True).start()
            if legacy.vehicle_state_machine.mode == DriveMode.AUTO_LOCAL:
                legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {
                "active": self.active,
                "map_id": self.map_id,
                "destination_id": self.destination_id,
                "destination": self.destination,
                "error": self.error,
                "started_at": self.started_at,
                "slam": None if self.engine is None else self.engine.snapshot(),
                "planner": None if self.planner is None else self.planner.snapshot(),
                "last_command": self.last_command,
                "lane": self.last_lane,
                "steering_tracking": self.steering_tracking_guard.snapshot(),
            }

    def current_pose(self):
        with self.lock:
            if self.engine is None or not self.engine.localized:
                return None
            return self.engine.pose

    def _run(self):
        previous_loop = None
        while not self.stop_event.wait(0.0):
            started = time.monotonic()
            loop_delay = 0.0 if previous_loop is None else max(0.0, started - previous_loop)
            previous_loop = started
            try:
                lidar, points = _lidar_points()
                imu = _fresh_imu_snapshot()
                heading = _relative_map_heading(
                    imu.get("global_heading_degrees"),
                    self.imu_reference_heading_degrees,
                )
                with self.lock:
                    if not self.active or self.engine is None or self.planner is None:
                        return
                    localization = self.engine.update_localization(points, heading)
                    if not localization.localized:
                        legacy.motor_controller.stop()
                        if self.localization_lost_since is None:
                            self.localization_lost_since = time.monotonic()
                        if time.monotonic() - self.localization_lost_since > self.LOCALIZATION_HOLD_SECONDS:
                            self._fault_locked("LOCALIZATION_LOST")
                            return
                        self.stop_event.wait(0.10)
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

                    command = self.planner.update(
                        self.engine.pose,
                        lidar.get("safety_points") or [],
                    )
                    if command.fault:
                        self._fault_locked(command.fault)
                        return
                    if command.finished:
                        legacy.record_manager.add_event("AUTO_LOCAL_COMPLETED", self.destination_id or "")
                        self.active = False
                        legacy.motor_controller.stop()
                        if self.owns_recording:
                            self.owns_recording = False
                            threading.Thread(target=legacy.record_manager.stop, daemon=True).start()
                        legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, "local_destination_reached")
                        return

                    steering_angle = command.steering_angle_degrees
                    self.last_lane = self._lane_assist()
                    if self.last_lane and self.last_lane.get("detected"):
                        confidence = float(self.last_lane.get("confidence") or 0.0)
                        if confidence >= legacy.AUTO_LANE_MIN_CONFIDENCE:
                            steering_angle += confidence * float(
                                self.last_lane.get("correction_angle_degrees") or 0.0
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
                    decision = legacy.safety_supervisor.evaluate(
                        request,
                        legacy.safety_context(loop_delay),
                    )
                    self.last_command = {
                        **command.as_dict(),
                        "steering_angle_degrees": steering_angle,
                        "calibrated_throttle": calibrated_throttle,
                        "localization": localization.as_dict(),
                        "safety": decision.as_dict(),
                    }
                    if not decision.allowed:
                        legacy.motor_controller.set_drive(0.0, True)
                        legacy.motor_controller.stop_steering()
                        if decision.stop_reason not in {
                            "CAMERA_OBJECT_STOP",
                            "OBSTACLE_STOP",
                            "OBSTACLE_RESTART_DELAY",
                        }:
                            self._fault_locked(decision.stop_reason or "AUTO_LOCAL_SAFETY_STOP")
                            return
                    else:
                        legacy.motor_controller.set_drive(0.0, True)
                        steering_result = legacy.motor_controller.set_steering(decision.final_steering)
                        if steering_result.get("steering_rejection"):
                            self._fault_locked("STEERING_COMMAND_REJECTED")
                            return
                        legacy.motor_controller.set_drive(decision.final_throttle, True)
            except Exception as error:
                with self.lock:
                    self._fault_locked(f"AUTO_LOCAL_RUNTIME_ERROR:{type(error).__name__}:{error}")
                return
            elapsed = time.monotonic() - started
            self.stop_event.wait(max(0.0, 0.10 - elapsed))

    def _lane_assist(self):
        if not legacy.auto_route_runtime.lane_controller.available:
            return None
        frame, _, frame_monotonic, _ = legacy.camera.snapshot_frame()
        if frame is None or frame_monotonic is None or time.monotonic() - frame_monotonic > 0.30:
            return None
        try:
            return legacy.auto_route_runtime.lane_controller.analyze_jpeg(frame).as_dict()
        except Exception as error:
            return {"detected": False, "confidence": 0.0, "error": str(error)}

    def _fault_locked(self, reason):
        self.active = False
        self.stop_event.set()
        self.error = reason
        legacy.motor_controller.stop()
        if self.owns_recording:
            self.owns_recording = False
            threading.Thread(target=legacy.record_manager.stop, daemon=True).start()
        if legacy.vehicle_state_machine.mode != DriveMode.FAULT:
            legacy.vehicle_state_machine.transition(DriveMode.FAULT, reason)


MAPPING_CONTROLLER = MappingController()
AUTO_LOCAL_CONTROLLER = AutoLocalController()


class AutoOrchestrator:
    def __init__(self):
        self.lock = threading.RLock()
        self.active = False
        self.strategy = None
        self.reason = None
        self.resource_id = None
        self.last_attempts = []
        self.started_at = None

    def start(self):
        with self.lock:
            self.active = True
            self.strategy = None
            self.reason = None
            self.resource_id = None
            self.last_attempts = []
            self.started_at = time.time()

        # 1) GPS/RTK route when it is truly ready.
        preflight = v2._route_preflight()
        self.last_attempts.append({"mode": "AUTO_GPS", "ready": preflight.get("ready"), "details": preflight})
        if preflight.get("ready"):
            result = ai.enhanced_select_mode("AUTO_GPS")
            if result.get("accepted"):
                return self._selected("AUTO_GPS", "gps_preflight_ready", None, result)

        # 2) Saved LOCAL map and successful localization.
        local_selection = _selected_local()
        if local_selection.get("map_id") and local_selection.get("destination_id"):
            check = AUTO_LOCAL_CONTROLLER.preflight()
            self.last_attempts.append({"mode": "AUTO_LOCAL", **check})
            if check.get("ready"):
                result = AUTO_LOCAL_CONTROLLER.start()
                return self._selected(
                    "AUTO_LOCAL",
                    "saved_map_localized",
                    local_selection.get("map_id"),
                    result,
                )

        # 3) Environment-compatible AUTO_ALLOWED learned model.
        tags = _auto_config().get("environment_tags") or []
        compatible = ai.MODEL_REGISTRY.compatible_for_auto(tags)
        self.last_attempts.append(
            {
                "mode": "AUTO_AI",
                "environment_tags": tags,
                "compatible_models": [model.get("model_id") for model in compatible],
            }
        )
        if compatible:
            selected_id = ai.ai_status().get("selected_model_id")
            model = next(
                (item for item in compatible if item.get("model_id") == selected_id),
                compatible[0],
            )
            ai.select_ai_model(model["model_id"])
            result = ai.AUTO_AI_CONTROLLER.start(model["model_id"])
            return self._selected(
                "AUTO_AI",
                "auto_allowed_ai_environment_match",
                model["model_id"],
                result,
            )

        self.active = False
        self.reason = "no_safe_autonomous_strategy"
        legacy.motor_controller.stop()
        raise ValueError("AUTO found no ready GPS, LOCAL, or AUTO_ALLOWED AI strategy")

    def stop(self):
        with self.lock:
            self.active = False
            self.strategy = None
            self.reason = "operator_stop"
            self.resource_id = None

    def snapshot(self):
        with self.lock:
            return {
                "active": self.active,
                "strategy": self.strategy,
                "reason": self.reason,
                "resource_id": self.resource_id,
                "started_at": self.started_at,
                "last_attempts": list(self.last_attempts),
            }

    def _selected(self, strategy, reason, resource_id, result):
        with self.lock:
            self.strategy = strategy
            self.reason = reason
            self.resource_id = resource_id
            return {
                "accepted": True,
                "target": "AUTO",
                "selected_strategy": strategy,
                "reason": reason,
                "resource_id": resource_id,
                "runtime": result,
            }


AUTO_ORCHESTRATOR = AutoOrchestrator()


def _current_pose():
    pose = AUTO_LOCAL_CONTROLLER.current_pose()
    if pose is not None:
        return pose
    return MAPPING_CONTROLLER.current_pose()


def add_destination(payload):
    map_id = payload.get("map_id") or _selected_local().get("map_id") or MAPPING_CONTROLLER.map_id
    if not map_id:
        raise ValueError("Map ID is required")
    if payload.get("use_current_pose", False):
        pose = _current_pose()
        if pose is None:
            raise ValueError("No current LOCAL/mapping pose is available")
        x, y = pose.x, pose.y
        heading = pose.heading_degrees
    else:
        x = payload.get("x")
        y = payload.get("y")
        heading = payload.get("heading_degrees")
    return MAP_STORE.upsert_destination(
        map_id,
        payload.get("destination_id") or payload.get("name"),
        payload.get("name") or payload.get("destination_id"),
        x,
        y,
        heading,
    )


def full_status():
    status = ai.enhanced_status()
    selected = _selected_local()
    maps = MAP_STORE.list_maps()
    selected_map = None
    if selected.get("map_id"):
        try:
            selected_map = MAP_STORE.get_map(selected["map_id"])
        except MapStoreError:
            selected_map = None
    local_configured = bool(
        selected_map
        and selected.get("destination_id")
        and selected_map.get("map_file")
        and any(
            item.get("destination_id") == selected.get("destination_id")
            for item in selected_map.get("destinations") or []
        )
    )
    status["local"] = {
        "maps": maps,
        "selected": selected,
        "configured": local_configured,
        "mapping": MAPPING_CONTROLLER.snapshot(),
        "controller": AUTO_LOCAL_CONTROLLER.snapshot(),
        "current_pose": None if _current_pose() is None else _current_pose().as_dict(),
    }
    status["auto"] = {
        "config": _auto_config(),
        "selector": AUTO_ORCHESTRATOR.snapshot(),
    }
    status["capabilities"]["AUTO_LOCAL"] = {
        "implemented": True,
        "ready": local_configured,
        "reason": "selected_map_destination_configured" if local_configured else "select map and destination",
    }
    compatible_ai = ai.MODEL_REGISTRY.compatible_for_auto(
        _auto_config().get("environment_tags") or []
    )
    gps_ready = bool((status.get("gps") or {}).get("preflight_ready"))
    status["capabilities"]["AUTO"] = {
        "implemented": True,
        "ready": gps_ready or local_configured or bool(compatible_ai),
        "reason": "strategy_available" if gps_ready or local_configured or compatible_ai else "no strategy configured",
    }
    return status


def full_select_mode(mode_name, record_gps=True):
    target = DriveMode(str(mode_name).strip().upper())
    if target == DriveMode.AUTO:
        if MAPPING_CONTROLLER.active:
            raise ValueError("Stop mapping before AUTO")
        if AUTO_LOCAL_CONTROLLER.active:
            AUTO_LOCAL_CONTROLLER.stop("auto_reselect")
        if ai.AUTO_AI_CONTROLLER.active:
            ai.AUTO_AI_CONTROLLER.stop("auto_reselect")
        if legacy.auto_route_runtime.active:
            legacy.auto_route_runtime.stop("auto_reselect")
        return AUTO_ORCHESTRATOR.start()

    AUTO_ORCHESTRATOR.stop()
    if target == DriveMode.AUTO_LOCAL:
        if ai.AUTO_AI_CONTROLLER.active:
            ai.AUTO_AI_CONTROLLER.stop("auto_local_selected")
        if legacy.auto_route_runtime.active:
            legacy.auto_route_runtime.stop("auto_local_selected")
        return {
            "accepted": True,
            "target": target.value,
            "local": AUTO_LOCAL_CONTROLLER.start(),
            "status": full_status(),
        }

    if AUTO_LOCAL_CONTROLLER.active:
        AUTO_LOCAL_CONTROLLER.stop("v2_mode_change")
    return ai.enhanced_select_mode(target.value, record_gps=record_gps)


FULL_HTML = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GNSS Autonomy V2</title><style>
:root{color-scheme:dark;--bg:#0b0e0c;--p:#141915;--l:#303932;--t:#eef3ee;--m:#919d93;--a:#b8d89a;--w:#e2c27d;--b:#ff8493}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--t);font-family:system-ui,sans-serif}header{padding:14px 18px;border-bottom:1px solid var(--l);display:flex;justify-content:space-between;position:sticky;top:0;background:#090c0a;z-index:5}h1{font-size:15px;margin:0}a{color:var(--a)}main{max-width:1300px;margin:auto;padding:16px}.grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}.panel{background:var(--p);border:1px solid var(--l);border-radius:12px;padding:13px;margin-bottom:12px}.mode{min-height:145px;display:flex;flex-direction:column}.mode p{color:var(--m);font-size:12px;flex:1}.mode.active{border-color:var(--a)}button,input,select{background:#202820;color:var(--t);border:1px solid #465248;border-radius:8px;padding:9px}button{cursor:pointer;font-weight:700}button.primary{border-color:#67845d}button.danger{border-color:#8a3f4a;color:#ffb0b9}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}.row>*{min-width:120px}.status{white-space:pre-wrap;font:11px ui-monospace,monospace;color:var(--m);max-height:300px;overflow:auto}.resources{display:grid;grid-template-columns:1fr 1fr;gap:12px}label{font-size:11px;color:var(--m)}table{width:100%;border-collapse:collapse;font-size:11px}td,th{border-bottom:1px solid var(--l);padding:6px;text-align:left}@media(max-width:850px){.grid,.resources{grid-template-columns:1fr}}
</style></head><body><header><div><h1>GNSS AUTONOMY V2 · FULL</h1><small>MANUAL · RECORD · AUTO_AI · AUTO_GPS · AUTO_LOCAL · AUTO</small></div><a href="/legacy">기존 상세 대시보드</a></header><main>
<section class="grid" id="modes">
<div class="panel mode" data-card="MANUAL"><b>1. MANUAL</b><p>사람이 100% 운전. 자동 차선/장애물/사람 개입 OFF. 하드 Safety만 유지.</p><button data-mode="MANUAL" class="primary">MANUAL</button></div>
<div class="panel mode" data-card="RECORD"><b>2. RECORD</b><p>완전 수동 + AI 학습 데이터 기록.</p><label><input type="checkbox" id="record-gps" checked> GPS/RTK 기록</label><button data-mode="RECORD" class="primary">RECORD</button></div>
<div class="panel mode" data-card="AUTO_AI"><b>3. AUTO_AI</b><p>학습 모델이 주행/일반 장애물 판단. 사람만 외부 STOP.</p><button data-mode="AUTO_AI">AUTO_AI</button></div>
<div class="panel mode" data-card="AUTO_GPS"><b>4. AUTO_GPS</b><p>RTK 경로 + 자동 Lane Assist + LiDAR 우회/복귀.</p><button data-mode="AUTO_GPS">AUTO_GPS</button></div>
<div class="panel mode" data-card="AUTO_LOCAL"><b>5. AUTO_LOCAL</b><p>저장 SLAM 지도 + 목적지 + A* + 회피 + Lane Assist.</p><button data-mode="AUTO_LOCAL">AUTO_LOCAL</button></div>
<div class="panel mode" data-card="AUTO"><b>6. AUTO</b><p>GPS → LOCAL → AUTO_ALLOWED AI 순으로 안전한 전략 자동 선택.</p><button data-mode="AUTO" class="primary">AUTO</button></div>
</section>
<div class="row panel"><button id="disarm">STOP / DISARM</button><button id="estop" class="danger">EMERGENCY STOP</button><button id="reset">SAFETY RESET</button><button id="refresh">REFRESH</button></div>
<section class="resources">
<div class="panel"><h3>LOCAL MAP</h3><div class="row"><input id="map-name" placeholder="새 지도 이름"><button id="map-create">지도 생성</button></div><div class="row"><select id="map-select"></select><select id="dest-select"></select><button id="local-select">선택 저장</button></div><div class="row"><button id="map-start">새 Mapping 시작</button><button id="map-refine">선택 지도 정밀화</button><button id="map-stop">Mapping 저장/종료</button></div><div class="row"><input id="dest-name" placeholder="현재 위치 목적지 이름"><button id="dest-current">현재 위치 저장</button></div><div id="map-status" class="status"></div></div>
<div class="panel"><h3>AI MODEL / AUTO</h3><div class="row"><select id="model-select"></select><button id="model-use">AI 모델 선택</button></div><div class="row"><select id="stage"><option>TRAINED</option><option>OFFLINE_VALIDATED</option><option>CLOSED_AREA_VALIDATED</option><option>AUTO_ALLOWED</option></select><button id="stage-set">Lifecycle 변경</button></div><div class="row"><input id="env-tags" placeholder="indoor,warehouse"><button id="env-save">AUTO 환경 태그 저장</button></div><div id="ai-status" class="status"></div></div>
</section>
<div class="panel"><h3>STATUS</h3><div id="status" class="status">loading...</div></div>
<script>
let S=null;const $=id=>document.getElementById(id);async function api(p,o={}){const r=await fetch(p,{cache:'no-store',...o});let d={};try{d=await r.json()}catch{}if(!r.ok)throw new Error(d.error||`HTTP ${r.status}`);return d}function post(p,b={}){return api(p,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})}
function fillSelect(el,items,valueKey,labelFn,selected){el.innerHTML='';for(const it of items){const o=document.createElement('option');o.value=it[valueKey];o.textContent=labelFn(it);if(o.value===selected)o.selected=true;el.appendChild(o)}}
function render(s){S=s;document.querySelectorAll('[data-card]').forEach(e=>e.classList.toggle('active',e.dataset.card===s.state.canonical_mode));const maps=s.local.maps||[];fillSelect($('map-select'),maps,'map_id',m=>`${m.name} (${m.map_id})`,s.local.selected.map_id);const map=maps.find(m=>m.map_id===$('map-select').value)||maps.find(m=>m.map_id===s.local.selected.map_id);fillSelect($('dest-select'),map?.destinations||[],'destination_id',d=>d.name,s.local.selected.destination_id);const models=s.ai?.models||[];fillSelect($('model-select'),models,'model_id',m=>`${m.model_id} · ${m.validation_stage}`,s.ai?.selected_model_id);$('env-tags').value=(s.auto.config.environment_tags||[]).join(',');$('map-status').textContent=JSON.stringify({selected:s.local.selected,mapping:s.local.mapping,controller:s.local.controller,current_pose:s.local.current_pose},null,2);$('ai-status').textContent=JSON.stringify({selected:s.ai?.selected_model,controller:s.ai?.controller,auto:s.auto.selector},null,2);$('status').textContent=JSON.stringify(s,null,2)}
async function refresh(){try{render(await api('/api/v2/status'))}catch(e){$('status').textContent=e.message}}
document.querySelectorAll('[data-mode]').forEach(b=>b.onclick=async()=>{try{await post('/api/v2/mode',{mode:b.dataset.mode,record_gps:$('record-gps').checked});await refresh()}catch(e){alert(e.message)}});$('disarm').onclick=()=>post('/api/v2/mode',{mode:'DISARMED'}).then(refresh).catch(e=>alert(e.message));$('estop').onclick=()=>{if(confirm('EMERGENCY STOP?'))post('/api/safety/emergency-stop',{}).then(refresh)};$('reset').onclick=()=>post('/api/safety/reset',{}).then(refresh);$('refresh').onclick=refresh;
$('map-select').onchange=()=>{const map=(S?.local.maps||[]).find(m=>m.map_id===$('map-select').value);fillSelect($('dest-select'),map?.destinations||[],'destination_id',d=>d.name,null)};$('map-create').onclick=async()=>{try{await post('/api/v2/maps/create',{name:$('map-name').value});await refresh()}catch(e){alert(e.message)}};$('local-select').onclick=async()=>{try{await post('/api/v2/maps/select',{map_id:$('map-select').value,destination_id:$('dest-select').value});await refresh()}catch(e){alert(e.message)}};$('map-start').onclick=async()=>{try{let id=$('map-select').value||null;await post('/api/v2/maps/mapping/start',{map_id:id,name:$('map-name').value,refine:false});await refresh()}catch(e){alert(e.message)}};$('map-refine').onclick=async()=>{try{await post('/api/v2/maps/mapping/start',{map_id:$('map-select').value,refine:true});await refresh()}catch(e){alert(e.message)}};$('map-stop').onclick=()=>post('/api/v2/maps/mapping/stop',{save:true}).then(refresh).catch(e=>alert(e.message));$('dest-current').onclick=async()=>{try{await post('/api/v2/maps/destination',{map_id:$('map-select').value,name:$('dest-name').value,use_current_pose:true});await refresh()}catch(e){alert(e.message)}};
$('model-use').onclick=()=>post('/api/v2/ai/select',{model_id:$('model-select').value}).then(refresh).catch(e=>alert(e.message));$('stage-set').onclick=async()=>{const stage=$('stage').value;if(!confirm(`모델 lifecycle을 ${stage}(으)로 변경? 실차 검증 상태와 일치해야 합니다.`))return;try{await post('/api/v2/ai/lifecycle',{model_id:$('model-select').value,stage,confirm:stage});await refresh()}catch(e){alert(e.message)}};$('env-save').onclick=()=>post('/api/v2/auto/environment',{tags:$('env-tags').value}).then(refresh).catch(e=>alert(e.message));refresh();setInterval(refresh,1500);
</script></main></body></html>""".encode("utf-8")


class FullHandler(ai.V2AiHandler):
    def do_GET(self):
        if self.path == "/":
            self._send_html(FULL_HTML)
            return
        if self.path == "/api/v2/status":
            status = full_status()
            # Surface models at the top-level AI object for the full dashboard.
            status["ai"]["models"] = ai.MODEL_REGISTRY.list_models()
            self._send_json(status)
            return
        if self.path == "/api/v2/maps":
            self._send_json({"maps": MAP_STORE.list_maps(), "selected": _selected_local()})
            return
        super().do_GET()

    def do_POST(self):
        try:
            if self.path == "/api/v2/maps/create":
                payload = self._read_json()
                self._send_json(MAP_STORE.create_map(payload.get("name"), payload.get("map_id")), 202)
                return
            if self.path == "/api/v2/maps/delete":
                payload = self._read_json()
                if MAPPING_CONTROLLER.active or AUTO_LOCAL_CONTROLLER.active:
                    raise ValueError("Stop LOCAL runtime before deleting maps")
                self._send_json(MAP_STORE.delete_map(payload.get("map_id")), 202)
                return
            if self.path == "/api/v2/maps/select":
                payload = self._read_json()
                self._send_json(select_local_target(payload.get("map_id"), payload.get("destination_id")), 202)
                return
            if self.path == "/api/v2/maps/mapping/start":
                payload = self._read_json()
                self._send_json(
                    MAPPING_CONTROLLER.start(
                        payload.get("map_id"),
                        payload.get("name"),
                        payload.get("refine", False),
                    ),
                    202,
                )
                return
            if self.path == "/api/v2/maps/mapping/stop":
                payload = self._read_json()
                self._send_json(MAPPING_CONTROLLER.stop(payload.get("save", True)), 202)
                return
            if self.path == "/api/v2/maps/destination":
                payload = self._read_json()
                self._send_json(add_destination(payload), 202)
                return
            if self.path == "/api/v2/maps/destination/delete":
                payload = self._read_json()
                self._send_json(
                    MAP_STORE.remove_destination(payload.get("map_id"), payload.get("destination_id")),
                    202,
                )
                return
            if self.path == "/api/v2/auto/environment":
                payload = self._read_json()
                self._send_json(set_environment_tags(payload.get("tags")), 202)
                return
            if self.path == "/api/v2/ai/lifecycle":
                payload = self._read_json()
                stage = str(payload.get("stage") or "").upper()
                if stage not in MODEL_LIFECYCLE:
                    raise ValueError(f"Unknown lifecycle stage: {stage}")
                if payload.get("confirm") != stage:
                    raise ValueError("Lifecycle change requires exact confirmation")
                if ai.AUTO_AI_CONTROLLER.active:
                    raise ValueError("Stop AUTO_AI before changing model lifecycle")
                self._send_json(
                    ai.MODEL_REGISTRY.update_lifecycle(payload.get("model_id"), stage),
                    202,
                )
                return
            if self.path == "/api/v2/mode":
                payload = self._read_json()
                result = full_select_mode(
                    payload.get("mode"),
                    record_gps=payload.get("record_gps", True),
                )
                self._send_json(result, 202 if result.get("accepted", False) else 409)
                return
            if self.path == "/api/safety/emergency-stop":
                AUTO_ORCHESTRATOR.stop()
                if MAPPING_CONTROLLER.active:
                    MAPPING_CONTROLLER.stop(save=False)
                if AUTO_LOCAL_CONTROLLER.active:
                    AUTO_LOCAL_CONTROLLER.stop("emergency_stop")
        except (ValueError, OSError, TypeError, MapStoreError, ModelRegistryError, json.JSONDecodeError) as error:
            self._send_json({"error": str(error), "status": full_status()}, 409)
            return
        super().do_POST()


def main():
    legacy.camera.start()
    legacy.gps_monitor.start()
    legacy.ntrip_client.start()
    legacy.imu_monitor.start()
    legacy.lidar_monitor.start()
    legacy.motor_controller.start()
    legacy.perception_monitor.start()
    httpd = legacy.ThreadingHTTPServer((legacy.HOST, legacy.PORT), FullHandler)
    print(
        f"GNSS Autonomy V2 FULL listening on http://{legacy.HOST}:{legacy.PORT} "
        f"(legacy dashboard: /legacy)",
        flush=True,
    )
    httpd.serve_forever()


if __name__ == "__main__":
    main()
