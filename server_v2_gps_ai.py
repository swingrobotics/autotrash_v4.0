#!/usr/bin/env python3
"""GPS-conditioned learned-driving integration for the final V2 server."""

import importlib.util
import json
import os
import threading
import time

from autonomous_car import ControlRequest, DriveMode
from autonomous_car.ai import GpsAiRuntime, ModelRegistryError
from autonomous_car.modes import AutoRoutePlanner
from autonomous_car.routes import GpsRouteFeatureExtractor, GpsRouteNormalizer, NormalizedGpsRoute


class GpsAiController:
    CAMERA_TIMEOUT_SECONDS = 0.30
    GNSS_TIMEOUT_SECONDS = 0.30
    IMU_TIMEOUT_SECONDS = 0.20
    MAXIMUM_ROUTE_DEVIATION_M = 3.0
    DESTINATION_HOLD_DISTANCE_M = 0.35

    def __init__(self, integration):
        self.integration = integration
        self.full = integration.full
        self.legacy = integration.legacy
        self.lock = threading.RLock()
        self.active = False
        self.route_id = None
        self.model_id = None
        self.route = None
        self.extractor = None
        self.runtime = None
        self.last_route_features = None
        self.last_inference = None
        self.error = None
        self.started_at = None
        self.previous_index = None
        self.owns_recording = False
        self._generation = 0
        self.stop_event = threading.Event()
        self.thread = None

    def preflight(self, auto_only=False):
        try:
            selected = self.integration.selected()
            route_id = selected.get("route_id")
            model_id = selected.get("model_id")
            if not route_id or not model_id:
                return {
                    "route_loaded": bool(route_id),
                    "ready": False,
                    "details": {"error": "GPS_ROUTE_AND_MODEL_REQUIRED"},
                }
            route = self.integration.load_route(route_id)
            model = self.full.ai.MODEL_REGISTRY.get(model_id)
            if str(model.get("policy_type") or "AUTO_AI") != "AUTO_GPS":
                return {
                    "route_loaded": True,
                    "ready": False,
                    "details": {"error": "SELECTED_MODEL_NOT_AUTO_GPS"},
                }
            if str(model.get("route_id") or "") != route.route_id:
                return {
                    "route_loaded": True,
                    "ready": False,
                    "details": {"error": "GPS_MODEL_ROUTE_MISMATCH"},
                }
            allowed_stages = (
                {"AUTO_ALLOWED"}
                if auto_only
                else {"CLOSED_AREA_VALIDATED", "AUTO_ALLOWED"}
            )
            if model.get("validation_stage") not in allowed_stages:
                return {
                    "route_loaded": True,
                    "ready": False,
                    "details": {
                        "error": "GPS_MODEL_VALIDATION_STAGE",
                        "stage": model.get("validation_stage"),
                    },
                }

            missing = [
                module
                for module in ("cv2", "numpy", "onnxruntime")
                if importlib.util.find_spec(module) is None
            ]
            if missing:
                return {
                    "route_loaded": True,
                    "ready": False,
                    "details": {
                        "error": "GPS_AI_RUNTIME_DEPENDENCY_MISSING",
                        "missing": missing,
                    },
                }
            try:
                self.integration.model_artifacts(model)
            except Exception as error:
                return {
                    "route_loaded": True,
                    "ready": False,
                    "details": {
                        "error": str(error) or type(error).__name__,
                    },
                }

            gps = self.legacy.gps_monitor.snapshot()
            imu = self.legacy.imu_monitor.snapshot()
            lidar = self.legacy.lidar_monitor.snapshot()
            motor = self.legacy.motor_controller.snapshot()
            result = AutoRoutePlanner(route).preflight(
                gps,
                imu,
                lidar_connected=bool(lidar.get("connected")),
                arduino_connected=bool(motor.get("connected")),
                steering_connected=bool(motor.get("encoder_connected")),
                emergency_stop_active=bool(motor.get("hardware_estop_active")),
            )
            frame, _, frame_monotonic, _ = self.legacy.camera.snapshot_frame()
            errors = list(result.errors)
            if (
                frame is None
                or frame_monotonic is None
                or time.monotonic() - frame_monotonic > self.CAMERA_TIMEOUT_SECONDS
            ):
                errors.append("CAMERA_TIMEOUT")
            return {
                "route_loaded": True,
                "ready": not errors,
                "details": {
                    "errors": errors,
                    "start_distance_m": result.start_distance_m,
                    "heading_error_degrees": result.heading_error_degrees,
                    "route_id": route_id,
                    "model_id": model_id,
                    "model_stage": model.get("validation_stage"),
                    "policy_type": "AUTO_GPS",
                    "model_artifacts_ready": True,
                },
            }
        except Exception as error:
            return {
                "route_loaded": False,
                "ready": False,
                "details": {"error": f"{type(error).__name__}: {error}"},
            }

    def start(self):
        with self.lock:
            if self.active:
                return self.snapshot()
            check = self.preflight(auto_only=False)
            if not check.get("ready"):
                raise ValueError(f"AUTO_GPS preflight failed: {check.get('details')}")
            selected = self.integration.selected()
            route = self.integration.load_route(selected["route_id"])
            model = self.full.ai.MODEL_REGISTRY.get(selected["model_id"])
            model_path, manifest_path, _ = self.integration.model_artifacts(model)
            runtime = GpsAiRuntime(model_path, manifest_path)
            if runtime.route_id and runtime.route_id != route.route_id:
                raise ValueError(
                    "ONNX manifest route does not match selected normalized route"
                )

            if self.full.AUTO_LOCAL_CONTROLLER.active:
                self.full.AUTO_LOCAL_CONTROLLER.stop("auto_gps_takeover")
            if self.full.ai.AUTO_AI_CONTROLLER.active:
                self.full.ai.AUTO_AI_CONTROLLER.stop("auto_gps_takeover")
            if self.legacy.auto_route_runtime.active:
                self.legacy.auto_route_runtime.stop("auto_gps_takeover")
            if self.legacy.record_manager.active:
                self.legacy.record_manager.stop()
            mode = self.legacy.vehicle_state_machine.mode
            if mode in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
                raise ValueError("Safety reset is required before AUTO_GPS")
            if mode == DriveMode.RECORD:
                self.legacy.vehicle_state_machine.transition(
                    DriveMode.MANUAL_ASSIST,
                    "auto_gps_prepare",
                )
                mode = self.legacy.vehicle_state_machine.mode
            if mode not in {
                DriveMode.DISARMED,
                DriveMode.MANUAL,
                DriveMode.MANUAL_ASSIST,
            }:
                self.legacy.vehicle_state_machine.transition(
                    DriveMode.DISARMED,
                    "auto_gps_prepare",
                )
            self.legacy.motor_controller.stop()
            self.legacy.vehicle_state_machine.transition(
                DriveMode.AUTO_GPS,
                "gps_conditioned_ai_started",
            )

            metadata = dict(self.legacy.recording_metadata())
            metadata.update(
                purpose="AUTO_GPS",
                record_gps=True,
                route_id=route.route_id,
                model_id=model["model_id"],
                policy_type="AUTO_GPS",
                autonomy_schema="v2",
            )
            self.legacy.record_manager.start(metadata)
            self.owns_recording = True
            self.route_id = route.route_id
            self.model_id = model["model_id"]
            self.route = route
            extractor = GpsRouteFeatureExtractor(route)
            self.extractor = extractor
            self.runtime = runtime
            self.last_route_features = None
            self.last_inference = None
            self.error = None
            self.previous_index = None
            self.started_at = time.time()
            self.active = True
            self._generation += 1
            generation = self._generation
            stop_event = threading.Event()
            self.stop_event = stop_event
            self.thread = threading.Thread(
                target=self._run,
                args=(generation, stop_event, runtime, extractor),
                daemon=True,
            )
            self.thread.start()
            return self.snapshot()

    def stop(self, reason="operator_stop"):
        stop_event = self.stop_event
        stop_event.set()
        with self.lock:
            self.active = False
            self._generation += 1
            self.legacy.motor_controller.stop()
            if self.owns_recording:
                self.owns_recording = False
                threading.Thread(
                    target=self.legacy.record_manager.stop,
                    daemon=True,
                ).start()
            if self.legacy.vehicle_state_machine.mode == DriveMode.AUTO_GPS:
                self.legacy.vehicle_state_machine.transition(
                    DriveMode.MANUAL_ASSIST,
                    reason,
                )
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {
                "active": self.active,
                "route_id": self.route_id,
                "model_id": self.model_id,
                "runtime": None if self.runtime is None else self.runtime.snapshot(),
                "last_route_features": self.last_route_features,
                "last_inference": self.last_inference,
                "error": self.error,
                "started_at": self.started_at,
                "generation": self._generation,
            }

    def _run_is_current_locked(self, generation, stop_event, runtime, extractor):
        return (
            self.active
            and self._generation == generation
            and self.stop_event is stop_event
            and not stop_event.is_set()
            and self.runtime is runtime
            and self.extractor is extractor
        )

    def _run(self, generation, stop_event, runtime, extractor):
        previous_loop = None
        previous_index = None
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
                with self.lock:
                    if not self._run_is_current_locked(
                        generation,
                        stop_event,
                        runtime,
                        extractor,
                    ):
                        return

                sensor_started = time.perf_counter()
                frame, _, frame_monotonic, _ = self.legacy.camera.snapshot_frame()
                if (
                    frame is None
                    or frame_monotonic is None
                    or time.monotonic() - frame_monotonic > self.CAMERA_TIMEOUT_SECONDS
                ):
                    self._fault(generation, stop_event, "CAMERA_TIMEOUT")
                    return
                gps = self.legacy.gps_monitor.snapshot()
                imu = self.legacy.imu_monitor.snapshot()
                lidar = self.legacy.lidar_monitor.snapshot()
                perception = self.legacy.perception_monitor.snapshot()
                sensor_seconds = time.perf_counter() - sensor_started

                if gps.get("fix") != "RTK FIXED":
                    self._fault(generation, stop_event, "RTK_FIX_LOST")
                    return
                if (
                    gps.get("received_at") is None
                    or time.time() - float(gps["received_at"])
                    > self.GNSS_TIMEOUT_SECONDS
                ):
                    self._fault(generation, stop_event, "GNSS_TIMEOUT")
                    return
                if (
                    imu.get("last_update") is None
                    or time.time() - float(imu["last_update"])
                    > self.IMU_TIMEOUT_SECONDS
                ):
                    self._fault(generation, stop_event, "IMU_TIMEOUT")
                    return
                if (
                    gps.get("latitude") is None
                    or gps.get("longitude") is None
                    or imu.get("global_heading_degrees") is None
                ):
                    self._fault(generation, stop_event, "GPS_AI_POSE_UNAVAILABLE")
                    return
                if stop_event.is_set():
                    return

                route_started = time.perf_counter()
                route_features = extractor.extract(
                    gps["latitude"],
                    gps["longitude"],
                    imu["global_heading_degrees"],
                    previous_index,
                )
                route_seconds = time.perf_counter() - route_started
                previous_index = route_features.nearest_index
                route_dict = route_features.as_dict()

                if abs(route_features.cross_track_error_m) > self.MAXIMUM_ROUTE_DEVIATION_M:
                    self._fault(generation, stop_event, "GPS_ROUTE_DEVIATION")
                    return
                if (
                    route_features.remaining_distance_m
                    <= self.DESTINATION_HOLD_DISTANCE_M
                    and route_features.route_progress >= 0.99
                ):
                    stop_event.set()
                    with self.lock:
                        if not self._run_is_current_locked(
                            generation,
                            stop_event,
                            runtime,
                            extractor,
                        ):
                            # stop_event is set intentionally for completion;
                            # validate identity/generation without the flag.
                            if (
                                self._generation != generation
                                or self.stop_event is not stop_event
                                or self.runtime is not runtime
                            ):
                                return
                        self.active = False
                        self._generation += 1
                        self.previous_index = previous_index
                        self.last_route_features = route_dict
                        self.legacy.motor_controller.stop()
                        self.legacy.record_manager.add_event(
                            "AUTO_GPS_COMPLETED",
                            self.route_id or "",
                        )
                        if self.owns_recording:
                            self.owns_recording = False
                            threading.Thread(
                                target=self.legacy.record_manager.stop,
                                daemon=True,
                            ).start()
                        self.legacy.vehicle_state_machine.transition(
                            DriveMode.MANUAL_ASSIST,
                            "gps_route_completed",
                        )
                    return

                inference_started = time.perf_counter()
                inference = runtime.infer_jpeg(
                    frame,
                    lidar.get("safety_points") or [],
                    imu.get("yaw_rate_dps"),
                    route_features,
                    person_hazard=bool(perception.get("hazard")),
                )
                inference_call_seconds = time.perf_counter() - inference_started
                if stop_event.is_set():
                    return

                request = ControlRequest(
                    throttle=inference.throttle,
                    steering=self.legacy.normalized_steering_for_angle(
                        inference.steering_degrees
                    ),
                    enabled=True,
                    source="auto_gps_ai",
                )
                safety_started = time.perf_counter()
                decision = self.legacy.safety_supervisor.evaluate(
                    request,
                    self.legacy.safety_context(loop_delay),
                )
                safety_seconds = time.perf_counter() - safety_started
                inference_data = {
                    **inference.as_dict(),
                    "route": route_dict,
                    "safety": decision.as_dict(),
                    "timing": {
                        "sensor_snapshot_seconds": sensor_seconds,
                        "route_feature_seconds": route_seconds,
                        "inference_call_seconds": inference_call_seconds,
                        "onnx_seconds": inference.inference_seconds,
                        "preprocess_feature_seconds": max(
                            0.0,
                            inference_call_seconds - inference.inference_seconds,
                        ),
                        "safety_seconds": safety_seconds,
                        "loop_period_seconds": loop_delay,
                    },
                }

                with self.lock:
                    if not self._run_is_current_locked(
                        generation,
                        stop_event,
                        runtime,
                        extractor,
                    ):
                        return
                    self.previous_index = previous_index
                    self.last_route_features = route_dict
                    actuation_started = time.perf_counter()
                    drive_result = None
                    steering_result = None
                    if not decision.allowed:
                        drive_result = self.legacy.motor_controller.set_drive(0.0, True)
                        steering_result = self.legacy.motor_controller.stop_steering()
                        inference_data["actuation"] = self._actuation_snapshot(
                            decision,
                            drive_result,
                            steering_result,
                        )
                        if decision.stop_reason != "CAMERA_OBJECT_STOP":
                            self.last_inference = inference_data
                            self._fault_locked(
                                decision.stop_reason or "AUTO_GPS_SAFETY_STOP"
                            )
                            return
                    else:
                        self.legacy.motor_controller.set_drive(0.0, True)
                        steering_result = self.legacy.motor_controller.set_steering(
                            decision.final_steering
                        )
                        if steering_result.get("steering_rejection"):
                            inference_data["actuation"] = self._actuation_snapshot(
                                decision,
                                None,
                                steering_result,
                            )
                            self.last_inference = inference_data
                            self._fault_locked("STEERING_COMMAND_REJECTED")
                            return
                        drive_result = self.legacy.motor_controller.set_drive(
                            decision.final_throttle,
                            True,
                        )
                        inference_data["actuation"] = self._actuation_snapshot(
                            decision,
                            drive_result,
                            steering_result,
                        )
                    inference_data["timing"]["actuation_seconds"] = (
                        time.perf_counter() - actuation_started
                    )
                    inference_data["timing"]["control_loop_seconds"] = (
                        time.perf_counter() - loop_started
                    )
                    self.last_inference = inference_data
            except Exception as error:
                self._fault(
                    generation,
                    stop_event,
                    f"AUTO_GPS_RUNTIME_ERROR:{type(error).__name__}:{error}",
                )
                return
            elapsed = time.perf_counter() - loop_started
            stop_event.wait(max(0.0, 0.10 - elapsed))

    @staticmethod
    def _actuation_snapshot(decision, drive_result, steering_result):
        drive_result = drive_result or {}
        steering_result = steering_result or {}
        applied_throttle = drive_result.get("throttle")
        if applied_throttle is None:
            applied_throttle = drive_result.get("drive_throttle")
        drive_pwm = drive_result.get("drive_pwm")
        if drive_pwm is None:
            drive_pwm = drive_result.get("pwm")
        return {
            "safety_final_throttle": decision.final_throttle,
            "safety_final_steering": decision.final_steering,
            "applied_throttle": applied_throttle,
            "drive_pwm": drive_pwm,
            "drive_enabled": drive_result.get("enabled"),
            "steering_rejection": steering_result.get("steering_rejection"),
        }

    def _fault(self, generation, stop_event, reason):
        stop_event.set()
        with self.lock:
            if generation != self._generation:
                return
            self._fault_locked(reason)

    def _fault_locked(self, reason):
        self.active = False
        self.stop_event.set()
        self._generation += 1
        self.error = reason
        self.legacy.motor_controller.stop()
        if self.owns_recording:
            self.owns_recording = False
            threading.Thread(
                target=self.legacy.record_manager.stop,
                daemon=True,
            ).start()
        if self.legacy.vehicle_state_machine.mode != DriveMode.FAULT:
            self.legacy.vehicle_state_machine.transition(DriveMode.FAULT, reason)


class GpsAiIntegration:
    def __init__(self, release):
        self.release = release
        self.full = release.full
        self.legacy = self.full.legacy
        self.project_root = os.path.dirname(os.path.abspath(__file__))
        self.routes_root = os.environ.get(
            "AUTONOMY_GPS_ROUTES_PATH",
            os.path.join(self.project_root, "gps-routes"),
        )
        self.selection_path = os.path.join(self.routes_root, "selected.json")
        self.controller = GpsAiController(self)
        self._original_full_select = self.full.full_select_mode
        self._original_full_status = self.full.full_status

    def route_path(self, route_id):
        route_id = str(route_id or "").strip()
        if not route_id or os.path.basename(route_id) != route_id:
            raise ValueError("Invalid GPS route ID")
        root = os.path.abspath(self.routes_root)
        path = os.path.abspath(os.path.join(root, f"{route_id}.json"))
        if os.path.commonpath([root, path]) != root:
            raise ValueError("GPS route path escapes route store")
        return path

    def load_route(self, route_id):
        return NormalizedGpsRoute.load(self.route_path(route_id))

    def list_routes(self):
        if not os.path.isdir(self.routes_root):
            return []
        result = []
        for name in sorted(os.listdir(self.routes_root)):
            if not name.endswith(".json") or name == "selected.json":
                continue
            try:
                route = NormalizedGpsRoute.load(os.path.join(self.routes_root, name))
            except Exception:
                continue
            result.append(
                {
                    "route_id": route.route_id,
                    "source_sessions": route.source_sessions,
                    "quality": route.quality,
                    "point_count": len(route.points),
                    "created_at": route.created_at,
                }
            )
        return result

    def build_route(self, sessions, route_id):
        os.makedirs(self.routes_root, exist_ok=True)
        path = self.route_path(route_id)
        if os.path.exists(path):
            raise FileExistsError(f"GPS route already exists: {route_id}")
        route = GpsRouteNormalizer().build(
            self.legacy.RECORDINGS_PATH,
            sessions,
            route_id,
            output_path=path,
        )
        return route.as_dict()

    def model_artifacts(self, model):
        model_path = self.full.ai._safe_model_path(model.get("model_file"))
        manifest_file = model.get("manifest_file")
        if not os.path.isfile(model_path):
            raise FileNotFoundError(model_path)
        if not manifest_file:
            raise ValueError("AUTO_GPS_MODEL_MANIFEST_MISSING")
        manifest_path = self.full.ai._safe_model_path(manifest_file)
        if not os.path.isfile(manifest_path):
            raise FileNotFoundError(manifest_path)
        try:
            with open(manifest_path, "r", encoding="utf-8") as file:
                manifest = json.load(file)
        except json.JSONDecodeError as error:
            raise ValueError(f"AUTO_GPS_MANIFEST_INVALID: {error}") from error
        if manifest.get("policy_type") != "AUTO_GPS":
            raise ValueError("AUTO_GPS_MANIFEST_POLICY_MISMATCH")
        export = manifest.get("export") or {}
        if export.get("external_data") is not False or export.get("self_contained") is not True:
            raise ValueError("AUTO_GPS_MODEL_NOT_SELF_CONTAINED")
        return model_path, manifest_path, manifest

    def selected(self):
        try:
            with open(self.selection_path, "r", encoding="utf-8") as file:
                document = json.load(file)
            return {
                "route_id": str(document.get("route_id") or "").strip() or None,
                "model_id": str(document.get("model_id") or "").strip() or None,
            }
        except (OSError, ValueError, json.JSONDecodeError):
            return {"route_id": None, "model_id": None}

    def select(self, route_id, model_id):
        if self.controller.active:
            raise ValueError("Stop AUTO_GPS before changing route/model")
        route = self.load_route(route_id)
        model = self.full.ai.MODEL_REGISTRY.get(model_id)
        if str(model.get("policy_type") or "AUTO_AI") != "AUTO_GPS":
            raise ValueError("Selected model is not AUTO_GPS")
        if str(model.get("route_id") or "") != route.route_id:
            raise ValueError(
                "GPS AI model was trained for a different normalized route"
            )
        self.model_artifacts(model)
        os.makedirs(self.routes_root, exist_ok=True)
        temporary = self.selection_path + ".tmp"
        document = {
            "route_id": route.route_id,
            "model_id": model["model_id"],
            "updated_at": time.time(),
        }
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(document, file, indent=2)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, self.selection_path)
        return document

    def status(self):
        selected = self.selected()
        models = self.full.ai.MODEL_REGISTRY.list_models("AUTO_GPS")
        return {
            "selected": selected,
            "routes": self.list_routes(),
            "models": models,
            "manual_preflight": self.controller.preflight(auto_only=False),
            "auto_preflight": self.controller.preflight(auto_only=True),
            "controller": self.controller.snapshot(),
        }

    @staticmethod
    def _preflight_reason(preflight):
        if preflight.get("ready"):
            return "gps_conditioned_ai_ready"
        details = preflight.get("details") or {}
        error = str(details.get("error") or "").strip()
        if error:
            return error
        errors = details.get("errors") or []
        if errors:
            return str(errors[0])
        return "select normalized GPS route + validated matching GPS AI model"

    def install(self):
        original_ai_select = self.full.ai.enhanced_select_mode

        def ai_select(mode_name, record_gps=True):
            target = DriveMode(str(mode_name).strip().upper())
            if target == DriveMode.AUTO_GPS:
                return {
                    "accepted": True,
                    "target": "AUTO_GPS",
                    "gps_ai": self.controller.start(),
                    "status": self.full.full_status(),
                }
            if self.controller.active:
                self.controller.stop("v2_mode_change")
            return original_ai_select(target.value, record_gps=record_gps)

        def full_select(mode_name, record_gps=True):
            target = DriveMode(str(mode_name).strip().upper())
            if target == DriveMode.AUTO_GPS:
                self.full.AUTO_ORCHESTRATOR.stop()
                if self.full.AUTO_LOCAL_CONTROLLER.active:
                    self.full.AUTO_LOCAL_CONTROLLER.stop("auto_gps_selected")
                return ai_select("AUTO_GPS", record_gps=record_gps)
            if self.controller.active:
                self.controller.stop("v2_mode_change")
            return self._original_full_select(
                target.value,
                record_gps=record_gps,
            )

        self.full.ai.enhanced_select_mode = ai_select
        self.full.full_select_mode = full_select
        self.full.v2._route_preflight = lambda: self.controller.preflight(auto_only=True)

        def full_status():
            status = self._original_full_status()
            status["gps_ai"] = self.status()
            status.setdefault("gps", {})["conditioned_ai"] = status["gps_ai"]
            manual_preflight = status["gps_ai"]["manual_preflight"]
            status["capabilities"]["AUTO_GPS"] = {
                "implemented": True,
                "ready": bool(manual_preflight.get("ready")),
                "reason": self._preflight_reason(manual_preflight),
            }
            return status

        self.full.full_status = full_status

        original_handler = self.release.ReleaseHandler
        integration = self

        class GpsReleaseHandler(original_handler):
            def do_GET(self):
                if self.path == "/api/v2/gps-ai":
                    self._send_json(integration.status())
                    return
                if self.path == "/gps-ai":
                    self._send_html(GPS_AI_HTML)
                    return
                if self.path == "/":
                    body = integration.full.FULL_HTML.replace(
                        b"</header>",
                        '<a href="/gps-ai">GPS AI 관리</a></header>'.encode("utf-8"),
                        1,
                    )
                    self._send_html(body)
                    return
                super().do_GET()

            def do_POST(self):
                try:
                    if self.path == "/api/v2/gps-ai/routes/build":
                        payload = self._read_json()
                        self._send_json(
                            integration.build_route(
                                payload.get("sessions"),
                                payload.get("route_id"),
                            ),
                            202,
                        )
                        return
                    if self.path == "/api/v2/gps-ai/select":
                        payload = self._read_json()
                        self._send_json(
                            integration.select(
                                payload.get("route_id"),
                                payload.get("model_id"),
                            ),
                            202,
                        )
                        return
                    if self.path == "/api/v2/ai/select":
                        payload = self._read_json()
                        model = integration.full.ai.MODEL_REGISTRY.get(
                            payload.get("model_id")
                        )
                        if str(model.get("policy_type") or "AUTO_AI") != "AUTO_AI":
                            raise ValueError(
                                "Use GPS AI selection for AUTO_GPS models"
                            )
                except (
                    ValueError,
                    OSError,
                    TypeError,
                    ModelRegistryError,
                    json.JSONDecodeError,
                ) as error:
                    self._send_json(
                        {"error": str(error), "gps_ai": integration.status()},
                        409,
                    )
                    return
                super().do_POST()

        self.release.ReleaseHandler = GpsReleaseHandler
        return self


GPS_AI_HTML = r"""<!doctype html><html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>GPS AI</title>
<style>body{font-family:system-ui;background:#0b0e0c;color:#eee;max-width:900px;margin:auto;padding:20px}
section{border:1px solid #394239;border-radius:12px;padding:14px;margin:12px 0}input,select,button,textarea{background:#202820;color:#eee;border:1px solid #566356;border-radius:7px;padding:8px;margin:3px}textarea{width:95%;height:80px}pre{white-space:pre-wrap;font-size:11px}</style></head><body>
<a href="/">← V2</a><h2>AUTO_GPS · GPS-conditioned AI</h2>
<section><h3>1. 여러 GPS ON RECORD → 정규화 Route</h3>
<input id="route-id" placeholder="route_id"><textarea id="sessions" placeholder="run_001, run_002, run_003"></textarea>
<button onclick="buildRoute()">Route 생성</button></section>
<section><h3>2. Route + GPS AI 모델 선택</h3><select id="route"></select><select id="model"></select>
<button onclick="selectPair()">선택 저장</button></section><section><h3>상태</h3><pre id="status"></pre></section>
<script>async function api(p,o={}){const r=await fetch(p,{cache:'no-store',...o});const d=await r.json();if(!r.ok)throw new Error(d.error||`HTTP ${r.status}`);return d}
function post(p,b){return api(p,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})}
async function refresh(){const s=await api('/api/v2/gps-ai');route.innerHTML=s.routes.map(x=>`<option value="${x.route_id}">${x.route_id}</option>`).join('');
model.innerHTML=s.models.map(x=>`<option value="${x.model_id}">${x.model_id} · ${x.validation_stage}</option>`).join('');
if(s.selected.route_id)route.value=s.selected.route_id;if(s.selected.model_id)model.value=s.selected.model_id;
status.textContent=JSON.stringify(s,null,2)}
async function buildRoute(){try{await post('/api/v2/gps-ai/routes/build',{route_id:document.getElementById('route-id').value,sessions:document.getElementById('sessions').value.split(',').map(x=>x.trim()).filter(Boolean)});await refresh()}catch(e){alert(e.message)}}
async function selectPair(){try{await post('/api/v2/gps-ai/select',{route_id:route.value,model_id:model.value});await refresh()}catch(e){alert(e.message)}}refresh()</script></body></html>""".encode("utf-8")


def install_gps_ai(release):
    return GpsAiIntegration(release).install()
