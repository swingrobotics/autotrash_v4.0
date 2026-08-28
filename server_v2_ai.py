#!/usr/bin/env python3
"""AUTO_AI integration layer for the Autonomy V2 server."""

import json
import os
import threading
import time

import server_v2 as v2
from autonomous_car import ControlRequest, DriveMode
from autonomous_car.ai import AutoAiRuntime, ModelRegistry, ModelRegistryError


legacy = v2.legacy
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
MODELS_ROOT = os.environ.get(
    "AUTONOMY_MODELS_PATH",
    os.path.join(PROJECT_ROOT, "models"),
)
SELECTED_MODEL_PATH = os.path.join(MODELS_ROOT, "selected-model.json")
MODEL_REGISTRY = ModelRegistry(MODELS_ROOT)


def _safe_model_path(filename):
    filename = os.path.basename(str(filename or ""))
    if not filename:
        raise ValueError("Model file is not configured")
    root = os.path.abspath(MODELS_ROOT)
    path = os.path.abspath(os.path.join(root, filename))
    if os.path.commonpath([root, path]) != root:
        raise ValueError("Model path escapes model registry")
    return path


def _selected_model_id():
    try:
        with open(SELECTED_MODEL_PATH, "r", encoding="utf-8") as file:
            document = json.load(file)
        return str(document.get("model_id") or "").strip() or None
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def _model_artifacts(model):
    """Validate the install/runtime artifact contract without starting inference."""

    model_path = _safe_model_path(model.get("model_file"))
    manifest_file = model.get("manifest_file")
    if not os.path.isfile(model_path):
        raise FileNotFoundError(model_path)
    if not manifest_file:
        raise ValueError("AUTO_AI_MODEL_MANIFEST_MISSING")
    manifest_path = _safe_model_path(manifest_file)
    if not os.path.isfile(manifest_path):
        raise FileNotFoundError(manifest_path)
    try:
        with open(manifest_path, "r", encoding="utf-8") as file:
            manifest = json.load(file)
    except json.JSONDecodeError as error:
        raise ValueError(f"AUTO_AI_MANIFEST_INVALID: {error}") from error

    export = manifest.get("export") or {}
    # The rover install API transfers one ONNX artifact. Therefore a candidate
    # that depends on an external .onnx.data sidecar is not a valid vehicle
    # artifact even if the registry JSON and main ONNX file exist.
    if export.get("external_data") is not False or export.get("self_contained") is not True:
        raise ValueError("AUTO_AI_MODEL_NOT_SELF_CONTAINED")
    return model_path, manifest_path, manifest


def select_ai_model(model_id):
    model = MODEL_REGISTRY.get(model_id)
    _model_artifacts(model)
    os.makedirs(MODELS_ROOT, exist_ok=True)
    temporary = SELECTED_MODEL_PATH + ".tmp"
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump({"model_id": model["model_id"]}, file, indent=2)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, SELECTED_MODEL_PATH)
    return model


class AutoAiController:
    CAMERA_TIMEOUT_SECONDS = 0.30

    def __init__(self):
        self.lock = threading.RLock()
        self.active = False
        self.model_id = None
        self.runtime = None
        self.thread = None
        self.last_inference = None
        self.error = None
        self.started_monotonic = None
        self._generation = 0
        self._stop_event = threading.Event()

    def start(self, model_id=None):
        with self.lock:
            if self.active:
                return self.snapshot()
            model_id = model_id or _selected_model_id()
            if not model_id:
                raise ValueError("Select an AUTO_AI model before starting")
            model = MODEL_REGISTRY.get(model_id)
            stage = model.get("validation_stage", "TRAINED")
            if stage not in {"CLOSED_AREA_VALIDATED", "AUTO_ALLOWED"}:
                raise ValueError(
                    "AUTO_AI vehicle runtime requires CLOSED_AREA_VALIDATED or AUTO_ALLOWED model"
                )
            model_path, manifest_path, _ = _model_artifacts(model)
            runtime = AutoAiRuntime(model_path, manifest_path)

            if legacy.auto_route_runtime.active:
                legacy.auto_route_runtime.stop("auto_ai_takeover")
            if legacy.record_manager.active:
                legacy.record_manager.stop()
            mode = legacy.vehicle_state_machine.mode
            if mode in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
                raise ValueError("Safety reset is required before AUTO_AI")
            if mode == DriveMode.RECORD:
                legacy.vehicle_state_machine.transition(
                    DriveMode.MANUAL_ASSIST,
                    "auto_ai_prepare",
                )
                mode = legacy.vehicle_state_machine.mode
            if mode not in {
                DriveMode.DISARMED,
                DriveMode.MANUAL,
                DriveMode.MANUAL_ASSIST,
            }:
                legacy.vehicle_state_machine.transition(
                    DriveMode.DISARMED,
                    "auto_ai_prepare",
                )
            legacy.motor_controller.stop()
            legacy.vehicle_state_machine.transition(DriveMode.AUTO_AI, "auto_ai_started")

            self.runtime = runtime
            self.model_id = model["model_id"]
            self.last_inference = None
            self.error = None
            self.started_monotonic = time.monotonic()
            self.active = True
            self._generation += 1
            generation = self._generation
            stop_event = threading.Event()
            self._stop_event = stop_event
            self.thread = threading.Thread(
                target=self._run,
                args=(generation, stop_event, runtime),
                daemon=True,
            )
            self.thread.start()
            return self.snapshot()

    def stop(self, reason="operator_stop"):
        # Signal cancellation before waiting for the short output critical
        # section. The worker performs camera/preprocess/ONNX work outside the
        # controller lock, so STOP does not wait for inference to finish.
        stop_event = self._stop_event
        stop_event.set()
        with self.lock:
            self.active = False
            self._generation += 1
            legacy.motor_controller.stop()
            if legacy.vehicle_state_machine.mode == DriveMode.AUTO_AI:
                legacy.vehicle_state_machine.transition(
                    DriveMode.MANUAL_ASSIST,
                    reason,
                )
            return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {
                "active": self.active,
                "model_id": self.model_id,
                "runtime": self.runtime.snapshot() if self.runtime else None,
                "last_inference": self.last_inference,
                "error": self.error,
                "generation": self._generation,
                "duration_seconds": (
                    time.monotonic() - self.started_monotonic
                    if self.started_monotonic is not None
                    else 0.0
                ),
            }

    def _run_is_current_locked(self, generation, stop_event, runtime):
        return (
            self.active
            and self._generation == generation
            and self._stop_event is stop_event
            and not stop_event.is_set()
            and self.runtime is runtime
        )

    def _run(self, generation, stop_event, runtime):
        previous_loop_started = None
        while not stop_event.is_set():
            loop_started = time.perf_counter()
            monotonic_started = time.monotonic()
            loop_delay = (
                0.0
                if previous_loop_started is None
                else max(0.0, monotonic_started - previous_loop_started)
            )
            previous_loop_started = monotonic_started
            try:
                with self.lock:
                    if not self._run_is_current_locked(generation, stop_event, runtime):
                        return

                sensor_started = time.perf_counter()
                frame, _, frame_monotonic, _ = legacy.camera.snapshot_frame()
                if frame is None or frame_monotonic is None:
                    self._fault(generation, stop_event, "CAMERA_UNAVAILABLE")
                    return
                frame_age = time.monotonic() - frame_monotonic
                if frame_age > self.CAMERA_TIMEOUT_SECONDS:
                    self._fault(generation, stop_event, "CAMERA_TIMEOUT")
                    return
                lidar = legacy.lidar_monitor.snapshot()
                imu = legacy.imu_monitor.snapshot()
                perception = legacy.perception_monitor.snapshot()
                sensor_seconds = time.perf_counter() - sensor_started

                if stop_event.is_set():
                    return

                inference_started = time.perf_counter()
                inference = runtime.infer_jpeg(
                    frame,
                    lidar.get("safety_points") or [],
                    imu.get("yaw_rate_dps"),
                    person_hazard=bool(perception.get("hazard")),
                )
                inference_call_seconds = time.perf_counter() - inference_started

                if stop_event.is_set():
                    return

                request = ControlRequest(
                    throttle=inference.throttle,
                    steering=legacy.normalized_steering_for_angle(
                        inference.steering_degrees
                    ),
                    enabled=True,
                    source="auto_ai",
                )
                safety_started = time.perf_counter()
                decision = legacy.safety_supervisor.evaluate(
                    request,
                    legacy.safety_context(loop_delay),
                )
                safety_seconds = time.perf_counter() - safety_started
                inference_data = inference.as_dict()
                inference_data["safety"] = decision.as_dict()
                inference_data["timing"] = {
                    "sensor_snapshot_seconds": sensor_seconds,
                    "inference_call_seconds": inference_call_seconds,
                    "onnx_seconds": inference.inference_seconds,
                    "preprocess_feature_seconds": max(
                        0.0,
                        inference_call_seconds - inference.inference_seconds,
                    ),
                    "safety_seconds": safety_seconds,
                    "loop_period_seconds": loop_delay,
                }

                # Only the final state check and actuator writes are serialized.
                # This guarantees a STOP that wins the lock leaves the actuator
                # stopped, while avoiding an inference-sized lock hold.
                with self.lock:
                    if not self._run_is_current_locked(generation, stop_event, runtime):
                        return
                    actuation_started = time.perf_counter()
                    drive_result = None
                    steering_result = None
                    if not decision.allowed:
                        drive_result = legacy.motor_controller.set_drive(0.0, True)
                        steering_result = legacy.motor_controller.set_steering(0.0)
                        inference_data["actuation"] = self._actuation_snapshot(
                            decision,
                            drive_result,
                            steering_result,
                        )
                        if decision.stop_reason != "CAMERA_OBJECT_STOP":
                            self.last_inference = inference_data
                            self._fault_locked(
                                decision.stop_reason or "AUTO_AI_SAFETY_STOP"
                            )
                            return
                    else:
                        # Neutralize propulsion while steering changes, then
                        # apply the safety-approved propulsion command.
                        legacy.motor_controller.set_drive(0.0, True)
                        steering_result = legacy.motor_controller.set_steering(
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
                        drive_result = legacy.motor_controller.set_drive(
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
                    f"AUTO_AI_RUNTIME_ERROR:{type(error).__name__}:{error}",
                )
                return
            elapsed = time.perf_counter() - loop_started
            stop_event.wait(max(0.0, 0.1 - elapsed))

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
        self._stop_event.set()
        self._generation += 1
        self.error = reason
        legacy.motor_controller.stop()
        if legacy.vehicle_state_machine.mode != DriveMode.FAULT:
            legacy.vehicle_state_machine.transition(DriveMode.FAULT, reason)


AUTO_AI_CONTROLLER = AutoAiController()


def ai_status():
    selected_id = _selected_model_id()
    selected = None
    selected_error = None
    artifacts_ready = False
    if selected_id:
        try:
            selected = MODEL_REGISTRY.get(selected_id)
            _model_artifacts(selected)
            artifacts_ready = True
        except (ValueError, OSError, ModelRegistryError, json.JSONDecodeError) as error:
            selected_error = str(error)
    ready = bool(
        selected
        and artifacts_ready
        and selected.get("validation_stage")
        in {"CLOSED_AREA_VALIDATED", "AUTO_ALLOWED"}
    )
    return {
        "selected_model_id": selected_id,
        "selected_model": selected,
        "selected_error": selected_error,
        "artifacts_ready": artifacts_ready,
        "ready": ready,
        "controller": AUTO_AI_CONTROLLER.snapshot(),
    }


def enhanced_status():
    status = v2.v2_status()
    status.pop("message", None)
    ai = ai_status()
    status["ai"] = ai
    status["capabilities"]["AUTO_AI"] = {
        "implemented": True,
        "ready": ai["ready"],
        "reason": (
            "selected_model_ready"
            if ai["ready"]
            else (
                ai.get("selected_error")
                or "select a CLOSED_AREA_VALIDATED/AUTO_ALLOWED installed model"
            )
        ),
    }
    return status


def enhanced_select_mode(mode_name, record_gps=True):
    target = DriveMode(str(mode_name).strip().upper())
    if target == DriveMode.AUTO_AI:
        return {
            "accepted": True,
            "target": target.value,
            "ai": AUTO_AI_CONTROLLER.start(),
            "status": enhanced_status(),
        }
    if AUTO_AI_CONTROLLER.active:
        AUTO_AI_CONTROLLER.stop("v2_mode_change")
    return v2.select_mode(target.value, record_gps=record_gps)


class V2AiHandler(v2.V2Handler):
    def do_GET(self):
        if self.path == "/api/v2/status":
            self._send_json(enhanced_status())
            return
        if self.path == "/api/v2/modes":
            status = enhanced_status()
            self._send_json(
                {
                    "modes": [mode.value for mode in v2.MODE_ORDER],
                    "capabilities": status["capabilities"],
                    "state": status["state"],
                }
            )
            return
        if self.path == "/api/v2/ai/models":
            self._send_json(
                {
                    "models": MODEL_REGISTRY.list_models(),
                    "ai": ai_status(),
                }
            )
            return
        super().do_GET()

    def do_POST(self):
        if self.path == "/api/v2/ai/select":
            try:
                payload = self._read_json()
                if AUTO_AI_CONTROLLER.active:
                    raise ValueError("Stop AUTO_AI before changing models")
                model = select_ai_model(payload.get("model_id"))
                self._send_json({"selected": model, "ai": ai_status()}, 202)
            except (ValueError, OSError, ModelRegistryError, json.JSONDecodeError) as error:
                self._send_json({"error": str(error), "ai": ai_status()}, 409)
            return
        if self.path == "/api/v2/mode":
            try:
                payload = self._read_json()
                result = enhanced_select_mode(
                    payload.get("mode"),
                    record_gps=payload.get("record_gps", True),
                )
                status = 202 if result.get("accepted", False) else 409
                self._send_json(result, status)
            except NotImplementedError as error:
                self._send_json({"error": str(error), "status": enhanced_status()}, 501)
            except (ValueError, OSError, ModelRegistryError, json.JSONDecodeError) as error:
                self._send_json({"error": str(error), "status": enhanced_status()}, 409)
            return
        if self.path == "/api/safety/emergency-stop" and AUTO_AI_CONTROLLER.active:
            AUTO_AI_CONTROLLER.stop("emergency_stop")
        super().do_POST()


def main():
    legacy.camera.start()
    legacy.gps_monitor.start()
    legacy.ntrip_client.start()
    legacy.imu_monitor.start()
    legacy.lidar_monitor.start()
    legacy.motor_controller.start()
    legacy.perception_monitor.start()
    httpd = legacy.ThreadingHTTPServer((legacy.HOST, legacy.PORT), V2AiHandler)
    print(
        f"GNSS Autonomy V2+AI listening on http://{legacy.HOST}:{legacy.PORT} "
        f"(legacy dashboard: /legacy)",
        flush=True,
    )
    httpd.serve_forever()


if __name__ == "__main__":
    main()
