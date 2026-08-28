"""Install lightweight pretrained lane perception and no-training AUTO strategy.

This module is installed only by the final V2 service. It deliberately avoids
changing AUTO_AI: user-trained driving inference remains independent. Instead it
adds a final AUTO-orchestrator fallback named PRETRAINED_ROAD and enables LSTR
as the primary lane backend only while AUTO_LOCAL or the pretrained AUTO
controller needs it. The classical lane detector remains the immediate fallback.
"""

from __future__ import annotations

import os
import threading
import time
import types

from autonomous_car import ControlRequest, DriveMode
from autonomous_car.control.hybrid_lane_controller import HybridLaneController
from autonomous_car.perception.pretrained_road import (
    DEFAULT_MODEL_FILENAME,
    PretrainedRoadPerception,
)
from autonomous_car.safety import SteeringTrackingGuard


class PretrainedRoadAutoController:
    """Conservative pretrained-lane cruise for AUTO without user training."""

    MIN_CONFIDENCE = 0.58
    LANE_LOSS_STOP_SECONDS = 0.65
    TARGET_SPEED_NEURAL_MPS = 0.20
    TARGET_SPEED_FALLBACK_MPS = 0.11

    def __init__(self, full, hybrid_lane):
        self.full = full
        self.legacy = full.legacy
        self.hybrid_lane = hybrid_lane
        self.lock = threading.RLock()
        self.active = False
        self.started_at = None
        self.thread = None
        self.stop_event = threading.Event()
        self.error = None
        self.last_lane = None
        self.last_command = None
        self.last_safety = None
        self.last_preflight = None
        self.lane_lost_since = None
        self.loop_seconds = None
        self.steering_tracking_guard = SteeringTrackingGuard(
            self.legacy.AUTO_STEERING_MAX_ERROR_DEGREES,
            self.legacy.AUTO_STEERING_ERROR_TIMEOUT_SECONDS,
        )

    def _fresh_camera_frame(self, maximum_age_seconds=0.35):
        frame, sequence, frame_monotonic, _ = self.legacy.camera.snapshot_frame()
        if frame is None or frame_monotonic is None:
            raise ValueError("PRETRAINED_AUTO_CAMERA_UNAVAILABLE")
        age = time.monotonic() - float(frame_monotonic)
        if age > maximum_age_seconds:
            raise ValueError(f"PRETRAINED_AUTO_CAMERA_STALE:{age:.3f}s")
        return frame, sequence, age

    def _lidar_ready(self):
        snapshot = self.legacy.lidar_monitor.snapshot()
        points = snapshot.get("safety_points") or snapshot.get("points") or []
        return bool(snapshot.get("connected")) and len(points) >= 20, snapshot

    def preflight(self, probe_frame=True):
        details = {
            "ready": False,
            "strategy": "PRETRAINED_ROAD",
            "required_neural_backend": self.hybrid_lane.NEURAL_BACKEND,
            "maximum_neural_inference_ms": (
                self.hybrid_lane.maximum_neural_inference_ms
            ),
            "perception": self.hybrid_lane.snapshot(),
        }
        try:
            if not self.hybrid_lane.pretrained.ensure_loaded():
                raise ValueError(
                    self.hybrid_lane.pretrained.snapshot().get("error")
                    or "PRETRAINED_MODEL_UNAVAILABLE"
                )
            lidar_ready, lidar = self._lidar_ready()
            details["lidar_connected"] = bool(lidar.get("connected"))
            details["lidar_points"] = len(
                lidar.get("safety_points") or lidar.get("points") or []
            )
            if not lidar_ready:
                raise ValueError("PRETRAINED_AUTO_LIDAR_NOT_READY")
            # This validates steering calibration without commanding motion.
            self.legacy.normalized_steering_for_angle(0.0)
            frame, sequence, camera_age = self._fresh_camera_frame()
            details["camera_sequence"] = sequence
            details["camera_age_seconds"] = camera_age
            if probe_frame:
                # Warm and benchmark before any autonomous state transition.
                # The first run may include allocator/graph warmup; the final
                # warmed sample still has to leave margin under the 200 ms soft
                # control-loop timing guard.
                latency = self.hybrid_lane.probe_neural_latency_jpeg(
                    frame, attempts=2
                )
                details["neural_latency"] = latency
                if not latency.get("allowed"):
                    raise ValueError(
                        latency.get("error")
                        or "PRETRAINED_AUTO_NEURAL_LATENCY_REJECTED"
                    )

                previous = self.hybrid_lane.neural_enabled
                self.hybrid_lane.set_neural_enabled(True)
                try:
                    lane = self.hybrid_lane.analyze_jpeg(frame)
                finally:
                    if not self.active:
                        self.hybrid_lane.set_neural_enabled(previous)
                lane_document = lane.as_dict()
                details["lane"] = lane_document
                details["perception"] = self.hybrid_lane.snapshot()
                if str(lane.backend) != self.hybrid_lane.NEURAL_BACKEND:
                    raise ValueError(
                        "PRETRAINED_AUTO_NEURAL_BACKEND_REQUIRED:"
                        + str(
                            details["perception"].get("fallback_reason")
                            or lane.backend
                        )
                    )
                if not lane.detected or float(lane.confidence) < self.MIN_CONFIDENCE:
                    raise ValueError(
                        lane.error
                        or f"PRETRAINED_AUTO_LANE_CONFIDENCE_LOW:{lane.confidence:.3f}"
                    )
            details["ready"] = True
            details["error"] = None
            self.last_preflight = details
            return details
        except Exception as error:
            details["error"] = f"{type(error).__name__}: {error}"
            details["perception"] = self.hybrid_lane.snapshot()
            self.last_preflight = details
            return details

    def start(self, preflight=None):
        with self.lock:
            if self.active:
                return self.snapshot()
            check = preflight or self.preflight(probe_frame=True)
            if not check.get("ready"):
                raise ValueError(check.get("error") or "PRETRAINED_AUTO_NOT_READY")
            mode = self.legacy.vehicle_state_machine.mode
            if mode in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
                raise ValueError("Safety reset is required before PRETRAINED_ROAD AUTO")
            if mode not in {
                DriveMode.DISARMED,
                DriveMode.MANUAL,
                DriveMode.MANUAL_ASSIST,
                DriveMode.AUTO,
            }:
                self.legacy.vehicle_state_machine.transition(
                    DriveMode.DISARMED, "pretrained_auto_prepare"
                )
            self.legacy.motor_controller.stop()
            self.legacy.vehicle_state_machine.transition(
                DriveMode.AUTO, "pretrained_road_auto_started"
            )
            self.hybrid_lane.reset()
            self.hybrid_lane.set_neural_enabled(True)
            self.error = None
            self.last_lane = None
            self.last_command = None
            self.last_safety = None
            self.lane_lost_since = None
            self.loop_seconds = None
            self.steering_tracking_guard.reset()
            self.stop_event = threading.Event()
            self.active = True
            self.started_at = time.time()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return {**self.snapshot(), "preflight": check}

    def stop(self, reason="operator_stop"):
        with self.lock:
            if not self.active:
                self.hybrid_lane.set_neural_enabled(False)
                return self.snapshot()
            self.active = False
            self.stop_event.set()
            thread = self.thread
        self.legacy.motor_controller.stop()
        self.steering_tracking_guard.reset()
        self.hybrid_lane.set_neural_enabled(False)
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self.lock:
            if self.legacy.vehicle_state_machine.mode == DriveMode.AUTO:
                self.legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)
            return self.snapshot()

    def _fail(self, reason):
        with self.lock:
            self.error = str(reason)
            self.active = False
            self.stop_event.set()
        self.legacy.motor_controller.stop()
        self.steering_tracking_guard.reset()
        self.hybrid_lane.set_neural_enabled(False)
        try:
            if self.legacy.vehicle_state_machine.mode == DriveMode.AUTO:
                self.legacy.vehicle_state_machine.transition(DriveMode.FAULT, str(reason))
        except Exception:
            pass

    def _run(self):
        previous_started = time.monotonic()
        while not self.stop_event.is_set():
            loop_started = time.monotonic()
            try:
                with self.lock:
                    if not self.active:
                        return
                motor = self.legacy.motor_controller.snapshot()
                steering_fault = self.steering_tracking_guard.evaluate(
                    motor.get("target_steering_angle_degrees"),
                    motor.get("steering_angle_degrees"),
                    active=motor.get("steering_control_mode") == "ANGLE",
                )
                if steering_fault:
                    self._fail("PRETRAINED_AUTO_STEERING_TRACKING_FAULT")
                    return

                frame, sequence, camera_age = self._fresh_camera_frame()
                lane = self.hybrid_lane.analyze_jpeg(frame)
                lane_document = lane.as_dict()
                with self.lock:
                    self.last_lane = lane_document

                confidence = float(lane.confidence or 0.0)
                if not lane.detected or confidence < self.MIN_CONFIDENCE:
                    if self.lane_lost_since is None:
                        self.lane_lost_since = time.monotonic()
                    self.legacy.motor_controller.stop_steering()
                    self.legacy.motor_controller.set_drive(0.0, True)
                    if (
                        time.monotonic() - self.lane_lost_since
                        > self.LANE_LOSS_STOP_SECONDS
                    ):
                        self._fail(
                            "PRETRAINED_AUTO_LANE_LOST:"
                            + str(lane.error or f"confidence={confidence:.3f}")
                        )
                        return
                    self.stop_event.wait(0.05)
                    continue
                self.lane_lost_since = None

                angle = max(
                    -8.0,
                    min(8.0, float(lane.correction_angle_degrees or 0.0)),
                )
                steering = self.legacy.normalized_steering_for_angle(angle)
                neural_primary = (
                    str(lane.backend) == self.hybrid_lane.NEURAL_BACKEND
                )
                base_speed = (
                    self.TARGET_SPEED_NEURAL_MPS
                    if neural_primary
                    else self.TARGET_SPEED_FALLBACK_MPS
                )
                confidence_scale = max(0.55, min(1.0, confidence))
                turn_scale = max(0.45, 1.0 - abs(angle) / 12.0)
                target_speed = base_speed * confidence_scale * turn_scale
                throttle = self.legacy.throttle_calibration.throttle_for_speed(
                    target_speed
                )
                request = ControlRequest(
                    throttle=throttle,
                    steering=steering,
                    enabled=True,
                    source="pretrained_road_auto",
                )
                # Safety must see both start-to-start delay and the time already
                # consumed by the current perception/geometry iteration. This
                # prevents one suddenly slow inference from issuing a command
                # before the next loop gets a chance to notice the stall.
                period_delay = max(0.0, loop_started - previous_started)
                current_delay = max(0.0, time.monotonic() - loop_started)
                loop_delay = max(period_delay, current_delay)
                previous_started = loop_started
                decision = self.legacy.safety_supervisor.evaluate(
                    request,
                    self.legacy.safety_context(loop_delay),
                )
                with self.lock:
                    self.last_safety = decision.as_dict()
                    self.last_command = {
                        "camera_sequence": sequence,
                        "camera_age_seconds": camera_age,
                        "backend": lane.backend,
                        "confidence": confidence,
                        "steering_angle_degrees": angle,
                        "target_speed_mps": target_speed,
                        "requested_throttle": throttle,
                        "final_throttle": decision.final_throttle,
                        "allowed": decision.allowed,
                        "stop_reason": decision.stop_reason,
                        "loop_delay_seconds": loop_delay,
                    }

                if not decision.allowed:
                    self.legacy.motor_controller.stop_steering()
                    self.legacy.motor_controller.set_drive(0.0, True)
                else:
                    # Match the proven AUTO_ROUTE ordering: no propulsion while
                    # a steering command is being accepted/rejected.
                    self.legacy.motor_controller.set_drive(0.0, True)
                    steering_result = self.legacy.motor_controller.set_steering(
                        decision.final_steering
                    )
                    if steering_result.get("steering_rejection"):
                        self.legacy.motor_controller.set_drive(0.0, False)
                        self._fail("PRETRAINED_AUTO_STEERING_COMMAND_REJECTED")
                        return
                    self.legacy.motor_controller.set_drive(
                        decision.final_throttle, True
                    )
            except Exception as error:
                self._fail(
                    f"PRETRAINED_AUTO_RUNTIME_ERROR:{type(error).__name__}:{error}"
                )
                return
            elapsed = time.monotonic() - loop_started
            with self.lock:
                self.loop_seconds = elapsed
            self.stop_event.wait(max(0.0, 0.10 - elapsed))

    def snapshot(self):
        with self.lock:
            return {
                "active": self.active,
                "strategy": "PRETRAINED_ROAD",
                "started_at": self.started_at,
                "error": self.error,
                "lane": self.last_lane,
                "last_command": self.last_command,
                "safety": self.last_safety,
                "last_preflight": self.last_preflight,
                "loop_seconds": self.loop_seconds,
                "steering_tracking": self.steering_tracking_guard.snapshot(),
                "perception": self.hybrid_lane.snapshot(),
            }


def install_pretrained_auto_runtime(full, gps_ai=None):
    """Install hybrid LSTR perception and append PRETRAINED_ROAD to AUTO."""
    legacy = full.legacy
    project_root = getattr(full, "PROJECT_ROOT", os.getcwd())
    model_path = os.environ.get(
        "SWING_PRETRAINED_ROAD_MODEL",
        os.path.join(project_root, "models", "pretrained", DEFAULT_MODEL_FILENAME),
    )
    threads = max(1, int(os.environ.get("SWING_PRETRAINED_ROAD_THREADS", "2")))
    maximum_neural_inference_ms = max(
        20.0,
        float(os.environ.get("SWING_PRETRAINED_ROAD_MAX_INFERENCE_MS", "160")),
    )
    pretrained = PretrainedRoadPerception(model_path, threads=threads)

    previous_lane = legacy.auto_route_runtime.lane_controller
    hybrid = HybridLaneController(
        pretrained,
        camera_calibration=getattr(legacy, "camera_calibration", None),
        expected_lane_width_m=float(
            getattr(previous_lane, "expected_lane_width_m", 1.0)
        ),
        vehicle_width_m=float(getattr(previous_lane, "vehicle_width_m", 0.4826)),
        processing_width=int(getattr(previous_lane, "processing_width", 640)),
        processing_height=int(getattr(previous_lane, "processing_height", 360)),
        maximum_neural_inference_ms=maximum_neural_inference_ms,
    )
    legacy.auto_route_runtime.lane_controller = hybrid
    controller = PretrainedRoadAutoController(full, hybrid)

    # AUTO_LOCAL uses lane assistance only as a secondary correction. Warm the
    # neural model while still stopped; if it exceeds the explicit latency
    # budget, leave AUTO_LOCAL on the existing classical lane detector rather
    # than injecting a repeatedly slow network into its localization loop.
    local = full.AUTO_LOCAL_CONTROLLER
    original_local_start = local.start
    original_local_stop = local.stop

    def local_start(self, *args, **kwargs):
        neural_allowed = False
        try:
            # Mode selection is an explicit handoff; stop propulsion before a
            # potentially non-trivial model warmup so latency qualification can
            # never delay a manual drive command that is still energizing wheels.
            legacy.motor_controller.stop()
            frame, _, frame_monotonic, _ = legacy.camera.snapshot_frame()
            if (
                frame is not None
                and frame_monotonic is not None
                and time.monotonic() - float(frame_monotonic) <= 0.35
            ):
                probe = hybrid.probe_neural_latency_jpeg(frame, attempts=2)
                neural_allowed = bool(probe.get("allowed"))
        except Exception:
            neural_allowed = False
        hybrid.set_neural_enabled(neural_allowed)
        try:
            return original_local_start(*args, **kwargs)
        except Exception:
            if not controller.active:
                hybrid.set_neural_enabled(False)
            raise

    def local_stop(self, *args, **kwargs):
        try:
            return original_local_stop(*args, **kwargs)
        finally:
            if not controller.active:
                hybrid.set_neural_enabled(False)

    local.start = types.MethodType(local_start, local)
    local.stop = types.MethodType(local_stop, local)

    orchestrator = full.AUTO_ORCHESTRATOR
    original_auto_start = orchestrator.start
    original_auto_stop = orchestrator.stop

    def auto_start(self):
        try:
            return original_auto_start()
        except ValueError:
            # Only append the generic pretrained strategy after the established
            # priority order (GPS > LOCAL > compatible user-trained AI) has been
            # exhausted. Do not mask an unrelated orchestrator failure.
            if getattr(self, "reason", None) != "no_safe_autonomous_strategy":
                raise
        check = controller.preflight(probe_frame=True)
        self.last_attempts.append({"mode": "PRETRAINED_ROAD", **check})
        if not check.get("ready"):
            self.active = False
            self.reason = "no_safe_autonomous_strategy"
            legacy.motor_controller.stop()
            raise ValueError(
                "AUTO found no ready GPS, LOCAL, AUTO_ALLOWED AI, or PRETRAINED_ROAD strategy"
            )
        self.active = True
        result = controller.start(preflight=check)
        return self._selected(
            "PRETRAINED_ROAD",
            "pretrained_road_model_ready",
            pretrained.snapshot().get("model"),
            result,
        )

    def auto_stop(self):
        if controller.active:
            controller.stop("auto_orchestrator_stop")
        return original_auto_stop()

    orchestrator.start = types.MethodType(auto_start, orchestrator)
    orchestrator.stop = types.MethodType(auto_stop, orchestrator)

    original_full_status = full.full_status

    def full_status_with_pretrained():
        status = original_full_status()
        status["pretrained_road"] = {
            "runtime": controller.snapshot(),
            "perception": hybrid.snapshot(),
        }
        capability = status.setdefault("capabilities", {}).setdefault("AUTO", {})
        base_ready = bool(capability.get("ready"))
        pretrained_ready = bool(
            pretrained.available
            and legacy.lidar_monitor.snapshot().get("connected")
        )
        capability["ready"] = base_ready or pretrained_ready
        if pretrained_ready and not base_ready:
            capability["reason"] = "pretrained_road_available"
        return status

    full.full_status = full_status_with_pretrained
    full.PRETRAINED_ROAD_PERCEPTION = pretrained
    full.HYBRID_LANE_CONTROLLER = hybrid
    full.PRETRAINED_AUTO_CONTROLLER = controller
    return controller


__all__ = ["PretrainedRoadAutoController", "install_pretrained_auto_runtime"]
