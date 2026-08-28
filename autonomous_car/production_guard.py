import os
import shutil
import threading
import time
import types

from autonomous_car import DriveMode
from autonomous_car.pretrained_auto_runtime import install_pretrained_auto_runtime


class ProductionRuntimeGuard:
    """Independent fail-closed monitor for final Raspberry Pi runtime.

    Controllers already validate their own required sensors. This guard adds a
    final service-level backstop for stale LiDAR/IMU, Arduino/steering loss and
    recording/storage failures so an autonomous worker cannot keep issuing
    commands after a required runtime dependency has disappeared.
    """

    def __init__(self, release, gps_ai=None):
        self.release = release
        self.full = release.full
        self.legacy = self.full.legacy
        self.gps_ai = gps_ai
        # server_v2_final constructs this guard after install_full_runtime_hardening,
        # so this is the first production-safe point where the pretrained AUTO
        # layer can wrap the final AUTO_LOCAL/orchestrator instances.
        self.pretrained_auto = install_pretrained_auto_runtime(
            self.full,
            gps_ai=gps_ai,
        )
        self.lidar_max_age_seconds = max(
            0.10, float(os.environ.get("AUTONOMY_LIDAR_MAX_AGE_SECONDS", "0.50"))
        )
        self.imu_max_age_seconds = max(
            0.05, float(os.environ.get("AUTONOMY_IMU_MAX_AGE_SECONDS", "0.30"))
        )
        self.minimum_record_free_bytes = max(
            64 * 1024 * 1024,
            int(os.environ.get("RECORD_MIN_FREE_BYTES", str(256 * 1024 * 1024))),
        )
        self.lock = threading.RLock()
        self.stop_event = threading.Event()
        self.thread = None
        self.last_fault = None
        self.last_fault_at = None
        self.last_storage_free_bytes = None
        self.last_sensor_check = None
        self._handled_record_error = None
        self._install_durable_json_writes()
        self._install_fresh_lidar_gate()
        self._install_record_flush_guard()

    def start(self):
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                return self.snapshot()
            self.stop_event = threading.Event()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return self.snapshot()

    def stop(self):
        self.stop_event.set()
        thread = self.thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        return self.snapshot()

    def snapshot(self):
        with self.lock:
            return {
                "active": bool(self.thread and self.thread.is_alive() and not self.stop_event.is_set()),
                "lidar_max_age_seconds": self.lidar_max_age_seconds,
                "imu_max_age_seconds": self.imu_max_age_seconds,
                "minimum_record_free_bytes": self.minimum_record_free_bytes,
                "last_storage_free_bytes": self.last_storage_free_bytes,
                "last_sensor_check": self.last_sensor_check,
                "last_fault": self.last_fault,
                "last_fault_at": self.last_fault_at,
                "pretrained_auto": self.pretrained_auto.snapshot(),
            }

    @staticmethod
    def _fsync_parent(path):
        parent = os.path.dirname(os.path.abspath(path)) or "."
        try:
            descriptor = os.open(parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        except OSError:
            return
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _install_durable_json_writes(self):
        full = self.full
        if not getattr(full._atomic_json, "_production_durable", False):
            original_atomic_json = full._atomic_json

            def durable_atomic_json(path, document):
                original_atomic_json(path, document)
                self._fsync_parent(path)

            durable_atomic_json._production_durable = True
            full._atomic_json = durable_atomic_json

        ai = full.ai
        if not getattr(ai.select_ai_model, "_production_durable", False):
            original_select_ai_model = ai.select_ai_model

            def durable_select_ai_model(model_id):
                result = original_select_ai_model(model_id)
                self._fsync_parent(ai.SELECTED_MODEL_PATH)
                return result

            durable_select_ai_model._production_durable = True
            ai.select_ai_model = durable_select_ai_model

        if self.gps_ai is not None and not getattr(self.gps_ai.select, "_production_durable", False):
            original_gps_select = self.gps_ai.select

            def durable_gps_select(route_id, model_id):
                result = original_gps_select(route_id, model_id)
                self._fsync_parent(self.gps_ai.selection_path)
                return result

            durable_gps_select._production_durable = True
            self.gps_ai.select = durable_gps_select

    def _install_fresh_lidar_gate(self):
        full = self.full
        if getattr(full._lidar_points, "_production_freshness_guard", False):
            return
        original_lidar_points = full._lidar_points

        def fresh_lidar_points(require_connected=True):
            snapshot, points = original_lidar_points(require_connected=require_connected)
            if require_connected:
                updated = snapshot.get("last_update")
                age = None if updated is None else max(0.0, time.time() - float(updated))
                if age is None or age > self.lidar_max_age_seconds:
                    raise ValueError("AUTO_LOCAL requires fresh LiDAR data")
            return snapshot, points

        fresh_lidar_points._production_freshness_guard = True
        full._lidar_points = fresh_lidar_points

    def _install_record_flush_guard(self):
        recorder = self.legacy.record_manager
        current = recorder._flush_streams
        if getattr(current, "_production_flush_guard", False):
            return

        def guarded_flush(recorder_self, sync=False):
            first_error = None
            for name, file in list(recorder_self._files.items()):
                try:
                    file.flush()
                    if sync:
                        os.fsync(file.fileno())
                except OSError as error:
                    if first_error is None:
                        first_error = error
                    recorder_self.error = (
                        f"RECORD_FLUSH_FAILED:{name}:{type(error).__name__}:{error}"
                    )
            # During the writer loop, propagate the failure so RecordManager's
            # existing outer exception handler terminates the current generation.
            # During final close/fsync keep closing the remaining descriptors.
            if first_error is not None and not sync:
                raise OSError(recorder_self.error) from first_error

        guarded_flush._production_flush_guard = True
        recorder._flush_streams = types.MethodType(guarded_flush, recorder)

    def _active_autonomy(self):
        return any(
            (
                bool(self.full.ai.AUTO_AI_CONTROLLER.active),
                bool(self.full.AUTO_LOCAL_CONTROLLER.active),
                bool(self.full.AUTO_ORCHESTRATOR.active),
                bool(getattr(self.full, "PRETRAINED_AUTO_CONTROLLER", None) is not None and self.full.PRETRAINED_AUTO_CONTROLLER.active),
                bool(self.legacy.auto_route_runtime.active),
                bool(self.gps_ai is not None and self.gps_ai.controller.active),
            )
        )

    def _stop_autonomy(self, reason):
        controllers = [
            getattr(self.full, "AUTO_ORCHESTRATOR", None),
            getattr(self.full, "PRETRAINED_AUTO_CONTROLLER", None),
            None if self.gps_ai is None else self.gps_ai.controller,
            getattr(self.full, "AUTO_LOCAL_CONTROLLER", None),
            getattr(self.full.ai, "AUTO_AI_CONTROLLER", None),
            getattr(self.legacy, "auto_route_runtime", None),
        ]
        for controller in controllers:
            if controller is None or not getattr(controller, "active", False):
                continue
            try:
                controller.stop(reason)
            except TypeError:
                try:
                    controller.stop()
                except Exception:
                    pass
            except Exception:
                pass
        try:
            self.legacy.motor_controller.stop()
        except Exception:
            pass
        try:
            mode = self.legacy.vehicle_state_machine.mode
            if mode not in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
                self.legacy.vehicle_state_machine.transition(DriveMode.FAULT, reason)
        except Exception:
            pass
        with self.lock:
            self.last_fault = reason
            self.last_fault_at = time.time()

    def _sensor_fault_reason(self):
        now = time.time()
        lidar = self.legacy.lidar_monitor.snapshot()
        imu = self.legacy.imu_monitor.snapshot()
        motor = self.legacy.motor_controller.snapshot()
        lidar_update = lidar.get("last_update")
        imu_update = imu.get("last_update")
        lidar_age = None if lidar_update is None else max(0.0, now - float(lidar_update))
        imu_age = None if imu_update is None else max(0.0, now - float(imu_update))
        with self.lock:
            self.last_sensor_check = {
                "lidar_age_seconds": lidar_age,
                "imu_age_seconds": imu_age,
                "arduino_connected": bool(motor.get("connected")),
                "steering_connected": bool(motor.get("encoder_connected")),
            }
        if lidar_age is None or lidar_age > self.lidar_max_age_seconds:
            return "AUTONOMY_LIDAR_STALE"
        if imu_age is None or imu_age > self.imu_max_age_seconds:
            return "AUTONOMY_IMU_STALE"
        if not motor.get("connected"):
            return "AUTONOMY_ARDUINO_DISCONNECTED"
        if not motor.get("encoder_connected"):
            return "AUTONOMY_STEERING_DISCONNECTED"
        return None

    def _check_recording_storage(self):
        recorder = self.legacy.record_manager
        root = recorder.session_path or recorder.root_path
        try:
            free = shutil.disk_usage(root).free
        except OSError:
            free = None
        with self.lock:
            self.last_storage_free_bytes = free
        if recorder.active and free is not None and free < self.minimum_record_free_bytes:
            with recorder.lock:
                recorder.error = "RECORD_STORAGE_LOW"
            try:
                recorder.add_event("RECORD_STORAGE_LOW", str(free))
            except Exception:
                pass
            recorder.stop()
            if self._active_autonomy():
                self._stop_autonomy("RECORD_STORAGE_LOW")
            else:
                try:
                    if self.legacy.vehicle_state_machine.mode == DriveMode.RECORD:
                        self.legacy.vehicle_state_machine.transition(
                            DriveMode.MANUAL_ASSIST,
                            "record_storage_low",
                        )
                except Exception:
                    pass
            return

        error = str(recorder.error or "").strip()
        if error and error != self._handled_record_error:
            self._handled_record_error = error
            if self._active_autonomy():
                self._stop_autonomy(f"RECORDING_FAILED:{error}")
            elif self.legacy.vehicle_state_machine.mode == DriveMode.RECORD:
                try:
                    self.legacy.vehicle_state_machine.transition(
                        DriveMode.MANUAL_ASSIST,
                        "recording_failed",
                    )
                except Exception:
                    pass

    def _run(self):
        next_storage_check = 0.0
        while not self.stop_event.wait(0.05):
            if self._active_autonomy():
                try:
                    reason = self._sensor_fault_reason()
                except Exception as error:
                    reason = f"AUTONOMY_SENSOR_GUARD_ERROR:{type(error).__name__}"
                if reason:
                    self._stop_autonomy(reason)
            now = time.monotonic()
            if now >= next_storage_check:
                next_storage_check = now + 1.0
                try:
                    self._check_recording_storage()
                except Exception as error:
                    with self.lock:
                        self.last_fault = f"STORAGE_GUARD_ERROR:{type(error).__name__}:{error}"
                        self.last_fault_at = time.time()


def shutdown_runtime(release, gps_ai=None, production_guard=None):
    """Best-effort process shutdown that de-energizes actuators first."""
    full = release.full
    legacy = full.legacy
    controllers = [
        getattr(full, "AUTO_ORCHESTRATOR", None),
        getattr(full, "PRETRAINED_AUTO_CONTROLLER", None),
        None if gps_ai is None else gps_ai.controller,
        getattr(full, "AUTO_LOCAL_CONTROLLER", None),
        getattr(full.ai, "AUTO_AI_CONTROLLER", None),
        getattr(legacy, "auto_route_runtime", None),
    ]
    for controller in controllers:
        if controller is None or not getattr(controller, "active", False):
            continue
        try:
            controller.stop("service_shutdown")
        except TypeError:
            try:
                controller.stop()
            except Exception:
                pass
        except Exception:
            pass
    try:
        legacy.motor_controller.stop()
    except Exception:
        pass
    try:
        legacy.motor_controller.stop_steering()
    except Exception:
        pass
    try:
        if legacy.record_manager.active:
            legacy.record_manager.stop()
    except Exception:
        pass
    try:
        if full.MAPPING_CONTROLLER.active:
            full.MAPPING_CONTROLLER.stop(save=False)
    except Exception:
        pass
    if production_guard is not None:
        try:
            production_guard.stop()
        except Exception:
            pass


__all__ = ["ProductionRuntimeGuard", "shutdown_runtime"]
