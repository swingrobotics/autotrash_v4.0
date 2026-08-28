from __future__ import annotations

from .state import DriveMode


AUTONOMOUS_CANONICAL_MODES = {
    DriveMode.AUTO_AI,
    DriveMode.AUTO_GPS,
    DriveMode.AUTO_LOCAL,
    DriveMode.AUTO,
}


def install_manual_takeover_guards(
    legacy,
    *,
    auto_ai_controller,
    auto_local_controller,
    auto_orchestrator,
    auto_gps_controller=None,
):
    """Patch legacy manual commands so V2 autonomy cannot race human input."""

    original_drive = legacy.apply_safe_drive
    original_steering = legacy.apply_safe_steering

    def autonomous_active():
        mode = legacy.vehicle_state_machine.mode
        try:
            canonical = mode.canonical
        except AttributeError:
            canonical = DriveMode(mode).canonical
        return canonical in AUTONOMOUS_CANONICAL_MODES

    def ignored_result():
        result = legacy.motor_controller.snapshot()
        result["safety"] = legacy.safety_snapshot()
        result["state_machine"] = legacy.vehicle_state_machine.snapshot()
        result["manual_command_ignored"] = True
        result["manual_takeover_required"] = True
        return result

    def takeover(reason="manual_override"):
        auto_orchestrator.stop()
        if auto_gps_controller is not None and auto_gps_controller.active:
            auto_gps_controller.stop(reason)
        if auto_ai_controller.active:
            auto_ai_controller.stop(reason)
        if auto_local_controller.active:
            auto_local_controller.stop(reason)
        if legacy.auto_route_runtime.active:
            legacy.auto_route_runtime.stop(reason)

        mode = legacy.vehicle_state_machine.mode
        if mode in {
            DriveMode.AUTO_AI,
            DriveMode.AUTO_GPS,
            DriveMode.AUTO_LOCAL,
            DriveMode.AUTO,
            DriveMode.AUTO_ROUTE,
            DriveMode.AUTO_HYBRID,
        }:
            try:
                legacy.vehicle_state_machine.transition(DriveMode.MANUAL_ASSIST, reason)
            except Exception:
                legacy.motor_controller.stop()
                raise

    def scale_manual_throttle(throttle):
        """Use the whole stick travel for the configured manual speed range.

        The dashboard sends a normalized stick request in [-1, 1].  The legacy
        SafetySupervisor caps output at MANUAL_MAX_THROTTLE, which previously
        meant every stick position above that cap felt identical.  Scale first
        so 0..100% stick maps linearly to 0..configured maximum throttle.
        """
        normalized = max(-1.0, min(1.0, float(throttle)))
        maximum = max(
            0.0,
            min(1.0, float(getattr(legacy, "MANUAL_MAX_THROTTLE", 1.0))),
        )
        return normalized * maximum

    def apply_safe_drive_v2(throttle, enabled, deadman=False):
        explicit_takeover = bool(enabled) and bool(deadman)
        if autonomous_active() or auto_orchestrator.active:
            if not explicit_takeover:
                return ignored_result()
            takeover("manual_override")
        return original_drive(scale_manual_throttle(throttle), enabled, deadman)

    def apply_safe_steering_v2(direction):
        if autonomous_active() or auto_orchestrator.active:
            if not bool(getattr(legacy, "manual_deadman_pressed", False)):
                return ignored_result()
            takeover("manual_override")
        return original_steering(direction)

    legacy.apply_safe_drive = apply_safe_drive_v2
    legacy.apply_safe_steering = apply_safe_steering_v2
    return {
        "installed": True,
        "policy": "linear_manual_throttle_ignore_without_deadman_takeover_with_deadman",
    }
