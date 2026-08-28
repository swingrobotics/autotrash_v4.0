"""Read-only external UFLD preview endpoint for the operator dashboard.

The preview is diagnostic-only and is allowed while stationary or in manual
operation. RECORD explicitly rejects the endpoint: recording is capture-only and
UFLD analysis happens after the session on the Compute Worker.
"""

from __future__ import annotations

import os

import server_v2_release as release
from autonomous_car import DriveMode
from lane_observer import UFLD_LANE_OBSERVER
from manual_control_hardening import (
    MANUAL_CONTROL_HARDENING,
    install_manual_control_priority,
)
from manual_perception_priority import install_manual_perception_priority
from record_storage_runtime import install_record_storage_runtime


# Install removable RECORD storage, control-priority scheduling and the
# mode-aware perception gate before sensors/server threads start.
RECORD_STORAGE = install_record_storage_runtime(release.full.legacy)
MANUAL_PERCEPTION_PRIORITY = install_manual_perception_priority(release.full.legacy)
MANUAL_CONTROL_TIMING = install_manual_control_priority(release.full.legacy)
if b'id="manual-control-priority-hardening"' not in release.full.legacy.INDEX_HTML:
    release.full.legacy.INDEX_HTML = release.full.legacy.INDEX_HTML.replace(
        b"</body>", MANUAL_CONTROL_HARDENING + b"</body>", 1
    )


_INSTALLED = False


def _observer_period_seconds():
    try:
        value = float(os.environ.get("SWING_UFLD_OBSERVER_PERIOD_SECONDS", "0.50"))
    except (TypeError, ValueError):
        value = 0.50
    return max(0.20, min(2.0, value))


def _preview_allowed_modes():
    modes = {DriveMode.DISARMED, DriveMode.MANUAL_ASSIST}
    manual = getattr(DriveMode, "MANUAL", None)
    if manual is not None:
        modes.add(manual)
    return modes


def install_lane_neural_preview_endpoint():
    """Patch the legacy GET handler with a diagnostic-only neural lane route."""
    global _INSTALLED
    if _INSTALLED:
        return

    handler = release.full.legacy.CameraHandler
    original_do_get = handler.do_GET
    if getattr(original_do_get, "_swing_neural_preview", False):
        _INSTALLED = True
        return

    def do_get_with_neural_preview(self):
        path = str(self.path or "").split("?", 1)[0]
        if path != "/api/lane/neural-preview":
            return original_do_get(self)

        mode = release.full.legacy.vehicle_state_machine.mode
        if mode == DriveMode.RECORD:
            self._send_json(
                {
                    "error": "NEURAL_PREVIEW_DISABLED_DURING_RECORD",
                    "mode": getattr(mode, "value", str(mode)),
                    "preview": True,
                    "control_authority": "NONE",
                    "record_policy": "OFFLINE_UFLD_ANALYSIS_ONLY",
                },
                409,
            )
            return
        if mode not in _preview_allowed_modes():
            self._send_json(
                {
                    "error": "NEURAL_PREVIEW_REQUIRES_MANUAL_OR_DISARMED",
                    "mode": getattr(mode, "value", str(mode)),
                    "preview": True,
                    "control_authority": "NONE",
                },
                409,
            )
            return

        hybrid = getattr(release.full, "HYBRID_LANE_CONTROLLER", None)
        if hybrid is None:
            self._send_json(
                {
                    "error": "NEURAL_PREVIEW_RUNTIME_UNAVAILABLE",
                    "preview": True,
                    "control_authority": "NONE",
                },
                503,
            )
            return

        try:
            frame, sequence, frame_monotonic, _ = (
                release.full.legacy.camera.snapshot_frame()
            )
            observation = UFLD_LANE_OBSERVER.observe(
                hybrid,
                frame,
                sequence=sequence,
                frame_monotonic=frame_monotonic,
                minimum_interval_seconds=_observer_period_seconds(),
            )
            payload = dict(observation.get("lane") or {})
            payload["preview"] = True
            payload["control_authority"] = "NONE"
            payload["frame_sequence"] = observation.get("frame_sequence")
            payload["data_age"] = observation.get("frame_data_age_seconds")
            diagnostics = dict(observation.get("diagnostics") or {})
            diagnostics["observer_age_seconds"] = observation.get(
                "observer_age_seconds"
            )
            diagnostics["frame_data_age_seconds"] = observation.get(
                "frame_data_age_seconds"
            )
            diagnostics["shared_with_record"] = False
            diagnostics["record_policy"] = "OFFLINE_UFLD_ANALYSIS_ONLY"
            payload["preview_diagnostics"] = diagnostics
            self._send_json(payload)
        except Exception as error:
            self._send_json(
                {
                    "detected": False,
                    "confidence": 0.0,
                    "backend": "UFLD_ONNX",
                    "error": f"NEURAL_PREVIEW_ERROR:{type(error).__name__}:{error}",
                    "preview": True,
                    "control_authority": "NONE",
                },
                200,
            )

    do_get_with_neural_preview._swing_neural_preview = True
    handler.do_GET = do_get_with_neural_preview
    _INSTALLED = True


__all__ = ["install_lane_neural_preview_endpoint"]
