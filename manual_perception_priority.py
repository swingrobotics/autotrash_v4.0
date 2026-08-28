"""Keep expensive person detection off the manual/RECORD control path."""

from __future__ import annotations

import threading
import time
import types

from autonomous_car import DriveMode
from autonomous_car.mode_policy import policy_for


_INSTALLED = False


def install_manual_perception_priority(legacy):
    """Run person detection only when the current mode policy can consume it."""
    global _INSTALLED
    if _INSTALLED:
        return True

    monitor = legacy.perception_monitor

    def perception_run(self):
        last_sequence = -1
        while True:
            mode = legacy.vehicle_state_machine.mode
            try:
                enabled = bool(policy_for(mode).person_stop)
            except Exception:
                enabled = False
            if not enabled:
                # MANUAL, MANUAL_ASSIST, RECORD and DISARMED do not use camera
                # person detection in SafetySupervisor. Publish an explicit idle
                # state and leave Pi CPU to joystick/control/recording.
                with self.lock:
                    self.state.update(
                        detections=[],
                        hazard=False,
                        error=None,
                        suspended=True,
                        suspended_reason="MODE_POLICY_PERSON_STOP_DISABLED",
                    )
                last_sequence = -1
                time.sleep(0.5)
                continue

            time.sleep(0.25)
            try:
                frame, sequence, _, _ = legacy.camera.snapshot_frame()
                if frame is None or sequence == last_sequence:
                    continue
                last_sequence = sequence
                detections = self.detector.detect_people(frame)
                fused = self.detector.fuse_lidar(
                    detections,
                    legacy.lidar_monitor.snapshot().get("points", []),
                )
                with self.lock:
                    self.state.update(
                        detections=[item.as_dict() for item in fused],
                        hazard=any(item.in_vehicle_path for item in fused),
                        last_update=time.time(),
                        frame_sequence=sequence,
                        error=None,
                        suspended=False,
                        suspended_reason=None,
                    )
            except Exception as error:
                with self.lock:
                    self.state.update(
                        hazard=False,
                        error=str(error),
                        suspended=False,
                        suspended_reason=None,
                    )

    monitor._run = types.MethodType(perception_run, monitor)
    monitor.state.setdefault("suspended", False)
    monitor.state.setdefault("suspended_reason", None)
    _INSTALLED = True
    return True


__all__ = ["install_manual_perception_priority"]
