import threading
import time

from .state import DriveMode


class InvalidStateTransition(ValueError):
    pass


_MANUAL_MODES = {DriveMode.MANUAL, DriveMode.MANUAL_ASSIST}
_AUTONOMOUS_MODES = {
    DriveMode.AUTO_AI,
    DriveMode.AUTO_GPS,
    DriveMode.AUTO_LOCAL,
    DriveMode.AUTO,
    # Legacy modes during migration.
    DriveMode.AUTO_ROUTE,
    DriveMode.AUTO_HYBRID,
}


class VehicleStateMachine:
    _ALLOWED_TRANSITIONS = {
        DriveMode.DISARMED: _MANUAL_MODES | _AUTONOMOUS_MODES,

        DriveMode.MANUAL: {
            DriveMode.DISARMED,
            DriveMode.RECORD,
        } | _AUTONOMOUS_MODES,
        DriveMode.MANUAL_ASSIST: {
            DriveMode.DISARMED,
            DriveMode.RECORD,
        } | _AUTONOMOUS_MODES,

        DriveMode.RECORD: _MANUAL_MODES | {DriveMode.DISARMED},

        DriveMode.AUTO_AI: _MANUAL_MODES | {
            DriveMode.DISARMED,
            DriveMode.AUTO,
        },
        DriveMode.AUTO_GPS: _MANUAL_MODES | {
            DriveMode.DISARMED,
            DriveMode.AUTO,
            DriveMode.AUTO_HYBRID,
        },
        DriveMode.AUTO_LOCAL: _MANUAL_MODES | {
            DriveMode.DISARMED,
            DriveMode.AUTO,
        },
        DriveMode.AUTO: _MANUAL_MODES | {
            DriveMode.DISARMED,
            DriveMode.AUTO_AI,
            DriveMode.AUTO_GPS,
            DriveMode.AUTO_LOCAL,
        },

        # Legacy transition support while server.py is migrated.
        DriveMode.AUTO_ROUTE: _MANUAL_MODES | {
            DriveMode.AUTO_HYBRID,
            DriveMode.AUTO_GPS,
            DriveMode.AUTO,
            DriveMode.DISARMED,
        },
        DriveMode.AUTO_HYBRID: _MANUAL_MODES | {
            DriveMode.AUTO_ROUTE,
            DriveMode.AUTO_GPS,
            DriveMode.AUTO,
            DriveMode.DISARMED,
        },

        DriveMode.EMERGENCY_STOP: {DriveMode.DISARMED},
        DriveMode.FAULT: {DriveMode.DISARMED},
    }

    def __init__(self):
        self._lock = threading.Lock()
        self._mode = DriveMode.DISARMED
        self._changed_at = time.monotonic()
        self._reason = "system_start"

    @property
    def mode(self):
        with self._lock:
            return self._mode

    def transition(self, target, reason=None):
        target = DriveMode(target)
        with self._lock:
            if target == self._mode:
                return self.snapshot_unlocked()
            if target in {DriveMode.EMERGENCY_STOP, DriveMode.FAULT}:
                pass
            elif target not in self._ALLOWED_TRANSITIONS[self._mode]:
                raise InvalidStateTransition(f"{self._mode.value} -> {target.value}")
            self._mode = target
            self._changed_at = time.monotonic()
            self._reason = reason
            return self.snapshot_unlocked()

    def snapshot_unlocked(self):
        return {
            "mode": self._mode.value,
            "canonical_mode": self._mode.canonical.value,
            "changed_at": self._changed_at,
            "reason": self._reason,
        }

    def snapshot(self):
        with self._lock:
            return self.snapshot_unlocked()
