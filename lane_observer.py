"""Shared read-only UFLD observation cache for dashboard and RECORD.

This module deliberately has no motor/control authority. It serializes diagnostic
UFLD inference so the dashboard preview and RECORD writer can share one result
instead of running duplicate neural inference on the Raspberry Pi.
"""

from __future__ import annotations

import threading
import time


class SharedLaneObserver:
    def __init__(self):
        self._lock = threading.RLock()
        self._lane = None
        self._diagnostics = None
        self._frame_sequence = None
        self._frame_monotonic = None
        self._observed_monotonic = None

    def reset(self):
        with self._lock:
            self._lane = None
            self._diagnostics = None
            self._frame_sequence = None
            self._frame_monotonic = None
            self._observed_monotonic = None

    def _snapshot_locked(self, now=None):
        now = time.monotonic() if now is None else float(now)
        observer_age = (
            None
            if self._observed_monotonic is None
            else max(0.0, now - float(self._observed_monotonic))
        )
        frame_age = (
            None
            if self._frame_monotonic is None
            else max(0.0, now - float(self._frame_monotonic))
        )
        return {
            "lane": dict(self._lane or {}),
            "diagnostics": dict(self._diagnostics or {}),
            "frame_sequence": self._frame_sequence,
            "frame_monotonic": self._frame_monotonic,
            "observed_monotonic": self._observed_monotonic,
            "observer_age_seconds": observer_age,
            "frame_data_age_seconds": frame_age,
            "control_authority": "NONE",
        }

    def snapshot(self):
        with self._lock:
            return self._snapshot_locked()

    def observe(
        self,
        hybrid,
        frame,
        *,
        sequence=None,
        frame_monotonic=None,
        minimum_interval_seconds=0.45,
    ):
        """Return a cached or newly computed diagnostic UFLD lane observation."""
        interval = max(0.0, float(minimum_interval_seconds))
        now = time.monotonic()
        with self._lock:
            if self._lane is not None:
                if sequence is not None and sequence == self._frame_sequence:
                    return self._snapshot_locked(now)
                if (
                    self._observed_monotonic is not None
                    and now - float(self._observed_monotonic) < interval
                ):
                    return self._snapshot_locked(now)

            try:
                result = hybrid.analyze_neural_preview_jpeg(frame)
                lane = result.as_dict()
                diagnostics = dict(hybrid.preview_snapshot() or {})
            except Exception as error:
                lane = {
                    "detected": False,
                    "confidence": 0.0,
                    "backend": "UFLD_ONNX",
                    "marking": "NEURAL_PREVIEW",
                    "error": f"UFLD_OBSERVER_ERROR:{type(error).__name__}:{error}",
                }
                diagnostics = {
                    "backend": "UFLD_ONNX",
                    "error": lane["error"],
                }

            self._lane = dict(lane)
            self._diagnostics = dict(diagnostics)
            self._frame_sequence = sequence
            self._frame_monotonic = frame_monotonic
            self._observed_monotonic = time.monotonic()
            return self._snapshot_locked(self._observed_monotonic)


UFLD_LANE_OBSERVER = SharedLaneObserver()


__all__ = ["SharedLaneObserver", "UFLD_LANE_OBSERVER"]
