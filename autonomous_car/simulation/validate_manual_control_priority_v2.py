"""Regression for manual-control heartbeat priority over dashboard telemetry."""

import time
from pathlib import Path

from manual_control_hardening import ManualControlTimingMonitor


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    monitor = ManualControlTimingMonitor(0.30)
    first = monitor.note(True)
    _require(first["gap_seconds"] is None, first)
    with monitor.lock:
        monitor.last_arrival_monotonic = time.monotonic() - 0.31
    delayed = monitor.note(True)
    snap = monitor.snapshot()
    _require(delayed["watchdog_risk"] is True, delayed)
    _require(snap["gaps_over_watchdog"] == 1, snap)
    _require(snap["maximum_gap_seconds"] >= 0.30, snap)

    source = Path("manual_control_hardening.py").read_text(encoding="utf-8")
    config = Path("camera_stream/config.py").read_text(encoding="utf-8")
    preview = Path("lane_neural_preview.py").read_text(encoding="utf-8")

    _require("motorInFlight" in source and "motorPending" in source, "motor coalescer missing")
    _require("latestMotorFetch" in source, "latest-command-wins motor scheduler missing")
    _require("'/api/lidar':500" in source, "manual LiDAR pacing missing")
    _require("'/api/status':1500" in source, "manual status pacing missing")
    _require("/api/manual-control/timing" in source, "server timing endpoint missing")
    _require(
        'MOTOR_TIMEOUT_SECONDS = float(os.environ.get("MOTOR_TIMEOUT_SECONDS", "0.30"))'
        in config,
        "300 ms motor watchdog was loosened instead of fixing scheduling",
    )
    _require(
        "MANUAL_CONTROL_HARDENING" in preview,
        "primary operator dashboard does not load control-priority scheduler",
    )

    print("Manual control priority V2 regression: PASS")
    print(
        {
            "motor_watchdog_seconds": 0.30,
            "motor_http": "latest-command-wins",
            "dashboard_reads": "paced_during_manual_drive",
            "server_gap_metrics": True,
        }
    )


if __name__ == "__main__":
    main()
