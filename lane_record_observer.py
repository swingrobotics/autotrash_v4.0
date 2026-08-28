"""Compatibility hook for the retired live-UFLD-during-RECORD path.

Human RECORD is now deliberately capture-only: camera, timestamps, operator
controls, vehicle state and raw sensors are recorded with control latency as the
highest priority. UFLD is run after the session on the Compute Worker so a slow
or disconnected Worker can never stall manual driving.

Keep ``install_lane_record_observer`` as a no-op because older integration code
and regression tooling may still import it.
"""

from __future__ import annotations


LIVE_RECORD_UFLD_ENABLED = False
RECORD_PERCEPTION_POLICY = "OFFLINE_UFLD_ANALYSIS_ONLY"


def install_lane_record_observer():
    """Mark the compatibility hook installed without wrapping RECORD samples."""
    try:
        import server_v2_release as release
        release._ufld_record_observer_installed = True
        release._ufld_record_observer_policy = RECORD_PERCEPTION_POLICY
    except Exception:
        # Importing this compatibility module must never make the vehicle server
        # unavailable. There is intentionally no live inference to initialize.
        pass
    return True


__all__ = [
    "LIVE_RECORD_UFLD_ENABLED",
    "RECORD_PERCEPTION_POLICY",
    "install_lane_record_observer",
]
