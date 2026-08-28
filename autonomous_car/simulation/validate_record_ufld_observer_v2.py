"""Regression for capture-only human RECORD and offline UFLD analysis."""

from pathlib import Path

from lane_record_observer import (
    LIVE_RECORD_UFLD_ENABLED,
    RECORD_PERCEPTION_POLICY,
    install_lane_record_observer,
)


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    _require(LIVE_RECORD_UFLD_ENABLED is False, LIVE_RECORD_UFLD_ENABLED)
    _require(
        RECORD_PERCEPTION_POLICY == "OFFLINE_UFLD_ANALYSIS_ONLY",
        RECORD_PERCEPTION_POLICY,
    )
    _require(install_lane_record_observer() is True, "compatibility hook failed")

    preview = Path("lane_neural_preview.py").read_text(encoding="utf-8")
    record = Path("lane_record_observer.py").read_text(encoding="utf-8")
    offline = Path("record_replay_ufld.py").read_text(encoding="utf-8")
    replay_hmi = Path("record_replay_auto_hmi.py").read_text(encoding="utf-8")
    candidate_hmi = Path("record_replay_candidate_overlay_hmi.py").read_text(
        encoding="utf-8"
    )
    manager = Path("autonomous_car/recording/record_manager.py").read_text(
        encoding="utf-8"
    )

    _require(
        "NEURAL_PREVIEW_DISABLED_DURING_RECORD" in preview,
        "dashboard UFLD preview is not rejected during RECORD",
    )
    _require(
        "install_lane_record_observer" not in preview,
        "preview endpoint still installs a live RECORD observer",
    )
    _require(
        "UFLD_LANE_OBSERVER.observe" not in record,
        "RECORD compatibility hook can still execute UFLD inference",
    )
    _require(
        "analyze_neural_preview_jpeg" not in record,
        "RECORD compatibility hook can still execute neural preview",
    )
    _require(
        'live_record_ufld=False' in manager,
        "RECORD metadata does not publish the capture-only UFLD policy",
    )
    _require(
        "Offline UFLD analysis for finalized RECORD videos" in offline,
        "offline UFLD replay analysis path is missing",
    )
    _require(
        "camera_timestamps.csv" in offline,
        "offline UFLD analysis is not aligned to real camera time",
    )
    _require(
        "/api/recordings/ufld-analysis" in replay_hmi,
        "replay overlay is not reading the offline UFLD sidecar directly",
    )
    _require(
        "pendingReplayOffset" in replay_hmi and "Promise.allSettled" in replay_hmi,
        "replay UFLD synchronization is not latest-offset-wins",
    )
    _require(
        "if(ufld.available)row=ufld.row||null" in replay_hmi,
        "offline UFLD row does not override stale RECORD perception",
    )
    _require(
        "else if(!ufld.native_ufld)row=null" in replay_hmi,
        "stale non-UFLD RECORD perception can still be drawn as UFLD",
    )
    _require(
        "observeReplayClock" in candidate_hmi
        and "setInterval(()=>observeReplayClock(false),120)" in candidate_hmi,
        "JPEG fallback does not synchronize raw UFLD candidates to programmatic replay time",
    )
    _require(
        "/api/recordings/ufld-analysis" in candidate_hmi,
        "candidate overlay is not reading the offline UFLD sidecar",
    )

    print("RECORD capture-only UFLD V2 regression: PASS")
    print(
        {
            "live_record_ufld": False,
            "policy": RECORD_PERCEPTION_POLICY,
            "offline_reanalysis": True,
            "replay_sidecar_sync": "latest_offset_wins",
            "jpeg_candidate_sync": "replay_clock_observer",
        }
    )


if __name__ == "__main__":
    main()
