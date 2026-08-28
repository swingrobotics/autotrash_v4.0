from dataclasses import dataclass

from .state import DriveMode


@dataclass(frozen=True)
class ModePolicy:
    """Behavior switches for each operating mode.

    Driving assistance and hard safety are intentionally separate. Hard safety
    (E-stop, command watchdog, Arduino availability, steering limits/freshness)
    stays active in every drive-capable mode. The switches below only control
    assistance/perception layers and sensors required by the selected driving
    strategy.
    """

    driver_controlled: bool = False
    records_data: bool = False
    learned_driving: bool = False
    gps_navigation: bool = False
    local_navigation: bool = False
    automatic_selector: bool = False

    lane_assist: bool = False
    local_avoidance: bool = False
    person_stop: bool = False
    obstacle_stop_fallback: bool = False

    require_deadman: bool = False
    require_lidar: bool = False


_POLICIES = {
    DriveMode.DISARMED: ModePolicy(),
    # MANUAL/RECORD are explicitly selected from the V2 drive-mode UI. Once the
    # operator has selected/started a human-driven mode, the gamepad A button is
    # not an additional deadman gate. E-stop, command watchdog, Arduino/steering
    # health and output limits remain hard-safety requirements.
    DriveMode.MANUAL: ModePolicy(
        driver_controlled=True,
    ),
    DriveMode.MANUAL_ASSIST: ModePolicy(
        driver_controlled=True,
    ),
    DriveMode.RECORD: ModePolicy(
        driver_controlled=True,
        records_data=True,
    ),
    DriveMode.AUTO_AI: ModePolicy(
        learned_driving=True,
        person_stop=True,
        require_lidar=True,
    ),
    DriveMode.AUTO_GPS: ModePolicy(
        learned_driving=True,
        gps_navigation=True,
        person_stop=True,
        # AUTO_GPS is now a GPS-conditioned learned driving policy. Ordinary
        # obstacle behavior must come from the learned model, exactly as in
        # AUTO_AI; only person STOP and hard safety remain external.
        require_lidar=True,
    ),
    # Legacy route controllers retain their original deterministic behavior
    # during migration/testing, but they are no longer the V2 AUTO_GPS policy.
    DriveMode.AUTO_ROUTE: ModePolicy(
        gps_navigation=True,
        lane_assist=False,
        local_avoidance=True,
        person_stop=True,
        obstacle_stop_fallback=True,
        require_lidar=True,
    ),
    DriveMode.AUTO_HYBRID: ModePolicy(
        gps_navigation=True,
        lane_assist=True,
        local_avoidance=True,
        person_stop=True,
        obstacle_stop_fallback=True,
        require_lidar=True,
    ),
    DriveMode.AUTO_LOCAL: ModePolicy(
        local_navigation=True,
        lane_assist=True,
        local_avoidance=True,
        person_stop=True,
        obstacle_stop_fallback=True,
        require_lidar=True,
    ),
    DriveMode.AUTO: ModePolicy(
        automatic_selector=True,
        lane_assist=True,
        local_avoidance=True,
        person_stop=True,
        obstacle_stop_fallback=True,
        require_lidar=True,
    ),
    DriveMode.EMERGENCY_STOP: ModePolicy(),
    DriveMode.FAULT: ModePolicy(),
}


def policy_for(mode) -> ModePolicy:
    return _POLICIES[DriveMode(mode)]
