from dataclasses import dataclass

from autonomous_car.state import DriveMode


@dataclass(frozen=True)
class AutoCapabilities:
    gps_ready: bool = False
    local_map_id: str | None = None
    local_localization_ready: bool = False
    ai_model_id: str | None = None
    ai_model_validated: bool = False
    ai_environment_match: bool = False


@dataclass(frozen=True)
class AutoSelection:
    ready: bool
    target_mode: DriveMode | None
    reason: str
    resource_id: str | None = None


class AutoModeSelector:
    """Selects the safest available autonomous strategy.

    Priority is intentionally deterministic:
    1. GPS/RTK navigation when the route/preflight is ready.
    2. LOCAL navigation when a saved map can localize the vehicle.
    3. AI only when the model is validated and matches the current environment.
    4. Otherwise refuse autonomous motion.
    """

    def select(self, capabilities: AutoCapabilities) -> AutoSelection:
        if capabilities.gps_ready:
            return AutoSelection(
                True,
                DriveMode.AUTO_GPS,
                "gps_ready",
            )

        if capabilities.local_map_id and capabilities.local_localization_ready:
            return AutoSelection(
                True,
                DriveMode.AUTO_LOCAL,
                "local_map_localized",
                capabilities.local_map_id,
            )

        if (
            capabilities.ai_model_id
            and capabilities.ai_model_validated
            and capabilities.ai_environment_match
        ):
            return AutoSelection(
                True,
                DriveMode.AUTO_AI,
                "validated_ai_environment_match",
                capabilities.ai_model_id,
            )

        return AutoSelection(
            False,
            None,
            "no_safe_autonomous_strategy",
        )
