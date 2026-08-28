from dataclasses import dataclass, field
from enum import Enum
import time
from typing import Any


class DriveMode(str, Enum):
    DISARMED = "DISARMED"

    # V2 user-facing modes
    MANUAL = "MANUAL"
    RECORD = "RECORD"
    AUTO_AI = "AUTO_AI"
    AUTO_GPS = "AUTO_GPS"
    AUTO_LOCAL = "AUTO_LOCAL"
    AUTO = "AUTO"

    # Legacy runtime names kept during migration so the current dashboard/server
    # can be upgraded incrementally without breaking existing field operation.
    MANUAL_ASSIST = "MANUAL_ASSIST"
    AUTO_ROUTE = "AUTO_ROUTE"
    AUTO_HYBRID = "AUTO_HYBRID"

    EMERGENCY_STOP = "EMERGENCY_STOP"
    FAULT = "FAULT"

    @property
    def canonical(self):
        return {
            DriveMode.MANUAL_ASSIST: DriveMode.MANUAL,
            DriveMode.AUTO_ROUTE: DriveMode.AUTO_GPS,
            DriveMode.AUTO_HYBRID: DriveMode.AUTO_GPS,
        }.get(self, self)


@dataclass(frozen=True)
class SensorStatus:
    value: Any = None
    timestamp: float | None = None
    is_valid: bool = False
    data_age: float | None = None
    error_code: str | None = None

    @classmethod
    def build(
        cls,
        value: Any,
        timestamp: float | None,
        is_valid: bool,
        error_code: str | None = None,
        now: float | None = None,
    ):
        current_time = time.time() if now is None else now
        age = None if timestamp is None else max(0.0, current_time - timestamp)
        return cls(value, timestamp, is_valid, age, error_code)

    def fresh(self, maximum_age: float) -> bool:
        return self.is_valid and self.data_age is not None and self.data_age <= maximum_age

    def as_dict(self):
        return {
            "value": self.value,
            "timestamp": self.timestamp,
            "is_valid": self.is_valid,
            "data_age": self.data_age,
            "error_code": self.error_code,
        }


@dataclass(frozen=True)
class ControlRequest:
    throttle: float = 0.0
    steering: float = 0.0
    enabled: bool = False
    deadman_pressed: bool = False
    source: str = "unknown"
    timestamp: float = field(default_factory=time.monotonic)


@dataclass(frozen=True)
class SafetyContext:
    mode: DriveMode
    arduino: SensorStatus
    lidar: SensorStatus
    steering: SensorStatus
    emergency_stop: bool = False
    camera_hazard: bool = False
    loop_delay_seconds: float = 0.0


@dataclass(frozen=True)
class SafetyDecision:
    requested_throttle: float
    requested_steering: float
    final_throttle: float
    final_steering: float
    allowed: bool
    stop_reason: str | None = None
    obstacle_distance_m: float | None = None
    throttle_limit: float = 1.0

    def as_dict(self):
        return {
            "requested_throttle": self.requested_throttle,
            "requested_steering": self.requested_steering,
            "final_throttle": self.final_throttle,
            "final_steering": self.final_steering,
            "allowed": self.allowed,
            "stop_reason": self.stop_reason,
            "obstacle_detected": self.obstacle_distance_m is not None,
            "obstacle_distance_m": self.obstacle_distance_m,
            "throttle_limit": self.throttle_limit,
        }
