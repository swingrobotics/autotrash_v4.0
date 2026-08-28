"""GNSS quality policy shared by AUTO_GPS dataset building and HMI previews."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import os


RTK_FIXED = "RTK FIXED"
RTK_FLOAT = "RTK FLOAT"
DGPS_FIX = "DGPS FIX"


def _finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _explicit_false(value):
    if isinstance(value, bool):
        return not value
    text = str(value or "").strip().lower()
    return text in {"0", "false", "no", "invalid", "bad"}


def normalize_gnss_status(value):
    """Normalize recorder/gpsd fix labels without treating plain GPS as DGPS."""
    raw = " ".join(str(value or "").strip().upper().replace("_", " ").split())
    if not raw:
        return "UNKNOWN"
    if "RTK" in raw and "FLOAT" in raw:
        return RTK_FLOAT
    if "RTK" in raw and ("FIXED" in raw or raw.endswith(" FIX")):
        return RTK_FIXED
    if (
        raw in {"DGPS", "DGNSS", "DIFFERENTIAL"}
        or (("DGPS" in raw or "DGNSS" in raw or "DIFFERENTIAL" in raw) and "FIX" in raw)
    ):
        return DGPS_FIX
    return raw


@dataclass(frozen=True)
class GpsTrainingQualityPolicy:
    """Quality policy for non-fixed GNSS samples used only in training.

    The normalized reference route and live AUTO_GPS runtime remain RTK FIXED
    only. RTK FLOAT requires a good reported HDOP plus route proximity. DGPS /
    DGNSS may contribute imitation labels even when the receiver does not report
    HDOP, provided the position remains close to the RTK-FIXED-derived route.
    """

    maximum_conditional_hdop: float = 1.5
    maximum_fixed_route_deviation_m: float = 3.0
    maximum_conditional_route_deviation_m: float = 1.5

    @classmethod
    def from_environment(cls, maximum_fixed_route_deviation_m=3.0):
        return cls(
            maximum_conditional_hdop=max(
                0.1,
                float(os.environ.get("GPS_AI_MAX_CONDITIONAL_HDOP", "1.5")),
            ),
            maximum_fixed_route_deviation_m=max(
                0.25,
                float(maximum_fixed_route_deviation_m),
            ),
            maximum_conditional_route_deviation_m=max(
                0.25,
                float(
                    os.environ.get(
                        "GPS_AI_MAX_CONDITIONAL_ROUTE_DEVIATION_M",
                        "1.5",
                    )
                ),
            ),
        )

    def as_dict(self):
        return {
            **asdict(self),
            "reference_route_fix_policy": "RTK_FIXED_ONLY",
            "runtime_fix_policy": "RTK_FIXED_ONLY",
            "training_fixed_status": RTK_FIXED,
            "training_conditional_statuses": [RTK_FLOAT, DGPS_FIX],
            "conditional_requirements": {
                RTK_FLOAT: [
                    "GNSS is not explicitly invalid",
                    "HDOP is present and within threshold",
                    "position is within conditional route-deviation threshold",
                ],
                DGPS_FIX: [
                    "GNSS is not explicitly invalid",
                    "HDOP is optional",
                    "position is within conditional route-deviation threshold",
                ],
            },
        }

    def evaluate_row(self, row):
        """Return normalized quality metadata before route-distance gating."""
        row = row or {}
        status = normalize_gnss_status(row.get("rtk_status") or row.get("fix"))
        hdop = _finite_float(row.get("hdop"))
        if _explicit_false(row.get("is_valid")):
            return {
                "accepted": False,
                "status": status,
                "tier": "REJECTED",
                "hdop": hdop,
                "reason": "GNSS_EXPLICITLY_INVALID",
            }
        if status == RTK_FIXED:
            return {
                "accepted": True,
                "status": status,
                "tier": "FIXED",
                "hdop": hdop,
                "reason": None,
            }
        if status == DGPS_FIX:
            # Some GNSS receivers/gpsd paths expose the differential-fix state
            # without a usable HDOP field. Do not discard those frames solely
            # because HDOP is absent; GpsDatasetBuilder still enforces the much
            # more important RTK-reference-route proximity gate afterwards.
            return {
                "accepted": True,
                "status": status,
                "tier": "CONDITIONAL",
                "hdop": hdop,
                "reason": None,
            }
        if status == RTK_FLOAT:
            if hdop is None:
                return {
                    "accepted": False,
                    "status": status,
                    "tier": "CONDITIONAL",
                    "hdop": None,
                    "reason": "CONDITIONAL_FIX_HDOP_MISSING",
                }
            if hdop > self.maximum_conditional_hdop:
                return {
                    "accepted": False,
                    "status": status,
                    "tier": "CONDITIONAL",
                    "hdop": hdop,
                    "reason": "CONDITIONAL_FIX_HDOP_TOO_HIGH",
                }
            return {
                "accepted": True,
                "status": status,
                "tier": "CONDITIONAL",
                "hdop": hdop,
                "reason": None,
            }
        return {
            "accepted": False,
            "status": status,
            "tier": "REJECTED",
            "hdop": hdop,
            "reason": "GNSS_FIX_QUALITY_UNSUPPORTED",
        }

    def route_deviation_limit_m(self, status):
        return (
            self.maximum_fixed_route_deviation_m
            if normalize_gnss_status(status) == RTK_FIXED
            else self.maximum_conditional_route_deviation_m
        )


__all__ = [
    "DGPS_FIX",
    "GpsTrainingQualityPolicy",
    "RTK_FIXED",
    "RTK_FLOAT",
    "normalize_gnss_status",
]
