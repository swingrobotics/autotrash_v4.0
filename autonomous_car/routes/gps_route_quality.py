"""GNSS-quality-aware normalized route builder.

RTK FIXED remains preferred. When a RECORD session does not contain enough RTK
FIXED samples, DGPS/DGNSS samples may be used as a fallback so AUTO_GPS training
can still proceed. Live AUTO_GPS runtime safety remains RTK FIXED only.
"""

from __future__ import annotations

import csv
import json
import math
import os

from .gps_route import GpsRouteNormalizer as _StrictGpsRouteNormalizer


RTK_FIXED = "RTK FIXED"
DGPS_FIX = "DGPS FIX"


def _normalize_fix(value):
    raw = " ".join(str(value or "").strip().upper().replace("_", " ").split())
    if not raw:
        return "UNKNOWN"
    if "RTK" in raw and ("FIXED" in raw or raw.endswith(" FIX")):
        return RTK_FIXED
    if (
        raw in {"DGPS", "DGNSS", "DIFFERENTIAL"}
        or (("DGPS" in raw or "DGNSS" in raw or "DIFFERENTIAL" in raw) and "FIX" in raw)
    ):
        return DGPS_FIX
    return raw


def _explicit_false(value):
    if isinstance(value, bool):
        return not value
    return str(value or "").strip().lower() in {"0", "false", "no", "invalid", "bad"}


class GpsRouteNormalizer(_StrictGpsRouteNormalizer):
    """Prefer RTK FIXED route points and fall back to DGPS when necessary."""

    def build(self, recordings_root, session_names, route_id, output_path=None):
        self._source_fix_policies = {}
        route = super().build(recordings_root, session_names, route_id, output_path=None)
        policies = dict(self._source_fix_policies)
        contains_fallback = any(
            value != "RTK_FIXED_ONLY" for value in policies.values()
        )
        route.quality["source_fix_policies"] = policies
        route.quality["contains_dgps_fallback"] = contains_fallback
        route.quality["reference_fix_policy"] = (
            "PREFER_RTK_FIXED_DGPS_FALLBACK"
            if contains_fallback
            else "RTK_FIXED_ONLY"
        )
        if output_path:
            route.save(output_path)
        return route

    def _read_session(self, root, session):
        path = os.path.join(root, session)
        metadata_path = os.path.join(path, "metadata.json")
        if os.path.isfile(metadata_path):
            with open(metadata_path, "r", encoding="utf-8") as file:
                metadata = json.load(file)
            if metadata.get("record_gps") is False:
                raise ValueError(f"{session}: GPS recording was disabled")
            if str(metadata.get("purpose") or "RECORD").upper().startswith("AUTO"):
                raise ValueError(
                    f"{session}: autonomous run cannot define a human reference route"
                )

        gnss_path = os.path.join(path, "gnss.csv")
        if not os.path.isfile(gnss_path):
            raise FileNotFoundError(f"{session}: gnss.csv not found")

        fixed = []
        dgps = []
        with open(gnss_path, "r", encoding="utf-8", newline="") as file:
            for row in csv.DictReader(file):
                if _explicit_false(row.get("is_valid")):
                    continue
                status = _normalize_fix(row.get("rtk_status") or row.get("fix"))
                if status not in {RTK_FIXED, DGPS_FIX}:
                    continue
                try:
                    values = (
                        float(row["latitude"]),
                        float(row["longitude"]),
                        float(row.get("altitude_m") or 0.0),
                        float(row.get("speed_mps") or 0.0),
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                if not all(math.isfinite(value) for value in values):
                    continue
                (fixed if status == RTK_FIXED else dgps).append(values)

        minimum = self.minimum_fixed_samples
        if len(fixed) >= minimum:
            self._source_fix_policies[session] = "RTK_FIXED_ONLY"
            return fixed
        if len(dgps) >= minimum:
            self._source_fix_policies[session] = "DGPS_FALLBACK"
            return dgps
        if len(fixed) + len(dgps) >= minimum:
            self._source_fix_policies[session] = "RTK_FIXED_PLUS_DGPS_FALLBACK"
            return fixed + dgps

        raise ValueError(
            f"{session}: expected at least {minimum} RTK FIXED or DGPS samples; "
            f"got RTK FIXED={len(fixed)}, DGPS={len(dgps)}"
        )


__all__ = ["GpsRouteNormalizer"]
