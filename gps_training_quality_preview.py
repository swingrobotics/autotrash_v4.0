"""Rover-side GNSS quality preview for AUTO_GPS training selection."""

from __future__ import annotations

import bisect
import csv
import json
import math
import os
from pathlib import Path

import server_v2_release as release
from autonomous_car.ai.gps_quality import GpsTrainingQualityPolicy, normalize_gnss_status
from autonomous_car.routes import GpsRouteFeatureExtractor, NormalizedGpsRoute


_INSTALLED = False
MAXIMUM_GNSS_SKEW_SECONDS = 0.20


def _safe_leaf(value, label="VALUE"):
    raw = str(value or "").strip()
    name = os.path.basename(raw)
    if not raw or raw != name or name in {".", ".."}:
        raise ValueError(f"INVALID_{label}")
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.")
    if any(character not in allowed for character in raw):
        raise ValueError(f"INVALID_{label}")
    return raw


def _routes_root():
    return Path(
        os.environ.get(
            "AUTONOMY_GPS_ROUTES_PATH",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "gps-routes"),
        )
    ).resolve()


def _route_path(route_id):
    route_id = _safe_leaf(route_id, "GPS_ROUTE")
    root = _routes_root()
    path = (root / f"{route_id}.json").resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("GPS_ROUTE_PATH_ESCAPE") from error
    if not path.is_file():
        raise FileNotFoundError(f"GPS route not found: {route_id}")
    return path


def _float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _read_timed_csv(path):
    rows = []
    with open(path, "r", encoding="utf-8", newline="") as file:
        for row in csv.DictReader(file):
            timestamp = _float(row.get("monotonic"))
            if timestamp is None:
                continue
            row["_timestamp"] = timestamp
            rows.append(row)
    rows.sort(key=lambda row: row["_timestamp"])
    return rows


def _nearest(times, rows, timestamp):
    if not times:
        return None, math.inf
    position = bisect.bisect_left(times, timestamp)
    candidates = []
    if position < len(times):
        candidates.append(position)
    if position > 0:
        candidates.append(position - 1)
    if not candidates:
        return None, math.inf
    best = min(candidates, key=lambda index: abs(times[index] - timestamp))
    return rows[best], abs(times[best] - timestamp)


def _increment(mapping, key):
    mapping[key] = int(mapping.get(key) or 0) + 1


def _session_preview(session_name, session_path, route, policy):
    gnss_path = session_path / "gnss.csv"
    camera_path = session_path / "camera_timestamps.csv"
    if not gnss_path.is_file():
        raise FileNotFoundError(f"{session_name}: gnss.csv not found")
    if not camera_path.is_file():
        raise FileNotFoundError(f"{session_name}: camera_timestamps.csv not found")

    gnss_rows = _read_timed_csv(gnss_path)
    camera_rows = _read_timed_csv(camera_path)
    times = [row["_timestamp"] for row in gnss_rows]
    raw_status_counts = {}
    for row in gnss_rows:
        _increment(
            raw_status_counts,
            normalize_gnss_status(row.get("rtk_status") or row.get("fix")),
        )

    extractor = GpsRouteFeatureExtractor(route)
    matched_status_counts = {}
    eligible_by_status = {}
    rejected_by_reason = {}
    matched_frames = 0
    eligible_frames = 0
    conditional_eligible_frames = 0
    previous_index = None

    for camera_row in camera_rows:
        timestamp = camera_row["_timestamp"]
        gnss_row, skew = _nearest(times, gnss_rows, timestamp)
        if gnss_row is None or skew > MAXIMUM_GNSS_SKEW_SECONDS:
            _increment(rejected_by_reason, "GNSS_NOT_SYNCHRONIZED")
            continue
        matched_frames += 1
        quality = policy.evaluate_row(gnss_row)
        _increment(matched_status_counts, quality["status"])
        if not quality["accepted"]:
            _increment(rejected_by_reason, quality["reason"])
            continue

        latitude = _float(gnss_row.get("latitude"))
        longitude = _float(gnss_row.get("longitude"))
        if latitude is None or longitude is None:
            _increment(rejected_by_reason, "GNSS_POSITION_MISSING")
            continue
        try:
            route_features = extractor.extract(
                latitude,
                longitude,
                0.0,
                previous_index,
            )
        except (TypeError, ValueError, OverflowError):
            _increment(rejected_by_reason, "GPS_ROUTE_PROJECTION_FAILED")
            continue
        previous_index = route_features.nearest_index
        route_limit = policy.route_deviation_limit_m(quality["status"])
        if abs(route_features.cross_track_error_m) > route_limit:
            _increment(
                rejected_by_reason,
                "TOO_FAR_FROM_NORMALIZED_ROUTE"
                if quality["tier"] == "FIXED"
                else "CONDITIONAL_FIX_TOO_FAR_FROM_ROUTE",
            )
            continue
        eligible_frames += 1
        if quality["tier"] == "CONDITIONAL":
            conditional_eligible_frames += 1
        _increment(eligible_by_status, quality["status"])

    total_frames = len(camera_rows)
    raw_total = len(gnss_rows)
    fixed_raw = int(raw_status_counts.get("RTK FIXED") or 0)
    return {
        "session": session_name,
        "camera_frames": total_frames,
        "gnss_rows": raw_total,
        "raw_status_counts": raw_status_counts,
        "rtk_fixed_raw_ratio": fixed_raw / raw_total if raw_total else 0.0,
        "gnss_matched_frames": matched_frames,
        "matched_status_counts": matched_status_counts,
        "eligible_frames": eligible_frames,
        "conditional_eligible_frames": conditional_eligible_frames,
        "excluded_frames": max(0, total_frames - eligible_frames),
        "eligibility_ratio": eligible_frames / total_frames if total_frames else 0.0,
        "eligible_by_status": eligible_by_status,
        "rejected_by_reason": rejected_by_reason,
    }


def _quality_preview(sessions, route_id):
    route_id = _safe_leaf(route_id, "GPS_ROUTE")
    normalized_sessions = list(
        dict.fromkeys(_safe_leaf(value, "SESSION") for value in sessions or [])
    )
    if not normalized_sessions:
        raise ValueError("GPS_TRAINING_SESSIONS_REQUIRED")
    resolver = getattr(release.full.legacy, "recording_session_path", None)
    if not callable(resolver):
        raise RuntimeError("RECORDING_SESSION_RESOLVER_UNAVAILABLE")

    route = NormalizedGpsRoute.load(_route_path(route_id))
    policy = GpsTrainingQualityPolicy.from_environment(3.0)
    previews = []
    for session in normalized_sessions:
        path = Path(resolver(session)).resolve()
        if not path.is_dir() or path.name != session:
            raise FileNotFoundError(f"RECORD session not found: {session}")
        previews.append(_session_preview(session, path, route, policy))

    aggregate = {
        "camera_frames": 0,
        "gnss_rows": 0,
        "gnss_matched_frames": 0,
        "eligible_frames": 0,
        "conditional_eligible_frames": 0,
        "excluded_frames": 0,
        "raw_status_counts": {},
        "matched_status_counts": {},
        "eligible_by_status": {},
        "rejected_by_reason": {},
    }
    for item in previews:
        for key in (
            "camera_frames",
            "gnss_rows",
            "gnss_matched_frames",
            "eligible_frames",
            "conditional_eligible_frames",
            "excluded_frames",
        ):
            aggregate[key] += int(item.get(key) or 0)
        for key in (
            "raw_status_counts",
            "matched_status_counts",
            "eligible_by_status",
            "rejected_by_reason",
        ):
            for name, count in (item.get(key) or {}).items():
                aggregate[key][name] = int(aggregate[key].get(name) or 0) + int(count)
    total_frames = aggregate["camera_frames"]
    raw_total = aggregate["gnss_rows"]
    aggregate["eligibility_ratio"] = (
        aggregate["eligible_frames"] / total_frames if total_frames else 0.0
    )
    aggregate["rtk_fixed_raw_ratio"] = (
        int(aggregate["raw_status_counts"].get("RTK FIXED") or 0) / raw_total
        if raw_total
        else 0.0
    )
    return {
        "schema": "gps_training_quality_preview_v1",
        "route_id": route_id,
        "route_fix_policy": "RTK_FIXED_ONLY",
        "runtime_fix_policy": "RTK_FIXED_ONLY",
        "training_policy": policy.as_dict(),
        "note": (
            "Eligible frame counts apply GNSS sync, fix/HDOP and route-distance checks. "
            "Final dataset acceptance can be lower after camera/LiDAR/IMU/label purity checks."
        ),
        "aggregate": aggregate,
        "sessions": previews,
    }


def install_gps_training_quality_preview():
    global _INSTALLED
    if _INSTALLED:
        return True
    original = release.ReleaseHandler

    class GpsTrainingQualityPreviewHandler(original):
        def do_POST(self):
            path = str(self.path or "").split("?", 1)[0]
            if path != "/api/v2/compute/gps-quality":
                super().do_POST()
                return
            try:
                payload = self._read_json()
                self._send_json(
                    _quality_preview(
                        payload.get("sessions"),
                        payload.get("route_id"),
                    )
                )
            except (
                ValueError,
                OSError,
                RuntimeError,
                TypeError,
                json.JSONDecodeError,
            ) as error:
                self._send_json({"error": str(error)}, 409)

    release.ReleaseHandler = GpsTrainingQualityPreviewHandler
    _INSTALLED = True
    return True


__all__ = ["install_gps_training_quality_preview"]
