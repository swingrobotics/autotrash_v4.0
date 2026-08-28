"""Read-only normalized GPS route preview endpoint for the operator HMI."""

from __future__ import annotations

import math
import os
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import server_v2_release as release
from autonomous_car.routes import NormalizedGpsRoute


_INSTALLED = False
_EARTH_RADIUS_M = 6378137.0
_MAX_PREVIEW_POINTS = 600


def _safe_route_id(value):
    route_id = str(value or "").strip()
    if not route_id or os.path.basename(route_id) != route_id or route_id in {".", ".."}:
        raise ValueError("INVALID_GPS_ROUTE_ID")
    allowed = set("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.")
    if any(character not in allowed for character in route_id):
        raise ValueError("INVALID_GPS_ROUTE_ID")
    return route_id


def _routes_root():
    return Path(
        os.environ.get(
            "AUTONOMY_GPS_ROUTES_PATH",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "gps-routes"),
        )
    ).resolve()


def _route_path(route_id):
    route_id = _safe_route_id(route_id)
    root = _routes_root()
    path = (root / f"{route_id}.json").resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("GPS_ROUTE_PATH_ESCAPE") from error
    if not path.is_file():
        raise FileNotFoundError(f"GPS_ROUTE_NOT_FOUND:{route_id}")
    return path


def _sample_indices(count, maximum):
    if count <= maximum:
        return list(range(count))
    if maximum <= 1:
        return [0]
    result = []
    for index in range(maximum):
        source = int(round(index * (count - 1) / (maximum - 1)))
        if not result or source != result[-1]:
            result.append(source)
    if result[-1] != count - 1:
        result.append(count - 1)
    return result


def route_preview(route_id):
    route = NormalizedGpsRoute.load(_route_path(route_id))
    origin_latitude = float(route.origin["origin_latitude"])
    origin_longitude = float(route.origin["origin_longitude"])
    latitude_radians = math.radians(origin_latitude)
    longitude_scale = _EARTH_RADIUS_M * max(1e-9, abs(math.cos(latitude_radians)))

    points = []
    for index in _sample_indices(len(route.points), _MAX_PREVIEW_POINTS):
        point = route.points[index]
        latitude = origin_latitude + math.degrees(float(point.y) / _EARTH_RADIUS_M)
        longitude = origin_longitude + math.degrees(float(point.x) / longitude_scale)
        points.append(
            {
                "index": index,
                "x_m": float(point.x),
                "y_m": float(point.y),
                "latitude": latitude,
                "longitude": longitude,
                "speed_mps": None if point.speed_mps is None else float(point.speed_mps),
            }
        )

    return {
        "schema": "swing_gps_route_preview_v1",
        "route_id": route.route_id,
        "point_count": len(route.points),
        "preview_point_count": len(points),
        "source_sessions": list(route.source_sessions),
        "quality": dict(route.quality),
        "origin": dict(route.origin),
        "points": points,
        "display_contract": {
            "meaning": "normalized reference route expected by the selected AUTO_GPS model",
            "not_a_model_rollout": True,
        },
    }


def install_gps_route_preview_api():
    global _INSTALLED
    if _INSTALLED:
        return True

    original = release.ReleaseHandler

    class GpsRoutePreviewHandler(original):
        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path != "/api/v2/gps-ai/route-preview":
                super().do_GET()
                return
            try:
                query = parse_qs(parsed.query)
                route_id = query.get("route_id", [""])[0]
                self._send_json(route_preview(route_id))
            except (ValueError, OSError, TypeError, KeyError) as error:
                self._send_json({"error": str(error)}, 404)

    release.ReleaseHandler = GpsRoutePreviewHandler
    _INSTALLED = True
    return True


__all__ = ["install_gps_route_preview_api", "route_preview"]
