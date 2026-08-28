import copy
import os
import threading
import time


class TimedSnapshotCache:
    def __init__(self, provider, ttl_seconds=0.35):
        self.provider = provider
        self.ttl_seconds = max(0.0, float(ttl_seconds))
        self._lock = threading.Lock()
        self._expires_at = 0.0
        self._value = None

    def invalidate(self):
        with self._lock:
            self._expires_at = 0.0
            self._value = None

    def get(self):
        now = time.monotonic()
        with self._lock:
            if self._value is not None and now < self._expires_at:
                return copy.deepcopy(self._value)
        value = self.provider()
        with self._lock:
            self._value = copy.deepcopy(value)
            self._expires_at = time.monotonic() + self.ttl_seconds
        return value


def install_gps_status_cache(integration, ttl_seconds=None):
    """Replace UI status with one-preflight cached status.

    Runtime AUTO start/preflight paths continue calling controller.preflight
    directly and are never served cached readiness. Only status/dashboard reads
    are cached, so safety decisions always use fresh sensor state.
    """
    if ttl_seconds is None:
        ttl_seconds = float(os.environ.get("AUTONOMY_STATUS_CACHE_SECONDS", "0.35"))

    def build_status():
        selected = integration.selected()
        models = integration.full.ai.MODEL_REGISTRY.list_models("AUTO_GPS")
        manual = integration.controller.preflight(auto_only=False)
        auto = copy.deepcopy(manual)
        if auto.get("ready"):
            details = auto.setdefault("details", {})
            if details.get("model_stage") != "AUTO_ALLOWED":
                auto["ready"] = False
                details["error"] = "GPS_MODEL_VALIDATION_STAGE"
                details["required_stage"] = "AUTO_ALLOWED"
        return {
            "selected": selected,
            "routes": integration.list_routes(),
            "models": models,
            "manual_preflight": manual,
            "auto_preflight": auto,
            "controller": integration.controller.snapshot(),
            "status_cache_seconds": max(0.0, float(ttl_seconds)),
        }

    cache = TimedSnapshotCache(build_status, ttl_seconds)
    original_select = integration.select
    original_build_route = integration.build_route

    def select(*args, **kwargs):
        result = original_select(*args, **kwargs)
        cache.invalidate()
        return result

    def build_route(*args, **kwargs):
        result = original_build_route(*args, **kwargs)
        cache.invalidate()
        return result

    integration.status = cache.get
    integration.select = select
    integration.build_route = build_route
    integration.status_cache = cache
    return cache


__all__ = ["TimedSnapshotCache", "install_gps_status_cache"]
