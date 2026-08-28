'''Final operator-dashboard readiness augmentation.

This module is intentionally small and layered on top of the existing V2 stack.
It makes the status exposed to the operator use the same AUTO_LOCAL preflight
that the controller uses at start time, and prevents AUTO_AI from being shown
as ready when the Pi inference dependencies or packaged ONNX contract are not
actually usable. AUTO_GPS readiness remains owned by its controller preflight,
which validates route, sensors, inference dependencies and model artifacts.
'''

from __future__ import annotations

import importlib.util
import json
import os
import threading
import time


_LOCAL_PREFLIGHT_TTL_SECONDS = 5.0
_lock = threading.RLock()
_cache = {
    "key": None,
    "checked_at": 0.0,
    "result": None,
    "worker_running": False,
}


def _local_selection_key(full):
    selected = full._selected_local()
    return (
        str(selected.get("map_id") or "").strip(),
        str(selected.get("destination_id") or "").strip(),
    )


def _local_failure_reason(result):
    error = str(result.get("error") or "").strip()
    if not error:
        return "local_preflight_not_ready"
    return error


def _refresh_local_preflight(full, key):
    try:
        result = dict(full.AUTO_LOCAL_CONTROLLER.preflight())
    except Exception as error:
        result = {
            "ready": False,
            "error": f"{type(error).__name__}: {error}",
        }

    result["ready"] = bool(result.get("ready"))
    result["checking"] = False
    result["checked_at"] = time.time()

    with _lock:
        if _cache["key"] == key:
            _cache["result"] = result
            _cache["checked_at"] = time.monotonic()
        _cache["worker_running"] = False


def _local_preflight_snapshot(full):
    key = _local_selection_key(full)
    map_id, destination_id = key
    if not map_id or not destination_id:
        with _lock:
            _cache.update(
                key=key,
                checked_at=time.monotonic(),
                result=None,
                worker_running=False,
            )
        return {
            "ready": False,
            "checking": False,
            "configured": False,
            "error": "LOCAL_MAP_AND_DESTINATION_REQUIRED",
        }

    now = time.monotonic()
    with _lock:
        if _cache["key"] != key:
            _cache.update(key=key, checked_at=0.0, result=None, worker_running=False)

        result = _cache["result"]
        fresh = (
            result is not None
            and now - float(_cache["checked_at"]) <= _LOCAL_PREFLIGHT_TTL_SECONDS
        )
        if fresh:
            snapshot = dict(result)
            snapshot["configured"] = True
            return snapshot

        if not _cache["worker_running"]:
            _cache["worker_running"] = True
            threading.Thread(
                target=_refresh_local_preflight,
                args=(full, key),
                daemon=True,
                name="auto-local-preflight",
            ).start()

        snapshot = dict(result or {})
        snapshot.update(
            ready=False,
            checking=True,
            configured=True,
            stale_ready=bool((result or {}).get("ready")),
        )
        snapshot.setdefault("error", "LOCAL_PREFLIGHT_CHECKING")
        return snapshot


def _ai_runtime_readiness(full, status):
    """Cheap operator readiness check without constructing an ORT session."""

    ai = status.get("ai") or {}
    if not ai.get("ready"):
        return False, (
            ai.get("selected_error")
            or "select a CLOSED_AREA_VALIDATED/AUTO_ALLOWED installed model"
        )

    missing = [
        module
        for module in ("cv2", "numpy", "onnxruntime")
        if importlib.util.find_spec(module) is None
    ]
    if missing:
        return False, "AI_RUNTIME_DEPENDENCY_MISSING:" + ",".join(missing)

    selected = ai.get("selected_model") or {}
    manifest_file = selected.get("manifest_file")
    model_file = selected.get("model_file")
    if not manifest_file or not model_file:
        return False, "AI_MODEL_ARTIFACT_MISSING"
    try:
        model_path = full.ai._safe_model_path(model_file)
        manifest_path = full.ai._safe_model_path(manifest_file)
        if not os.path.isfile(model_path) or not os.path.isfile(manifest_path):
            return False, "AI_MODEL_ARTIFACT_MISSING"
        with open(manifest_path, "r", encoding="utf-8") as file:
            manifest = json.load(file)
    except Exception as error:
        return False, f"AI_MODEL_MANIFEST_INVALID:{type(error).__name__}:{error}"

    export = manifest.get("export") or {}
    if export.get("external_data") is not False or export.get("self_contained") is not True:
        return False, "AI_MODEL_NOT_SELF_CONTAINED"
    return True, "selected_model_runtime_ready"


def install_operator_mode_status_patch():
    import sys

    release = sys.modules.get("server_v2_release")
    if release is None:
        return False

    full = release.full
    if getattr(full, "_operator_mode_status_patched", False):
        return True

    original_full_status = full.full_status

    def operator_full_status():
        status = original_full_status()
        # Lower V2 layers still carry an obsolete migration-era message saying
        # AUTO_AI/AUTO_LOCAL are unimplemented. The final stack implements both;
        # do not expose contradictory status to the operator.
        status.pop("message", None)

        capabilities = status.setdefault("capabilities", {})
        ai_ready, ai_reason = _ai_runtime_readiness(full, status)
        ai = status.setdefault("ai", {})
        ai["ready"] = ai_ready
        ai["readiness_reason"] = ai_reason
        ai_cap = capabilities.setdefault("AUTO_AI", {})
        ai_cap.update(
            {
                "implemented": True,
                "ready": ai_ready,
                "reason": ai_reason,
            }
        )

        local_preflight = _local_preflight_snapshot(full)
        local_ready = bool(local_preflight.get("ready"))
        local_checking = bool(local_preflight.get("checking"))

        local = status.setdefault("local", {})
        local["preflight"] = local_preflight
        local["preflight_ready"] = local_ready

        local_cap = capabilities.setdefault("AUTO_LOCAL", {})
        local_cap.update(
            {
                "implemented": True,
                "ready": local_ready,
                "checking": local_checking,
                "reason": (
                    "local_preflight_ready"
                    if local_ready
                    else (
                        "local_preflight_checking"
                        if local_checking
                        else _local_failure_reason(local_preflight)
                    )
                ),
            }
        )

        # AUTO_GPS controller preflight is the single authority. server_v2_gps_ai
        # replaces v2._route_preflight before requests are served, so this lower
        # capability already includes route/stage/sensor/runtime-artifact checks.
        gps_ready = bool((capabilities.get("AUTO_GPS") or {}).get("ready"))
        gps_reason = str((capabilities.get("AUTO_GPS") or {}).get("reason") or "")
        try:
            environment_tags = full._auto_config().get("environment_tags") or []
            compatible_ai = (
                full.ai.MODEL_REGISTRY.compatible_for_auto(environment_tags)
                if ai_ready
                else []
            )
        except Exception:
            compatible_ai = []

        if gps_ready:
            auto_reason = "gps_strategy_available"
        elif local_ready:
            auto_reason = "local_strategy_available"
        elif compatible_ai:
            auto_reason = "ai_strategy_available"
        elif local_checking:
            auto_reason = "local_preflight_checking"
        else:
            auto_reason = "no_strategy_ready"

        auto_cap = capabilities.setdefault("AUTO", {})
        auto_cap.update(
            {
                "implemented": True,
                "ready": bool(gps_ready or local_ready or compatible_ai),
                "checking": bool(local_checking and not gps_ready and not compatible_ai),
                "reason": auto_reason,
            }
        )

        auto = status.setdefault("auto", {})
        auto["readiness"] = {
            "gps_ready": gps_ready,
            "gps_runtime_reason": gps_reason,
            "local_ready": local_ready,
            "local_checking": local_checking,
            "ai_runtime_ready": ai_ready,
            "ai_runtime_reason": ai_reason,
            "compatible_auto_ai_models": [
                model.get("model_id") for model in compatible_ai if model.get("model_id")
            ],
        }
        return status

    full.full_status = operator_full_status
    full._operator_mode_status_patched = True
    return True
