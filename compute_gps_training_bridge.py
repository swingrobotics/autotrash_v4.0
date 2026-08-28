"""Rover-side bridge for route-bound AUTO_GPS Compute Worker training."""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs, urlparse

import server_v2_release as release
from server_v2_gps_ai import GpsAiIntegration
from autonomous_car.ai import ModelRegistryError
from autonomous_car.routes import GpsRouteNormalizer
from compute_rover_api import (
    MAX_CHECKPOINT_BYTES,
    MAX_MODEL_BYTES,
    TRANSFER_MANAGER,
    _download_worker_artifact,
    _safe_leaf,
    _vehicle_safe_for_training_transfer,
    _worker_get_json,
    _worker_url_allowed,
)


_INSTALLED = False
_ROUTE_PATCHED = False
_GRANT_LOCK = threading.RLock()
_GPS_GRANTS = {}


def _routes_root():
    return Path(
        os.environ.get(
            "AUTONOMY_GPS_ROUTES_PATH",
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "gps-routes"),
        )
    ).resolve()


def _route_path(route_id):
    name = _safe_leaf(route_id, label="GPS_ROUTE")
    root = _routes_root()
    path = (root / f"{name}.json").resolve()
    try:
        path.relative_to(root)
    except ValueError as error:
        raise ValueError("GPS_ROUTE_PATH_ESCAPE") from error
    if not path.is_file():
        raise FileNotFoundError(f"GPS route not found: {name}")
    return path


def _load_json(path):
    with open(path, "r", encoding="utf-8") as file:
        value = json.load(file)
    if not isinstance(value, dict):
        raise ValueError(f"JSON object required: {path}")
    return value


def _cleanup_grants():
    now = time.time()
    with _GRANT_LOCK:
        for token in [
            key
            for key, value in _GPS_GRANTS.items()
            if float(value.get("expires_at") or 0) <= now
        ]:
            _GPS_GRANTS.pop(token, None)


def _issue_gps_transfer(sessions, route_id):
    if not _vehicle_safe_for_training_transfer():
        raise ValueError(
            "Stop RECORD, dataset build, mapping and autonomous driving before GPS training"
        )
    route_id = _safe_leaf(route_id, label="GPS_ROUTE")
    route = _load_json(_route_path(route_id))
    if str(route.get("route_id") or "") != route_id:
        raise ValueError("GPS_ROUTE_DOCUMENT_ID_MISMATCH")
    grant = TRANSFER_MANAGER.issue(sessions)
    token = str(grant["token"])
    with _GRANT_LOCK:
        _cleanup_grants()
        _GPS_GRANTS[token] = {
            "route_id": route_id,
            "expires_at": float(grant["expires_at"]),
        }
    result = dict(grant)
    result["policy_type"] = "AUTO_GPS"
    result["route_id"] = route_id
    return result


def _authorize_route(token, route_id):
    token = str(token or "").strip()
    route_id = _safe_leaf(route_id, label="GPS_ROUTE")
    TRANSFER_MANAGER.authorize(token)
    _cleanup_grants()
    with _GRANT_LOCK:
        grant = _GPS_GRANTS.get(token)
        if not grant:
            raise PermissionError("GPS_TRANSFER_TOKEN_INVALID_OR_EXPIRED")
        if str(grant.get("route_id") or "") != route_id:
            raise PermissionError("GPS_ROUTE_NOT_AUTHORIZED")
    return _route_path(route_id)


def _token_from(handler, query):
    return str(
        handler.headers.get("X-SWING-Transfer-Token")
        or query.get("token", [""])[0]
    ).strip()


def _patch_usb_aware_route_build():
    global _ROUTE_PATCHED
    if _ROUTE_PATCHED:
        return

    def build_route(self, sessions, route_id):
        route_id = _safe_leaf(route_id, label="GPS_ROUTE")
        normalized = list(
            dict.fromkeys(
                _safe_leaf(value, label="SESSION") for value in sessions or []
            )
        )
        if len(normalized) < 2:
            raise ValueError("GPS_ROUTE_REQUIRES_AT_LEAST_2_RECORD_SESSIONS")
        resolver = getattr(self.legacy, "recording_session_path", None)
        if not callable(resolver):
            raise RuntimeError("RECORDING_SESSION_RESOLVER_UNAVAILABLE")
        roots = set()
        for session in normalized:
            path = Path(resolver(session)).resolve()
            if not path.is_dir() or path.name != session:
                raise FileNotFoundError(f"RECORD session not found: {session}")
            roots.add(path.parent)
        if len(roots) != 1:
            raise ValueError("GPS_ROUTE_SOURCE_SESSIONS_SPAN_MULTIPLE_STORAGE_ROOTS")

        os.makedirs(self.routes_root, exist_ok=True)
        path = self.route_path(route_id)
        if os.path.exists(path):
            raise FileExistsError(f"GPS route already exists: {route_id}")
        route = GpsRouteNormalizer().build(
            str(next(iter(roots))),
            normalized,
            route_id,
            output_path=path,
        )
        return route.as_dict()

    GpsAiIntegration.build_route = build_route
    _ROUTE_PATCHED = True


def _install_gps_candidate(worker_urls, job_id, model_id, artifact_token, route_id):
    if not _vehicle_safe_for_training_transfer():
        raise ValueError(
            "Stop RECORD, dataset build, mapping and autonomous driving before installing a GPS model"
        )
    job_id = _safe_leaf(job_id, label="JOB")
    artifact_token = str(artifact_token or "").strip()
    if len(artifact_token) < 24:
        raise ValueError("ARTIFACT_TOKEN_REQUIRED")
    model_id = release.full.ai.MODEL_REGISTRY._normalize_id(model_id)
    route_id = _safe_leaf(route_id, label="GPS_ROUTE")
    _route_path(route_id)
    try:
        release.full.ai.MODEL_REGISTRY.get(model_id)
    except ModelRegistryError:
        pass
    else:
        raise ValueError("MODEL_ID_ALREADY_EXISTS")

    urls = [
        str(value).rstrip("/")
        for value in worker_urls or []
        if _worker_url_allowed(value)
    ]
    if not urls:
        raise ValueError("NO_PRIVATE_WORKER_URL")

    worker_url = None
    job = None
    last_error = None
    for url in urls:
        try:
            candidate = _worker_get_json(
                url,
                f"/api/v1/jobs/{job_id}",
                artifact_token,
            )
            if candidate.get("state") != "SUCCEEDED":
                raise ValueError("WORKER_JOB_NOT_SUCCEEDED")
            result = candidate.get("result") or {}
            if str(result.get("model_id") or "") != model_id:
                raise ValueError("WORKER_MODEL_ID_MISMATCH")
            if str(result.get("policy_type") or "") != "AUTO_GPS":
                raise ValueError("WORKER_MODEL_POLICY_MISMATCH")
            if str(result.get("route_id") or "") != route_id:
                raise ValueError("WORKER_GPS_ROUTE_MISMATCH")
            worker_url, job = url, candidate
            break
        except Exception as error:
            last_error = error
    if worker_url is None:
        raise OSError(f"WORKER_UNREACHABLE:{last_error}")

    models_root = Path(release.full.ai.MODELS_ROOT).resolve()
    models_root.mkdir(parents=True, exist_ok=True)
    paths = {
        "model": models_root / f"{model_id}.onnx",
        "manifest": models_root / f"{model_id}.manifest.json",
        "evaluation": models_root / f"{model_id}.evaluation.json",
        "checkpoint": models_root / f"{model_id}.checkpoint.pt",
        "context": models_root / f"{model_id}.training.json",
    }
    limits = {
        "model": MAX_MODEL_BYTES,
        "manifest": 4 * 1024 * 1024,
        "evaluation": 8 * 1024 * 1024,
        "checkpoint": MAX_CHECKPOINT_BYTES,
        "context": 8 * 1024 * 1024,
    }
    downloaded = {}
    try:
        for artifact, path in paths.items():
            downloaded[artifact] = _download_worker_artifact(
                worker_url,
                job_id,
                artifact,
                artifact_token,
                path,
                limits[artifact],
            )
        manifest = _load_json(paths["manifest"])
        evaluation = _load_json(paths["evaluation"])
        context = _load_json(paths["context"])
        if manifest.get("policy_type") != "AUTO_GPS":
            raise ValueError("GPS_WORKER_MANIFEST_POLICY_MISMATCH")
        if str(manifest.get("route_id") or "") != route_id:
            raise ValueError("GPS_WORKER_MANIFEST_ROUTE_MISMATCH")
        if manifest.get("model_file") not in {
            "gps_drive_model.onnx",
            paths["model"].name,
        }:
            raise ValueError("INVALID_GPS_WORKER_MODEL_MANIFEST")
        export = manifest.get("export") or {}
        if export.get("external_data") is not False or export.get("self_contained") is not True:
            raise ValueError("GPS_WORKER_MODEL_NOT_SELF_CONTAINED")

        metadata = {
            "policy_type": "AUTO_GPS",
            "route_id": route_id,
            "manifest_file": paths["manifest"].name,
            "checkpoint_file": paths["checkpoint"].name,
            "training_context_file": paths["context"].name,
            "training": context,
            "input": manifest.get("inputs") or {},
            "output": manifest.get("output") or {},
            "metrics": evaluation,
            "worker_job_id": job_id,
            "artifact_sha256": {
                name: item["sha256"] for name, item in downloaded.items()
            },
        }
        registered = release.full.ai.MODEL_REGISTRY.register(
            model_id,
            paths["model"].name,
            metadata=metadata,
            validation_stage="TRAINED",
            policy_type="AUTO_GPS",
        )
        if evaluation.get("criteria_passed") is True:
            registered = release.full.ai.MODEL_REGISTRY.update_lifecycle(
                model_id,
                "OFFLINE_VALIDATED",
                metrics=evaluation,
            )
        return {
            "model": registered,
            "evaluation": evaluation,
            "worker_job": job,
        }
    except Exception:
        registry_path = models_root / f"{model_id}.json"
        if not registry_path.exists():
            for path in paths.values():
                try:
                    path.unlink()
                except OSError:
                    pass
        raise


def install_compute_gps_training_bridge():
    global _INSTALLED
    if _INSTALLED:
        return True
    _patch_usb_aware_route_build()
    original = release.ReleaseHandler

    class ComputeGpsTrainingHandler(original):
        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path != "/api/v2/compute/gps-route":
                super().do_GET()
                return
            query = parse_qs(parsed.query)
            try:
                route_id = _safe_leaf(
                    query.get("route_id", [""])[0],
                    label="GPS_ROUTE",
                )
                path = _authorize_route(_token_from(self, query), route_id)
                document = _load_json(path)
                self._send_json(document)
            except PermissionError as error:
                self._send_json({"error": str(error)}, 403)
            except (ValueError, OSError, TypeError, json.JSONDecodeError) as error:
                self._send_json({"error": str(error)}, 404)

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == "/api/v2/compute/gps-transfer":
                try:
                    payload = self._read_json()
                    self._send_json(
                        _issue_gps_transfer(
                            payload.get("sessions"),
                            payload.get("route_id"),
                        ),
                        202,
                    )
                except (ValueError, OSError, TypeError, json.JSONDecodeError) as error:
                    self._send_json({"error": str(error)}, 409)
                return
            if parsed.path == "/api/v2/compute/gps-model/install":
                try:
                    payload = self._read_json()
                    result = _install_gps_candidate(
                        payload.get("worker_urls"),
                        payload.get("job_id"),
                        payload.get("model_id"),
                        payload.get("artifact_token"),
                        payload.get("route_id"),
                    )
                    self._send_json(result, 202)
                except (
                    ValueError,
                    OSError,
                    TypeError,
                    ModelRegistryError,
                    json.JSONDecodeError,
                ) as error:
                    self._send_json({"error": str(error)}, 409)
                return
            super().do_POST()

    release.ReleaseHandler = ComputeGpsTrainingHandler
    _INSTALLED = True
    return True


__all__ = ["install_compute_gps_training_bridge"]
