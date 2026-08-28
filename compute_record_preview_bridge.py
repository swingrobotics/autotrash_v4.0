"""Rover-side bridge for synchronized RECORD model previews on the PC worker."""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import parse_qs, urlparse
from urllib.request import Request, urlopen

import server_v2_release as release
from autonomous_car.ai import ModelRegistryError
from compute_rover_api import (
    TRANSFER_MANAGER,
    _safe_leaf,
    _send_binary,
    _vehicle_safe_for_training_transfer,
    _worker_url_allowed,
)
from compute_gps_training_bridge import _issue_gps_transfer


_INSTALLED = False
_GRANT_LOCK = threading.RLock()
_PREVIEW_GRANTS = {}
_PREVIEW_ARTIFACTS = {
    "preview_video": ("video/mp4", 1024 * 1024 * 1024),
    "preview_csv": ("text/csv; charset=utf-8", 128 * 1024 * 1024),
}


def _cleanup_grants():
    now = time.time()
    with _GRANT_LOCK:
        for token in [
            key
            for key, value in _PREVIEW_GRANTS.items()
            if float(value.get("expires_at") or 0.0) <= now
        ]:
            _PREVIEW_GRANTS.pop(token, None)


def _model_document(model_id):
    model_id = release.full.ai.MODEL_REGISTRY._normalize_id(model_id)
    model = release.full.ai.MODEL_REGISTRY.get(model_id)
    policy = str(model.get("policy_type") or "AUTO_AI").strip().upper()
    if policy not in {"AUTO_AI", "AUTO_GPS"}:
        raise ValueError("PREVIEW_MODEL_POLICY_UNSUPPORTED")

    model_file = str(model.get("model_file") or "").strip()
    manifest_file = str(model.get("manifest_file") or "").strip()
    if not model_file:
        raise FileNotFoundError("PREVIEW_MODEL_FILE_UNAVAILABLE")
    if not manifest_file:
        fallback = f"{model_id}.manifest.json"
        try:
            fallback_path = Path(release.full.ai._safe_model_path(fallback))
        except (OSError, ValueError):
            fallback_path = None
        if fallback_path is not None and fallback_path.is_file():
            manifest_file = fallback
    if not manifest_file:
        raise FileNotFoundError("PREVIEW_MODEL_MANIFEST_UNAVAILABLE")

    model_path = Path(release.full.ai._safe_model_path(model_file)).resolve()
    manifest_path = Path(release.full.ai._safe_model_path(manifest_file)).resolve()
    if not model_path.is_file():
        raise FileNotFoundError("PREVIEW_MODEL_FILE_UNAVAILABLE")
    if not manifest_path.is_file():
        raise FileNotFoundError("PREVIEW_MODEL_MANIFEST_UNAVAILABLE")

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        raise ValueError("PREVIEW_MODEL_MANIFEST_INVALID") from error
    if not isinstance(manifest, dict):
        raise ValueError("PREVIEW_MODEL_MANIFEST_INVALID")
    manifest_policy = str(manifest.get("policy_type") or policy).strip().upper()
    if manifest_policy != policy:
        raise ValueError("PREVIEW_MODEL_POLICY_MISMATCH")

    route_id = str(model.get("route_id") or manifest.get("route_id") or "").strip() or None
    if policy == "AUTO_GPS" and not route_id:
        raise ValueError("PREVIEW_GPS_MODEL_ROUTE_REQUIRED")
    if policy == "AUTO_GPS" and str(manifest.get("route_id") or "").strip() != route_id:
        raise ValueError("PREVIEW_GPS_MODEL_ROUTE_MISMATCH")

    model_spec = manifest.get("model_spec") or {}
    try:
        auxiliary_feature_size = int(model_spec.get("auxiliary_feature_size") or 2)
    except (TypeError, ValueError):
        auxiliary_feature_size = 2
    try:
        temporal_history_steps = int(model_spec.get("temporal_history_steps") or 1)
    except (TypeError, ValueError):
        temporal_history_steps = 1
    temporal_gps = (
        policy == "AUTO_GPS"
        and auxiliary_feature_size > 2
        and temporal_history_steps > 1
    )

    return {
        "model_id": model_id,
        "policy_type": policy,
        "route_id": route_id,
        "validation_stage": str(model.get("validation_stage") or "TRAINED"),
        "model_path": model_path,
        "manifest_path": manifest_path,
        "temporal_gps": temporal_gps,
        "temporal_history_steps": temporal_history_steps if temporal_gps else None,
        "auxiliary_feature_size": auxiliary_feature_size,
    }


def _public_model(document):
    return {
        "model_id": document["model_id"],
        "policy_type": document["policy_type"],
        "route_id": document.get("route_id"),
        "validation_stage": document.get("validation_stage"),
        "preview_available": True,
        "temporal_gps": bool(document.get("temporal_gps")),
        "temporal_history_steps": document.get("temporal_history_steps"),
        "auxiliary_feature_size": document.get("auxiliary_feature_size"),
    }


def _list_preview_models():
    result = []
    for model in release.full.ai.MODEL_REGISTRY.list_models():
        model_id = str(model.get("model_id") or "").strip()
        if not model_id:
            continue
        try:
            result.append(_public_model(_model_document(model_id)))
        except (ValueError, OSError, ModelRegistryError):
            result.append(
                {
                    "model_id": model_id,
                    "policy_type": str(model.get("policy_type") or "AUTO_AI").upper(),
                    "route_id": model.get("route_id"),
                    "validation_stage": model.get("validation_stage"),
                    "preview_available": False,
                    "temporal_gps": False,
                    "temporal_history_steps": None,
                    "auxiliary_feature_size": None,
                }
            )
    return result


def _issue_preview_transfer(session, model_id):
    if not _vehicle_safe_for_training_transfer():
        raise ValueError(
            "Stop RECORD, dataset build, mapping and autonomous driving before model preview"
        )
    session = _safe_leaf(session, label="SESSION")
    model = _model_document(model_id)
    if model["policy_type"] == "AUTO_GPS":
        grant = _issue_gps_transfer([session], model["route_id"])
    else:
        grant = TRANSFER_MANAGER.issue([session])
    token = str(grant["token"])
    with _GRANT_LOCK:
        _cleanup_grants()
        _PREVIEW_GRANTS[token] = {
            "model_id": model["model_id"],
            "policy_type": model["policy_type"],
            "route_id": model.get("route_id"),
            "expires_at": float(grant["expires_at"]),
        }
    result = dict(grant)
    result["model"] = _public_model(model)
    return result


def _authorize_model(token, model_id, kind):
    token = str(token or "").strip()
    model_id = release.full.ai.MODEL_REGISTRY._normalize_id(model_id)
    TRANSFER_MANAGER.authorize(token)
    _cleanup_grants()
    with _GRANT_LOCK:
        grant = _PREVIEW_GRANTS.get(token)
        if not grant:
            raise PermissionError("PREVIEW_TRANSFER_TOKEN_INVALID_OR_EXPIRED")
        if str(grant.get("model_id") or "") != model_id:
            raise PermissionError("PREVIEW_MODEL_NOT_AUTHORIZED")
    document = _model_document(model_id)
    if kind == "model":
        return document["model_path"]
    if kind == "manifest":
        return document["manifest_path"]
    raise ValueError("PREVIEW_MODEL_FILE_KIND_INVALID")


def _token_from(handler, query):
    return str(
        handler.headers.get("X-SWING-Transfer-Token")
        or query.get("token", [""])[0]
    ).strip()


def _proxy_worker_artifact(handler, payload):
    job_id = _safe_leaf(payload.get("job_id"), label="JOB")
    artifact = str(payload.get("artifact") or "").strip()
    if artifact not in _PREVIEW_ARTIFACTS:
        raise ValueError("PREVIEW_ARTIFACT_INVALID")
    artifact_token = str(payload.get("artifact_token") or "").strip()
    if len(artifact_token) < 24:
        raise ValueError("ARTIFACT_TOKEN_REQUIRED")
    urls = [
        str(value).rstrip("/")
        for value in payload.get("worker_urls") or []
        if _worker_url_allowed(value)
    ]
    if not urls:
        raise ValueError("NO_PRIVATE_WORKER_URL")

    content_type, maximum_bytes = _PREVIEW_ARTIFACTS[artifact]
    last_error = None
    response = None
    for base_url in urls:
        request = Request(
            base_url
            + f"/api/v1/jobs/{job_id}/artifacts/{artifact}",
            headers={"X-SWING-Artifact-Token": artifact_token},
            method="GET",
        )
        try:
            response = urlopen(request, timeout=30)
            break
        except (HTTPError, URLError, OSError) as error:
            last_error = error
    if response is None:
        raise OSError(f"WORKER_PREVIEW_ARTIFACT_UNREACHABLE:{last_error}")

    with response:
        try:
            declared = int(response.headers.get("Content-Length") or 0)
        except (TypeError, ValueError):
            declared = 0
        if declared < 0 or declared > maximum_bytes:
            raise ValueError("PREVIEW_ARTIFACT_TOO_LARGE")
        handler.send_response(200)
        handler.send_header("Content-Type", content_type)
        handler.send_header("Cache-Control", "private, no-store")
        if declared:
            handler.send_header("Content-Length", str(declared))
        handler.end_headers()
        total = 0
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            total += len(chunk)
            if total > maximum_bytes:
                raise OSError("PREVIEW_ARTIFACT_STREAM_TOO_LARGE")
            handler.wfile.write(chunk)


def install_compute_record_preview_bridge():
    global _INSTALLED
    if _INSTALLED:
        return True
    original = release.ReleaseHandler

    class ComputeRecordPreviewHandler(original):
        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == "/api/v2/compute/preview-models":
                self._send_json({"models": _list_preview_models()})
                return
            if parsed.path == "/api/v2/compute/preview-model-file":
                query = parse_qs(parsed.query)
                try:
                    model_id = query.get("model_id", [""])[0]
                    kind = str(query.get("kind", ["model"])[0]).strip().lower()
                    path = _authorize_model(
                        _token_from(self, query),
                        model_id,
                        kind,
                    )
                    _send_binary(self, path)
                except PermissionError as error:
                    self._send_json({"error": str(error)}, 403)
                except (ValueError, OSError, ModelRegistryError) as error:
                    self._send_json({"error": str(error)}, 404)
                return
            super().do_GET()

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == "/api/v2/compute/preview-artifact":
                try:
                    payload = self._read_json()
                    _proxy_worker_artifact(self, payload)
                except (
                    ValueError,
                    OSError,
                    TypeError,
                    json.JSONDecodeError,
                ) as error:
                    self._send_json({"error": str(error)}, 409)
                return
            if parsed.path != "/api/v2/compute/preview-transfer":
                super().do_POST()
                return
            try:
                payload = self._read_json()
                result = _issue_preview_transfer(
                    payload.get("session"),
                    payload.get("model_id"),
                )
                self._send_json(result, 202)
            except (
                ValueError,
                OSError,
                ModelRegistryError,
                TypeError,
                json.JSONDecodeError,
            ) as error:
                self._send_json({"error": str(error)}, 409)

    release.ReleaseHandler = ComputeRecordPreviewHandler
    _INSTALLED = True
    return True


__all__ = ["install_compute_record_preview_bridge"]
