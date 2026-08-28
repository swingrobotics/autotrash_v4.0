"""Secure rover-side bridge for the PC SWING Compute Worker.

The vehicle remains the authority for RECORD ownership and model installation.
The PC receives short-lived read grants for selected, closed RECORD sessions and
returns candidate artifacts that the rover pulls and registers at TRAINED stage.
No endpoint in this module grants closed-area or AUTO permission.
"""

from __future__ import annotations

import hashlib
import ipaddress
import json
import os
import secrets
import threading
import time
from pathlib import Path
from urllib.parse import parse_qs, urlparse
from urllib.request import Request, urlopen

import server_v2_release as release
from autonomous_car.ai import ModelRegistryError
from autonomous_car.recording.record_transfer import (
    SOURCE_TOP_LEVEL_FILES,
    iter_record_source_files,
    record_source_path,
)


TRANSFER_TTL_SECONDS = 15 * 60
# Backward-compatible public name. New sessions additionally expose validated
# camera_frames/** JPEGs through the manifest and file endpoint.
TRANSFER_FILES = SOURCE_TOP_LEVEL_FILES
MAX_MODEL_BYTES = 128 * 1024 * 1024
MAX_CHECKPOINT_BYTES = 512 * 1024 * 1024
_INSTALLED = False


def _safe_leaf(value, *, label="ID"):
    text = str(value or "").strip()
    if (
        not text
        or len(text) > 128
        or os.path.basename(text) != text
        or text in {".", ".."}
        or any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_." for ch in text)
    ):
        raise ValueError(f"INVALID_{label}")
    return text


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class TransferManager:
    """Short-lived capability grants plus an mtime/size keyed hash cache."""

    def __init__(self):
        self.lock = threading.RLock()
        self.tokens = {}
        self.hash_cache = {}

    def _cleanup(self):
        now = time.time()
        for token in [key for key, value in self.tokens.items() if value["expires_at"] <= now]:
            self.tokens.pop(token, None)

    def issue(self, sessions, base_model_id=None):
        normalized = []
        seen = set()
        for value in sessions or ():
            name = _safe_leaf(value, label="SESSION")
            if name in seen:
                continue
            path = self.session_path(name)
            if not path.is_dir():
                raise FileNotFoundError(f"RECORD session not found: {name}")
            if release.full.legacy.record_manager.active:
                active = release.full.legacy.record_manager.snapshot().get("session_path")
                if active and Path(active).name == name:
                    raise ValueError("ACTIVE_RECORD_CANNOT_BE_TRANSFERRED")
            seen.add(name)
            normalized.append(name)
        if not normalized:
            raise ValueError("Select at least one RECORD session")

        base_model_id = str(base_model_id or "").strip() or None
        if base_model_id:
            model = release.full.ai.MODEL_REGISTRY.get(base_model_id)
            if str(model.get("policy_type") or "AUTO_AI") != "AUTO_AI":
                raise ValueError("BASE_MODEL_MUST_BE_AUTO_AI")

        token = secrets.token_urlsafe(32)
        expires = time.time() + TRANSFER_TTL_SECONDS
        with self.lock:
            self._cleanup()
            self.tokens[token] = {
                "sessions": set(normalized),
                "base_model_id": base_model_id,
                "expires_at": expires,
            }
        return {
            "token": token,
            "expires_at": expires,
            "sessions": [self.manifest(name) for name in normalized],
            "base_model_id": base_model_id,
        }

    def authorize(self, token, *, session=None, base_model_id=None):
        token = str(token or "").strip()
        with self.lock:
            self._cleanup()
            grant = self.tokens.get(token)
            if not grant:
                raise PermissionError("TRANSFER_TOKEN_INVALID_OR_EXPIRED")
            if session is not None and session not in grant["sessions"]:
                raise PermissionError("SESSION_NOT_AUTHORIZED")
            if base_model_id is not None and base_model_id != grant.get("base_model_id"):
                raise PermissionError("BASE_MODEL_NOT_AUTHORIZED")
            return dict(grant)

    @staticmethod
    def session_path(session):
        """Resolve through the runtime storage bridge, not the legacy SD root."""
        name = _safe_leaf(session, label="SESSION")
        resolver = getattr(release.full.legacy, "recording_session_path", None)
        if not callable(resolver):
            raise RuntimeError("RECORDING_SESSION_RESOLVER_UNAVAILABLE")
        path = Path(resolver(name)).resolve()
        if path.name != name:
            raise ValueError("SESSION_PATH_ESCAPE")
        if not path.is_dir():
            raise FileNotFoundError(f"RECORD session not found: {name}")

        roots_provider = getattr(release.full.legacy, "recording_roots", None)
        if callable(roots_provider):
            roots = []
            for item in roots_provider() or []:
                root = item.get("path") if isinstance(item, dict) else item
                if root:
                    roots.append(Path(root).resolve())
            if roots and not any(path.parent == root for root in roots):
                raise ValueError("SESSION_PATH_OUTSIDE_RECORDING_ROOTS")
        else:
            root = Path(release.full.legacy.RECORDINGS_PATH).resolve()
            try:
                path.relative_to(root)
            except ValueError as error:
                raise ValueError("SESSION_PATH_ESCAPE") from error
        return path

    def file_path(self, session, filename):
        return record_source_path(self.session_path(session), filename)

    def digest(self, path):
        stat = path.stat()
        key = (str(path), int(stat.st_size), int(stat.st_mtime_ns))
        with self.lock:
            value = self.hash_cache.get(key)
        if value:
            return value
        value = _sha256(path)
        with self.lock:
            self.hash_cache[key] = value
        return value

    def manifest(self, session):
        path = self.session_path(session)
        files = []
        for name, candidate in iter_record_source_files(path):
            stat = candidate.stat()
            files.append(
                {
                    "name": name,
                    "size_bytes": int(stat.st_size),
                    "mtime_ns": int(stat.st_mtime_ns),
                    "sha256": self.digest(candidate),
                }
            )
        return {
            "session": path.name,
            "files": files,
            "total_bytes": sum(item["size_bytes"] for item in files),
            "camera_frame_files": sum(
                1 for item in files if str(item.get("name") or "").startswith("camera_frames/")
            ),
        }


TRANSFER_MANAGER = TransferManager()


def _vehicle_safe_for_training_transfer():
    mode = str(
        release.full.legacy.vehicle_state_machine.snapshot().get("canonical_mode")
        or release.full.legacy.vehicle_state_machine.snapshot().get("mode")
        or ""
    ).upper()
    return (
        not release.full.legacy.record_manager.active
        and not release.DATASET_BUILD_CONTROLLER.active
        and not release.full.MAPPING_CONTROLLER.active
        and not release.full.ai.AUTO_AI_CONTROLLER.active
        and not release.full.AUTO_LOCAL_CONTROLLER.active
        and mode in {"DISARMED", "MANUAL", "MANUAL_ASSIST"}
    )


def _worker_url_allowed(url):
    parsed = urlparse(str(url or ""))
    if parsed.scheme != "http" or not parsed.hostname:
        return False
    if parsed.port not in {8765}:
        return False
    try:
        ip = ipaddress.ip_address(parsed.hostname.split("%", 1)[0])
    except ValueError:
        return False
    return bool(ip.is_private or ip.is_loopback or ip.is_link_local)


def _worker_get_json(base_url, path, artifact_token=None):
    headers = {"Accept": "application/json"}
    if artifact_token:
        headers["X-SWING-Artifact-Token"] = artifact_token
    request = Request(base_url.rstrip("/") + path, headers=headers, method="GET")
    with urlopen(request, timeout=8) as response:
        if response.status != 200:
            raise OSError(f"WORKER_HTTP_{response.status}")
        return json.loads(response.read(2 * 1024 * 1024).decode("utf-8"))


def _download_worker_artifact(base_url, job_id, artifact, token, target, maximum):
    request = Request(
        base_url.rstrip("/")
        + f"/api/v1/jobs/{job_id}/artifacts/{artifact}",
        headers={"X-SWING-Artifact-Token": token},
        method="GET",
    )
    temporary = Path(str(target) + ".part")
    total = 0
    digest = hashlib.sha256()
    with urlopen(request, timeout=30) as response, open(temporary, "wb") as output:
        if response.status != 200:
            raise OSError(f"WORKER_ARTIFACT_HTTP_{response.status}")
        while True:
            chunk = response.read(1024 * 1024)
            if not chunk:
                break
            total += len(chunk)
            if total > maximum:
                raise ValueError("WORKER_ARTIFACT_TOO_LARGE")
            digest.update(chunk)
            output.write(chunk)
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, target)
    return {"bytes": total, "sha256": digest.hexdigest()}


def _install_candidate(worker_urls, job_id, model_id, artifact_token):
    if not _vehicle_safe_for_training_transfer():
        raise ValueError("Stop RECORD, mapping and autonomous driving before installing a model")
    job_id = _safe_leaf(job_id, label="JOB")
    model_id = release.full.ai.MODEL_REGISTRY._normalize_id(model_id)
    urls = [str(url).rstrip("/") for url in (worker_urls or []) if _worker_url_allowed(url)]
    if not urls:
        raise ValueError("NO_PRIVATE_WORKER_URL")

    worker_url = None
    job = None
    last_error = None
    for url in urls:
        try:
            candidate = _worker_get_json(url, f"/api/v1/jobs/{job_id}", artifact_token)
            if candidate.get("state") != "SUCCEEDED":
                raise ValueError("WORKER_JOB_NOT_SUCCEEDED")
            result = candidate.get("result") or {}
            if str(result.get("model_id") or "") != model_id:
                raise ValueError("WORKER_MODEL_ID_MISMATCH")
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
                worker_url, job_id, artifact, artifact_token, path, limits[artifact]
            )
        with open(paths["manifest"], "r", encoding="utf-8") as file:
            manifest = json.load(file)
        with open(paths["evaluation"], "r", encoding="utf-8") as file:
            evaluation = json.load(file)
        with open(paths["context"], "r", encoding="utf-8") as file:
            context = json.load(file)
        if manifest.get("model_file") not in {"drive_model.onnx", paths["model"].name}:
            raise ValueError("INVALID_WORKER_MODEL_MANIFEST")
        metadata = {
            "policy_type": "AUTO_AI",
            "manifest_file": paths["manifest"].name,
            "checkpoint_file": paths["checkpoint"].name,
            "training_context_file": paths["context"].name,
            "training": context,
            "input": manifest.get("inputs") or {},
            "output": manifest.get("output") or {},
            "metrics": evaluation,
            "worker_job_id": job_id,
            "artifact_sha256": {name: item["sha256"] for name, item in downloaded.items()},
        }
        registered = release.full.ai.MODEL_REGISTRY.register(
            model_id,
            paths["model"].name,
            metadata=metadata,
            validation_stage="TRAINED",
            policy_type="AUTO_AI",
        )
        # Offline evaluation is attached as evidence but never silently grants
        # vehicle permission. Explicit project criteria may promote only one
        # step to OFFLINE_VALIDATED.
        if evaluation.get("criteria_passed") is True:
            registered = release.full.ai.MODEL_REGISTRY.update_lifecycle(
                model_id, "OFFLINE_VALIDATED", metrics=evaluation
            )
        return {"model": registered, "evaluation": evaluation, "worker_job": job}
    except Exception:
        # Avoid half-installed artifacts looking like a valid registry entry.
        registry_path = models_root / f"{model_id}.json"
        if not registry_path.exists():
            for path in paths.values():
                try:
                    path.unlink()
                except OSError:
                    pass
        raise


def _token_from(handler, query):
    return str(handler.headers.get("X-SWING-Transfer-Token") or query.get("token", [""])[0]).strip()


def _send_binary(handler, path):
    size = path.stat().st_size
    start = 0
    range_header = str(handler.headers.get("Range") or "").strip()
    if range_header:
        if not range_header.startswith("bytes=") or "," in range_header:
            handler.send_error(416)
            return
        first = range_header[6:].split("-", 1)[0].strip()
        try:
            start = int(first)
        except ValueError:
            handler.send_error(416)
            return
        if start < 0 or start >= size:
            handler.send_response(416)
            handler.send_header("Content-Range", f"bytes */{size}")
            handler.send_header("Content-Length", "0")
            handler.end_headers()
            return
    length = size - start
    handler.send_response(206 if start else 200)
    handler.send_header("Content-Type", "application/octet-stream")
    handler.send_header("Accept-Ranges", "bytes")
    handler.send_header("Content-Length", str(length))
    if start:
        handler.send_header("Content-Range", f"bytes {start}-{size - 1}/{size}")
    handler.send_header("Cache-Control", "private, no-store")
    handler.end_headers()
    with open(path, "rb") as file:
        if start:
            file.seek(start)
        while True:
            chunk = file.read(1024 * 1024)
            if not chunk:
                break
            handler.wfile.write(chunk)


def install_compute_rover_api():
    """Wrap server_v2_release.ReleaseHandler before GPS/final wrappers inherit it."""
    global _INSTALLED
    if _INSTALLED:
        return True
    original = release.ReleaseHandler

    class ComputeRoverApiHandler(original):
        def do_GET(self):
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)
            try:
                if parsed.path == "/api/v2/compute/recording-manifest":
                    session = _safe_leaf(query.get("session", [""])[0], label="SESSION")
                    token = _token_from(self, query)
                    TRANSFER_MANAGER.authorize(token, session=session)
                    self._send_json(TRANSFER_MANAGER.manifest(session))
                    return
                if parsed.path == "/api/v2/compute/recording-file":
                    session = _safe_leaf(query.get("session", [""])[0], label="SESSION")
                    filename = str(query.get("file", [""])[0])
                    token = _token_from(self, query)
                    TRANSFER_MANAGER.authorize(token, session=session)
                    _send_binary(self, TRANSFER_MANAGER.file_path(session, filename))
                    return
                if parsed.path == "/api/v2/compute/base-file":
                    model_id = release.full.ai.MODEL_REGISTRY._normalize_id(
                        query.get("model_id", [""])[0]
                    )
                    kind = str(query.get("kind", ["checkpoint"])[0])
                    token = _token_from(self, query)
                    TRANSFER_MANAGER.authorize(token, base_model_id=model_id)
                    model = release.full.ai.MODEL_REGISTRY.get(model_id)
                    field = {
                        "checkpoint": "checkpoint_file",
                        "context": "training_context_file",
                    }.get(kind)
                    if not field or not model.get(field):
                        raise FileNotFoundError("BASE_TRAINING_ARTIFACT_UNAVAILABLE")
                    path = Path(release.full.ai._safe_model_path(model[field]))
                    _send_binary(self, path)
                    return
            except PermissionError as error:
                self._send_json({"error": str(error)}, 403)
                return
            except (ValueError, OSError, ModelRegistryError) as error:
                self._send_json({"error": str(error)}, 404)
                return
            super().do_GET()

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == "/api/v2/compute/transfer":
                try:
                    if not _vehicle_safe_for_training_transfer():
                        raise ValueError(
                            "Stop RECORD, dataset build, mapping and autonomous driving before training"
                        )
                    payload = self._read_json()
                    self._send_json(
                        TRANSFER_MANAGER.issue(
                            payload.get("sessions"), payload.get("base_model_id")
                        ),
                        202,
                    )
                except (ValueError, OSError, ModelRegistryError) as error:
                    self._send_json({"error": str(error)}, 409)
                return
            if parsed.path == "/api/v2/compute/model/install":
                try:
                    payload = self._read_json()
                    result = _install_candidate(
                        payload.get("worker_urls"),
                        payload.get("job_id"),
                        payload.get("model_id"),
                        payload.get("artifact_token"),
                    )
                    self._send_json(result, 202)
                except (ValueError, OSError, ModelRegistryError, json.JSONDecodeError) as error:
                    self._send_json({"error": str(error)}, 409)
                return
            super().do_POST()

    release.ReleaseHandler = ComputeRoverApiHandler
    _INSTALLED = True
    return True


__all__ = [
    "TRANSFER_FILES",
    "TRANSFER_MANAGER",
    "TransferManager",
    "install_compute_rover_api",
]
