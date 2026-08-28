"""Production-facing SWING Compute Worker pipeline server.

The browser submits a small training job to localhost. The worker then pulls only
selected rover RECORD files over the private LAN, verifies SHA-256, reuses its
cache, trains/evaluates/exports locally, and exposes capability-protected
candidate artifacts for the rover to pull back.
"""

from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import socket
import threading
import time
import traceback
import uuid
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.parse import quote, urlparse
from urllib.request import Request, urlopen

from .training_pipeline import TrainingPipeline
from .worker import ComputeWorker, WorkerConfig, WORKER_VERSION, _status_page


JOB_TERMINAL = {"SUCCEEDED", "FAILED", "CANCELED"}


def _private_rover_url(value):
    parsed = urlparse(str(value or "").rstrip("/"))
    if parsed.scheme != "http" or not parsed.hostname:
        raise ValueError("ROVER_URL_MUST_BE_PRIVATE_HTTP")
    host = parsed.hostname
    addresses = []
    try:
        addresses = [ipaddress.ip_address(host.split("%", 1)[0])]
    except ValueError:
        try:
            addresses = [ipaddress.ip_address(item[4][0]) for item in socket.getaddrinfo(host, parsed.port or 80)]
        except OSError as error:
            raise ValueError("ROVER_HOST_NOT_RESOLVABLE") from error
    if not addresses or not all(ip.is_private or ip.is_loopback or ip.is_link_local for ip in addresses):
        raise ValueError("ROVER_URL_NOT_PRIVATE")
    return str(value).rstrip("/")


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class PipelineJobManager:
    def __init__(self, worker):
        self.worker = worker
        self.lock = threading.RLock()
        self.jobs = {}
        self.queue = []
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def create(self, payload):
        kind = str(payload.get("kind") or "").strip().lower()
        if kind not in {"diagnostic", "train_rover_records"}:
            raise ValueError("UNSUPPORTED_JOB_KIND")
        if kind == "train_rover_records":
            payload = self._validate_training_request(payload)
        job_id = uuid.uuid4().hex[:12]
        token = uuid.uuid4().hex + uuid.uuid4().hex
        now = time.time()
        job = {
            "job_id": job_id,
            "kind": kind,
            "state": "QUEUED",
            "phase": "QUEUED",
            "created_at": now,
            "started_at": None,
            "finished_at": None,
            "progress": 0.0,
            "message": "대기 중",
            "request": payload,
            "result": None,
            "error": None,
            "cancel_requested": False,
            "artifact_token": token,
            "worker_urls": self.worker.advertise_urls(),
        }
        with self.lock:
            # One training job at a time prevents CPU/GPU overcommit and keeps
            # perception/training resource policy deterministic.
            if kind == "train_rover_records" and any(
                item["kind"] == kind and item["state"] not in JOB_TERMINAL
                for item in self.jobs.values()
            ):
                raise ValueError("A_TRAINING_JOB_IS_ALREADY_ACTIVE")
            self.jobs[job_id] = job
            self.queue.append(job_id)
        return self.snapshot(job_id)

    def _validate_training_request(self, payload):
        request = dict(payload)
        request["rover_url"] = _private_rover_url(request.get("rover_url"))
        request["transfer_token"] = str(request.get("transfer_token") or "").strip()
        if len(request["transfer_token"]) < 24:
            raise ValueError("TRANSFER_TOKEN_REQUIRED")
        request["model_id"] = self.worker.safe_id(request.get("model_id"))
        mode = str(request.get("mode") or "BASE").strip().upper()
        if mode not in {"BASE", "QUICK"}:
            raise ValueError("TRAINING_MODE_MUST_BE_BASE_OR_QUICK")
        request["mode"] = mode
        request["sessions"] = list(
            dict.fromkeys(self.worker.safe_id(item) for item in request.get("sessions") or [])
        )
        request["correction_sessions"] = list(
            dict.fromkeys(
                self.worker.safe_id(item) for item in request.get("correction_sessions") or []
            )
        )
        if mode == "BASE" and len(request["sessions"]) < 3:
            raise ValueError("BASE_TRAINING_REQUIRES_AT_LEAST_3_RECORD_SESSIONS")
        if mode == "QUICK":
            request["base_model_id"] = self.worker.safe_id(request.get("base_model_id"))
            if not request["correction_sessions"]:
                raise ValueError("QUICK_TRAINING_REQUIRES_CORRECTION_SESSIONS")
        return request

    def snapshot(self, job_id):
        with self.lock:
            job = self.jobs.get(job_id)
            if not job:
                raise KeyError(job_id)
            # Do not expose the rover transfer token again through status APIs.
            value = json.loads(json.dumps(job))
            if value.get("request"):
                value["request"]["transfer_token"] = "<redacted>"
            return value

    def list(self):
        with self.lock:
            values = sorted(self.jobs.values(), key=lambda item: item["created_at"], reverse=True)[:50]
            result = []
            for item in values:
                copy = json.loads(json.dumps(item))
                if copy.get("request"):
                    copy["request"]["transfer_token"] = "<redacted>"
                copy.pop("artifact_token", None)
                result.append(copy)
            return result

    def cancel(self, job_id):
        with self.lock:
            job = self.jobs.get(job_id)
            if not job:
                raise KeyError(job_id)
            if job["state"] in JOB_TERMINAL:
                return self.snapshot(job_id)
            job["cancel_requested"] = True
            job["message"] = "취소 요청됨"
            return self.snapshot(job_id)

    def artifact(self, job_id, name, supplied_token):
        with self.lock:
            job = self.jobs.get(job_id)
            if not job:
                raise KeyError(job_id)
            if job["state"] != "SUCCEEDED":
                raise ValueError("JOB_NOT_SUCCEEDED")
            if not supplied_token or supplied_token != job.get("artifact_token"):
                raise PermissionError("ARTIFACT_TOKEN_REQUIRED")
            result = dict(job.get("result") or {})
        key = {
            "model": "model",
            "manifest": "manifest",
            "evaluation": "evaluation",
            "checkpoint": "checkpoint",
            "context": "context",
            "comparison": "comparison",
        }.get(name)
        if not key or not result.get(key):
            raise FileNotFoundError("ARTIFACT_NOT_FOUND")
        path = Path(result[key]).resolve()
        job_root = (self.worker.jobs_root / job_id).resolve()
        if job_root not in path.parents or not path.is_file():
            raise PermissionError("ARTIFACT_PATH_REJECTED")
        return path

    def _set(self, job_id, *, state=None, phase=None, progress=None, message=None, result=None, error=None, **extra):
        with self.lock:
            job = self.jobs[job_id]
            if state is not None:
                job["state"] = state
            if phase is not None:
                job["phase"] = phase
            if progress is not None:
                job["progress"] = max(0.0, min(1.0, float(progress)))
            if message is not None:
                job["message"] = str(message)
            if result is not None:
                job["result"] = result
            if error is not None:
                job["error"] = str(error)
            if extra:
                job.setdefault("telemetry", {}).update(extra)

    def _cancelled(self, job_id):
        with self.lock:
            return bool(self.jobs[job_id].get("cancel_requested"))

    def _loop(self):
        while not self.stop_event.is_set():
            with self.lock:
                job_id = self.queue.pop(0) if self.queue else None
            if not job_id:
                self.stop_event.wait(0.15)
                continue
            try:
                self._run(job_id)
            except Exception as error:
                with self.lock:
                    job = self.jobs.get(job_id)
                    if job and job["state"] not in {"SUCCEEDED", "CANCELED"}:
                        if job.get("cancel_requested") or str(error) == "JOB_CANCELLED":
                            job["state"] = "CANCELED"
                            job["phase"] = "CANCELED"
                            job["message"] = "취소됨"
                        else:
                            job["state"] = "FAILED"
                            job["phase"] = "FAILED"
                            job["message"] = "작업 실패"
                            job["error"] = f"{type(error).__name__}: {error}\n{traceback.format_exc(limit=6)}"
                        job["finished_at"] = time.time()

    def _run(self, job_id):
        with self.lock:
            job = self.jobs[job_id]
            if job["cancel_requested"]:
                job["state"] = "CANCELED"
                job["finished_at"] = time.time()
                return
            job["state"] = "RUNNING"
            job["started_at"] = time.time()
        if job["kind"] == "diagnostic":
            for index in range(5):
                if self._cancelled(job_id):
                    raise RuntimeError("JOB_CANCELLED")
                self._set(
                    job_id,
                    phase="DIAGNOSTIC",
                    progress=(index + 1) / 5,
                    message=f"연결 시험 {index + 1}/5",
                )
                time.sleep(0.08)
            result = {"status": self.worker.status()}
        else:
            result = self._run_training(job_id)
        self._set(
            job_id,
            state="SUCCEEDED",
            phase="SUCCEEDED",
            progress=1.0,
            message="완료",
            result=result,
        )
        with self.lock:
            self.jobs[job_id]["finished_at"] = time.time()

    def _run_training(self, job_id):
        with self.lock:
            request = dict(self.jobs[job_id]["request"])
        job_root = self.worker.jobs_root / job_id
        if job_root.exists():
            import shutil
            shutil.rmtree(job_root)
        job_root.mkdir(parents=True)
        cache = self.worker.recordings_root

        self._set(job_id, phase="SYNCING", progress=0.03, message="차량 RECORD 확인 중")
        sync = self.worker.sync_recordings(
            rover_url=request["rover_url"],
            token=request["transfer_token"],
            sessions=request["sessions"],
            progress=lambda done, total, message: self._set(
                job_id,
                phase="SYNCING",
                progress=0.03 + 0.12 * (done / max(1, total)),
                message=message,
            ),
            cancelled=lambda: self._cancelled(job_id),
        )

        base_checkpoint = None
        base_context = None
        if request["mode"] == "QUICK":
            self._set(job_id, phase="SYNCING", progress=0.15, message="기준 모델 학습 상태 가져오는 중")
            base_checkpoint = job_root / "base_checkpoint.pt"
            base_context = job_root / "base_context.json"
            self.worker.download_base_artifact(
                request["rover_url"], request["transfer_token"], request["base_model_id"],
                "checkpoint", base_checkpoint
            )
            self.worker.download_base_artifact(
                request["rover_url"], request["transfer_token"], request["base_model_id"],
                "context", base_context
            )

        def pipeline_progress(phase, percent, message=None, **extra):
            self._set(
                job_id,
                phase=phase,
                progress=max(0.16, min(0.98, float(percent) / 100.0)),
                message=message or phase,
                **extra,
            )

        pipeline = TrainingPipeline(
            str(cache),
            str(job_root),
            progress=pipeline_progress,
            cancelled=lambda: self._cancelled(job_id),
        )
        result = pipeline.run(
            model_id=request["model_id"],
            mode=request["mode"],
            sessions=request["sessions"],
            correction_sessions=request.get("correction_sessions") or [],
            base_model_id=request.get("base_model_id"),
            base_checkpoint=None if base_checkpoint is None else str(base_checkpoint),
            base_context=None if base_context is None else str(base_context),
            epochs=request.get("epochs"),
            target_hz=float(request.get("target_hz") or 10.0),
        )
        result["sync"] = sync
        result["worker_urls"] = self.worker.advertise_urls()
        result["model_id"] = request["model_id"]
        return result


class PipelineComputeWorker(ComputeWorker):
    def __init__(self, config=None):
        super().__init__(config)
        # Replace the legacy v0.1 queue cleanly; its thread exits on the flag.
        self.jobs.stop_event.set()
        self.recordings_root = self.cache_root / "recordings"
        self.jobs_root = self.data_root / "jobs"
        self.recordings_root.mkdir(parents=True, exist_ok=True)
        self.jobs_root.mkdir(parents=True, exist_ok=True)
        self.jobs = PipelineJobManager(self)

    def advertise_urls(self):
        addresses = set()
        try:
            for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                addresses.add(item[4][0])
        except OSError:
            pass
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.connect(("8.8.8.8", 80))
                addresses.add(sock.getsockname()[0])
            finally:
                sock.close()
        except OSError:
            pass
        result = []
        for address in sorted(addresses):
            try:
                ip = ipaddress.ip_address(address)
            except ValueError:
                continue
            if ip.is_private or ip.is_link_local:
                result.append(f"http://{address}:{self.config.port}")
        return result

    def status(self):
        value = super().status()
        value["version"] = "0.2.0"
        value["capabilities"].update(
            {
                "record_incremental_sync": True,
                "training_package_materialization": True,
                "base_training": True,
                "quick_fine_tune": True,
                "candidate_artifact_return": True,
            }
        )
        value["advertise_urls"] = self.advertise_urls()
        return value

    @staticmethod
    def _rover_request(url, token, *, range_start=None, timeout=20):
        headers = {"X-SWING-Transfer-Token": token, "Accept": "application/octet-stream"}
        if range_start is not None and range_start > 0:
            headers["Range"] = f"bytes={range_start}-"
        return Request(url, headers=headers, method="GET")

    def _get_manifest(self, rover_url, token, session):
        url = (
            rover_url.rstrip("/")
            + "/api/v2/compute/recording-manifest?session="
            + quote(session, safe="")
        )
        request = self._rover_request(url, token)
        request.add_header("Accept", "application/json")
        with urlopen(request, timeout=15) as response:
            return json.loads(response.read(4 * 1024 * 1024).decode("utf-8"))

    def sync_recordings(self, *, rover_url, token, sessions, progress, cancelled):
        rover_url = _private_rover_url(rover_url)
        manifests = [self._get_manifest(rover_url, token, session) for session in sessions]
        files = [
            (manifest["session"], item)
            for manifest in manifests
            for item in manifest.get("files") or []
        ]
        transferred = 0
        reused = 0
        for index, (session, remote) in enumerate(files, start=1):
            if cancelled():
                raise RuntimeError("JOB_CANCELLED")
            session_dir = self.recordings_root / self.safe_id(session)
            session_dir.mkdir(parents=True, exist_ok=True)
            destination = session_dir / remote["name"]
            manifest_path = session_dir / ".swing-cache.json"
            try:
                cache_document = json.loads(manifest_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, json.JSONDecodeError):
                cache_document = {"files": {}}
            cached = (cache_document.get("files") or {}).get(remote["name"]) or {}
            valid = (
                destination.is_file()
                and destination.stat().st_size == int(remote["size_bytes"])
                and cached.get("sha256") == remote["sha256"]
            )
            if valid:
                reused += int(remote["size_bytes"])
            else:
                self._download_record_file(
                    rover_url, token, session, remote, destination, cancelled
                )
                transferred += int(remote["size_bytes"])
                cache_document.setdefault("files", {})[remote["name"]] = remote
                cache_document["session"] = session
                temporary = Path(str(manifest_path) + ".tmp")
                temporary.write_text(
                    json.dumps(cache_document, ensure_ascii=False, indent=2), encoding="utf-8"
                )
                os.replace(temporary, manifest_path)
            progress(index, len(files), f"RECORD 동기화 {index}/{len(files)} · {session}")
        return {
            "sessions": sessions,
            "files": len(files),
            "transferred_bytes": transferred,
            "reused_bytes": reused,
        }

    def _download_record_file(self, rover_url, token, session, remote, destination, cancelled):
        part = Path(str(destination) + ".part")
        expected_size = int(remote["size_bytes"])
        offset = part.stat().st_size if part.is_file() else 0
        if offset > expected_size:
            part.unlink()
            offset = 0
        url = (
            rover_url.rstrip("/")
            + "/api/v2/compute/recording-file?session="
            + quote(session, safe="")
            + "&file="
            + quote(remote["name"], safe="")
        )
        request = self._rover_request(url, token, range_start=offset)
        try:
            response = urlopen(request, timeout=30)
        except HTTPError as error:
            if offset and error.code == 416:
                part.unlink(missing_ok=True)
                return self._download_record_file(
                    rover_url, token, session, remote, destination, cancelled
                )
            raise
        with response:
            append = offset > 0 and response.status == 206
            if offset > 0 and response.status == 200:
                offset = 0
            mode = "ab" if append else "wb"
            with open(part, mode) as output:
                while True:
                    if cancelled():
                        raise RuntimeError("JOB_CANCELLED")
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    output.write(chunk)
                output.flush()
                os.fsync(output.fileno())
        if part.stat().st_size != expected_size:
            raise OSError(f"RECORD_SIZE_MISMATCH:{session}/{remote['name']}")
        if _sha256(part) != remote["sha256"]:
            part.unlink(missing_ok=True)
            raise OSError(f"RECORD_SHA256_MISMATCH:{session}/{remote['name']}")
        os.replace(part, destination)

    def download_base_artifact(self, rover_url, token, model_id, kind, destination):
        rover_url = _private_rover_url(rover_url)
        url = (
            rover_url.rstrip("/")
            + "/api/v2/compute/base-file?model_id="
            + quote(self.safe_id(model_id), safe="")
            + "&kind="
            + quote(kind, safe="")
        )
        request = self._rover_request(url, token)
        temporary = Path(str(destination) + ".part")
        with urlopen(request, timeout=30) as response, open(temporary, "wb") as output:
            while True:
                chunk = response.read(1024 * 1024)
                if not chunk:
                    break
                output.write(chunk)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, destination)

    def serve_forever(self):
        worker = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "SWINGComputeWorker/0.2"

            def log_message(self, fmt, *args):
                return

            def _origin_allowed(self):
                origin = str(self.headers.get("Origin") or "").strip()
                if not origin:
                    return True
                try:
                    parsed = urlparse(origin)
                    if parsed.hostname in {"localhost", "127.0.0.1", "::1"}:
                        return True
                    addresses = [ipaddress.ip_address(parsed.hostname)]
                except ValueError:
                    try:
                        addresses = [
                            ipaddress.ip_address(item[4][0])
                            for item in socket.getaddrinfo(parsed.hostname, parsed.port or 80)
                        ]
                    except (OSError, UnboundLocalError):
                        return False
                return bool(addresses) and all(
                    ip.is_private or ip.is_loopback or ip.is_link_local for ip in addresses
                )

            def _write_allowed(self):
                try:
                    ip = ipaddress.ip_address(str(self.client_address[0]).split("%", 1)[0])
                    return bool(ip.is_private or ip.is_loopback or ip.is_link_local)
                except ValueError:
                    return False

            def _cors(self):
                origin = str(self.headers.get("Origin") or "").strip()
                if origin and self._origin_allowed():
                    self.send_header("Access-Control-Allow-Origin", origin)
                    self.send_header("Vary", "Origin")
                self.send_header(
                    "Access-Control-Allow-Headers",
                    "Content-Type,X-SWING-Artifact-Token",
                )
                self.send_header("Access-Control-Allow-Methods", "GET,POST,DELETE,OPTIONS")

            def _json(self, payload, status=200):
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self._cors()
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self, maximum=256 * 1024):
                length = int(self.headers.get("Content-Length") or 0)
                if length < 0 or length > maximum:
                    raise ValueError("REQUEST_BODY_TOO_LARGE")
                value = json.loads((self.rfile.read(length) if length else b"{}").decode("utf-8"))
                if not isinstance(value, dict):
                    raise ValueError("JSON_OBJECT_REQUIRED")
                return value

            def do_OPTIONS(self):
                if not self._origin_allowed():
                    self.send_error(403)
                    return
                self.send_response(204)
                self._cors()
                self.send_header("Content-Length", "0")
                self.end_headers()

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path in {"/health", "/api/v1/health"}:
                    self._json({"ok": True, "service": "swing-compute-worker", "version": "0.2.0"})
                    return
                if parsed.path == "/api/v1/status":
                    self._json(worker.status())
                    return
                if parsed.path == "/api/v1/jobs":
                    self._json({"jobs": worker.jobs.list()})
                    return
                parts = parsed.path.strip("/").split("/")
                if len(parts) == 4 and parts[:3] == ["api", "v1", "jobs"]:
                    try:
                        self._json(worker.jobs.snapshot(parts[3]))
                    except KeyError:
                        self._json({"error": "JOB_NOT_FOUND"}, 404)
                    return
                if len(parts) == 6 and parts[:3] == ["api", "v1", "jobs"] and parts[4] == "artifacts":
                    try:
                        path = worker.jobs.artifact(
                            parts[3], parts[5], self.headers.get("X-SWING-Artifact-Token")
                        )
                        size = path.stat().st_size
                        self.send_response(200)
                        self.send_header("Content-Type", "application/octet-stream")
                        self.send_header("Cache-Control", "private, no-store")
                        self.send_header("Content-Length", str(size))
                        self.end_headers()
                        with open(path, "rb") as file:
                            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                                self.wfile.write(chunk)
                    except KeyError:
                        self._json({"error": "JOB_NOT_FOUND"}, 404)
                    except PermissionError as error:
                        self._json({"error": str(error)}, 403)
                    except (ValueError, OSError) as error:
                        self._json({"error": str(error)}, 404)
                    return
                if parsed.path in {"/", "/status"}:
                    body = _status_page().encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self._json({"error": "NOT_FOUND"}, 404)

            def do_POST(self):
                if not self._origin_allowed() or not self._write_allowed():
                    self._json({"error": "PRIVATE_NETWORK_WRITE_REQUIRED"}, 403)
                    return
                if urlparse(self.path).path == "/api/v1/jobs":
                    try:
                        self._json(worker.jobs.create(self._read_json()), 202)
                    except (ValueError, TypeError, json.JSONDecodeError) as error:
                        self._json({"error": str(error)}, 400)
                    return
                self._json({"error": "NOT_FOUND"}, 404)

            def do_DELETE(self):
                if not self._origin_allowed() or not self._write_allowed():
                    self._json({"error": "PRIVATE_NETWORK_WRITE_REQUIRED"}, 403)
                    return
                parts = urlparse(self.path).path.strip("/").split("/")
                if len(parts) == 4 and parts[:3] == ["api", "v1", "jobs"]:
                    try:
                        self._json(worker.jobs.cancel(parts[3]), 202)
                    except KeyError:
                        self._json({"error": "JOB_NOT_FOUND"}, 404)
                    return
                self._json({"error": "NOT_FOUND"}, 404)

        self.httpd = ThreadingHTTPServer((self.config.host, self.config.port), Handler)
        self.httpd.daemon_threads = True
        self.httpd.serve_forever(poll_interval=0.25)


def main(argv=None):
    parser = argparse.ArgumentParser(description="SWING Compute Worker")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--data-root", default=None)
    parser.add_argument("--background", action="store_true")
    parser.add_argument("--open-status", action="store_true")
    args = parser.parse_args(argv)
    config = WorkerConfig.from_environment()
    if args.host:
        config.host = args.host
    if args.port:
        config.port = args.port
    if args.data_root:
        config.data_root = args.data_root
    if args.open_status:
        webbrowser.open(f"http://127.0.0.1:{config.port}/")
        return 0
    worker = PipelineComputeWorker(config)
    try:
        worker.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        worker.shutdown()
    return 0


__all__ = ["PipelineComputeWorker", "PipelineJobManager", "main"]
