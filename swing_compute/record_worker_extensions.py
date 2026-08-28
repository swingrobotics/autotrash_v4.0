"""Extend the existing Compute Worker job queue with portable RECORD workflows."""

from __future__ import annotations

import json
import os
from pathlib import Path, PurePosixPath
import time
import uuid

from autonomous_car.recording.record_transfer import normalize_record_relative_path
from .frame_training_compat import install_frame_training_compat
from .record_import import discover_record_sources, import_sessions, list_usb_sessions
from .record_postprocess import postprocess_session
from . import pipeline_worker as pipeline_module


_INSTALLED = False
_EXTRA_KINDS = {
    "scan_usb_records",
    "import_usb_records",
    "sync_rover_records",
    "postprocess_record",
}


def _write_cache(path, document):
    temporary = Path(str(path) + ".tmp")
    temporary.parent.mkdir(parents=True, exist_ok=True)
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump(document, file, ensure_ascii=False, indent=2, sort_keys=True)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)


def install_record_worker_extensions():
    global _INSTALLED
    if _INSTALLED:
        return True

    install_frame_training_compat()
    manager_class = pipeline_module.PipelineJobManager
    worker_class = pipeline_module.PipelineComputeWorker
    original_create = manager_class.create
    original_run = manager_class._run
    original_status = worker_class.status

    def create(self, payload):
        request = dict(payload or {})
        kind = str(request.get("kind") or "").strip().lower()
        if kind not in _EXTRA_KINDS:
            return original_create(self, payload)

        if kind == "import_usb_records":
            request["sessions"] = list(
                dict.fromkeys(
                    self.worker.safe_id(item) for item in request.get("sessions") or []
                )
            )
            source_root = str(request.get("source_root") or "").strip()
            if source_root:
                resolved = str(Path(source_root).resolve())
                allowed = {str(Path(item).resolve()) for item in discover_record_sources()}
                if resolved not in allowed:
                    raise ValueError("USB_RECORD_SOURCE_NOT_DISCOVERED")
                request["source_root"] = resolved
            else:
                request["source_root"] = None
        elif kind == "sync_rover_records":
            request["rover_url"] = pipeline_module._private_rover_url(
                request.get("rover_url")
            )
            request["transfer_token"] = str(
                request.get("transfer_token") or ""
            ).strip()
            if len(request["transfer_token"]) < 24:
                raise ValueError("TRANSFER_TOKEN_REQUIRED")
            request["sessions"] = list(
                dict.fromkeys(
                    self.worker.safe_id(item) for item in request.get("sessions") or []
                )
            )
            if not request["sessions"]:
                raise ValueError("RECORD_SESSIONS_REQUIRED")
        elif kind == "postprocess_record":
            request["session"] = self.worker.safe_id(request.get("session"))
            request["make_h264"] = bool(request.get("make_h264", True))
            request["make_mcap"] = bool(request.get("make_mcap", False))

        job_id = uuid.uuid4().hex[:12]
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
            "request": request,
            "result": None,
            "error": None,
            "cancel_requested": False,
            "artifact_token": uuid.uuid4().hex + uuid.uuid4().hex,
            "worker_urls": self.worker.advertise_urls(),
        }
        with self.lock:
            self.jobs[job_id] = job
            self.queue.append(job_id)
        return self.snapshot(job_id)

    def _run(self, job_id):
        with self.lock:
            kind = self.jobs[job_id].get("kind")
        if kind not in _EXTRA_KINDS:
            return original_run(self, job_id)

        with self.lock:
            job = self.jobs[job_id]
            if job.get("cancel_requested"):
                job["state"] = "CANCELED"
                job["phase"] = "CANCELED"
                job["finished_at"] = time.time()
                return
            job["state"] = "RUNNING"
            job["phase"] = "RUNNING"
            job["started_at"] = time.time()
            request = dict(job.get("request") or {})

        if kind == "scan_usb_records":
            self._set(job_id, phase="SCANNING", progress=0.2, message="USB RECORD 검색 중")
            result = {
                "sources": discover_record_sources(),
                "sessions": list_usb_sessions(),
            }
        elif kind == "import_usb_records":
            self._set(job_id, phase="IMPORTING", progress=0.03, message="USB RECORD 가져오는 중")
            result = import_sessions(
                self.worker.recordings_root,
                sessions=request.get("sessions") or None,
                source_root=request.get("source_root"),
                cancelled=lambda: self._cancelled(job_id),
                progress=lambda done, total, message: self._set(
                    job_id,
                    phase="IMPORTING",
                    progress=0.03 + 0.94 * (done / max(1, total)),
                    message=message,
                ),
            )
        elif kind == "sync_rover_records":
            self._set(job_id, phase="SYNCING", progress=0.03, message="Pi RECORD 동기화 중")
            result = self.worker.sync_recordings(
                rover_url=request["rover_url"],
                token=request["transfer_token"],
                sessions=request["sessions"],
                progress=lambda done, total, message: self._set(
                    job_id,
                    phase="SYNCING",
                    progress=0.03 + 0.94 * (done / max(1, total)),
                    message=message,
                ),
                cancelled=lambda: self._cancelled(job_id),
            )
        else:
            session = request["session"]
            path = (self.worker.recordings_root / session).resolve()
            try:
                path.relative_to(self.worker.recordings_root.resolve())
            except ValueError as error:
                raise ValueError("RECORD_SESSION_PATH_REJECTED") from error
            if not path.is_dir():
                raise FileNotFoundError(f"CACHED_RECORD_NOT_FOUND:{session}")
            self._set(job_id, phase="POSTPROCESSING", progress=0.02, message="RECORD 후처리 중")
            result = postprocess_session(
                path,
                make_h264=request.get("make_h264", True),
                make_mcap=request.get("make_mcap", False),
                progress=lambda value, message: self._set(
                    job_id,
                    phase="POSTPROCESSING",
                    progress=max(0.02, min(0.98, float(value))),
                    message=message,
                ),
            )

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

    def sync_recordings(self, *, rover_url, token, sessions, progress, cancelled):
        """Incrementally mirror top-level telemetry plus nested camera frames.

        Cache metadata is kept in memory during bulk JPEG transfer and flushed
        periodically instead of rewriting a growing JSON file for every frame.
        Existing files with missing cache metadata are SHA-verified once and then
        adopted, so interrupted transfers resume without needless downloads.
        """

        rover_url = pipeline_module._private_rover_url(rover_url)
        manifests = [self._get_manifest(rover_url, token, session) for session in sessions]
        files = [
            (manifest["session"], dict(item))
            for manifest in manifests
            for item in manifest.get("files") or []
        ]
        transferred = 0
        reused = 0
        cache_states = {}

        def state_for(session):
            safe_session = self.safe_id(session)
            session_dir = (self.recordings_root / safe_session).resolve()
            try:
                session_dir.relative_to(self.recordings_root.resolve())
            except ValueError as error:
                raise ValueError("RECORD_SESSION_PATH_REJECTED") from error
            session_dir.mkdir(parents=True, exist_ok=True)
            state = cache_states.get(safe_session)
            if state is not None:
                return state
            manifest_path = session_dir / ".swing-cache.json"
            try:
                document = json.loads(manifest_path.read_text(encoding="utf-8"))
                if not isinstance(document, dict):
                    raise ValueError("invalid cache")
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                document = {"session": safe_session, "files": {}}
            if not isinstance(document.get("files"), dict):
                document["files"] = {}
            state = {
                "session_dir": session_dir,
                "manifest_path": manifest_path,
                "document": document,
                "dirty": 0,
            }
            cache_states[safe_session] = state
            return state

        def flush_state(state):
            if state["dirty"] <= 0:
                return
            _write_cache(state["manifest_path"], state["document"])
            state["dirty"] = 0

        try:
            for index, (session, remote) in enumerate(files, start=1):
                if cancelled():
                    raise RuntimeError("JOB_CANCELLED")
                relative_name = normalize_record_relative_path(remote.get("name"))
                remote["name"] = relative_name
                try:
                    expected_size = int(remote.get("size_bytes"))
                except (TypeError, ValueError) as error:
                    raise ValueError("INVALID_RECORD_REMOTE_SIZE") from error
                expected_sha = str(remote.get("sha256") or "").strip().lower()
                if (
                    expected_size < 0
                    or len(expected_sha) != 64
                    or any(ch not in "0123456789abcdef" for ch in expected_sha)
                ):
                    raise ValueError("INVALID_RECORD_REMOTE_DIGEST")

                state = state_for(session)
                session_dir = state["session_dir"]
                destination = (
                    session_dir / Path(*PurePosixPath(relative_name).parts)
                ).resolve()
                try:
                    destination.relative_to(session_dir)
                except ValueError as error:
                    raise ValueError("RECORD_CACHE_PATH_REJECTED") from error
                destination.parent.mkdir(parents=True, exist_ok=True)

                cached = (state["document"].get("files") or {}).get(relative_name) or {}
                valid = False
                if destination.is_file() and destination.stat().st_size == expected_size:
                    if str(cached.get("sha256") or "").lower() == expected_sha:
                        valid = True
                    elif pipeline_module._sha256(destination) == expected_sha:
                        valid = True
                        state["document"].setdefault("files", {})[relative_name] = remote
                        state["dirty"] += 1

                if valid:
                    reused += expected_size
                else:
                    self._download_record_file(
                        rover_url,
                        token,
                        session,
                        remote,
                        destination,
                        cancelled,
                    )
                    transferred += expected_size
                    state["document"].setdefault("files", {})[relative_name] = remote
                    state["document"]["session"] = self.safe_id(session)
                    state["dirty"] += 1

                if state["dirty"] >= 250:
                    flush_state(state)
                progress(
                    index,
                    len(files),
                    f"RECORD 동기화 {index}/{len(files)} · {session} · {relative_name}",
                )
        finally:
            for state in cache_states.values():
                flush_state(state)

        return {
            "sessions": [manifest.get("session") for manifest in manifests],
            "files": len(files),
            "transferred_bytes": transferred,
            "reused_bytes": reused,
            "nested_camera_frames": sum(
                1
                for _, remote in files
                if str(remote.get("name") or "").startswith("camera_frames/")
            ),
        }

    def status(self):
        value = original_status(self)
        value.setdefault("capabilities", {}).update(
            {
                "portable_usb_record_import": True,
                "standalone_rover_record_sync": True,
                "recursive_camera_frame_sync": True,
                "segmented_jpeg_training": True,
                "worker_h264_postprocess": True,
                "worker_mcap_export": _module_available("mcap"),
                "worker_bundled_ffmpeg": _module_available("imageio_ffmpeg"),
            }
        )
        return value

    def _module_available(name):
        try:
            import importlib.util

            return importlib.util.find_spec(name) is not None
        except Exception:
            return False

    manager_class.create = create
    manager_class._run = _run
    worker_class.sync_recordings = sync_recordings
    worker_class.status = status
    _INSTALLED = True
    return True


__all__ = ["install_record_worker_extensions"]
