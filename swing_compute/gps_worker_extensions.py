"""Add route-bound AUTO_GPS training jobs to the Compute Worker."""

from __future__ import annotations

import json
from pathlib import Path
import time
from urllib.parse import quote
from urllib.request import urlopen
import uuid

from .gps_training_pipeline import GpsTrainingPipeline
from . import pipeline_worker as pipeline_module


_INSTALLED = False
_GPS_KIND = "train_gps_rover_records"
_TRAINING_KINDS = {"train_rover_records", _GPS_KIND}


def install_gps_worker_extensions():
    global _INSTALLED
    if _INSTALLED:
        return True

    manager_class = pipeline_module.PipelineJobManager
    worker_class = pipeline_module.PipelineComputeWorker
    original_create = manager_class.create
    original_run = manager_class._run
    original_status = worker_class.status

    def create(self, payload):
        request = dict(payload or {})
        kind = str(request.get("kind") or "").strip().lower()
        if kind != _GPS_KIND:
            # The pre-existing AUTO_AI path knows nothing about the GPS job
            # kind. Hold the shared RLock across its create call so a generic
            # training job cannot race into the queue while GPS training is
            # already queued/running.
            if kind == "train_rover_records":
                with self.lock:
                    if any(
                        item.get("kind") == _GPS_KIND
                        and item.get("state") not in pipeline_module.JOB_TERMINAL
                        for item in self.jobs.values()
                    ):
                        raise ValueError("A_TRAINING_JOB_IS_ALREADY_ACTIVE")
                    return original_create(self, payload)
            return original_create(self, payload)

        request["kind"] = _GPS_KIND
        request["rover_url"] = pipeline_module._private_rover_url(request.get("rover_url"))
        request["transfer_token"] = str(request.get("transfer_token") or "").strip()
        if len(request["transfer_token"]) < 24:
            raise ValueError("TRANSFER_TOKEN_REQUIRED")
        request["model_id"] = self.worker.safe_id(request.get("model_id"))
        request["route_id"] = self.worker.safe_id(request.get("route_id"))
        request["policy_type"] = "AUTO_GPS"
        request["mode"] = "BASE"
        request["sessions"] = list(
            dict.fromkeys(
                self.worker.safe_id(item) for item in request.get("sessions") or []
            )
        )
        if len(request["sessions"]) < 3:
            raise ValueError("GPS_BASE_TRAINING_REQUIRES_AT_LEAST_3_RECORD_SESSIONS")
        try:
            epochs = int(request.get("epochs") or 30)
        except (TypeError, ValueError) as error:
            raise ValueError("INVALID_GPS_TRAINING_EPOCHS") from error
        request["epochs"] = max(1, min(60, epochs))

        job_id = uuid.uuid4().hex[:12]
        token = uuid.uuid4().hex + uuid.uuid4().hex
        now = time.time()
        job = {
            "job_id": job_id,
            "kind": _GPS_KIND,
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
            "artifact_token": token,
            "worker_urls": self.worker.advertise_urls(),
        }
        with self.lock:
            if any(
                item.get("kind") in _TRAINING_KINDS
                and item.get("state") not in pipeline_module.JOB_TERMINAL
                for item in self.jobs.values()
            ):
                raise ValueError("A_TRAINING_JOB_IS_ALREADY_ACTIVE")
            self.jobs[job_id] = job
            self.queue.append(job_id)
        return self.snapshot(job_id)

    def _run(self, job_id):
        with self.lock:
            kind = self.jobs[job_id].get("kind")
        if kind != _GPS_KIND:
            return original_run(self, job_id)

        with self.lock:
            job = self.jobs[job_id]
            if job.get("cancel_requested"):
                job["state"] = "CANCELED"
                job["phase"] = "CANCELED"
                job["finished_at"] = time.time()
                return
            job["state"] = "RUNNING"
            job["started_at"] = time.time()
            request = dict(job.get("request") or {})

        job_root = self.worker.jobs_root / job_id
        if job_root.exists():
            import shutil
            shutil.rmtree(job_root)
        job_root.mkdir(parents=True, exist_ok=True)

        self._set(job_id, phase="SYNCING", progress=0.03, message="GPS 학습용 차량 RECORD 확인 중")
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

        route_path = job_root / "gps-route.json"
        self._set(job_id, phase="SYNCING_ROUTE", progress=0.16, message="정규화 GPS Route 가져오는 중")
        self.worker.download_gps_route(
            request["rover_url"],
            request["transfer_token"],
            request["route_id"],
            route_path,
        )
        if self._cancelled(job_id):
            raise RuntimeError("JOB_CANCELLED")

        def pipeline_progress(phase, percent, message=None, **extra):
            self._set(
                job_id,
                phase=phase,
                progress=max(0.17, min(0.98, float(percent) / 100.0)),
                message=message or phase,
                **extra,
            )

        result = GpsTrainingPipeline(
            str(self.worker.recordings_root),
            str(job_root),
            progress=pipeline_progress,
            cancelled=lambda: self._cancelled(job_id),
        ).run(
            model_id=request["model_id"],
            sessions=request["sessions"],
            route_path=str(route_path),
            epochs=request.get("epochs"),
        )
        if str(result.get("route_id") or "") != request["route_id"]:
            raise ValueError("GPS_WORKER_ROUTE_ID_MISMATCH")
        result["sync"] = sync
        result["worker_urls"] = self.worker.advertise_urls()
        result["model_id"] = request["model_id"]
        result["mode"] = "BASE"
        result["policy_type"] = "AUTO_GPS"

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

    def download_gps_route(self, rover_url, token, route_id, destination):
        rover_url = pipeline_module._private_rover_url(rover_url)
        route_id = self.safe_id(route_id)
        url = (
            rover_url.rstrip("/")
            + "/api/v2/compute/gps-route?route_id="
            + quote(route_id, safe="")
        )
        request = self._rover_request(url, token)
        request.add_header("Accept", "application/json")
        destination = Path(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(str(destination) + ".part")
        with urlopen(request, timeout=15) as response:
            raw = response.read(8 * 1024 * 1024)
            if len(raw) >= 8 * 1024 * 1024:
                raise OSError("GPS_ROUTE_RESPONSE_TOO_LARGE")
        document = json.loads(raw.decode("utf-8"))
        if not isinstance(document, dict):
            raise ValueError("GPS_ROUTE_RESPONSE_INVALID")
        if str(document.get("route_id") or "") != route_id:
            raise ValueError("GPS_ROUTE_RESPONSE_ID_MISMATCH")
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2)
            file.flush()
        temporary.replace(destination)
        return destination

    def status(self):
        value = original_status(self)
        value.setdefault("capabilities", {}).update(
            {
                "gps_conditioned_training": True,
                "gps_route_transfer": True,
                "gps_segmented_jpeg_training": True,
                "gps_conditional_fix_training": True,
            }
        )
        return value

    manager_class.create = create
    manager_class._run = _run
    worker_class.download_gps_route = download_gps_route
    worker_class.status = status
    _INSTALLED = True
    return True


__all__ = ["install_gps_worker_extensions"]
