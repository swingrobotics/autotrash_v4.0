"""Add synchronized RECORD model preview jobs to the Compute Worker."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import time
from urllib.parse import quote
from urllib.request import urlopen
import uuid

from autonomous_car.ai.record_preview import preview_record_session
from . import pipeline_worker as pipeline_module
from .record_postprocess import _ffmpeg_executable


_INSTALLED = False
_PREVIEW_KIND = "preview_record_model"
_MAX_MODEL_BYTES = 128 * 1024 * 1024
_MAX_MANIFEST_BYTES = 4 * 1024 * 1024


def _transcode_preview_h264(source, destination, cancelled):
    source = Path(source).resolve()
    destination = Path(destination).resolve()
    temporary = Path(str(destination) + ".part.mp4")
    if not source.is_file() or source.stat().st_size <= 0:
        raise OSError("PREVIEW_SOURCE_VIDEO_NOT_CREATED")
    command = [
        _ffmpeg_executable(),
        "-nostdin",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-i",
        str(source),
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "fast",
        "-crf",
        "23",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(temporary),
    ]
    process = subprocess.Popen(
        command,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
    )
    try:
        while process.poll() is None:
            if cancelled():
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
                raise RuntimeError("JOB_CANCELLED")
            time.sleep(0.20)
        details = (process.stderr.read() if process.stderr is not None else b"").decode(
            "utf-8", errors="replace"
        ).strip()
        if process.returncode != 0:
            raise RuntimeError(
                "PREVIEW_H264_TRANSCODE_FAILED"
                + (f":{details[-1600:]}" if details else "")
            )
        if not temporary.is_file() or temporary.stat().st_size <= 0:
            raise RuntimeError("PREVIEW_H264_TRANSCODE_EMPTY")
        os.replace(temporary, destination)
        return destination
    finally:
        temporary.unlink(missing_ok=True)
        if process.poll() is None:
            process.kill()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass


def install_record_preview_worker_extensions():
    global _INSTALLED
    if _INSTALLED:
        return True

    manager_class = pipeline_module.PipelineJobManager
    worker_class = pipeline_module.PipelineComputeWorker
    original_create = manager_class.create
    original_run = manager_class._run
    original_artifact = manager_class.artifact
    original_status = worker_class.status

    def create(self, payload):
        request = dict(payload or {})
        kind = str(request.get("kind") or "").strip().lower()
        if kind != _PREVIEW_KIND:
            return original_create(self, payload)

        request["kind"] = _PREVIEW_KIND
        request["rover_url"] = pipeline_module._private_rover_url(
            request.get("rover_url")
        )
        request["transfer_token"] = str(
            request.get("transfer_token") or ""
        ).strip()
        if len(request["transfer_token"]) < 24:
            raise ValueError("TRANSFER_TOKEN_REQUIRED")
        request["session"] = self.worker.safe_id(request.get("session"))
        request["model_id"] = self.worker.safe_id(request.get("model_id"))
        try:
            sample_every = int(request.get("sample_every") or 1)
        except (TypeError, ValueError) as error:
            raise ValueError("INVALID_PREVIEW_SAMPLE_EVERY") from error
        request["sample_every"] = max(1, min(10, sample_every))

        job_id = uuid.uuid4().hex[:12]
        now = time.time()
        job = {
            "job_id": job_id,
            "kind": _PREVIEW_KIND,
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
            if any(
                item.get("kind") == _PREVIEW_KIND
                and item.get("state") not in pipeline_module.JOB_TERMINAL
                for item in self.jobs.values()
            ):
                raise ValueError("A_RECORD_PREVIEW_JOB_IS_ALREADY_ACTIVE")
            self.jobs[job_id] = job
            self.queue.append(job_id)
        return self.snapshot(job_id)

    def _run(self, job_id):
        with self.lock:
            kind = self.jobs[job_id].get("kind")
        if kind != _PREVIEW_KIND:
            return original_run(self, job_id)

        with self.lock:
            job = self.jobs[job_id]
            if job.get("cancel_requested"):
                job["state"] = "CANCELED"
                job["phase"] = "CANCELED"
                job["finished_at"] = time.time()
                return
            job["state"] = "RUNNING"
            job["phase"] = "SYNCING"
            job["started_at"] = time.time()
            request = dict(job.get("request") or {})

        job_root = (self.worker.jobs_root / job_id).resolve()
        if job_root.exists():
            import shutil
            shutil.rmtree(job_root)
        job_root.mkdir(parents=True, exist_ok=False)

        self._set(
            job_id,
            phase="SYNCING",
            progress=0.03,
            message="선택한 RECORD를 PC로 동기화하는 중",
        )
        sync = self.worker.sync_recordings(
            rover_url=request["rover_url"],
            token=request["transfer_token"],
            sessions=[request["session"]],
            progress=lambda done, total, message: self._set(
                job_id,
                phase="SYNCING",
                progress=0.03 + 0.27 * (done / max(1, total)),
                message=message,
            ),
            cancelled=lambda: self._cancelled(job_id),
        )
        if self._cancelled(job_id):
            raise RuntimeError("JOB_CANCELLED")

        model_path = job_root / "preview-model.onnx"
        manifest_path = job_root / "model_manifest.json"
        self._set(
            job_id,
            phase="SYNCING_MODEL",
            progress=0.32,
            message="차량의 선택 모델을 가져오는 중",
        )
        self.worker.download_preview_model_file(
            request["rover_url"],
            request["transfer_token"],
            request["model_id"],
            "model",
            model_path,
            _MAX_MODEL_BYTES,
        )
        self.worker.download_preview_model_file(
            request["rover_url"],
            request["transfer_token"],
            request["model_id"],
            "manifest",
            manifest_path,
            _MAX_MANIFEST_BYTES,
        )

        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as error:
            raise ValueError("PREVIEW_MODEL_MANIFEST_INVALID") from error
        if not isinstance(manifest, dict):
            raise ValueError("PREVIEW_MODEL_MANIFEST_INVALID")
        policy_type = str(manifest.get("policy_type") or "AUTO_AI").strip().upper()
        if policy_type not in {"AUTO_AI", "AUTO_GPS"}:
            raise ValueError("PREVIEW_MODEL_POLICY_UNSUPPORTED")
        model_spec = manifest.get("model_spec") or {}
        try:
            auxiliary_size = int(model_spec.get("auxiliary_feature_size") or 2)
        except (TypeError, ValueError):
            auxiliary_size = 2
        temporal_gps = policy_type == "AUTO_GPS" and auxiliary_size > 2
        # Temporal GPS models were trained at the RECORD/control cadence. Never
        # sparsify their replay input: skipping frames changes both yaw history
        # and previous-steering history, making the preview incomparable to
        # training/live inference.
        effective_sample_every = 1 if temporal_gps else request["sample_every"]

        route_path = None
        route_id = str(manifest.get("route_id") or "").strip() or None
        if policy_type == "AUTO_GPS":
            if not route_id:
                raise ValueError("PREVIEW_GPS_ROUTE_REQUIRED")
            route_path = job_root / "gps-route.json"
            self._set(
                job_id,
                phase="SYNCING_ROUTE",
                progress=0.38,
                message="GPS 모델의 기준 Route를 가져오는 중",
            )
            self.worker.download_gps_route(
                request["rover_url"],
                request["transfer_token"],
                route_id,
                route_path,
            )

        cached_session = (self.worker.recordings_root / request["session"]).resolve()
        try:
            cached_session.relative_to(self.worker.recordings_root.resolve())
        except ValueError as error:
            raise ValueError("RECORD_SESSION_PATH_REJECTED") from error
        if not cached_session.is_dir():
            raise FileNotFoundError("CACHED_RECORD_NOT_FOUND")

        working_video = job_root / "record-model-preview-working.mp4"
        output_video = job_root / "record-model-preview.mp4"
        output_csv = job_root / "record-model-preview.csv"
        self._set(
            job_id,
            phase="MODEL_PREVIEW",
            progress=0.42,
            message=(
                "temporal GPS 이력을 유지하며 모든 프레임을 다시 계산하는 중"
                if temporal_gps
                else "기록된 센서로 AI 판단을 다시 계산하는 중"
            ),
        )

        def preview_progress(done, total):
            ratio = done / max(1, total)
            self._set(
                job_id,
                phase="MODEL_PREVIEW",
                progress=0.42 + 0.50 * ratio,
                message=f"AI 재계산 {done:,}/{max(1, total):,} frames",
            )

        summary = preview_record_session(
            str(cached_session),
            str(model_path),
            manifest_path=str(manifest_path),
            route_path=None if route_path is None else str(route_path),
            output_video=str(working_video),
            output_csv=str(output_csv),
            sample_every=effective_sample_every,
            progress_callback=preview_progress,
            cancelled=lambda: self._cancelled(job_id),
        ).as_dict()
        if self._cancelled(job_id):
            raise RuntimeError("JOB_CANCELLED")
        self._set(
            job_id,
            phase="ENCODING_PREVIEW",
            progress=0.94,
            message="브라우저용 H.264 영상으로 변환하는 중",
        )
        _transcode_preview_h264(
            working_video,
            output_video,
            cancelled=lambda: self._cancelled(job_id),
        )
        working_video.unlink(missing_ok=True)
        if self._cancelled(job_id):
            raise RuntimeError("JOB_CANCELLED")
        if not output_video.is_file() or output_video.stat().st_size <= 0:
            raise OSError("PREVIEW_VIDEO_NOT_CREATED")
        if not output_csv.is_file() or output_csv.stat().st_size <= 0:
            raise OSError("PREVIEW_CSV_NOT_CREATED")

        result = dict(summary)
        result.update(
            {
                "model_id": request["model_id"],
                "policy_type": policy_type,
                "route_id": route_id,
                "output_video": str(output_video),
                "preview_video": str(output_video),
                "preview_csv": str(output_csv),
                "video_codec": "H264_YUV420P_FASTSTART",
                "requested_sample_every": request["sample_every"],
                "effective_sample_every": effective_sample_every,
                "temporal_gps": temporal_gps,
                "sync": sync,
                "control_authority": "NONE",
            }
        )
        self._set(
            job_id,
            state="SUCCEEDED",
            phase="SUCCEEDED",
            progress=1.0,
            message="AI 계산 프리뷰 완료",
            result=result,
        )
        with self.lock:
            self.jobs[job_id]["finished_at"] = time.time()

    def artifact(self, job_id, name, supplied_token):
        if name not in {"preview_video", "preview_csv"}:
            return original_artifact(self, job_id, name, supplied_token)
        with self.lock:
            job = self.jobs.get(job_id)
            if not job:
                raise KeyError(job_id)
            if job.get("kind") != _PREVIEW_KIND:
                raise FileNotFoundError("ARTIFACT_NOT_FOUND")
            if job.get("state") != "SUCCEEDED":
                raise ValueError("JOB_NOT_SUCCEEDED")
            if not supplied_token or supplied_token != job.get("artifact_token"):
                raise PermissionError("ARTIFACT_TOKEN_REQUIRED")
            value = str((job.get("result") or {}).get(name) or "").strip()
        if not value:
            raise FileNotFoundError("ARTIFACT_NOT_FOUND")
        path = Path(value).resolve()
        job_root = (self.worker.jobs_root / job_id).resolve()
        try:
            path.relative_to(job_root)
        except ValueError as error:
            raise PermissionError("ARTIFACT_PATH_REJECTED") from error
        if not path.is_file():
            raise FileNotFoundError("ARTIFACT_NOT_FOUND")
        return path

    def download_preview_model_file(
        self,
        rover_url,
        token,
        model_id,
        kind,
        destination,
        maximum_bytes,
    ):
        rover_url = pipeline_module._private_rover_url(rover_url)
        model_id = self.safe_id(model_id)
        if kind not in {"model", "manifest"}:
            raise ValueError("PREVIEW_MODEL_FILE_KIND_INVALID")
        url = (
            rover_url.rstrip("/")
            + "/api/v2/compute/preview-model-file?model_id="
            + quote(model_id, safe="")
            + "&kind="
            + quote(kind, safe="")
        )
        request = self._rover_request(url, token)
        destination = Path(destination)
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(str(destination) + ".part")
        total = 0
        try:
            with urlopen(request, timeout=30) as response, open(temporary, "wb") as output:
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    total += len(chunk)
                    if total > int(maximum_bytes):
                        raise ValueError("PREVIEW_MODEL_FILE_TOO_LARGE")
                    output.write(chunk)
                output.flush()
                os.fsync(output.fileno())
            if total <= 0:
                raise OSError("PREVIEW_MODEL_FILE_EMPTY")
            os.replace(temporary, destination)
        except Exception:
            temporary.unlink(missing_ok=True)
            raise
        return destination

    def status(self):
        value = original_status(self)
        value.setdefault("capabilities", {})["record_model_preview"] = True
        value["capabilities"]["record_model_preview_artifacts"] = True
        value["capabilities"]["record_model_preview_h264"] = True
        value["capabilities"]["record_model_preview_temporal_gps"] = True
        return value

    manager_class.create = create
    manager_class._run = _run
    manager_class.artifact = artifact
    worker_class.download_preview_model_file = download_preview_model_file
    worker_class.status = status
    _INSTALLED = True
    return True


__all__ = ["install_record_preview_worker_extensions"]
