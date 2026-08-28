from __future__ import annotations

import argparse
import importlib.util
import ipaddress
import json
import os
import platform
import shutil
import socket
import threading
import time
import traceback
import uuid
import webbrowser
from dataclasses import asdict, dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
    import psutil
except ImportError:  # pragma: no cover - optional in source checkout
    psutil = None


WORKER_VERSION = "0.1.0"
DEFAULT_PORT = 8765


@dataclass
class WorkerConfig:
    host: str = "0.0.0.0"
    port: int = DEFAULT_PORT
    data_root: str | None = None
    allow_private_network_writes: bool = True

    @classmethod
    def from_environment(cls) -> "WorkerConfig":
        root = os.environ.get("SWING_COMPUTE_DATA_ROOT")
        return cls(
            host=os.environ.get("SWING_COMPUTE_HOST", "0.0.0.0"),
            port=int(os.environ.get("SWING_COMPUTE_PORT", str(DEFAULT_PORT))),
            data_root=root or None,
            allow_private_network_writes=os.environ.get(
                "SWING_COMPUTE_PRIVATE_WRITES", "1"
            ).strip().lower()
            not in {"0", "false", "no"},
        )


class JobManager:
    """Small single-worker queue.

    The first release intentionally supports only deterministic diagnostic jobs
    and training from datasets already present in the managed cache. Remote
    RECORD synchronization is added on top of this contract rather than
    exposing arbitrary command execution or arbitrary filesystem paths.
    """

    def __init__(self, worker: "ComputeWorker"):
        self.worker = worker
        self.lock = threading.RLock()
        self.jobs: dict[str, dict] = {}
        self.queue: list[str] = []
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def create(self, payload: dict) -> dict:
        kind = str(payload.get("kind") or "").strip().lower()
        if kind not in {"diagnostic", "train_cached_dataset"}:
            raise ValueError("UNSUPPORTED_JOB_KIND")
        job_id = uuid.uuid4().hex[:12]
        now = time.time()
        job = {
            "job_id": job_id,
            "kind": kind,
            "state": "QUEUED",
            "created_at": now,
            "started_at": None,
            "finished_at": None,
            "progress": 0.0,
            "message": "queued",
            "request": payload,
            "result": None,
            "error": None,
            "cancel_requested": False,
        }
        with self.lock:
            self.jobs[job_id] = job
            self.queue.append(job_id)
        return self.snapshot(job_id)

    def cancel(self, job_id: str) -> dict:
        with self.lock:
            job = self.jobs.get(job_id)
            if not job:
                raise KeyError(job_id)
            if job["state"] in {"SUCCEEDED", "FAILED", "CANCELED"}:
                return dict(job)
            job["cancel_requested"] = True
            job["message"] = "cancel requested"
            return dict(job)

    def snapshot(self, job_id: str) -> dict:
        with self.lock:
            job = self.jobs.get(job_id)
            if not job:
                raise KeyError(job_id)
            return json.loads(json.dumps(job))

    def list(self) -> list[dict]:
        with self.lock:
            ordered = sorted(
                self.jobs.values(), key=lambda item: item["created_at"], reverse=True
            )
            return json.loads(json.dumps(ordered[:50]))

    def _loop(self):
        while not self.stop_event.is_set():
            job_id = None
            with self.lock:
                if self.queue:
                    job_id = self.queue.pop(0)
            if not job_id:
                self.stop_event.wait(0.2)
                continue
            try:
                self._run(job_id)
            except Exception:
                with self.lock:
                    job = self.jobs.get(job_id)
                    if job and job["state"] not in {"SUCCEEDED", "CANCELED"}:
                        job["state"] = "FAILED"
                        job["finished_at"] = time.time()
                        job["error"] = traceback.format_exc(limit=8)
                        job["message"] = "job failed"

    def _run(self, job_id: str):
        with self.lock:
            job = self.jobs[job_id]
            if job["cancel_requested"]:
                job["state"] = "CANCELED"
                job["finished_at"] = time.time()
                return
            job["state"] = "RUNNING"
            job["started_at"] = time.time()
            job["message"] = "running"

        if job["kind"] == "diagnostic":
            for index in range(10):
                with self.lock:
                    if job["cancel_requested"]:
                        job["state"] = "CANCELED"
                        job["finished_at"] = time.time()
                        job["message"] = "canceled"
                        return
                    job["progress"] = (index + 1) / 10.0
                    job["message"] = f"diagnostic {index + 1}/10"
                time.sleep(0.08)
            result = {"status": self.worker.status()}
        else:
            result = self._train_cached_dataset(job)

        with self.lock:
            if job["cancel_requested"]:
                job["state"] = "CANCELED"
                job["message"] = "canceled"
            else:
                job["state"] = "SUCCEEDED"
                job["progress"] = 1.0
                job["message"] = "completed"
                job["result"] = result
            job["finished_at"] = time.time()

    def _train_cached_dataset(self, job: dict) -> dict:
        request = dict(job.get("request") or {})
        dataset_id = self.worker.safe_id(request.get("dataset_id"))
        output_id = self.worker.safe_id(request.get("output_id") or f"model-{job['job_id']}")
        dataset_path = self.worker.datasets_root / dataset_id
        output_path = self.worker.models_root / output_id
        if not (dataset_path / "dataset.json").is_file():
            raise FileNotFoundError(f"CACHED_DATASET_NOT_FOUND:{dataset_id}")

        try:
            from autonomous_car.ai import Trainer, TrainingConfig
        except Exception as error:
            raise RuntimeError("TRAINING_RUNTIME_UNAVAILABLE") from error

        epochs = max(1, min(100, int(request.get("epochs", 30))))
        batch_size = max(1, min(256, int(request.get("batch_size", 32))))
        learning_rate = max(1e-7, min(0.1, float(request.get("learning_rate", 1e-3))))
        device = str(request.get("device") or "auto")
        config = TrainingConfig(
            epochs=epochs,
            batch_size=batch_size,
            learning_rate=learning_rate,
            device=device,
        )
        with self.lock:
            job["message"] = "training cached dataset"
            job["progress"] = 0.05
        trainer = Trainer(config=config)
        metrics = trainer.train(str(dataset_path), str(output_path))
        return {
            "dataset_id": dataset_id,
            "output_id": output_id,
            "output_path": str(output_path),
            "metrics": metrics,
        }


class ComputeWorker:
    def __init__(self, config: WorkerConfig | None = None):
        self.config = config or WorkerConfig.from_environment()
        self.started_monotonic = time.monotonic()
        self.hostname = socket.gethostname()
        self.data_root = Path(self.config.data_root or self._default_data_root()).resolve()
        self.cache_root = self.data_root / "cache"
        self.datasets_root = self.data_root / "datasets"
        self.models_root = self.data_root / "models"
        self.logs_root = self.data_root / "logs"
        for path in (
            self.data_root,
            self.cache_root,
            self.datasets_root,
            self.models_root,
            self.logs_root,
        ):
            path.mkdir(parents=True, exist_ok=True)
        self.jobs = JobManager(self)
        self.httpd: ThreadingHTTPServer | None = None

    @staticmethod
    def _default_data_root() -> str:
        base = os.environ.get("LOCALAPPDATA") or os.environ.get("APPDATA")
        if base:
            return str(Path(base) / "SWING Robotics" / "Compute Worker")
        return str(Path.home() / ".swing-compute-worker")

    @staticmethod
    def safe_id(value) -> str:
        text = str(value or "").strip()
        if not text or len(text) > 96:
            raise ValueError("INVALID_ID")
        if any(ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_." for ch in text):
            raise ValueError("INVALID_ID")
        if text in {".", ".."}:
            raise ValueError("INVALID_ID")
        return text

    @staticmethod
    def _module_available(name: str) -> bool:
        try:
            return importlib.util.find_spec(name) is not None
        except (ImportError, ValueError):
            return False

    def capabilities(self) -> dict:
        result = {
            "local_cpu_training": self._module_available("torch"),
            "onnx_runtime": self._module_available("onnxruntime"),
            "openvino": self._module_available("openvino"),
            "opencv": self._module_available("cv2"),
            "remote_gpu_training": False,
            "record_cache": True,
        }
        if result["local_cpu_training"]:
            try:
                import torch

                result["cuda"] = bool(torch.cuda.is_available())
                result["mps"] = bool(
                    getattr(torch.backends, "mps", None)
                    and torch.backends.mps.is_available()
                )
                if result["cuda"]:
                    result["gpu_name"] = str(torch.cuda.get_device_name(0))
                else:
                    result["gpu_name"] = None
            except Exception:
                result["cuda"] = False
                result["mps"] = False
                result["gpu_name"] = None
        else:
            result.update({"cuda": False, "mps": False, "gpu_name": None})
        return result

    def status(self) -> dict:
        memory = {"total_bytes": None, "available_bytes": None, "percent": None}
        cpu_percent = None
        if psutil is not None:
            try:
                vm = psutil.virtual_memory()
                memory = {
                    "total_bytes": int(vm.total),
                    "available_bytes": int(vm.available),
                    "percent": float(vm.percent),
                }
                cpu_percent = float(psutil.cpu_percent(interval=None))
            except Exception:
                pass
        disk = shutil.disk_usage(self.data_root)
        return {
            "service": "swing-compute-worker",
            "version": WORKER_VERSION,
            "hostname": self.hostname,
            "platform": platform.platform(),
            "python": platform.python_version(),
            "pid": os.getpid(),
            "uptime_seconds": max(0.0, time.monotonic() - self.started_monotonic),
            "listen": {"host": self.config.host, "port": self.config.port},
            "cpu": {
                "logical_count": os.cpu_count(),
                "percent": cpu_percent,
                "processor": platform.processor(),
            },
            "memory": memory,
            "disk": {
                "total_bytes": int(disk.total),
                "free_bytes": int(disk.free),
                "used_bytes": int(disk.used),
            },
            "data_root": str(self.data_root),
            "capabilities": self.capabilities(),
            "jobs": self.jobs.list()[:8],
        }

    def serve_forever(self):
        worker = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "SWINGComputeWorker/0.1"

            def log_message(self, fmt, *args):
                return

            def _origin_allowed(self) -> bool:
                origin = str(self.headers.get("Origin") or "").strip()
                if not origin:
                    return True
                try:
                    parsed = urlparse(origin)
                    host = parsed.hostname or ""
                    if host in {"localhost", "127.0.0.1", "::1"}:
                        return True
                    ip = ipaddress.ip_address(host)
                    return bool(ip.is_private or ip.is_loopback or ip.is_link_local)
                except ValueError:
                    return False

            def _client_write_allowed(self) -> bool:
                if not worker.config.allow_private_network_writes:
                    return self.client_address[0] in {"127.0.0.1", "::1"}
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
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
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

            def _read_json(self, maximum=128 * 1024):
                raw_length = self.headers.get("Content-Length")
                length = int(raw_length or 0)
                if length < 0 or length > maximum:
                    raise ValueError("REQUEST_BODY_TOO_LARGE")
                raw = self.rfile.read(length) if length else b"{}"
                value = json.loads(raw.decode("utf-8"))
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
                if self.path in {"/health", "/api/v1/health"}:
                    self._json({"ok": True, "service": "swing-compute-worker", "version": WORKER_VERSION})
                    return
                if self.path == "/api/v1/status":
                    self._json(worker.status())
                    return
                if self.path == "/api/v1/jobs":
                    self._json({"jobs": worker.jobs.list()})
                    return
                if self.path.startswith("/api/v1/jobs/"):
                    job_id = self.path.rsplit("/", 1)[-1]
                    try:
                        self._json(worker.jobs.snapshot(job_id))
                    except KeyError:
                        self._json({"error": "JOB_NOT_FOUND"}, 404)
                    return
                if self.path in {"/", "/status"}:
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
                if not self._origin_allowed() or not self._client_write_allowed():
                    self._json({"error": "PRIVATE_NETWORK_WRITE_REQUIRED"}, 403)
                    return
                if self.path == "/api/v1/jobs":
                    try:
                        job = worker.jobs.create(self._read_json())
                        self._json(job, 202)
                    except (ValueError, TypeError, json.JSONDecodeError) as error:
                        self._json({"error": str(error)}, 400)
                    return
                self._json({"error": "NOT_FOUND"}, 404)

            def do_DELETE(self):
                if not self._origin_allowed() or not self._client_write_allowed():
                    self._json({"error": "PRIVATE_NETWORK_WRITE_REQUIRED"}, 403)
                    return
                if self.path.startswith("/api/v1/jobs/"):
                    job_id = self.path.rsplit("/", 1)[-1]
                    try:
                        self._json(worker.jobs.cancel(job_id), 202)
                    except KeyError:
                        self._json({"error": "JOB_NOT_FOUND"}, 404)
                    return
                self._json({"error": "NOT_FOUND"}, 404)

        self.httpd = ThreadingHTTPServer((self.config.host, self.config.port), Handler)
        self.httpd.daemon_threads = True
        self.httpd.serve_forever(poll_interval=0.25)

    def shutdown(self):
        self.jobs.stop_event.set()
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()


def _status_page() -> str:
    return r'''<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SWING Compute Worker</title><style>
:root{color-scheme:dark;background:#111;color:#eee;font-family:Inter,system-ui,sans-serif}body{margin:0;padding:24px}.wrap{max-width:820px;margin:auto}.card{border:1px solid #444;border-radius:8px;background:#181818;padding:16px;margin:10px 0}h1{font-size:20px}h2{font-size:14px;margin:0 0 10px}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.item{padding:9px;border:1px solid #333;border-radius:6px}.item span{display:block;color:#999;font-size:11px}.item b{display:block;margin-top:4px;font:600 13px ui-monospace,monospace}.ok{color:#8dcf83}.warn{color:#d9bc73}pre{white-space:pre-wrap;word-break:break-word;font:11px ui-monospace,monospace;color:#aaa}@media(max-width:600px){.grid{grid-template-columns:1fr}}</style></head>
<body><div class="wrap"><h1>SWING Compute Worker</h1><div class="card"><h2>상태</h2><div id="summary" class="grid"></div></div><div class="card"><h2>기능</h2><div id="caps" class="grid"></div></div><div class="card"><h2>최근 작업</h2><pre id="jobs">-</pre></div></div>
<script>
async function refresh(){try{const r=await fetch('/api/v1/status',{cache:'no-store'});const s=await r.json();const gib=n=>n==null?'-':(n/1073741824).toFixed(1)+' GB';document.getElementById('summary').innerHTML=`<div class="item"><span>WORKER</span><b class="ok">연결됨 · ${s.version}</b></div><div class="item"><span>PC</span><b>${s.hostname}</b></div><div class="item"><span>CPU</span><b>${s.cpu.processor||'-'} · ${s.cpu.logical_count||'-'} threads</b></div><div class="item"><span>RAM</span><b>${gib(s.memory.total_bytes)}</b></div><div class="item"><span>DISK FREE</span><b>${gib(s.disk.free_bytes)}</b></div><div class="item"><span>DATA</span><b>${s.data_root}</b></div>`;const c=s.capabilities;document.getElementById('caps').innerHTML=`<div class="item"><span>LOCAL TRAINING</span><b class="${c.local_cpu_training?'ok':'warn'}">${c.local_cpu_training?'사용 가능':'사용 불가'}</b></div><div class="item"><span>CUDA GPU</span><b>${c.cuda?(c.gpu_name||'사용 가능'):'없음'}</b></div><div class="item"><span>OPENVINO</span><b class="${c.openvino?'ok':'warn'}">${c.openvino?'사용 가능':'사용 불가'}</b></div><div class="item"><span>ONNX RUNTIME</span><b class="${c.onnx_runtime?'ok':'warn'}">${c.onnx_runtime?'사용 가능':'사용 불가'}</b></div>`;document.getElementById('jobs').textContent=JSON.stringify(s.jobs,null,2)}catch(e){document.getElementById('summary').innerHTML='<div class="item"><span>WORKER</span><b class="warn">연결 실패</b></div>'}}
refresh();setInterval(refresh,1500);
</script></body></html>'''


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

    worker = ComputeWorker(config)
    try:
        worker.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        worker.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
