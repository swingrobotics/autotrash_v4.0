"""Low-latency UFLD perception extension for the SWING Compute Worker.

This endpoint is intentionally separate from the long-running training job queue.
It never owns steering or motor authority.  When an inference is already running,
a newly arriving request is rejected immediately instead of being queued so the
rover can continue with the newest camera frame (latest-frame-wins policy).
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import socket
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from autonomous_car.perception.pretrained_road import DEFAULT_MODEL_FILENAME, DEFAULT_MODEL_NAME
from autonomous_car.perception.virtual_camera import (
    normalize_camera_mount_profile,
    project_points,
    warp_virtual_camera,
)
from third_party.ufld import decode_tusimple_output, prepare_tusimple_input
from .pipeline_worker import PipelineComputeWorker
from .worker import WorkerConfig, _status_page

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover
    cv2 = None
    np = None

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover
    ort = None


MAX_JPEG_BYTES = 4 * 1024 * 1024
MAX_MODEL_BYTES = 512 * 1024 * 1024


def _sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _decode_profile_header(value):
    if not value:
        return normalize_camera_mount_profile()
    try:
        padding = "=" * ((4 - len(value) % 4) % 4)
        document = json.loads(base64.urlsafe_b64decode((value + padding).encode("ascii")).decode("utf-8"))
    except Exception as error:
        raise ValueError("INVALID_CAMERA_PROFILE_HEADER") from error
    return normalize_camera_mount_profile(document)


def _decode_calibration_header(value):
    if not value:
        return None, None
    try:
        padding = "=" * ((4 - len(value) % 4) % 4)
        document = json.loads(base64.urlsafe_b64decode((value + padding).encode("ascii")).decode("utf-8"))
        matrix = document.get("camera_matrix")
        size = document.get("image_size")
        if matrix is None:
            return None, None
        return matrix, size
    except Exception as error:
        raise ValueError("INVALID_CAMERA_CALIBRATION_HEADER") from error


class UfldLiveEngine:
    def __init__(self, worker):
        self.worker = worker
        configured = str(os.environ.get("SWING_UFLD_MODEL_PATH") or "").strip()
        if configured:
            self.model_path = Path(configured).expanduser().resolve()
        else:
            self.model_path = (
                worker.data_root / "models" / "pretrained" / DEFAULT_MODEL_FILENAME
            ).resolve()
        self.model_path.parent.mkdir(parents=True, exist_ok=True)
        self.threshold = max(
            0.05,
            min(0.99, float(os.environ.get("SWING_UFLD_LANE_THRESHOLD", "0.55"))),
        )
        self.threads = max(1, int(os.environ.get("SWING_UFLD_WORKER_THREADS", str(max(2, (os.cpu_count() or 4) // 2)))))
        self._load_lock = threading.RLock()
        self._inference_lock = threading.Lock()
        self._session = None
        self._input_name = None
        self._output_name = None
        self._provider = None
        self._error = None
        self._runs = 0
        self._drops = 0
        self._last_inference_ms = None
        self._last_worker_ms = None
        self._last_lane_count = 0
        self._last_frame_id = None

    def _choose_providers(self):
        if ort is None:
            return []
        available = list(ort.get_available_providers())
        requested = str(os.environ.get("SWING_UFLD_ORT_PROVIDER") or "auto").strip()
        if requested and requested.lower() != "auto":
            if requested not in available:
                raise RuntimeError(f"UFLD_PROVIDER_UNAVAILABLE:{requested}")
            return [requested]
        preferred = [
            "CUDAExecutionProvider",
            "DmlExecutionProvider",
            "OpenVINOExecutionProvider",
            "CPUExecutionProvider",
        ]
        selected = [name for name in preferred if name in available]
        return selected or available

    def ensure_loaded(self):
        with self._load_lock:
            if self._session is not None:
                return True
            self._error = None
            if cv2 is None or np is None:
                self._error = "OpenCV/NumPy unavailable"
                return False
            if ort is None:
                self._error = "onnxruntime unavailable"
                return False
            if not self.model_path.is_file():
                self._error = f"model missing: {self.model_path}"
                return False
            try:
                options = ort.SessionOptions()
                options.intra_op_num_threads = self.threads
                options.inter_op_num_threads = 1
                options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
                options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
                providers = self._choose_providers()
                session = ort.InferenceSession(
                    str(self.model_path), sess_options=options, providers=providers
                )
                inputs = session.get_inputs()
                outputs = session.get_outputs()
                if len(inputs) != 1 or len(outputs) != 1:
                    raise RuntimeError("UFLD_MODEL_IO_CONTRACT_INVALID")
                self._session = session
                self._input_name = str(inputs[0].name)
                self._output_name = str(outputs[0].name)
                active = session.get_providers()
                self._provider = active[0] if active else None
                return True
            except Exception as error:
                self._session = None
                self._error = f"{type(error).__name__}: {error}"
                return False

    def unload(self):
        with self._load_lock:
            self._session = None
            self._input_name = None
            self._output_name = None
            self._provider = None

    def status(self):
        return {
            "service": "ufld-live",
            "model": DEFAULT_MODEL_NAME,
            "model_path": str(self.model_path),
            "model_present": self.model_path.is_file(),
            "model_bytes": self.model_path.stat().st_size if self.model_path.is_file() else 0,
            "loaded": self._session is not None,
            "available": bool(
                cv2 is not None
                and np is not None
                and ort is not None
                and self.model_path.is_file()
            ),
            "provider": self._provider,
            "available_providers": [] if ort is None else list(ort.get_available_providers()),
            "threads": self.threads,
            "lane_probability_threshold": self.threshold,
            "runs": self._runs,
            "busy_drops": self._drops,
            "last_inference_ms": self._last_inference_ms,
            "last_worker_ms": self._last_worker_ms,
            "last_lane_count": self._last_lane_count,
            "last_frame_id": self._last_frame_id,
            "error": self._error,
        }

    def install_model(self, stream, length, expected_sha256=None):
        length = int(length)
        if length <= 0 or length > MAX_MODEL_BYTES:
            raise ValueError("UFLD_MODEL_SIZE_REJECTED")
        if self._inference_lock.locked():
            raise RuntimeError("UFLD_INFERENCE_BUSY")
        temporary = Path(str(self.model_path) + ".part")
        digest = hashlib.sha256()
        remaining = length
        with open(temporary, "wb") as output:
            while remaining > 0:
                chunk = stream.read(min(1024 * 1024, remaining))
                if not chunk:
                    raise OSError("UFLD_MODEL_UPLOAD_INCOMPLETE")
                output.write(chunk)
                digest.update(chunk)
                remaining -= len(chunk)
            output.flush()
            os.fsync(output.fileno())
        actual = digest.hexdigest()
        if expected_sha256 and actual.lower() != str(expected_sha256).strip().lower():
            temporary.unlink(missing_ok=True)
            raise ValueError("UFLD_MODEL_SHA256_MISMATCH")
        os.replace(temporary, self.model_path)
        self.unload()
        if not self.ensure_loaded():
            raise RuntimeError(self._error or "UFLD_MODEL_LOAD_FAILED")
        return {
            "installed": True,
            "bytes": length,
            "sha256": actual,
            "status": self.status(),
        }

    def infer(self, jpeg, *, frame_id=None, profile=None, camera_matrix=None, calibration_size=None):
        if not jpeg:
            raise ValueError("CAMERA_FRAME_UNAVAILABLE")
        if len(jpeg) > MAX_JPEG_BYTES:
            raise ValueError("CAMERA_FRAME_TOO_LARGE")
        if not self._inference_lock.acquire(blocking=False):
            self._drops += 1
            raise BlockingIOError("UFLD_BUSY_DROP_FRAME")
        worker_started = time.perf_counter()
        try:
            if not self.ensure_loaded():
                raise RuntimeError(self._error or "UFLD_MODEL_UNAVAILABLE")
            image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("JPEG_DECODE_FAILED")
            profile = normalize_camera_mount_profile(profile)
            transformed, _, inverse = warp_virtual_camera(
                image,
                profile,
                camera_matrix=camera_matrix,
                calibration_size=calibration_size,
            )
            tensor = prepare_tusimple_input(transformed, cv2)
            started = time.perf_counter()
            output = self._session.run(
                [self._output_name], {self._input_name: tensor}
            )[0]
            inference_ms = (time.perf_counter() - started) * 1000.0
            height, width = transformed.shape[:2]
            lanes, confidences = decode_tusimple_output(
                output,
                (width, height),
                confidence_threshold=self.threshold,
            )
            restored = []
            for lane in lanes:
                points = project_points(lane.get("points") or [], inverse)
                restored.append({**lane, "points": points, "coordinate_space": "UNDISTORTED_SOURCE"})
            worker_ms = (time.perf_counter() - worker_started) * 1000.0
            self._runs += 1
            self._last_inference_ms = float(inference_ms)
            self._last_worker_ms = float(worker_ms)
            self._last_lane_count = len(restored)
            self._last_frame_id = frame_id
            return {
                "frame_id": frame_id,
                "lanes": restored,
                "inference_ms": float(inference_ms),
                "worker_processing_ms": float(worker_ms),
                "lane_count": len(restored),
                "max_lane_probability": max(confidences) if confidences else None,
                "model": DEFAULT_MODEL_NAME,
                "decoder": "EXTERNAL_UFLD_TUSIMPLE",
                "decoder_adapter": "third_party.ufld",
                "provider": self._provider,
                "camera_normalization": profile,
                "control_authority": "NONE",
            }
        finally:
            self._inference_lock.release()


class UfldPipelineComputeWorker(PipelineComputeWorker):
    def __init__(self, config=None):
        super().__init__(config)
        self.ufld = UfldLiveEngine(self)

    def status(self):
        value = super().status()
        value["version"] = "0.3.0"
        ufld = self.ufld.status()
        value["capabilities"].update(
            {
                "ufld_live_inference": True,
                "ufld_latest_frame_wins": True,
                "ufld_model_present": ufld["model_present"],
                "ufld_provider": ufld["provider"],
            }
        )
        value["ufld"] = ufld
        return value

    def serve_forever(self):
        worker = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "SWINGComputeWorker/0.3"

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
                    "Content-Type,X-SWING-Artifact-Token,X-SWING-Frame-Id,X-SWING-Camera-Profile,X-SWING-Camera-Calibration,X-SWING-SHA256",
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
                    self._json({"ok": True, "service": "swing-compute-worker", "version": "0.3.0"})
                    return
                if parsed.path == "/api/v1/status":
                    self._json(worker.status())
                    return
                if parsed.path == "/api/v1/perception/ufld/status":
                    self._json(worker.ufld.status())
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
                path = urlparse(self.path).path
                if path == "/api/v1/perception/ufld":
                    try:
                        length = int(self.headers.get("Content-Length") or 0)
                        if length <= 0 or length > MAX_JPEG_BYTES:
                            raise ValueError("CAMERA_FRAME_SIZE_REJECTED")
                        jpeg = self.rfile.read(length)
                        if len(jpeg) != length:
                            raise OSError("CAMERA_FRAME_INCOMPLETE")
                        profile = _decode_profile_header(self.headers.get("X-SWING-Camera-Profile"))
                        camera_matrix, calibration_size = _decode_calibration_header(
                            self.headers.get("X-SWING-Camera-Calibration")
                        )
                        result = worker.ufld.infer(
                            jpeg,
                            frame_id=self.headers.get("X-SWING-Frame-Id"),
                            profile=profile,
                            camera_matrix=camera_matrix,
                            calibration_size=calibration_size,
                        )
                        self._json(result)
                    except BlockingIOError as error:
                        self._json({"error": str(error), "drop_frame": True}, 429)
                    except (ValueError, TypeError, OSError) as error:
                        self._json({"error": str(error)}, 400)
                    except RuntimeError as error:
                        self._json({"error": str(error), "ufld": worker.ufld.status()}, 503)
                    return
                if path == "/api/v1/perception/ufld/model":
                    try:
                        length = int(self.headers.get("Content-Length") or 0)
                        result = worker.ufld.install_model(
                            self.rfile,
                            length,
                            self.headers.get("X-SWING-SHA256"),
                        )
                        self._json(result, 201)
                    except (ValueError, TypeError, OSError) as error:
                        self._json({"error": str(error)}, 400)
                    except RuntimeError as error:
                        self._json({"error": str(error)}, 409)
                    return
                if path == "/api/v1/jobs":
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
    parser = argparse.ArgumentParser(description="SWING Compute Worker + live UFLD")
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
    worker = UfldPipelineComputeWorker(config)
    try:
        worker.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        worker.shutdown()
    return 0


__all__ = ["UfldLiveEngine", "UfldPipelineComputeWorker", "main"]
