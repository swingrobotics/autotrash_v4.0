"""Rover-side Compute Worker UFLD bridge.

Heavy UFLD inference runs on the PC Worker.  The Raspberry Pi keeps lane-pair
validation, freshness/latency checks, SafetySupervisor, E-STOP, LiDAR safety,
steering and motor authority.  The classical OpenCV lane detector is not used as
a fallback.  Camera mount geometry comes from camera_mount_calibration.py so the
operator preview and Worker inference share one physical camera configuration.
"""

from __future__ import annotations

import base64
import hashlib
import http.client
import ipaddress
import json
import math
import os
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen

import server_v2_release as release
from camera_mount_calibration import CAMERA_MOUNT_SETTINGS
from autonomous_car.control.hybrid_lane_controller import HybridLaneController
from autonomous_car.control.lane_controller import LaneResult
from autonomous_car.perception.pretrained_road import (
    DEFAULT_MODEL_FILENAME,
    DEFAULT_MODEL_INPUT,
    DEFAULT_MODEL_NAME,
)
from autonomous_car.perception.virtual_camera import normalize_camera_mount_profile

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


_INSTALLED = False
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
WORKER_SETTINGS_PATH = os.environ.get(
    "SWING_UFLD_WORKER_SETTINGS_PATH",
    os.path.join(PROJECT_ROOT, "calibration", "worker-ufld.json"),
)


def _atomic_json(path, document):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump(document, file, ensure_ascii=False, indent=2, sort_keys=True)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)


def _private_worker_url(value):
    text = str(value or "").strip().rstrip("/")
    if not text:
        return ""
    parsed = urlparse(text)
    if parsed.scheme != "http" or not parsed.hostname or parsed.port != 8765:
        raise ValueError("WORKER_URL_MUST_BE_PRIVATE_HTTP_PORT_8765")
    try:
        ip = ipaddress.ip_address(parsed.hostname.split("%", 1)[0])
    except ValueError as error:
        raise ValueError("WORKER_URL_MUST_USE_PRIVATE_IP") from error
    if not (ip.is_private or ip.is_loopback or ip.is_link_local):
        raise ValueError("WORKER_URL_MUST_USE_PRIVATE_IP")
    return text


def _vehicle_disarmed():
    snapshot = release.full.legacy.vehicle_state_machine.snapshot()
    mode = str(snapshot.get("canonical_mode") or snapshot.get("mode") or "").upper()
    return mode == "DISARMED" and not release.full.legacy.record_manager.active


class WorkerUfldSettings:
    DEFAULTS = {
        "worker_enabled": True,
        "worker_url": "",
        "worker_timeout_ms": 350.0,
        # The primary camera-mount UI currently measures height/roll/pitch/yaw
        # and lateral offset.  Keep front/back offset here until that UI gains a
        # dedicated field; it is still part of the shared ground-plane profile.
        "longitudinal_offset_m": 0.0,
    }

    def __init__(self, path):
        self.path = path
        self.lock = threading.RLock()
        self.document = self._load()

    def _load(self):
        value = {}
        try:
            with open(self.path, "r", encoding="utf-8") as file:
                loaded = json.load(file)
            if isinstance(loaded, dict):
                value = loaded
        except (OSError, ValueError, json.JSONDecodeError):
            pass
        result = dict(self.DEFAULTS)
        result.update(value)
        result["worker_enabled"] = bool(result.get("worker_enabled", True))
        result["worker_url"] = _private_worker_url(result.get("worker_url")) if result.get("worker_url") else ""
        result["worker_timeout_ms"] = max(80.0, min(1000.0, float(result.get("worker_timeout_ms", 350.0))))
        result["longitudinal_offset_m"] = max(-2.0, min(2.0, float(result.get("longitudinal_offset_m", 0.0))))
        return result

    def worker_snapshot(self):
        with self.lock:
            return dict(self.document)

    def camera_profile(self):
        mount = CAMERA_MOUNT_SETTINGS.snapshot().get("settings") or {}
        worker = self.worker_snapshot()
        return normalize_camera_mount_profile(
            {
                "height_m": mount.get("height_m", 0.42),
                "pitch_degrees": mount.get("pitch_deg", -12.0),
                "roll_degrees": mount.get("roll_deg", 0.0),
                "yaw_degrees": mount.get("yaw_deg", 0.0),
                "lateral_offset_m": mount.get("lateral_offset_m", 0.0),
                "longitudinal_offset_m": worker.get("longitudinal_offset_m", 0.0),
                "target_height_m": mount.get("target_height_m", 1.20),
                "target_pitch_degrees": mount.get("target_pitch_deg", -4.0),
                "perspective_strength": mount.get("perspective_strength", 0.0),
            }
        )

    def snapshot(self):
        return {
            "worker": self.worker_snapshot(),
            "camera_profile": self.camera_profile(),
            "camera_mount": CAMERA_MOUNT_SETTINGS.snapshot(),
        }

    def update(self, payload):
        if not _vehicle_disarmed():
            raise ValueError("WORKER_UFLD_EDIT_REQUIRES_DISARMED")
        payload = dict(payload or {})
        with self.lock:
            result = dict(self.document)
            if "worker_enabled" in payload:
                result["worker_enabled"] = bool(payload.get("worker_enabled"))
            if "worker_url" in payload:
                result["worker_url"] = _private_worker_url(payload.get("worker_url"))
            if "worker_timeout_ms" in payload:
                result["worker_timeout_ms"] = max(80.0, min(1000.0, float(payload.get("worker_timeout_ms"))))
            if "longitudinal_offset_m" in payload:
                result["longitudinal_offset_m"] = max(-2.0, min(2.0, float(payload.get("longitudinal_offset_m"))))
            result["updated_at"] = time.time()
            _atomic_json(self.path, result)
            self.document = result
            return self.snapshot()


SETTINGS = WorkerUfldSettings(WORKER_SETTINGS_PATH)
_BASE_LANE_CONTROLLER = getattr(
    getattr(release.full.legacy, "auto_route_runtime", None),
    "lane_controller",
    None,
)
_ORIGINAL_PRETRAINED = getattr(_BASE_LANE_CONTROLLER, "pretrained", None)
LOCAL_UFLD_MODEL_PATH = getattr(_ORIGINAL_PRETRAINED, "model_path", None) or os.environ.get(
    "PRETRAINED_ROAD_MODEL_PATH",
    os.path.join(PROJECT_ROOT, "models", "pretrained", DEFAULT_MODEL_FILENAME),
)
CAMERA_CALIBRATION = getattr(release.full.legacy, "camera_calibration", None)


def _header_document(document):
    raw = json.dumps(document, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return base64.urlsafe_b64encode(raw).decode("ascii").rstrip("=")


class WorkerUfldPerception:
    """PretrainedRoadPerception-compatible remote inference adapter."""

    def __init__(
        self,
        model_path=None,
        input_size=DEFAULT_MODEL_INPUT,
        threads=2,
        lane_probability_threshold=0.55,
        profile_store=None,
        camera_calibration=None,
    ):
        self.model_path = os.path.abspath(str(model_path or LOCAL_UFLD_MODEL_PATH or "worker-ufld.onnx"))
        self.input_width = int(input_size[0])
        self.input_height = int(input_size[1])
        self.threads = max(1, int(threads))
        self.lane_probability_threshold = max(0.05, min(0.99, float(lane_probability_threshold)))
        self.settings = profile_store or SETTINGS
        self.camera_calibration = camera_calibration if camera_calibration is not None else CAMERA_CALIBRATION
        self._lock = threading.RLock()
        self._frame_id = 0
        self._error = None
        self._last = None
        self._runs = 0

    @property
    def available(self):
        worker = self.settings.worker_snapshot()
        return bool(worker.get("worker_enabled") and worker.get("worker_url"))

    @property
    def loaded(self):
        return bool(self._last and self._last.get("worker_model_loaded"))

    def _worker_status(self):
        worker = self.settings.worker_snapshot()
        url = _private_worker_url(worker.get("worker_url"))
        if not worker.get("worker_enabled"):
            raise RuntimeError("WORKER_UFLD_DISABLED")
        if not url:
            raise RuntimeError("WORKER_UFLD_URL_NOT_CONFIGURED")
        timeout = max(0.08, float(worker.get("worker_timeout_ms", 350.0)) / 1000.0)
        request = Request(url + "/api/v1/perception/ufld/status", headers={"Accept": "application/json"}, method="GET")
        with urlopen(request, timeout=timeout) as response:
            return json.loads(response.read(1024 * 1024).decode("utf-8"))

    def ensure_loaded(self):
        try:
            status = self._worker_status()
            ready = bool(status.get("model_present") and status.get("available"))
            with self._lock:
                self._error = None if ready else status.get("error") or "WORKER_UFLD_MODEL_MISSING"
                self._last = {**dict(self._last or {}), "worker_model_loaded": bool(status.get("loaded")), "worker_status": status}
            return ready
        except Exception as error:
            with self._lock:
                self._error = f"{type(error).__name__}: {error}"
            return False

    def unload(self):
        with self._lock:
            self._last = None

    def infer(self, image):
        if image is None:
            raise ValueError("camera image unavailable")
        if cv2 is None:
            raise RuntimeError("OpenCV unavailable for JPEG transport")
        worker = self.settings.worker_snapshot()
        if not worker.get("worker_enabled"):
            raise RuntimeError("WORKER_UFLD_DISABLED")
        worker_url = _private_worker_url(worker.get("worker_url"))
        if not worker_url:
            raise RuntimeError("WORKER_UFLD_URL_NOT_CONFIGURED")
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        if not ok:
            raise RuntimeError("WORKER_UFLD_JPEG_ENCODE_FAILED")
        jpeg = encoded.tobytes()
        with self._lock:
            self._frame_id += 1
            frame_id = str(self._frame_id)
        calibration_header = {}
        data = getattr(self.camera_calibration, "data", None)
        if isinstance(data, dict) and data.get("camera_matrix"):
            calibration_header = {
                "camera_matrix": data.get("camera_matrix"),
                "image_size": data.get("image_size"),
            }
        headers = {
            "Content-Type": "image/jpeg",
            "Accept": "application/json",
            "Content-Length": str(len(jpeg)),
            "X-SWING-Frame-Id": frame_id,
            "X-SWING-Camera-Profile": _header_document(self.settings.camera_profile()),
        }
        if calibration_header:
            headers["X-SWING-Camera-Calibration"] = _header_document(calibration_header)
        timeout = max(0.08, float(worker.get("worker_timeout_ms", 350.0)) / 1000.0)
        started = time.perf_counter()
        request = Request(worker_url + "/api/v1/perception/ufld", data=jpeg, headers=headers, method="POST")
        try:
            with urlopen(request, timeout=timeout) as response:
                result = json.loads(response.read(4 * 1024 * 1024).decode("utf-8"))
        except HTTPError as error:
            try:
                payload = json.loads(error.read(1024 * 1024).decode("utf-8"))
                message = payload.get("error") or f"WORKER_HTTP_{error.code}"
            except Exception:
                message = f"WORKER_HTTP_{error.code}"
            raise RuntimeError(message) from error
        except (URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"WORKER_UFLD_UNREACHABLE:{error}") from error
        end_to_end_ms = (time.perf_counter() - started) * 1000.0
        if str(result.get("frame_id")) != frame_id:
            raise RuntimeError("WORKER_UFLD_FRAME_ID_MISMATCH")
        worker_inference_ms = float(result.get("inference_ms") or 0.0)
        result["worker_inference_ms"] = worker_inference_ms
        result["worker_processing_ms"] = float(result.get("worker_processing_ms") or worker_inference_ms)
        result["network_roundtrip_ms"] = max(0.0, end_to_end_ms - result["worker_processing_ms"])
        result["end_to_end_ms"] = float(end_to_end_ms)
        # Remote safety uses complete camera->Worker->rover latency, not only
        # the ONNX session time inside the PC.
        result["inference_ms"] = float(end_to_end_ms)
        result["remote_worker"] = worker_url
        result["control_authority"] = "NONE"
        with self._lock:
            self._runs += 1
            self._error = None
            self._last = {**dict(result), "worker_model_loaded": True}
        return result

    def snapshot(self):
        with self._lock:
            worker = self.settings.worker_snapshot()
            return {
                "model": DEFAULT_MODEL_NAME,
                "model_path": self.model_path,
                "model_present_on_rover": bool(self.model_path and os.path.isfile(self.model_path)),
                "available": self.available,
                "loaded": self.loaded,
                "input_size": [self.input_width, self.input_height],
                "threads": self.threads,
                "lane_probability_threshold": self.lane_probability_threshold,
                "remote": True,
                "worker_url": worker.get("worker_url"),
                "worker_enabled": worker.get("worker_enabled"),
                "runs": self._runs,
                "last": dict(self._last or {}),
                "error": self._error,
            }


class UfldOnlyHybridLaneController(HybridLaneController):
    """UFLD-only controller: fail closed instead of classical CV fallback."""

    BACKEND = "UFLD_ROAD"

    @property
    def available(self):
        return bool(getattr(self.pretrained, "available", False))

    @staticmethod
    def _pair_score(left, right, center, width):
        confidence = left["confidence"] + right["confidence"]
        lane_center = (left["observed_bottom_x"] + right["observed_bottom_x"]) / 2.0
        center_error = abs(lane_center - center) / max(1.0, width)
        left_distance = max(0.0, center - left["observed_bottom_x"])
        right_distance = max(0.0, right["observed_bottom_x"] - center)
        symmetry_error = abs(left_distance - right_distance) / max(1.0, width)
        semantic_hint = (0.04 if left["lane_id"] == 1 else 0.0) + (0.04 if right["lane_id"] == 2 else 0.0)
        return confidence + semantic_hint - 0.75 * center_error - 0.30 * symmetry_error

    def _select_pair(self, candidates):
        width = self.processing_width
        center = width / 2.0
        guard = max(8.0, width * 0.012)
        lefts = [item for item in candidates if item["observed_bottom_x"] < center - guard]
        rights = [item for item in candidates if item["observed_bottom_x"] > center + guard]
        pairs = []
        for left in lefts:
            for right in rights:
                if left is right:
                    continue
                bottom_width = right["bottom_x"] - left["bottom_x"]
                top_width = right["top_x"] - left["top_x"]
                if bottom_width <= 18.0 or top_width <= 6.0:
                    continue
                ratio = bottom_width / max(1.0, top_width)
                if ratio < 1.005 or ratio > 14.0:
                    continue
                if not left["observed_bottom_x"] < center < right["observed_bottom_x"]:
                    continue
                center_bottom = (left["bottom_x"] + right["bottom_x"]) / 2.0
                center_error = abs(center_bottom - center) / max(1.0, width)
                if center_error > 0.42:
                    continue
                score = self._pair_score(left, right, center, width)
                score -= 0.08 * abs(math.log(max(1.0, ratio)) - math.log(2.5))
                pairs.append((score, left, right, ratio, center_error))
        if not pairs:
            return None, None
        pairs.sort(key=lambda item: item[0], reverse=True)
        score, left, right, ratio, center_error = pairs[0]
        target = self._last_preview if self._last_preview is not None else self._last_neural
        if isinstance(target, dict):
            target.update(
                {
                    "pair_score": float(score),
                    "pair_perspective_ratio": float(ratio),
                    "pair_center_error_normalized": float(center_error),
                    "pair_candidates_evaluated": len(pairs),
                }
            )
        return left, right

    def _classical_result(self, image, fallback=False):
        return LaneResult(
            False,
            0.0,
            error="CLASSICAL_LANE_DETECTION_REMOVED",
            backend=self.NEURAL_BACKEND,
            marking="UFLD_ONLY",
        )

    def analyze_image(self, image):
        if image is None:
            return LaneResult(False, 0.0, error="CAMERA_FRAME_UNAVAILABLE", backend=self.NEURAL_BACKEND)
        with self._lock:
            if not self._neural_enabled:
                self._last_backend = self.NEURAL_BACKEND
                self._fallback_reason = "NEURAL_DISABLED"
                return LaneResult(False, 0.0, error="NEURAL_DISABLED", backend=self.NEURAL_BACKEND)
            if self._neural_suspended_reason is not None:
                self._last_backend = self.NEURAL_BACKEND
                self._fallback_reason = self._neural_suspended_reason
                return LaneResult(False, 0.0, error=self._neural_suspended_reason, backend=self.NEURAL_BACKEND)
            try:
                result = self._neural_result(image)
                self._last_backend = result.backend
                self._fallback_reason = None
                return result
            except Exception as error:
                self._fallback_reason = f"{type(error).__name__}: {error}"
                self._last_backend = self.NEURAL_BACKEND
                return LaneResult(
                    False,
                    0.0,
                    error=str(error),
                    backend=self.NEURAL_BACKEND,
                    marking="UFLD_ONLY",
                    image_size=(self.processing_width, self.processing_height),
                )

    def snapshot(self):
        value = super().snapshot()
        value["classical_lane_detection"] = "REMOVED"
        value["fallback_policy"] = "FAIL_CLOSED_NO_OPENCV"
        return value


REMOTE_PRETRAINED = WorkerUfldPerception(
    model_path=LOCAL_UFLD_MODEL_PATH,
    input_size=(
        getattr(_ORIGINAL_PRETRAINED, "input_width", DEFAULT_MODEL_INPUT[0]),
        getattr(_ORIGINAL_PRETRAINED, "input_height", DEFAULT_MODEL_INPUT[1]),
    ),
    threads=getattr(_ORIGINAL_PRETRAINED, "threads", 2),
    lane_probability_threshold=getattr(_ORIGINAL_PRETRAINED, "lane_probability_threshold", 0.55),
    profile_store=SETTINGS,
    camera_calibration=CAMERA_CALIBRATION,
)


def _sync_model_to_worker():
    if not _vehicle_disarmed():
        raise ValueError("UFLD_MODEL_SYNC_REQUIRES_DISARMED")
    path = str(LOCAL_UFLD_MODEL_PATH or "")
    if not path or not os.path.isfile(path):
        raise FileNotFoundError("ROVER_UFLD_MODEL_MISSING")
    worker = SETTINGS.worker_snapshot()
    worker_url = _private_worker_url(worker.get("worker_url"))
    if not worker_url:
        raise ValueError("WORKER_UFLD_URL_NOT_CONFIGURED")
    parsed = urlparse(worker_url)
    size = os.path.getsize(path)
    digest = hashlib.sha256()
    with open(path, "rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    sha256 = digest.hexdigest()
    connection = http.client.HTTPConnection(parsed.hostname, parsed.port, timeout=180)
    try:
        connection.putrequest("POST", "/api/v1/perception/ufld/model")
        connection.putheader("Content-Type", "application/octet-stream")
        connection.putheader("Content-Length", str(size))
        connection.putheader("X-SWING-SHA256", sha256)
        connection.endheaders()
        with open(path, "rb") as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                connection.send(chunk)
        response = connection.getresponse()
        raw = response.read(2 * 1024 * 1024)
        try:
            document = json.loads(raw.decode("utf-8"))
        except Exception:
            document = {"error": raw.decode("utf-8", errors="replace")[:500]}
        if response.status not in {200, 201}:
            raise OSError(document.get("error") or f"WORKER_HTTP_{response.status}")
        return {"worker_url": worker_url, "source": path, **document}
    finally:
        connection.close()


def _build_runtime_controller(previous):
    return UfldOnlyHybridLaneController(
        pretrained=REMOTE_PRETRAINED,
        camera_calibration=getattr(previous, "camera_calibration", CAMERA_CALIBRATION),
        expected_lane_width_m=getattr(previous, "expected_lane_width_m", 1.0),
        vehicle_width_m=getattr(previous, "vehicle_width_m", 0.4826),
        processing_width=getattr(previous, "processing_width", 640),
        processing_height=getattr(previous, "processing_height", 360),
        maximum_neural_inference_ms=getattr(
            previous,
            "maximum_neural_inference_ms",
            160.0,
        ),
    )


def install_worker_ufld_bridge():
    global _INSTALLED
    if _INSTALLED:
        return True

    runtime = getattr(release.full.legacy, "auto_route_runtime", None)
    if runtime is None:
        raise RuntimeError("AUTO_ROUTE_RUNTIME_UNAVAILABLE")
    previous = getattr(runtime, "lane_controller", None)
    if isinstance(previous, UfldOnlyHybridLaneController):
        controller = previous
        controller.pretrained = REMOTE_PRETRAINED
    else:
        controller = _build_runtime_controller(previous)
        runtime.lane_controller = controller

    # This bridge is explicitly UFLD-only. Enable the neural path immediately;
    # missing/unreachable Worker results fail closed in analyze_image rather than
    # silently falling back to the classical detector.
    controller.set_neural_enabled(True)
    controller._last_backend = controller.NEURAL_BACKEND
    controller._fallback_reason = "WORKER_UFLD_NOT_READY"
    # Compatibility alias for diagnostics/older code. The authoritative slot is
    # legacy.auto_route_runtime.lane_controller.
    release.full.HYBRID_LANE_CONTROLLER = controller

    handler = release.ReleaseHandler
    original_get = handler.do_GET
    original_post = handler.do_POST
    if not getattr(original_get, "_swing_worker_ufld_bridge", False):
        def do_get_with_worker_ufld(self):
            path = str(self.path or "").split("?", 1)[0]
            if path == "/api/v2/perception/ufld-worker":
                self._send_json(
                    {
                        **SETTINGS.snapshot(),
                        "editable": _vehicle_disarmed(),
                        "remote_perception": REMOTE_PRETRAINED.snapshot(),
                        "local_model_path": LOCAL_UFLD_MODEL_PATH,
                        "local_model_present": bool(LOCAL_UFLD_MODEL_PATH and os.path.isfile(LOCAL_UFLD_MODEL_PATH)),
                        "control_authority": "PI_ONLY",
                    }
                )
                return
            return original_get(self)

        def do_post_with_worker_ufld(self):
            path = str(self.path or "").split("?", 1)[0]
            if path == "/api/v2/perception/ufld-worker":
                try:
                    self._send_json(SETTINGS.update(self._read_json()), 202)
                except Exception as error:
                    self._send_json({"error": str(error), **SETTINGS.snapshot()}, 409)
                return
            if path == "/api/v2/perception/ufld-worker/sync-model":
                try:
                    try:
                        self._read_json()
                    except Exception:
                        pass
                    self._send_json(_sync_model_to_worker(), 201)
                except Exception as error:
                    self._send_json({"error": str(error)}, 409)
                return
            return original_post(self)

        do_get_with_worker_ufld._swing_worker_ufld_bridge = True
        do_post_with_worker_ufld._swing_worker_ufld_bridge = True
        handler.do_GET = do_get_with_worker_ufld
        handler.do_POST = do_post_with_worker_ufld
    _INSTALLED = True
    return True


CAMERA_WORKER_UFLD_HMI = r'''
<style>
#camera-worker-ufld-panel .uw-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:7px}
#camera-worker-ufld-panel .uw-cell{padding:8px;border:1px solid var(--line);border-radius:5px;background:rgba(255,255,255,.012)}#camera-worker-ufld-panel .uw-cell span{display:block;color:var(--muted);font-size:8px}#camera-worker-ufld-panel .uw-cell strong{display:block;margin-top:4px;font:700 10px ui-monospace,monospace;overflow:hidden;text-overflow:ellipsis}
#camera-worker-ufld-panel .uw-fields{display:grid;grid-template-columns:2fr 1fr 1fr 1fr;gap:7px;margin-top:9px}#camera-worker-ufld-panel .uw-field{display:grid;gap:4px}#camera-worker-ufld-panel .uw-field label{font-size:8px;color:var(--muted)}#camera-worker-ufld-panel .uw-field input,#camera-worker-ufld-panel .uw-field select{width:100%}
#camera-worker-ufld-panel .uw-actions{display:flex;gap:7px;align-items:center;flex-wrap:wrap;margin-top:9px}#camera-worker-ufld-panel .uw-note{margin-top:8px;color:var(--muted);font-size:9px;line-height:1.5}
@media(max-width:850px){#camera-worker-ufld-panel .uw-grid,#camera-worker-ufld-panel .uw-fields{grid-template-columns:repeat(2,minmax(0,1fr))}}@media(max-width:520px){#camera-worker-ufld-panel .uw-grid,#camera-worker-ufld-panel .uw-fields{grid-template-columns:1fr}}
</style>
<script>
(function(){
 const grid=document.querySelector('#view-system .grid');if(!grid)return;
 const panel=document.createElement('div');panel.id='camera-worker-ufld-panel';panel.className='panel span12';panel.innerHTML=`
  <h2>Worker UFLD 실시간 차선 인식</h2>
  <p class="sectionnote">무거운 UFLD ONNX 추론은 Windows PC에서 실행합니다. Pi는 결과 frame-id·전체 지연·ego-lane geometry를 검증한 뒤에만 사용하며 모터/조향/Safety/E-STOP 권한은 넘기지 않습니다.</p>
  <div class="uw-grid">
   <div class="uw-cell"><span>WORKER</span><strong id="uw-state">확인 중</strong></div>
   <div class="uw-cell"><span>UFLD MODEL</span><strong id="uw-model">-</strong></div>
   <div class="uw-cell"><span>PROVIDER</span><strong id="uw-provider">-</strong></div>
   <div class="uw-cell"><span>LAST E2E</span><strong id="uw-latency">-</strong></div>
  </div>
  <div class="uw-fields">
   <div class="uw-field"><label>Worker LAN URL</label><input id="uw-url" placeholder="http://192.168.x.x:8765"></div>
   <div class="uw-field"><label>Timeout ms</label><input id="uw-timeout" type="number" step="10"></div>
   <div class="uw-field"><label>카메라 앞/뒤 Offset m</label><input id="uw-longitudinal" type="number" step="0.01"></div>
   <div class="uw-field"><label>Remote UFLD</label><select id="uw-enabled"><option value="1">사용</option><option value="0">사용 안 함</option></select></div>
  </div>
  <div class="uw-actions"><button id="uw-camera">카메라 장착값 열기</button><button id="uw-save" class="primary">Worker 설정 저장</button><button id="uw-sync">UFLD 모델 Worker로 동기화</button><span id="uw-pill" class="pill">localhost Worker 확인 중</span></div>
  <div id="uw-note" class="uw-note">OpenCV 차선 검출 fallback은 제거되었습니다. Worker 응답이 없거나 전체 지연이 기준을 넘으면 UFLD 결과를 사용하지 않습니다.</div>`;
 grid.appendChild(panel);
 const el=id=>document.getElementById(id);let rover=null,worker=null;
 async function api(path,options={}){const r=await fetch(path,{cache:'no-store',...options});let d={};try{d=await r.json()}catch{}if(!r.ok)throw new Error(d.error||`HTTP ${r.status}`);return d}
 const post=(path,body)=>api(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});
 function fill(){const w=rover?.worker||{};el('uw-url').value=w.worker_url||'';el('uw-timeout').value=w.worker_timeout_ms||350;el('uw-longitudinal').value=w.longitudinal_offset_m||0;el('uw-enabled').value=w.worker_enabled?'1':'0';el('uw-save').disabled=!rover?.editable;const last=rover?.remote_perception?.last||{};el('uw-latency').textContent=Number.isFinite(Number(last.end_to_end_ms))?`${Number(last.end_to_end_ms).toFixed(1)} ms`:'-'}
 async function refreshRover(){try{rover=await api('/api/v2/perception/ufld-worker');fill()}catch(e){el('uw-note').textContent='Pi Worker UFLD 상태 실패 · '+e.message}}
 async function refreshWorker(){try{const r=await fetch('http://127.0.0.1:8765/api/v1/status',{cache:'no-store'});if(!r.ok)throw new Error('HTTP '+r.status);worker=await r.json();const u=worker.ufld||{};const urls=worker.advertise_urls||[];if(!el('uw-url').value&&urls.length)el('uw-url').value=urls[0];el('uw-state').textContent=`v${worker.version||'-'} · ${worker.hostname||'PC'}`;el('uw-model').textContent=u.model_present?'READY':'동기화 필요';el('uw-provider').textContent=u.provider||((u.available_providers||[])[0]||'-');el('uw-pill').textContent=u.model_present?'Worker UFLD 준비됨':'Worker 연결됨 · 모델 필요';el('uw-pill').className='pill good'}catch(e){worker=null;el('uw-state').textContent='미연결';el('uw-model').textContent='-';el('uw-provider').textContent='-';el('uw-pill').textContent='Worker 미연결/업데이트 필요';el('uw-pill').className='pill warn'}}
 el('uw-camera').onclick=()=>{const m=document.getElementById('camera-mount-modal');if(m){m.hidden=false}else alert('카메라 장착 보정 UI를 찾지 못했습니다.')};
 el('uw-save').onclick=async()=>{try{await post('/api/v2/perception/ufld-worker',{worker_url:el('uw-url').value.trim(),worker_timeout_ms:Number(el('uw-timeout').value),longitudinal_offset_m:Number(el('uw-longitudinal').value),worker_enabled:el('uw-enabled').value==='1'});el('uw-note').textContent='Worker UFLD 설정 저장 완료';await refreshRover()}catch(e){el('uw-note').textContent='저장 실패 · '+e.message}};
 el('uw-sync').onclick=async()=>{if(!confirm('Pi의 UFLD 모델을 선택한 Worker PC로 한 번 복사할까요?'))return;el('uw-sync').disabled=true;el('uw-note').textContent='UFLD 모델을 Worker로 전송 중…';try{const r=await post('/api/v2/perception/ufld-worker/sync-model',{});el('uw-note').textContent=`모델 동기화 완료 · ${(Number(r.bytes||0)/1048576).toFixed(1)} MB`;await refreshWorker()}catch(e){el('uw-note').textContent='모델 동기화 실패 · '+e.message}finally{el('uw-sync').disabled=false}};
 refreshRover();refreshWorker();setInterval(()=>{refreshRover();refreshWorker()},3000);
})();
</script>
'''.encode('utf-8')


__all__ = [
    "CAMERA_WORKER_UFLD_HMI",
    "REMOTE_PRETRAINED",
    "SETTINGS",
    "WorkerUfldPerception",
    "install_worker_ufld_bridge",
]
