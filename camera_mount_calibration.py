"""Camera mount/extrinsic calibration and virtual-camera preview for SWING_CAR.

This module intentionally keeps camera mounting geometry separate from the existing
ChArUco intrinsic calibration.  The saved values describe how the physical camera
is installed on the rover plus a conservative virtual camera target used only for
preview/vision preprocessing.  It never owns motor or steering authority.
"""

from __future__ import annotations

import json
import math
import os
import threading
from urllib.parse import parse_qs, urlsplit

import server_v2_release as release
from autonomous_car.state import DriveMode

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - reported through the preview endpoint
    cv2 = None
    np = None


class CameraMountSettingsError(ValueError):
    pass


class CameraMountSettings:
    DEFAULT_PATH = "/home/gnss/camera-stream/calibration/camera-mount.json"
    DEFAULTS = {
        "height_m": 0.42,
        "pitch_deg": -12.0,
        "roll_deg": 0.0,
        "yaw_deg": 0.0,
        "lateral_offset_m": 0.0,
        "target_height_m": 1.20,
        "target_pitch_deg": -4.0,
        "perspective_strength": 0.0,
    }
    SCHEMA = {
        "height_m": {"min": 0.08, "max": 2.50, "step": 0.01, "unit": "m"},
        "pitch_deg": {"min": -45.0, "max": 20.0, "step": 0.1, "unit": "deg"},
        "roll_deg": {"min": -20.0, "max": 20.0, "step": 0.1, "unit": "deg"},
        "yaw_deg": {"min": -30.0, "max": 30.0, "step": 0.1, "unit": "deg"},
        "lateral_offset_m": {"min": -1.0, "max": 1.0, "step": 0.01, "unit": "m"},
        "target_height_m": {"min": 0.15, "max": 2.50, "step": 0.01, "unit": "m"},
        "target_pitch_deg": {"min": -30.0, "max": 15.0, "step": 0.1, "unit": "deg"},
        "perspective_strength": {"min": 0.0, "max": 1.0, "step": 0.05, "unit": "ratio"},
    }

    def __init__(self, legacy, path: str | None = None):
        self.legacy = legacy
        self.path = path or os.environ.get("CAMERA_MOUNT_SETTINGS_PATH", self.DEFAULT_PATH)
        self._lock = threading.RLock()
        self._values = dict(self.DEFAULTS)
        self._load()

    def _normalize(self, payload, *, base=None):
        if not isinstance(payload, dict):
            raise CameraMountSettingsError("camera mount settings must be a JSON object")
        values = dict(self._values if base is None else base)
        for key, raw in payload.items():
            if key not in self.SCHEMA:
                raise CameraMountSettingsError(f"Unknown camera mount setting: {key}")
            try:
                value = float(raw)
            except (TypeError, ValueError) as error:
                raise CameraMountSettingsError(f"{key} must be numeric") from error
            if not math.isfinite(value):
                raise CameraMountSettingsError(f"{key} must be finite")
            spec = self.SCHEMA[key]
            if value < spec["min"] or value > spec["max"]:
                raise CameraMountSettingsError(
                    f"{key} must be between {spec['min']} and {spec['max']}"
                )
            values[key] = value
        if values["target_height_m"] < 0.10:
            raise CameraMountSettingsError("target_height_m is invalid")
        return values

    def _load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as file:
                stored = json.load(file)
        except (FileNotFoundError, OSError, ValueError, json.JSONDecodeError):
            return
        try:
            self._values = self._normalize(stored, base=self.DEFAULTS)
        except CameraMountSettingsError:
            self._values = dict(self.DEFAULTS)

    def _persist(self, values):
        directory = os.path.dirname(os.path.abspath(self.path))
        os.makedirs(directory, exist_ok=True)
        temporary = f"{self.path}.tmp"
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(values, file, ensure_ascii=False, indent=2, sort_keys=True)
            file.flush()
            os.fsync(file.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, self.path)
        if hasattr(os, "O_DIRECTORY"):
            try:
                fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
            except OSError:
                pass

    def _editing_allowed(self):
        mode = self.legacy.vehicle_state_machine.mode
        canonical = getattr(mode, "canonical", mode)
        try:
            canonical = DriveMode(canonical)
        except (TypeError, ValueError):
            return False, "Unknown vehicle mode"
        if canonical != DriveMode.DISARMED:
            return False, "Camera mount calibration can only be saved while DISARMED"
        if self.legacy.record_manager.active:
            return False, "Stop RECORD before saving camera mount calibration"
        return True, None

    def _intrinsic_snapshot(self):
        calibration = getattr(self.legacy, "camera_calibration", None)
        if calibration is None:
            return {"calibrated": False, "error": "CAMERA_CALIBRATION_UNAVAILABLE"}
        try:
            return calibration.snapshot()
        except Exception as error:  # pragma: no cover - diagnostic only
            return {"calibrated": False, "error": f"{type(error).__name__}: {error}"}

    def snapshot(self):
        with self._lock:
            editable, reason = self._editing_allowed()
            return {
                "settings": dict(self._values),
                "defaults": dict(self.DEFAULTS),
                "schema": dict(self.SCHEMA),
                "editable": editable,
                "edit_block_reason": reason,
                "path": self.path,
                "intrinsic": self._intrinsic_snapshot(),
                "coordinate_convention": {
                    "height_m": "ground_to_lens_center",
                    "pitch_deg": "negative_points_camera_down",
                    "yaw_deg": "positive_turns_camera_right",
                    "lateral_offset_m": "positive_camera_right_of_vehicle_center",
                },
                "preview_only": True,
                "control_authority": "NONE",
            }

    def update(self, payload):
        with self._lock:
            editable, reason = self._editing_allowed()
            if not editable:
                raise CameraMountSettingsError(reason or "Camera mount settings are locked")
            if not isinstance(payload, dict):
                raise CameraMountSettingsError("camera mount payload must be a JSON object")
            if payload.get("reset") is True:
                values = dict(self.DEFAULTS)
            else:
                values = self._normalize(payload.get("settings", payload))
            self._persist(values)
            self._values = dict(values)
            return self.snapshot()

    def preview_values(self, query):
        with self._lock:
            incoming = {}
            for key in self.SCHEMA:
                values = query.get(key)
                if values:
                    incoming[key] = values[0]
            return self._normalize(incoming)

    @staticmethod
    def _rx(degrees):
        angle = math.radians(float(degrees))
        c, s = math.cos(angle), math.sin(angle)
        return np.asarray(
            [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
            dtype=np.float64,
        )

    @staticmethod
    def _rz(degrees):
        angle = math.radians(float(degrees))
        c, s = math.cos(angle), math.sin(angle)
        return np.asarray(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def _camera_to_world(self, values):
        # World axes: +X rover-right, +Y rover-forward, +Z up.
        # OpenCV camera axes: +x image-right, +y image-down, +z forward.
        base = np.asarray(
            [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]],
            dtype=np.float64,
        )
        # Positive yaw is defined as turning the camera toward rover-right.
        return (
            self._rz(-values["yaw_deg"])
            @ base
            @ self._rx(values["pitch_deg"])
            @ self._rz(values["roll_deg"])
        )

    def _scaled_camera_matrix(self, width, height):
        calibration = getattr(self.legacy, "camera_calibration", None)
        data = getattr(calibration, "data", None) if calibration is not None else None
        if data:
            original_width, original_height = [float(v) for v in data["image_size"]]
            matrix = np.asarray(data["camera_matrix"], dtype=np.float64).copy()
            matrix[0, 0] *= width / original_width
            matrix[0, 2] *= width / original_width
            matrix[1, 1] *= height / original_height
            matrix[1, 2] *= height / original_height
            return matrix, "CHARUCO_INTRINSIC"

        # Preview remains available before ChArUco calibration, but is explicitly
        # approximate.  This fallback must not be treated as a calibrated model.
        hfov = max(
            35.0,
            min(120.0, float(os.environ.get("SWING_CAMERA_APPROX_HFOV_DEG", "70.0"))),
        )
        focal = width / (2.0 * math.tan(math.radians(hfov) / 2.0))
        matrix = np.asarray(
            [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        return matrix, "APPROXIMATE_HFOV"

    def _ground_projection(self, matrix, values):
        c2w = self._camera_to_world(values)
        w2c = c2w.T
        center = np.asarray(
            [values["lateral_offset_m"], 0.0, values["height_m"]],
            dtype=np.float64,
        )
        translation = -w2c @ center
        plane = np.column_stack((w2c[:, 0], w2c[:, 1], translation))
        return matrix @ plane

    def _virtual_values(self, values):
        strength = float(values["perspective_strength"])
        return {
            **values,
            "height_m": values["height_m"]
            + (values["target_height_m"] - values["height_m"]) * strength,
            "pitch_deg": values["pitch_deg"]
            + (values["target_pitch_deg"] - values["pitch_deg"]) * strength,
            "roll_deg": values["roll_deg"] * (1.0 - strength),
            "yaw_deg": values["yaw_deg"] * (1.0 - strength),
            "lateral_offset_m": values["lateral_offset_m"] * (1.0 - strength),
        }

    def normalize_image(self, image, values=None):
        if cv2 is None or np is None:
            raise RuntimeError("OpenCV/NumPy unavailable")
        if image is None:
            raise ValueError("Camera image unavailable")
        values = self._normalize(values or {})
        calibration = getattr(self.legacy, "camera_calibration", None)
        intrinsic_calibrated = bool(getattr(calibration, "calibrated", False))
        source = calibration.undistort(image) if intrinsic_calibrated else image.copy()
        height, width = source.shape[:2]
        matrix, intrinsic_source = self._scaled_camera_matrix(width, height)
        strength = float(values["perspective_strength"])
        if strength <= 1e-6:
            return source, {
                "intrinsic_source": intrinsic_source,
                "intrinsic_calibrated": intrinsic_calibrated,
                "perspective_applied": False,
                "values": dict(values),
            }

        virtual = self._virtual_values(values)
        actual_projection = self._ground_projection(matrix, values)
        virtual_projection = self._ground_projection(matrix, virtual)
        try:
            determinant = float(np.linalg.det(actual_projection))
            if not math.isfinite(determinant) or abs(determinant) < 1e-9:
                raise ValueError("CAMERA_GROUND_PROJECTION_SINGULAR")
            homography = virtual_projection @ np.linalg.inv(actual_projection)
            homography = homography / homography[2, 2]
            if not np.all(np.isfinite(homography)):
                raise ValueError("CAMERA_HOMOGRAPHY_NONFINITE")
        except np.linalg.LinAlgError as error:
            raise ValueError("CAMERA_GROUND_PROJECTION_SINGULAR") from error

        corrected = cv2.warpPerspective(
            source,
            homography,
            (width, height),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        return corrected, {
            "intrinsic_source": intrinsic_source,
            "intrinsic_calibrated": intrinsic_calibrated,
            "perspective_applied": True,
            "homography": homography.tolist(),
            "values": dict(values),
            "virtual": virtual,
        }


CAMERA_MOUNT_SETTINGS = CameraMountSettings(release.full.legacy)
_INSTALLED = False


def _decode_camera_jpeg(jpeg):
    if cv2 is None or np is None:
        raise RuntimeError("OpenCV/NumPy unavailable")
    if not jpeg:
        raise RuntimeError("CAMERA_FRAME_UNAVAILABLE")
    image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("CAMERA_JPEG_DECODE_FAILED")
    return image


def _comparison_jpeg(frame, values):
    image = _decode_camera_jpeg(frame)
    corrected, metadata = CAMERA_MOUNT_SETTINGS.normalize_image(image, values)
    left = image.copy()
    right = corrected.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(left, "RAW CAMERA", (14, 28), font, 0.70, (255, 255, 255), 2, cv2.LINE_AA)
    label = "LENS + VIRTUAL PERSPECTIVE"
    cv2.putText(right, label, (14, 28), font, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
    comparison = cv2.hconcat([left, right])
    divider_x = left.shape[1]
    cv2.line(
        comparison,
        (divider_x, 0),
        (divider_x, comparison.shape[0] - 1),
        (0, 210, 255),
        2,
        cv2.LINE_AA,
    )
    status = (
        f"intrinsic={metadata['intrinsic_source']}  "
        f"strength={float(values['perspective_strength']):.2f}"
    )
    cv2.putText(
        comparison,
        status,
        (14, comparison.shape[0] - 14),
        font,
        0.48,
        (210, 225, 210),
        1,
        cv2.LINE_AA,
    )
    ok, encoded = cv2.imencode(
        ".jpg", comparison, [int(cv2.IMWRITE_JPEG_QUALITY), 86]
    )
    if not ok:
        raise RuntimeError("CAMERA_MOUNT_PREVIEW_ENCODE_FAILED")
    return encoded.tobytes()


def install_camera_mount_endpoints():
    global _INSTALLED
    if _INSTALLED:
        return True
    handler = release.full.legacy.CameraHandler
    original_do_get = handler.do_GET
    original_do_post = handler.do_POST
    if getattr(original_do_get, "_swing_camera_mount", False):
        _INSTALLED = True
        return True

    def do_get_with_camera_mount(self):
        parsed = urlsplit(str(self.path or ""))
        if parsed.path == "/api/camera/mount-calibration":
            self._send_json(CAMERA_MOUNT_SETTINGS.snapshot())
            return
        if parsed.path == "/api/camera/mount-preview":
            try:
                values = CAMERA_MOUNT_SETTINGS.preview_values(
                    parse_qs(parsed.query, keep_blank_values=False)
                )
                frame, sequence, frame_monotonic, _ = (
                    release.full.legacy.camera.snapshot_frame()
                )
                payload = _comparison_jpeg(frame, values)
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(payload)))
                self.send_header("Cache-Control", "private, no-store")
                self.send_header("X-SWING-Camera-Sequence", str(sequence))
                if frame_monotonic is not None:
                    self.send_header(
                        "X-SWING-Frame-Monotonic", f"{float(frame_monotonic):.6f}"
                    )
                self.end_headers()
                self.wfile.write(payload)
            except (
                CameraMountSettingsError,
                RuntimeError,
                ValueError,
                OSError,
                TypeError,
            ) as error:
                self._send_json({"error": f"{type(error).__name__}: {error}"}, 503)
            return
        return original_do_get(self)

    def do_post_with_camera_mount(self):
        path = str(self.path or "").split("?", 1)[0]
        if path != "/api/camera/mount-calibration":
            return original_do_post(self)
        try:
            payload = self._read_json()
            self._send_json(CAMERA_MOUNT_SETTINGS.update(payload), 202)
        except (
            CameraMountSettingsError,
            RuntimeError,
            ValueError,
            OSError,
            TypeError,
            json.JSONDecodeError,
        ) as error:
            self._send_json(
                {"error": str(error), **CAMERA_MOUNT_SETTINGS.snapshot()},
                409,
            )

    do_get_with_camera_mount._swing_camera_mount = True
    do_post_with_camera_mount._swing_camera_mount = True
    handler.do_GET = do_get_with_camera_mount
    handler.do_POST = do_post_with_camera_mount
    _INSTALLED = True
    return True


install_camera_mount_endpoints()


CAMERA_MOUNT_HMI = r"""
<style>
#camera-mount-open{min-height:28px;padding:5px 9px;border:1px solid rgba(84,180,220,.30);border-radius:4px;background:#101a1e;color:#b9e7f7;font:650 9px Inter,system-ui,sans-serif;cursor:pointer}
#camera-mount-open:hover{background:#152329;border-color:rgba(84,180,220,.55)}
.cmount-backdrop{position:fixed;inset:0;z-index:10020;background:rgba(0,0,0,.72);display:grid;place-items:center;padding:12px}
.cmount-backdrop[hidden]{display:none}
.cmount-card{width:min(1120px,calc(100vw - 24px));max-height:calc(100vh - 24px);overflow:hidden;border:1px solid rgba(255,255,255,.12);border-radius:8px;background:#0d1011;color:#dfe5e2;box-shadow:0 24px 80px rgba(0,0,0,.55);display:grid;grid-template-rows:auto minmax(0,1fr) auto}
.cmount-head{padding:12px 14px;border-bottom:1px solid rgba(255,255,255,.08);display:flex;justify-content:space-between;gap:12px;align-items:flex-start}
.cmount-head strong{display:block;font-size:13px}.cmount-head p{margin:4px 0 0;color:#8f9995;font-size:9px;line-height:1.45}
.cmount-close{border:0;background:transparent;color:#aeb7b3;font-size:24px;cursor:pointer}
.cmount-body{min-height:0;overflow:auto;padding:12px;display:grid;grid-template-columns:minmax(440px,1.25fr) minmax(330px,.8fr);gap:12px}
.cmount-preview,.cmount-controls{border:1px solid rgba(255,255,255,.08);border-radius:6px;background:#090c0d;overflow:hidden}
.cmount-preview-head,.cmount-section-head{padding:9px 10px;border-bottom:1px solid rgba(255,255,255,.07);display:flex;justify-content:space-between;align-items:center;gap:8px}
.cmount-preview-head strong,.cmount-section-head strong{font-size:10px}.cmount-preview-head span,.cmount-section-head span{font:650 8px ui-monospace,monospace;color:#7f8b87}
#cmount-preview{display:block;width:100%;aspect-ratio:32/9;object-fit:contain;background:#020303}
.cmount-help{padding:9px 10px;color:#96a19d;font-size:9px;line-height:1.55;border-top:1px solid rgba(255,255,255,.06)}
.cmount-section{border-bottom:1px solid rgba(255,255,255,.07)}.cmount-section:last-child{border-bottom:0}
.cmount-fields{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px;padding:10px}
.cmount-field{display:grid;gap:4px}.cmount-field label{color:#8c9692;font-size:8px}.cmount-field input{width:100%;box-sizing:border-box;padding:7px 8px;border:1px solid rgba(255,255,255,.11);border-radius:4px;background:#050707;color:#e2e8e5;font:700 10px ui-monospace,monospace}
.cmount-actions{display:flex;gap:7px;flex-wrap:wrap;padding:10px}.cmount-actions button,.cmount-footer button{min-height:32px;padding:7px 10px;border:1px solid rgba(255,255,255,.12);border-radius:4px;background:#171a1a;color:#d0d6d3;font-size:9px;cursor:pointer}.cmount-actions button.primary{background:#10242c;border-color:rgba(84,180,220,.40);color:#b9e7f7}.cmount-actions button:disabled{opacity:.45;cursor:not-allowed}
#cmount-status{padding:9px 10px;font-size:9px;line-height:1.5;color:#9ea7a3}#cmount-status.good{color:#acdcbc}#cmount-status.warn{color:#e4c47e}#cmount-status.bad{color:#e5a1a6}
.cmount-footer{padding:10px 12px;border-top:1px solid rgba(255,255,255,.08);display:flex;justify-content:space-between;align-items:center;gap:10px}.cmount-footer small{font-size:8px;color:#7f8985;line-height:1.5}
.cmount-launch-panel{padding:10px;border:1px solid rgba(84,180,220,.18);border-radius:6px;background:#0e171b;margin-top:8px}
@media(max-width:860px){.cmount-body{grid-template-columns:1fr}.cmount-fields{grid-template-columns:1fr}#cmount-preview{aspect-ratio:16/9}}
</style>

<div class="cmount-backdrop" id="camera-mount-modal" hidden>
  <div class="cmount-card">
    <div class="cmount-head">
      <div><strong>카메라 장착 보정 · UFLD Virtual Camera</strong><p>렌즈 ChArUco 보정과 별개입니다. 실제 장착 위치를 입력하고 낮은 카메라 시점을 지면 평면 homography로 완만하게 정규화해 봅니다.</p></div>
      <button class="cmount-close" id="cmount-close" type="button">×</button>
    </div>
    <div class="cmount-body">
      <section class="cmount-preview">
        <div class="cmount-preview-head"><strong>원본 / 보정 미리보기</strong><span id="cmount-preview-meta">왼쪽 RAW · 오른쪽 NORMALIZED</span></div>
        <img id="cmount-preview" alt="카메라 장착 보정 미리보기">
        <div class="cmount-help">왼쪽은 원본 카메라, 오른쪽은 렌즈 보정 후 가상 시점 변환입니다. 검은 영역이 크게 늘어나면 보정 강도가 과합니다. UFLD 적용 전 RECORD 영상으로 먼저 검증합니다.</div>
      </section>
      <section class="cmount-controls">
        <div class="cmount-section">
          <div class="cmount-section-head"><strong>실제 장착값</strong><span>줄자 + 수평계 측정값</span></div>
          <div class="cmount-fields">
            <div class="cmount-field"><label>렌즈 중심 높이 (m)</label><input id="cmount-height" type="number" step="0.01"></div>
            <div class="cmount-field"><label>Pitch (deg, 아래 방향 음수)</label><input id="cmount-pitch" type="number" step="0.1"></div>
            <div class="cmount-field"><label>Roll (deg)</label><input id="cmount-roll" type="number" step="0.1"></div>
            <div class="cmount-field"><label>Yaw (deg, 오른쪽 양수)</label><input id="cmount-yaw" type="number" step="0.1"></div>
            <div class="cmount-field"><label>차량 중심 대비 좌우 (m, 오른쪽 양수)</label><input id="cmount-lateral" type="number" step="0.01"></div>
          </div>
        </div>
        <div class="cmount-section">
          <div class="cmount-section-head"><strong>UFLD 가상 시점</strong><span>처음에는 0.00부터 조금씩</span></div>
          <div class="cmount-fields">
            <div class="cmount-field"><label>목표 카메라 높이 (m)</label><input id="cmount-target-height" type="number" step="0.01"></div>
            <div class="cmount-field"><label>목표 Pitch (deg)</label><input id="cmount-target-pitch" type="number" step="0.1"></div>
            <div class="cmount-field"><label>Perspective strength (0~1)</label><input id="cmount-strength" type="number" min="0" max="1" step="0.05"></div>
          </div>
          <div id="cmount-status">상태 불러오는 중</div>
          <div class="cmount-actions"><button class="primary" id="cmount-save" type="button">장착값 저장</button><button id="cmount-defaults" type="button">기본값 불러오기</button><button id="cmount-refresh" type="button">미리보기 새로고침</button></div>
        </div>
      </section>
    </div>
    <div class="cmount-footer"><small>저장은 DISARMED + RECORD 정지 상태에서만 허용됩니다. 현재 단계에서는 preview-only이며 모터/조향 제어권은 없습니다.</small><button id="cmount-footer-close" type="button">닫기</button></div>
  </div>
</div>

<script>
(function(){
 const modal=document.getElementById('camera-mount-modal');if(!modal)return;
 const ids={
   height:'cmount-height',pitch:'cmount-pitch',roll:'cmount-roll',yaw:'cmount-yaw',
   lateral_offset_m:'cmount-lateral',target_height_m:'cmount-target-height',
   target_pitch_deg:'cmount-target-pitch',perspective_strength:'cmount-strength'
 };
 ids.height_m=ids.height;delete ids.height;ids.pitch_deg=ids.pitch;delete ids.pitch;
 ids.roll_deg=ids.roll;delete ids.roll;ids.yaw_deg=ids.yaw;delete ids.yaw;
 const $=id=>document.getElementById(id),preview=$('cmount-preview'),status=$('cmount-status'),save=$('cmount-save');
 let snapshot=null,opened=false,timer=null,previewTimer=null;
 function values(){
   const out={};for(const [key,id] of Object.entries(ids)){const n=Number($(id)?.value);if(Number.isFinite(n))out[key]=n}return out;
 }
 function fill(source){
   const v=source||{};for(const [key,id] of Object.entries(ids)){if(v[key]!==undefined&&$(id))$(id).value=String(v[key])}
 }
 function setStatus(text,kind=''){status.textContent=text;status.className=kind}
 async function load(){
   try{
     const r=await fetch('/api/camera/mount-calibration',{cache:'no-store'}),data=await r.json();
     if(!r.ok)throw new Error(data.error||`HTTP ${r.status}`);snapshot=data;fill(data.settings||{});
     const intrinsic=data.intrinsic||{};
     setStatus(`${intrinsic.calibrated?'렌즈 intrinsic 보정 완료':'렌즈 intrinsic 미보정 · HFOV 근사 미리보기'} · ${data.editable?'저장 가능':'저장 잠김: '+(data.edit_block_reason||'')}`,data.editable?'good':'warn');
     save.disabled=!data.editable;refreshPreview();
   }catch(error){setStatus(String(error),'bad')}
 }
 function previewUrl(){
   const q=new URLSearchParams();for(const [key,value] of Object.entries(values()))q.set(key,String(value));q.set('_',String(Date.now()));return `/api/camera/mount-preview?${q.toString()}`;
 }
 function refreshPreview(){if(!opened)return;preview.src=previewUrl()}
 function schedulePreview(delay=160){clearTimeout(previewTimer);previewTimer=setTimeout(refreshPreview,delay)}
 function open(){
   opened=true;modal.hidden=false;load();clearInterval(timer);timer=setInterval(refreshPreview,900);
 }
 function close(){opened=false;modal.hidden=true;clearInterval(timer);timer=null;clearTimeout(previewTimer)}
 let launch=document.getElementById('camera-mount-open');
 if(!launch){
   const head=document.querySelector('.camera-panel .panel-head');
   if(head){
     let actions=head.querySelector('.panel-head-actions');if(!actions){actions=document.createElement('div');actions.className='panel-head-actions';head.appendChild(actions)}
     launch=document.createElement('button');launch.id='camera-mount-open';launch.type='button';launch.textContent='장착 보정';actions.appendChild(launch);
   }else{
     const grid=document.querySelector('#view-settings .grid');
     if(grid){
       const panel=document.createElement('div');panel.className='panel span12 cmount-launch-panel';panel.innerHTML='<h2>카메라 장착 보정</h2><p class="sectionnote">높이·Pitch·Roll·Yaw를 입력하고 UFLD용 가상 시점을 미리 봅니다.</p><button id="camera-mount-open" type="button">장착 보정 열기</button>';grid.appendChild(panel);launch=document.getElementById('camera-mount-open');
     }
   }
 }
 if(launch)launch.onclick=open;
 $('cmount-close').onclick=close;$('cmount-footer-close').onclick=close;
 modal.addEventListener('click',event=>{if(event.target===modal)close()});
 for(const id of Object.values(ids))$(id)?.addEventListener('input',()=>schedulePreview());
 $('cmount-refresh').onclick=refreshPreview;
 $('cmount-defaults').onclick=()=>{fill(snapshot?.defaults||{});schedulePreview(20)};
 save.onclick=async()=>{
   save.disabled=true;setStatus('저장 중…');
   try{
     const r=await fetch('/api/camera/mount-calibration',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({settings:values()})});
     const data=await r.json();if(!r.ok)throw new Error(data.error||`HTTP ${r.status}`);snapshot=data;fill(data.settings||{});
     setStatus(`저장 완료 · ${data.path||''}`,'good');save.disabled=!data.editable;refreshPreview();
   }catch(error){setStatus(String(error),'bad');save.disabled=!(snapshot?.editable)}
 };
 preview.addEventListener('error',()=>{if(opened)setStatus('미리보기를 만들지 못했습니다. 카메라/렌즈 보정 상태를 확인하세요.','bad')});
})();
</script>
""".encode("utf-8")


__all__ = [
    "CAMERA_MOUNT_HMI",
    "CAMERA_MOUNT_SETTINGS",
    "CameraMountSettings",
    "CameraMountSettingsError",
    "install_camera_mount_endpoints",
]
