"""Read-only media endpoints for legacy MP4 and segmented-JPEG RECORD replay.

New human RECORD sessions store timestamped 640x360 JPEG frames so the rover
never blocks control on MP4 muxing or H.264 encoding. camera_timestamps.csv is
the authoritative timeline. The browser can replay those frames immediately;
a Worker-generated H.264 asset is used when available. Legacy MJPEG-in-MP4
sessions remain supported and may still be transcoded lazily for compatibility.
"""

from __future__ import annotations

import bisect
import csv
import math
import os
from statistics import median
import subprocess
import threading
from urllib.parse import parse_qs, urlsplit

import server_v2_release as release

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


_INSTALLED = False
_FRAME_LOCK = threading.RLock()
_FRAME_CAPTURE = None
_FRAME_CAPTURE_PATH = None
_BROWSER_VIDEO_LOCK = threading.RLock()
_BROWSER_VIDEO_NAME = "camera_browser_v2.mp4"
_BROWSER_VIDEO_FFMPEG_TIMEOUT_SECONDS = 15 * 60
_TIMING_CACHE = {}
_CAMERA_ROWS_CACHE = {}


def _session_path(session_name):
    raw = str(session_name or "").strip()
    name = os.path.basename(raw)
    if not raw or name != raw or name in {".", ".."}:
        raise ValueError("INVALID_RECORDING_SESSION")
    # recording_session_path is patched by the final runtime to search removable
    # USB storage first and the historical microSD root second. It owns path
    # containment validation, so replay must not re-impose the legacy root here.
    path = os.path.realpath(release.full.legacy.recording_session_path(name))
    if not os.path.isdir(path):
        raise FileNotFoundError("Recording session not found")
    return path


def _assert_finalized(session_path):
    manager = release.full.legacy.record_manager
    active_path = os.path.realpath(manager.session_path) if manager.session_path else None
    if manager.active and active_path == os.path.realpath(session_path):
        raise RuntimeError("RECORDING_IS_STILL_ACTIVE")


def _finalized_media_path(session_name):
    """Return the legacy source MP4 when one exists.

    New segmented-JPEG sessions intentionally have no source MP4 until Worker
    post-processing. Callers that only need frames should use _record_frame_path
    or _read_record_frame instead.
    """
    path = _session_path(session_name)
    _assert_finalized(path)
    media = os.path.join(path, "camera.mp4")
    if not os.path.isfile(media) or os.path.getsize(media) <= 0:
        raise FileNotFoundError("Recorded camera video not found")
    return media


def _safe_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _camera_rows(session_path):
    path = os.path.join(session_path, "camera_timestamps.csv")
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return []
    cached = _CAMERA_ROWS_CACHE.get(path)
    if cached and cached.get("mtime") == mtime:
        return list(cached["rows"])
    rows = []
    try:
        with open(path, "r", newline="", encoding="utf-8") as file:
            for index, row in enumerate(csv.DictReader(file)):
                value = dict(row)
                value["_index"] = index
                value["_monotonic"] = _safe_float(row.get("monotonic"))
                rows.append(value)
    except OSError:
        rows = []
    _CAMERA_ROWS_CACHE[path] = {"mtime": mtime, "rows": list(rows)}
    return rows


def _camera_timing(session_path):
    path = os.path.join(session_path, "camera_timestamps.csv")
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        mtime = 0.0
    cached = _TIMING_CACHE.get(path)
    if cached and cached.get("mtime") == mtime:
        return dict(cached)

    rows = _camera_rows(session_path) if mtime else []
    monotonic_values = [row.get("_monotonic") for row in rows]
    first = next((value for value in monotonic_values if value is not None), None)
    offsets = [
        None if value is None or first is None else max(0.0, value - first)
        for value in monotonic_values
    ]
    finite = [value for value in offsets if value is not None]
    intervals = [
        b - a for a, b in zip(finite, finite[1:]) if 0.0001 < b - a < 10.0
    ]
    typical = median(intervals) if intervals else None
    duration = None
    effective_fps = None
    if len(finite) >= 2:
        span = finite[-1] - finite[0]
        if span > 0:
            effective_fps = (len(finite) - 1) / span
            duration = span + (typical or 1.0 / effective_fps)

    value = {
        "path": path,
        "mtime": mtime,
        "offsets": offsets,
        "duration_seconds": duration,
        "effective_fps": effective_fps,
        "median_interval_seconds": typical,
        "frame_count": len(rows),
    }
    _TIMING_CACHE[path] = dict(value)
    return value


def _frame_index_for_record_offset(timing, offset):
    pairs = [
        (float(value), index)
        for index, value in enumerate(timing.get("offsets") or [])
        if value is not None
    ]
    if not pairs:
        return None
    values = [item[0] for item in pairs]
    position = bisect.bisect_right(values, max(0.0, float(offset))) - 1
    position = max(0, min(position, len(pairs) - 1))
    return pairs[position][1]


def _saved_frame_path(session_path, row):
    raw = str((row or {}).get("filename") or "").strip().replace("\\", "/")
    if not raw:
        return None
    relative = raw if raw.startswith("camera_frames/") else "camera_frames/" + raw.lstrip("/")
    root = os.path.realpath(session_path)
    candidate = os.path.realpath(os.path.join(root, *relative.split("/")))
    try:
        if os.path.commonpath([root, candidate]) != root:
            return None
    except ValueError:
        return None
    return candidate if os.path.isfile(candidate) else None


def _record_frame_path(session_path, offset_seconds):
    rows = _camera_rows(session_path)
    if not rows:
        return None
    index = _frame_index_for_record_offset(
        _camera_timing(session_path), max(0.0, float(offset_seconds or 0.0))
    )
    if index is None or not 0 <= index < len(rows):
        return None
    return _saved_frame_path(session_path, rows[index])


def _probe_media_duration(path):
    try:
        result = subprocess.run(
            [
                "/usr/bin/ffprobe", "-v", "error",
                "-show_entries", "format=duration",
                "-of", "default=noprint_wrappers=1:nokey=1", path,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            timeout=15,
            check=False,
        )
        if result.returncode != 0:
            return None
        return _safe_float(result.stdout.decode("utf-8", errors="replace").strip())
    except (OSError, subprocess.TimeoutExpired):
        return None


def _browser_media_path(source_path):
    """Legacy compatibility transcoder; new RECORDs are post-processed on Worker."""
    session_path = os.path.dirname(source_path)
    output_path = os.path.join(session_path, _BROWSER_VIDEO_NAME)
    source_mtime = os.path.getmtime(source_path)
    timing = _camera_timing(session_path)
    input_mtime = max(source_mtime, float(timing.get("mtime") or 0.0))

    def valid():
        return bool(
            os.path.isfile(output_path)
            and os.path.getsize(output_path) > 0
            and os.path.getmtime(output_path) >= input_mtime
        )

    if valid():
        return output_path
    with _BROWSER_VIDEO_LOCK:
        if valid():
            return output_path
        temporary = output_path + ".part.mp4"
        try:
            if os.path.exists(temporary):
                os.remove(temporary)
            source_duration = _probe_media_duration(source_path)
            target_duration = _safe_float(timing.get("duration_seconds"))
            stretch = 1.0
            if source_duration and source_duration > 0.05 and target_duration and target_duration > 0.05:
                candidate = target_duration / source_duration
                if 0.25 <= candidate <= 20.0:
                    stretch = candidate
            filters = []
            if abs(stretch - 1.0) > 0.005:
                filters.append(f"setpts={stretch:.9f}*PTS")
            filters.append("scale=trunc(iw/2)*2:trunc(ih/2)*2")
            command = [
                "/usr/bin/ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
                "-y", "-i", source_path, "-an", "-vf", ",".join(filters),
                "-fps_mode", "vfr", "-c:v", "libx264", "-preset", "ultrafast",
                "-crf", "23", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
                "-threads", "2", temporary,
            ]
            result = subprocess.run(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                timeout=_BROWSER_VIDEO_FFMPEG_TIMEOUT_SECONDS,
                check=False,
            )
            if result.returncode != 0:
                details = result.stderr.decode("utf-8", errors="replace").strip()
                raise RuntimeError(
                    "RECORD_BROWSER_TRANSCODE_FAILED"
                    + (f": {details[-1200:]}" if details else "")
                )
            if not os.path.isfile(temporary) or os.path.getsize(temporary) <= 0:
                raise RuntimeError("RECORD_BROWSER_TRANSCODE_EMPTY")
            os.replace(temporary, output_path)
            return output_path
        except subprocess.TimeoutExpired as error:
            raise RuntimeError("RECORD_BROWSER_TRANSCODE_TIMEOUT") from error
        except OSError as error:
            raise RuntimeError(f"RECORD_BROWSER_TRANSCODE_IO:{error}") from error
        finally:
            try:
                if os.path.exists(temporary):
                    os.remove(temporary)
            except OSError:
                pass


def _finalized_browser_video(session_name):
    session_path = _session_path(session_name)
    _assert_finalized(session_path)
    worker_video = os.path.join(session_path, _BROWSER_VIDEO_NAME)
    if os.path.isfile(worker_video) and os.path.getsize(worker_video) > 0:
        return worker_video
    legacy_video = os.path.join(session_path, "camera.mp4")
    if os.path.isfile(legacy_video) and os.path.getsize(legacy_video) > 0:
        return _browser_media_path(legacy_video)
    raise FileNotFoundError("Browser video not generated yet; JPEG replay is available")


def _parse_single_range(value, size):
    if not value:
        return None
    text = str(value).strip()
    if not text.startswith("bytes=") or "," in text:
        raise ValueError("UNSUPPORTED_RANGE")
    spec = text[6:].strip()
    if "-" not in spec:
        raise ValueError("INVALID_RANGE")
    start_text, end_text = spec.split("-", 1)
    if not start_text:
        suffix = int(end_text)
        if suffix <= 0:
            raise ValueError("INVALID_RANGE")
        start = max(0, size - suffix)
        end = size - 1
    else:
        start = int(start_text)
        end = size - 1 if not end_text else int(end_text)
    if start < 0 or start >= size or end < start:
        raise ValueError("RANGE_NOT_SATISFIABLE")
    return start, min(end, size - 1)


def _send_video(handler, path):
    size = os.path.getsize(path)
    try:
        requested = _parse_single_range(handler.headers.get("Range"), size)
    except (TypeError, ValueError):
        handler.send_response(416)
        handler.send_header("Content-Range", f"bytes */{size}")
        handler.send_header("Content-Length", "0")
        handler.end_headers()
        return
    if requested is None:
        start, end, status = 0, size - 1, 200
    else:
        start, end = requested
        status = 206
    length = end - start + 1
    handler.send_response(status)
    handler.send_header("Content-Type", "video/mp4")
    handler.send_header("Accept-Ranges", "bytes")
    handler.send_header("Content-Length", str(length))
    handler.send_header("Cache-Control", "private, no-store")
    handler.send_header("X-SWING-Record-Replay", "h264-worker-or-legacy-v3")
    if status == 206:
        handler.send_header("Content-Range", f"bytes {start}-{end}/{size}")
    handler.end_headers()
    remaining = length
    try:
        with open(path, "rb") as file:
            file.seek(start)
            while remaining > 0:
                chunk = file.read(min(1024 * 1024, remaining))
                if not chunk:
                    break
                handler.wfile.write(chunk)
                remaining -= len(chunk)
    except (BrokenPipeError, ConnectionResetError, OSError):
        return


def _read_legacy_jpeg_frame(path, offset_seconds):
    global _FRAME_CAPTURE, _FRAME_CAPTURE_PATH
    if cv2 is None:
        raise RuntimeError("OPENCV_UNAVAILABLE")
    offset = max(0.0, float(offset_seconds or 0.0))
    with _FRAME_LOCK:
        if _FRAME_CAPTURE is None or _FRAME_CAPTURE_PATH != path:
            if _FRAME_CAPTURE is not None:
                try:
                    _FRAME_CAPTURE.release()
                except Exception:
                    pass
            capture = cv2.VideoCapture(path)
            if not capture.isOpened():
                capture.release()
                raise RuntimeError("RECORDED_VIDEO_OPEN_FAILED")
            _FRAME_CAPTURE = capture
            _FRAME_CAPTURE_PATH = path
        capture = _FRAME_CAPTURE
        timing = _camera_timing(os.path.dirname(path))
        frame_index = _frame_index_for_record_offset(timing, offset)
        if frame_index is not None:
            current_index = int(capture.get(cv2.CAP_PROP_POS_FRAMES) or 0)
            if abs(current_index - frame_index) > 1:
                capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        else:
            current = float(capture.get(cv2.CAP_PROP_POS_MSEC) or 0.0) / 1000.0
            if abs(current - offset) > 0.20:
                capture.set(cv2.CAP_PROP_POS_MSEC, offset * 1000.0)
        ok, frame = capture.read()
        if not ok or frame is None:
            if frame_index is not None:
                capture.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            else:
                capture.set(cv2.CAP_PROP_POS_MSEC, offset * 1000.0)
            ok, frame = capture.read()
        if not ok or frame is None:
            raise RuntimeError("RECORDED_FRAME_DECODE_FAILED")
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
        if not ok:
            raise RuntimeError("RECORDED_FRAME_JPEG_FAILED")
        return encoded.tobytes()


def _read_record_frame(session_path, offset_seconds):
    direct = _record_frame_path(session_path, offset_seconds)
    if direct:
        with open(direct, "rb") as file:
            return file.read()
    legacy = os.path.join(session_path, "camera.mp4")
    if os.path.isfile(legacy) and os.path.getsize(legacy) > 0:
        return _read_legacy_jpeg_frame(legacy, offset_seconds)
    raise FileNotFoundError("Recorded camera frame not found")


def install_record_replay_media_endpoints():
    global _INSTALLED
    if _INSTALLED:
        return True
    handler = release.full.legacy.CameraHandler
    original_do_get = handler.do_GET
    if getattr(original_do_get, "_swing_record_replay_media", False):
        _INSTALLED = True
        return True

    def do_get_with_record_replay_media(self):
        parsed = urlsplit(str(self.path or ""))
        if parsed.path not in {"/api/recordings/video", "/api/recordings/frame"}:
            return original_do_get(self)
        params = parse_qs(parsed.query, keep_blank_values=False)
        session = (params.get("session") or [""])[0]
        try:
            session_path = _session_path(session)
            _assert_finalized(session_path)
            if parsed.path == "/api/recordings/video":
                _send_video(self, _finalized_browser_video(session))
                return
            offset = float((params.get("offset") or ["0"])[0])
            payload = _read_record_frame(session_path, offset)
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "private, no-store")
            self.send_header("X-SWING-Record-Replay", "timestamped-jpeg-v3")
            self.end_headers()
            self.wfile.write(payload)
        except FileNotFoundError as error:
            self._send_json({"error": str(error)}, 404)
        except RuntimeError as error:
            status = 409 if str(error) == "RECORDING_IS_STILL_ACTIVE" else 503
            self._send_json({"error": str(error)}, status)
        except (TypeError, ValueError, OSError) as error:
            self._send_json({"error": str(error)}, 400)

    do_get_with_record_replay_media._swing_record_replay_media = True
    handler.do_GET = do_get_with_record_replay_media
    _INSTALLED = True
    return True


__all__ = [
    "_camera_rows",
    "_camera_timing",
    "_finalized_media_path",
    "_read_record_frame",
    "_record_frame_path",
    "_session_path",
    "install_record_replay_media_endpoints",
]
