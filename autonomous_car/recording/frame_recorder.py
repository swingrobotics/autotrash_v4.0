from __future__ import annotations

import json
import multiprocessing as mp
import os
import queue
import time
from pathlib import Path


_STOP = {"kind": "stop"}


def _atomic_json(path: Path, document):
    temporary = Path(str(path) + ".tmp")
    with open(temporary, "w", encoding="utf-8") as file:
        json.dump(document, file, ensure_ascii=False, indent=2, sort_keys=True)
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)


def _writer_main(
    frame_queue,
    status_queue,
    session_path,
    width,
    height,
    jpeg_quality,
):
    # The child intentionally runs at a lower scheduler priority than the rover
    # control process. If encoding/storage falls behind, the parent drops camera
    # frames instead of ever waiting here.
    try:
        try:
            os.nice(10)
        except OSError:
            pass
        try:
            import cv2
            import numpy as np
        except ImportError as error:
            raise RuntimeError("RECORD_FRAME_ENCODER_REQUIRES_OPENCV_NUMPY") from error

        root = Path(session_path)
        camera_root = root / "camera_frames"
        camera_root.mkdir(parents=True, exist_ok=True)
        written = 0
        written_bytes = 0
        started = time.monotonic()
        last_report = started

        while True:
            item = frame_queue.get()
            if item == _STOP:
                break
            if not isinstance(item, dict) or item.get("kind") != "frame":
                continue
            relative = Path(str(item["filename"]))
            destination = root / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            encoded = item.get("jpeg") or b""
            if width > 0 and height > 0:
                image = cv2.imdecode(np.frombuffer(encoded, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("RECORD_JPEG_DECODE_FAILED")
                if image.shape[1] != width or image.shape[0] != height:
                    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
                ok, result = cv2.imencode(
                    ".jpg",
                    image,
                    [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
                )
                if not ok:
                    raise OSError("RECORD_JPEG_ENCODE_FAILED")
                encoded = result.tobytes()

            temporary = Path(str(destination) + ".part")
            with open(temporary, "wb") as file:
                file.write(encoded)
            os.replace(temporary, destination)
            written += 1
            written_bytes += len(encoded)
            now = time.monotonic()
            if written % 25 == 0 or now - last_report >= 1.0:
                try:
                    status_queue.put_nowait(
                        {
                            "kind": "status",
                            "written_frames": written,
                            "written_bytes": written_bytes,
                            "last_filename": str(relative).replace("\\", "/"),
                            "encoder_pid": os.getpid(),
                            "elapsed_seconds": now - started,
                        }
                    )
                except queue.Full:
                    pass
                last_report = now

        try:
            status_queue.put_nowait(
                {
                    "kind": "finished",
                    "written_frames": written,
                    "written_bytes": written_bytes,
                    "encoder_pid": os.getpid(),
                    "elapsed_seconds": time.monotonic() - started,
                }
            )
        except queue.Full:
            pass
    except BaseException as error:
        try:
            status_queue.put_nowait(
                {
                    "kind": "error",
                    "error": f"{type(error).__name__}:{error}",
                    "encoder_pid": os.getpid(),
                }
            )
        except Exception:
            pass
        raise


class CameraFrameRecorder:
    """Bounded non-blocking camera writer isolated in a spawned process."""

    def __init__(
        self,
        *,
        width=640,
        height=360,
        jpeg_quality=82,
        segment_seconds=60.0,
        queue_frames=8,
    ):
        self.width = max(160, int(width))
        self.height = max(90, int(height))
        self.jpeg_quality = max(45, min(95, int(jpeg_quality)))
        self.segment_seconds = max(10.0, float(segment_seconds))
        self.queue_frames = max(2, min(64, int(queue_frames)))
        self._context = mp.get_context("spawn")
        self._queue = None
        self._status_queue = None
        self._process = None
        self._session_path = None
        self._started_monotonic = None
        self.enqueued_frames = 0
        self.dropped_frames = 0
        self.written_frames = 0
        self.written_bytes = 0
        self.error = None
        self.last_filename = None
        self.encoder_pid = None

    def start(self, session_path, started_monotonic):
        if self._process is not None and self._process.is_alive():
            raise RuntimeError("CAMERA_RECORDER_ALREADY_RUNNING")
        self._session_path = str(session_path)
        self._started_monotonic = float(started_monotonic)
        self._queue = self._context.Queue(maxsize=self.queue_frames)
        self._status_queue = self._context.Queue(maxsize=8)
        self.enqueued_frames = 0
        self.dropped_frames = 0
        self.written_frames = 0
        self.written_bytes = 0
        self.error = None
        self.last_filename = None
        self.encoder_pid = None
        self._process = self._context.Process(
            target=_writer_main,
            args=(
                self._queue,
                self._status_queue,
                self._session_path,
                self.width,
                self.height,
                self.jpeg_quality,
            ),
            daemon=True,
            name="swing-camera-recorder",
        )
        self._process.start()
        self.encoder_pid = self._process.pid

    def _relative_filename(self, frame_number, source_monotonic):
        elapsed = max(
            0.0,
            float(source_monotonic)
            - float(self._started_monotonic or source_monotonic),
        )
        segment = int(elapsed // self.segment_seconds)
        return f"camera_frames/segment_{segment:04d}/frame_{int(frame_number):08d}.jpg"

    def enqueue(self, *, frame_number, source_sequence, monotonic, wall_time, jpeg):
        self._drain_status()
        process = self._process
        if process is None or not process.is_alive() or self._queue is None:
            if process is not None and process.exitcode not in {None, 0}:
                self.error = self.error or f"CAMERA_RECORDER_EXIT_{process.exitcode}"
            return None
        filename = self._relative_filename(frame_number, monotonic)
        item = {
            "kind": "frame",
            "frame_number": int(frame_number),
            "source_sequence": source_sequence,
            "monotonic": monotonic,
            "wall_time": wall_time,
            "filename": filename,
            "jpeg": bytes(jpeg),
        }
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            # Drop the current frame only. Never evict a frame that was already
            # accepted by RecordManager, because its camera_timestamps.csv row
            # already points at the promised JPEG filename.
            self.dropped_frames += 1
            return None
        self.enqueued_frames += 1
        return filename

    def _drain_status(self):
        if self._status_queue is None:
            return
        while True:
            try:
                item = self._status_queue.get_nowait()
            except (queue.Empty, OSError, ValueError):
                break
            kind = item.get("kind")
            self.written_frames = max(
                self.written_frames,
                int(item.get("written_frames") or 0),
            )
            self.written_bytes = max(
                self.written_bytes,
                int(item.get("written_bytes") or 0),
            )
            self.last_filename = item.get("last_filename") or self.last_filename
            self.encoder_pid = item.get("encoder_pid") or self.encoder_pid
            if kind == "error":
                self.error = item.get("error") or "CAMERA_RECORDER_ERROR"

    def stop(self, timeout=5.0):
        self._drain_status()
        process = self._process
        if process is None:
            return self.snapshot()

        stop_sent = False
        if process.is_alive() and self._queue is not None:
            # RECORD is already stopping, so a short bounded wait is acceptable
            # here. It lets all previously accepted frames drain before the stop
            # sentinel, preserving timestamp/file consistency without affecting
            # the live control path.
            deadline = time.monotonic() + min(2.0, max(0.25, float(timeout) * 0.5))
            while process.is_alive() and time.monotonic() < deadline:
                try:
                    self._queue.put(_STOP, timeout=0.10)
                    stop_sent = True
                    break
                except queue.Full:
                    self._drain_status()
            if not stop_sent:
                self.error = self.error or "CAMERA_RECORDER_STOP_DRAIN_TIMEOUT"

        process.join(timeout=max(0.5, float(timeout)))
        if process.is_alive():
            process.terminate()
            process.join(timeout=1.0)
            self.error = self.error or "CAMERA_RECORDER_STOP_TIMEOUT"
        self._drain_status()
        if process.exitcode not in {None, 0}:
            self.error = self.error or f"CAMERA_RECORDER_EXIT_{process.exitcode}"
        self._process = None
        return self.snapshot()

    def snapshot(self):
        self._drain_status()
        process = self._process
        depth = None
        if self._queue is not None:
            try:
                depth = self._queue.qsize()
            except (NotImplementedError, OSError):
                pass
        return {
            "alive": bool(process and process.is_alive()),
            "pid": self.encoder_pid,
            "queue_capacity": self.queue_frames,
            "queue_depth": depth,
            "enqueued_frames": self.enqueued_frames,
            "written_frames": self.written_frames,
            "dropped_frames": self.dropped_frames,
            "written_bytes": self.written_bytes,
            "record_width": self.width,
            "record_height": self.height,
            "jpeg_quality": self.jpeg_quality,
            "segment_seconds": self.segment_seconds,
            "last_filename": self.last_filename,
            "error": self.error,
        }


__all__ = ["CameraFrameRecorder"]
