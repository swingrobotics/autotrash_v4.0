import csv
import json
import math
import os
import queue
import struct
import threading
import time
import zlib
from datetime import datetime

from .frame_recorder import CameraFrameRecorder


class RecordManager:
    STREAM_PERIODS = {
        "vehicle_state": 0.05,
        "gnss": 0.10,
        "imu": 0.02,
        "steering": 0.02,
        "arduino": 0.05,
        "control": 0.02,
        "lidar_summary": 0.10,
        "route": 0.10,
        "perception": 0.50,
    }
    STREAM_FIELDS = {
        "vehicle_state": [
            "monotonic", "wall_time", "mode", "system_state", "manual_override",
            "emergency_stop", "fault_code",
        ],
        "gnss": [
            "monotonic", "wall_time", "latitude", "longitude", "altitude_m",
            "rtk_status", "satellites", "hdop", "speed_mps", "course_degrees",
            "gnss_timestamp", "is_valid", "data_age", "error_code",
        ],
        "imu": [
            "monotonic", "wall_time", "yaw_degrees", "pitch_degrees",
            "roll_degrees", "yaw_rate_dps", "acceleration_x",
            "acceleration_y", "acceleration_z", "imu_timestamp",
            "is_valid", "data_age", "error_code",
        ],
        "steering": [
            "monotonic", "wall_time", "raw", "angle_degrees",
            "target_angle_degrees", "motor_command", "error_degrees",
            "is_valid", "data_age", "error_code",
        ],
        "arduino": [
            "monotonic", "wall_time", "port", "connected", "enabled",
            "drive_pwm", "steering_pwm", "hardware_estop_active",
            "watchdog_stop_reason", "last_response_at",
            "is_valid", "data_age", "error_code",
        ],
        "control": [
            "monotonic", "wall_time", "input_source", "gamepad_throttle",
            "requested_throttle", "limited_throttle",
            "final_throttle", "requested_steering", "final_steering", "stop_reason",
            "target_speed_mps", "cross_track_error_m", "target_index",
        ],
        "lidar_summary": [
            "monotonic", "wall_time", "front_min_distance_m", "front_left_distance_m",
            "front_center_distance_m", "front_right_distance_m", "obstacle_state",
            "rotation_hz", "point_count", "is_valid", "data_age", "error_code",
        ],
        "route": [
            "monotonic", "wall_time", "latitude", "longitude", "altitude_m",
            "rtk_status", "speed_mps", "course_degrees", "gnss_timestamp",
            "is_valid", "data_age", "error_code",
        ],
        "events": ["monotonic", "wall_time", "event", "details"],
        "perception": [
            "monotonic", "wall_time", "frame_sequence", "hazard",
            "detection_count", "detections_json", "error", "lane_detected",
            "lane_confidence", "lane_lateral_error_m",
            "lane_heading_error_degrees", "lane_correction_angle_degrees",
            "lane_error", "camera_is_valid", "camera_data_age",
            "camera_error_code",
        ],
    }

    def __init__(
        self,
        root_path,
        sample_provider,
        camera_provider,
        camera_fps=10.0,
        save_camera_frames=False,
        storage_manager=None,
    ):
        self.legacy_root_path = os.path.abspath(root_path)
        self.root_path = self.legacy_root_path
        self.storage_manager = storage_manager
        self.storage_status = None
        self.sample_provider = sample_provider
        self.camera_provider = camera_provider
        self.camera_period = 1.0 / max(1.0, float(camera_fps))
        # Compatibility flag retained for callers; V2 always stores JPEG frames
        # because frame-addressable files remove MP4 timestamp ambiguity.
        self.save_camera_frames = True
        sample_hz = float(os.environ.get("RECORD_SAMPLE_HZ", "20"))
        self.sample_hz = max(10.0, min(50.0, sample_hz))
        self.sample_period = 1.0 / self.sample_hz
        self.flush_interval_seconds = max(
            0.05,
            float(os.environ.get("RECORD_FLUSH_INTERVAL_SECONDS", "0.25")),
        )
        self.lidar_queue_capacity = max(
            1,
            min(32, int(os.environ.get("RECORD_LIDAR_QUEUE_FRAMES", "4"))),
        )
        self.record_camera_width = max(160, int(os.environ.get("RECORD_CAMERA_WIDTH", "640")))
        self.record_camera_height = max(90, int(os.environ.get("RECORD_CAMERA_HEIGHT", "360")))
        self.record_camera_jpeg_quality = max(
            45, min(95, int(os.environ.get("RECORD_CAMERA_JPEG_QUALITY", "82")))
        )
        self.camera_segment_seconds = max(
            10.0, float(os.environ.get("RECORD_CAMERA_SEGMENT_SECONDS", "60"))
        )
        self.camera_queue_capacity = max(
            2, min(64, int(os.environ.get("RECORD_CAMERA_QUEUE_FRAMES", "8")))
        )
        self.video_codec = "jpeg_segments"
        self.lock = threading.RLock()
        self._lidar_file_lock = threading.Lock()
        self.active = False
        self.session_path = None
        self.started_monotonic = None
        self.started_wall_time = None
        self.error = None
        self.sample_count = 0
        self.frame_count = 0
        self.last_camera_sequence = -1
        self.record_gps = True
        self._thread = None
        self._generation = 0
        self._stop_event = threading.Event()
        self._writers = {}
        self._files = {}
        self._last_stream_write = {}
        self._last_flush_monotonic = 0.0
        self.video_error = None
        self._lidar_queue = queue.Queue(maxsize=self.lidar_queue_capacity)
        self._lidar_thread = None
        self._lidar_stop_event = threading.Event()
        self._camera_recorder = CameraFrameRecorder(
            width=self.record_camera_width,
            height=self.record_camera_height,
            jpeg_quality=self.record_camera_jpeg_quality,
            segment_seconds=self.camera_segment_seconds,
            queue_frames=self.camera_queue_capacity,
        )
        self.lidar_raw_enqueued_frames = 0
        self.lidar_raw_written_frames = 0
        self.lidar_raw_dropped_frames = 0
        self.maximum_sample_duration_seconds = 0.0
        self.maximum_sample_schedule_lag_seconds = 0.0
        self.sample_overrun_count = 0
        self.last_camera_monotonic = None
        self.maximum_camera_gap_seconds = 0.0
        self.camera_gap_count_over_200ms = 0
        self.camera_gap_count_over_300ms = 0

    def storage_snapshot(self):
        if self.storage_manager is None:
            try:
                usage = __import__("shutil").disk_usage(self.root_path)
                return {
                    "ready": True,
                    "recordings_root": self.root_path,
                    "free_bytes": int(usage.free),
                    "total_bytes": int(usage.total),
                    "require_removable": False,
                    "error": None,
                }
            except OSError as error:
                return {"ready": False, "recordings_root": self.root_path, "error": str(error)}
        return self.storage_manager.snapshot(create=False, write_probe=False)

    def _prepare_root(self):
        if self.storage_manager is not None:
            root, status = self.storage_manager.require_recordings_root()
            self.root_path = os.path.abspath(root)
            self.storage_status = dict(status)
            return
        os.makedirs(self.root_path, exist_ok=True)
        usage = __import__("shutil").disk_usage(self.root_path)
        if usage.free < 512 * 1024 * 1024:
            raise OSError("At least 512 MB of free disk space is required")
        self.storage_status = self.storage_snapshot()

    def start(self, metadata=None):
        with self.lock:
            if self.active:
                return self.snapshot()
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("Previous recording writer is still stopping")
            if self._lidar_thread is not None and self._lidar_thread.is_alive():
                raise RuntimeError("Previous LiDAR writer is still stopping")

            self._prepare_root()
            document = dict(metadata or {})
            self.record_gps = bool(document.get("record_gps", True))
            session_name = datetime.now().strftime("run_%Y-%m-%d_%H-%M-%S")
            session_path = os.path.join(self.root_path, session_name)
            suffix = 1
            while os.path.exists(session_path):
                session_path = os.path.join(self.root_path, f"{session_name}_{suffix:02d}")
                suffix += 1
            os.makedirs(session_path)
            os.makedirs(os.path.join(session_path, "camera_frames"))
            self.session_path = session_path
            self.started_monotonic = time.monotonic()
            self.started_wall_time = time.time()
            self.error = None
            self.sample_count = 0
            self.frame_count = 0
            self.last_camera_sequence = -1
            self._last_stream_write = {}
            self._last_flush_monotonic = self.started_monotonic
            self.video_error = None
            self._lidar_queue = queue.Queue(maxsize=self.lidar_queue_capacity)
            self._lidar_stop_event = threading.Event()
            self.lidar_raw_enqueued_frames = 0
            self.lidar_raw_written_frames = 0
            self.lidar_raw_dropped_frames = 0
            self.maximum_sample_duration_seconds = 0.0
            self.maximum_sample_schedule_lag_seconds = 0.0
            self.sample_overrun_count = 0
            self.last_camera_monotonic = None
            self.maximum_camera_gap_seconds = 0.0
            self.camera_gap_count_over_200ms = 0
            self.camera_gap_count_over_300ms = 0
            self._open_streams()
            self._camera_recorder.start(session_path, self.started_monotonic)
            self._start_lidar_writer()
            document.update(
                session=os.path.basename(session_path),
                started_wall_time=self.started_wall_time,
                timebase="python_monotonic_seconds",
                camera_storage="segmented_jpeg_frames_v2",
                video_codec=self.video_codec,
                record_camera_fps=round(1.0 / self.camera_period, 3),
                record_camera_resolution=[self.record_camera_width, self.record_camera_height],
                record_camera_jpeg_quality=self.record_camera_jpeg_quality,
                record_camera_segment_seconds=self.camera_segment_seconds,
                record_camera_queue_frames=self.camera_queue_capacity,
                record_sample_hz=self.sample_hz,
                lidar_raw_encoding="zlib_json_frames_v1",
                lidar_raw_writer="bounded_async_latest_frames",
                lidar_raw_queue_frames=self.lidar_queue_capacity,
                live_record_ufld=False,
                record_gps=self.record_gps,
                stream_flush_interval_seconds=self.flush_interval_seconds,
                camera_label_alignment="source frame monotonic vs control/steering sample monotonic",
                recording_storage=self.storage_status,
                postprocess_policy="worker_h264_offline_ufld_dataset",
            )
            self._atomic_json(os.path.join(session_path, "metadata.json"), document)
            self._write_manifest("RECORDING")
            self.active = True
            self._generation += 1
            generation = self._generation
            stop_event = threading.Event()
            self._stop_event = stop_event
            self._write_row("events", self._event("RECORDING_STARTED", ""))
            self._thread = threading.Thread(
                target=self._run,
                args=(generation, stop_event),
                daemon=True,
                name="record-telemetry-writer",
            )
            self._thread.start()
            return self.snapshot()

    def stop(self):
        with self.lock:
            if not self.active:
                thread = self._thread
                stop_event = self._stop_event
            else:
                self._write_row("events", self._event("RECORDING_STOPPED", ""))
                self.active = False
                self._generation += 1
                stop_event = self._stop_event
                stop_event.set()
                thread = self._thread
        stop_event.set()
        if thread and thread is not threading.current_thread():
            thread.join(timeout=3)
        self._stop_lidar_writer()
        camera_status = self._camera_recorder.stop(timeout=5.0)
        with self.lock:
            if thread and thread.is_alive():
                self.error = self.error or "RECORD_THREAD_STOP_TIMEOUT"
            if camera_status.get("error"):
                self.video_error = camera_status.get("error")
            self._close_streams()
            self._write_manifest("FINALIZED" if not self.error else "FINALIZED_WITH_ERRORS")
            return self.snapshot()

    def snapshot(self):
        camera = self._camera_recorder.snapshot()
        with self.lock:
            return {
                "active": self.active,
                "session_path": self.session_path,
                "recordings_root": self.root_path,
                "started_wall_time": self.started_wall_time,
                "duration_seconds": (
                    time.monotonic() - self.started_monotonic
                    if self.started_monotonic is not None
                    else 0.0
                ),
                "sample_count": self.sample_count,
                "frame_count": self.frame_count,
                "record_gps": self.record_gps,
                "sample_hz": self.sample_hz,
                "sample_period_seconds": self.sample_period,
                "video_codec": self.video_codec,
                "record_camera_resolution": [self.record_camera_width, self.record_camera_height],
                "record_camera_fps": round(1.0 / self.camera_period, 3),
                "flush_interval_seconds": self.flush_interval_seconds,
                "generation": self._generation,
                "writer_alive": bool(self._thread and self._thread.is_alive()),
                "camera_recorder": camera,
                "lidar_writer_alive": bool(
                    self._lidar_thread and self._lidar_thread.is_alive()
                ),
                "lidar_raw_queue_capacity": self.lidar_queue_capacity,
                "lidar_raw_queue_depth": self._lidar_queue.qsize(),
                "lidar_raw_enqueued_frames": self.lidar_raw_enqueued_frames,
                "lidar_raw_written_frames": self.lidar_raw_written_frames,
                "lidar_raw_dropped_frames": self.lidar_raw_dropped_frames,
                "maximum_sample_duration_seconds": self.maximum_sample_duration_seconds,
                "maximum_sample_schedule_lag_seconds": self.maximum_sample_schedule_lag_seconds,
                "sample_overrun_count": self.sample_overrun_count,
                "maximum_camera_gap_seconds": self.maximum_camera_gap_seconds,
                "camera_gap_count_over_200ms": self.camera_gap_count_over_200ms,
                "camera_gap_count_over_300ms": self.camera_gap_count_over_300ms,
                "storage": self.storage_snapshot(),
                "live_record_ufld": False,
                "error": self.error,
                "video_error": self.video_error or camera.get("error"),
            }

    def add_event(self, name, details=""):
        with self.lock:
            if not self.active or "events" not in self._writers:
                return False
            self._write_row("events", self._event(str(name), str(details or "")))
            return True

    def _open_streams(self):
        for stream, fields in self.STREAM_FIELDS.items():
            if not self.record_gps and stream in {"gnss", "route"}:
                continue
            file = open(
                os.path.join(self.session_path, f"{stream}.csv"),
                "w",
                newline="",
                encoding="utf-8",
            )
            writer = csv.DictWriter(file, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            self._files[stream] = file
            self._writers[stream] = writer
        camera_file = open(
            os.path.join(self.session_path, "camera_timestamps.csv"),
            "w",
            newline="",
            encoding="utf-8",
        )
        camera_writer = csv.DictWriter(
            camera_file,
            fieldnames=[
                "frame_number",
                "source_sequence",
                "monotonic",
                "wall_time",
                "filename",
                "steering_angle_degrees",
                "target_steering_angle_degrees",
                "requested_throttle",
                "final_throttle",
                "steering_monotonic",
                "control_monotonic",
                "steering_skew_seconds",
                "control_skew_seconds",
            ],
        )
        camera_writer.writeheader()
        self._files["camera"] = camera_file
        self._writers["camera"] = camera_writer
        self._files["lidar_raw"] = open(
            os.path.join(self.session_path, "lidar_raw.bin"),
            "wb",
        )

    def _start_lidar_writer(self):
        stop_event = self._lidar_stop_event
        self._lidar_thread = threading.Thread(
            target=self._lidar_writer_loop,
            args=(stop_event,),
            daemon=True,
            name="record-lidar-writer",
        )
        self._lidar_thread.start()

    def _stop_lidar_writer(self):
        stop_event = self._lidar_stop_event
        stop_event.set()
        thread = self._lidar_thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self.lock:
            if thread and thread.is_alive():
                self.error = self.error or "LIDAR_WRITER_STOP_TIMEOUT"
            if not thread or not thread.is_alive():
                self._lidar_thread = None

    def _enqueue_lidar_raw(self, row, source_timestamp):
        item = (source_timestamp, row)
        try:
            self._lidar_queue.put_nowait(item)
        except queue.Full:
            try:
                self._lidar_queue.get_nowait()
                self._lidar_queue.task_done()
                self.lidar_raw_dropped_frames += 1
            except queue.Empty:
                pass
            try:
                self._lidar_queue.put_nowait(item)
            except queue.Full:
                self.lidar_raw_dropped_frames += 1
                return False
        self.lidar_raw_enqueued_frames += 1
        return True

    def _lidar_writer_loop(self, stop_event):
        while True:
            try:
                source_timestamp, row = self._lidar_queue.get(timeout=0.10)
            except queue.Empty:
                if stop_event.is_set():
                    return
                continue
            try:
                payload = zlib.compress(
                    json.dumps(row, separators=(",", ":")).encode("utf-8"),
                    level=1,
                )
                with self._lidar_file_lock:
                    file = self._files.get("lidar_raw")
                    if file is None or file.closed:
                        return
                    file.write(struct.pack("<dI", source_timestamp, len(payload)))
                    file.write(payload)
                with self.lock:
                    self.lidar_raw_written_frames += 1
            except Exception as error:
                with self.lock:
                    self.error = self.error or f"LIDAR_RAW_WRITE_ERROR:{type(error).__name__}:{error}"
            finally:
                self._lidar_queue.task_done()
            if stop_event.is_set() and self._lidar_queue.empty():
                return

    def _flush_streams(self, sync=False):
        for name, file in list(self._files.items()):
            try:
                if name == "lidar_raw":
                    with self._lidar_file_lock:
                        file.flush()
                        if sync:
                            os.fsync(file.fileno())
                else:
                    file.flush()
                    if sync:
                        os.fsync(file.fileno())
            except OSError:
                pass

    def _close_streams(self):
        self._flush_streams(sync=True)
        for name, file in list(self._files.items()):
            try:
                if name == "lidar_raw":
                    with self._lidar_file_lock:
                        file.close()
                else:
                    file.close()
            except OSError:
                pass
        self._files.clear()
        self._writers.clear()

    def _run_is_current_locked(self, generation, stop_event):
        return (
            self.active
            and self._generation == generation
            and self._stop_event is stop_event
            and not stop_event.is_set()
        )

    def _run(self, generation, stop_event):
        next_camera = time.monotonic()
        next_sample = time.monotonic()
        try:
            while not stop_event.is_set():
                now = time.monotonic()
                wait_seconds = max(0.0, next_sample - now)
                if stop_event.wait(wait_seconds):
                    break
                sample_started = time.monotonic()
                schedule_lag = max(0.0, sample_started - next_sample)
                samples = self.sample_provider()
                sample_finished = time.monotonic()
                sample_duration = max(0.0, sample_finished - sample_started)
                now = sample_finished
                record_camera = False
                flush_due = False
                with self.lock:
                    if not self._run_is_current_locked(generation, stop_event):
                        break
                    self.maximum_sample_duration_seconds = max(
                        self.maximum_sample_duration_seconds,
                        sample_duration,
                    )
                    self.maximum_sample_schedule_lag_seconds = max(
                        self.maximum_sample_schedule_lag_seconds,
                        schedule_lag,
                    )
                    if sample_duration > self.sample_period:
                        self.sample_overrun_count += 1

                    for stream, row in samples.items():
                        period = self.STREAM_PERIODS.get(stream, self.sample_period)
                        last_write = self._last_stream_write.get(stream, 0.0)
                        if stream == "lidar_raw" and row is not None and now - last_write >= 0.10:
                            source_timestamp = self._finite_float(
                                row.get("monotonic") if isinstance(row, dict) else None
                            )
                            if source_timestamp is None:
                                source_timestamp = now
                            self._enqueue_lidar_raw(row, source_timestamp)
                            self._last_stream_write[stream] = now
                            self.sample_count += 1
                            continue
                        if stream in self._writers and row is not None and now - last_write >= period:
                            self._write_row(stream, row)
                            self._last_stream_write[stream] = now
                            self.sample_count += 1
                    if now >= next_camera:
                        record_camera = True
                        next_camera = now + self.camera_period
                    if now - self._last_flush_monotonic >= self.flush_interval_seconds:
                        self._last_flush_monotonic = now
                        flush_due = True

                if record_camera:
                    self._record_camera_frame(samples)
                if flush_due:
                    self._flush_streams(sync=False)
                next_sample = max(
                    next_sample + self.sample_period,
                    sample_started + self.sample_period,
                )
        except Exception as error:
            with self.lock:
                if self._generation == generation and self._stop_event is stop_event:
                    self.error = str(error)
                    self.active = False
                    self._generation += 1
                    stop_event.set()

    def _record_camera_frame(self, samples=None):
        frame, sequence, monotonic_timestamp, wall_time = self.camera_provider()
        if frame is None or sequence == self.last_camera_sequence:
            return
        samples = samples or {}
        steering = samples.get("steering") or {}
        control = samples.get("control") or {}
        steering_monotonic = self._finite_float(steering.get("monotonic"))
        control_monotonic = self._finite_float(control.get("monotonic"))
        source_monotonic = self._finite_float(monotonic_timestamp)
        if source_monotonic is None:
            return
        steering_skew = self._absolute_skew(source_monotonic, steering_monotonic)
        control_skew = self._absolute_skew(source_monotonic, control_monotonic)

        with self.lock:
            if self.last_camera_monotonic is not None:
                gap = max(0.0, source_monotonic - self.last_camera_monotonic)
                self.maximum_camera_gap_seconds = max(self.maximum_camera_gap_seconds, gap)
                if gap > 0.20:
                    self.camera_gap_count_over_200ms += 1
                if gap > 0.30:
                    self.camera_gap_count_over_300ms += 1
            self.last_camera_monotonic = source_monotonic
            self.last_camera_sequence = sequence
            frame_number = self.frame_count + 1

        filename = self._camera_recorder.enqueue(
            frame_number=frame_number,
            source_sequence=sequence,
            monotonic=source_monotonic,
            wall_time=wall_time,
            jpeg=frame,
        )
        if not filename:
            status = self._camera_recorder.snapshot()
            if status.get("error"):
                with self.lock:
                    self.video_error = status.get("error")
            return

        with self.lock:
            writer = self._writers.get("camera")
            if writer is None:
                return
            self.frame_count = frame_number
            writer.writerow(
                {
                    "frame_number": frame_number,
                    "source_sequence": sequence,
                    "monotonic": monotonic_timestamp,
                    "wall_time": wall_time,
                    "filename": filename,
                    "steering_angle_degrees": steering.get("angle_degrees"),
                    "target_steering_angle_degrees": steering.get("target_angle_degrees"),
                    "requested_throttle": control.get("requested_throttle"),
                    "final_throttle": control.get("final_throttle"),
                    "steering_monotonic": steering_monotonic,
                    "control_monotonic": control_monotonic,
                    "steering_skew_seconds": steering_skew,
                    "control_skew_seconds": control_skew,
                }
            )

    def _write_manifest(self, state):
        path = self.session_path
        if not path:
            return
        camera = self._camera_recorder.snapshot()
        segments = []
        camera_root = os.path.join(path, "camera_frames")
        if os.path.isdir(camera_root):
            for entry in sorted(os.scandir(camera_root), key=lambda item: item.name):
                if not entry.is_dir(follow_symlinks=False) or not entry.name.startswith("segment_"):
                    continue
                count = 0
                size = 0
                try:
                    for frame in os.scandir(entry.path):
                        if frame.is_file(follow_symlinks=False) and frame.name.endswith(".jpg"):
                            count += 1
                            try:
                                size += frame.stat().st_size
                            except OSError:
                                pass
                except OSError:
                    pass
                segments.append({"name": entry.name, "frames": count, "size_bytes": size})
        document = {
            "schema": "swing_record_manifest_v2",
            "session": os.path.basename(path),
            "state": state,
            "started_wall_time": self.started_wall_time,
            "duration_seconds": (
                max(0.0, time.monotonic() - self.started_monotonic)
                if self.started_monotonic is not None
                else 0.0
            ),
            "camera": {
                **camera,
                "accepted_frames": self.frame_count,
                "maximum_source_gap_seconds": self.maximum_camera_gap_seconds,
                "gaps_over_200ms": self.camera_gap_count_over_200ms,
                "gaps_over_300ms": self.camera_gap_count_over_300ms,
                "segments": segments,
            },
            "telemetry": {
                "sample_count": self.sample_count,
                "sample_overrun_count": self.sample_overrun_count,
                "maximum_sample_duration_seconds": self.maximum_sample_duration_seconds,
                "maximum_schedule_lag_seconds": self.maximum_sample_schedule_lag_seconds,
                "lidar_raw_enqueued_frames": self.lidar_raw_enqueued_frames,
                "lidar_raw_written_frames": self.lidar_raw_written_frames,
                "lidar_raw_dropped_frames": self.lidar_raw_dropped_frames,
            },
            "storage": self.storage_snapshot(),
            "error": self.error,
        }
        self._atomic_json(os.path.join(path, "record_manifest.json"), document)

    def _write_row(self, stream, row):
        self._writers[stream].writerow(row)

    @staticmethod
    def _atomic_json(path, document):
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as file:
            json.dump(document, file, ensure_ascii=False, indent=2, sort_keys=True)
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)

    @staticmethod
    def _finite_float(value):
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None

    @staticmethod
    def _absolute_skew(first, second):
        if first is None or second is None:
            return None
        return abs(float(first) - float(second))

    @staticmethod
    def _event(name, details):
        return {
            "monotonic": time.monotonic(),
            "wall_time": time.time(),
            "event": name,
            "details": details,
        }
