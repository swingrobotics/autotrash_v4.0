"""Regression for control-priority JPEG-first RECORD policy."""

import os
import tempfile

from autonomous_car.recording.record_manager import RecordManager


def _require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    previous = {
        name: os.environ.get(name)
        for name in (
            "RECORD_SAMPLE_HZ",
            "RECORD_LIDAR_QUEUE_FRAMES",
            "RECORD_CAMERA_WIDTH",
            "RECORD_CAMERA_HEIGHT",
            "RECORD_CAMERA_QUEUE_FRAMES",
        )
    }
    os.environ["RECORD_SAMPLE_HZ"] = "20"
    os.environ["RECORD_LIDAR_QUEUE_FRAMES"] = "2"
    os.environ["RECORD_CAMERA_WIDTH"] = "640"
    os.environ["RECORD_CAMERA_HEIGHT"] = "360"
    os.environ["RECORD_CAMERA_QUEUE_FRAMES"] = "2"
    try:
        with tempfile.TemporaryDirectory() as directory:
            manager = RecordManager(
                directory,
                sample_provider=lambda: {},
                camera_provider=lambda: (None, -1, None, None),
                camera_fps=10.0,
            )
            manager.session_path = directory

            _require(manager.sample_hz == 20.0, manager.snapshot())
            _require(abs(manager.sample_period - 0.05) < 1e-9, manager.sample_period)
            _require(
                manager.sample_period <= manager.camera_period / 2.0 + 1e-9,
                "record control/steering sampling must be at least 2x camera FPS",
            )
            _require(manager.video_codec == "jpeg_segments", manager.video_codec)
            _require(not hasattr(manager, "_video_command"), "Pi RECORD still owns an ffmpeg video path")
            _require(manager.record_camera_width == 640, manager.record_camera_width)
            _require(manager.record_camera_height == 360, manager.record_camera_height)
            _require(manager.camera_queue_capacity == 2, manager.camera_queue_capacity)
            _require(
                manager._camera_recorder.queue_frames == 2,
                manager._camera_recorder.snapshot(),
            )
            manager._camera_recorder._started_monotonic = 100.0
            _require(
                manager._camera_recorder._relative_filename(1, 161.0)
                == "camera_frames/segment_0001/frame_00000001.jpg",
                "60-second camera segmentation changed",
            )

            # Bounded latest LiDAR queue may drop stale LiDAR because no CSV row
            # promises a one-to-one raw-frame file. Camera is different: an
            # accepted camera filename is timestamped, so CameraFrameRecorder
            # must drop only the current not-yet-accepted frame when full.
            _require(manager.lidar_queue_capacity == 2, manager.lidar_queue_capacity)
            _require(manager._enqueue_lidar_raw({"points": [1]}, 1.0), "enqueue 1")
            _require(manager._enqueue_lidar_raw({"points": [2]}, 2.0), "enqueue 2")
            _require(manager._enqueue_lidar_raw({"points": [3]}, 3.0), "enqueue 3")
            _require(manager._lidar_queue.qsize() == 2, manager.snapshot())
            _require(manager.lidar_raw_dropped_frames == 1, manager.snapshot())

            record_source = open(
                "autonomous_car/recording/record_manager.py",
                encoding="utf-8",
            ).read()
            recorder_source = open(
                "autonomous_car/recording/frame_recorder.py",
                encoding="utf-8",
            ).read()
            _require(
                "next_sample = max(" in record_source,
                "RECORD writer can still create a CPU catch-up burst after an overrun",
            )
            _require(
                "def _lidar_writer_loop" in record_source,
                "LiDAR JSON/zlib remains synchronous in the main RECORD loop",
            )
            _require(
                "camera_gap_count_over_300ms" in record_source,
                "camera-gap telemetry missing",
            )
            _require(
                "sample_overrun_count" in record_source,
                "RECORD sample-overrun telemetry missing",
            )
            _require(
                "subprocess" not in record_source and "ffmpeg" not in record_source.lower(),
                "Pi RecordManager still performs video mux/transcode work",
            )
            _require(
                'mp.get_context("spawn")' in recorder_source,
                "camera storage is not isolated in a spawned process",
            )
            _require("os.nice(10)" in recorder_source, "recorder child priority is not lowered")
            _require("put_nowait(item)" in recorder_source, "camera enqueue became blocking")
            _require(
                "self._queue.get_nowait()" not in recorder_source,
                "camera backlog can evict an already timestamped frame",
            )
            _require(
                "CAMERA_RECORDER_STOP_DRAIN_TIMEOUT" in recorder_source,
                "accepted camera frames are not bounded-drained at RECORD stop",
            )
    finally:
        for name, value in previous.items():
            if value is None:
                os.environ.pop(name, None)
            else:
                os.environ[name] = value

    print("RECORD control-priority JPEG V2 regression: PASS")
    print(
        {
            "record_sample_hz": 20.0,
            "camera_fps": 10.0,
            "camera_storage": "segmented_jpeg_frames_v2",
            "camera_resolution": [640, 360],
            "camera_writer": "spawned_bounded_process",
            "lidar_writer": "bounded_async_latest_frames",
            "live_record_ufld": False,
        }
    )


if __name__ == "__main__":
    main()
