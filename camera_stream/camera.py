import subprocess
import threading
import time

from .config import CAMERA_DEVICE, CAMERA_FRAMERATE, CAMERA_SIZE


class Camera:
    def __init__(self):
        self.condition = threading.Condition()
        self.frame = None
        self.sequence = 0
        self.frame_monotonic = None
        self.frame_wall_time = None

    def start(self):
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def wait_for_frame(self, previous_sequence):
        with self.condition:
            self.condition.wait_for(
                lambda: self.frame is not None and self.sequence != previous_sequence,
                timeout=5,
            )
            return self.frame, self.sequence

    def snapshot_frame(self):
        with self.condition:
            return self.frame, self.sequence, self.frame_monotonic, self.frame_wall_time

    def _capture_loop(self):
        command = [
            "/usr/bin/ffmpeg",
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "warning",
            "-f",
            "v4l2",
            "-input_format",
            "mjpeg",
            "-video_size",
            CAMERA_SIZE,
            "-framerate",
            CAMERA_FRAMERATE,
            "-i",
            CAMERA_DEVICE,
            "-an",
            "-c:v",
            "copy",
            "-f",
            "mjpeg",
            "pipe:1",
        ]

        while True:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
            buffer = bytearray()

            try:
                while process.stdout:
                    chunk = process.stdout.read(65536)
                    if not chunk:
                        break
                    buffer.extend(chunk)

                    while True:
                        start = buffer.find(b"\xff\xd8")
                        if start < 0:
                            if len(buffer) > 1:
                                del buffer[:-1]
                            break

                        end = buffer.find(b"\xff\xd9", start + 2)
                        if end < 0:
                            if start:
                                del buffer[:start]
                            break

                        frame = bytes(buffer[start : end + 2])
                        del buffer[: end + 2]
                        with self.condition:
                            self.frame = frame
                            self.sequence += 1
                            self.frame_monotonic = time.monotonic()
                            self.frame_wall_time = time.time()
                            self.condition.notify_all()
            finally:
                process.kill()
                process.wait()

            time.sleep(2)
