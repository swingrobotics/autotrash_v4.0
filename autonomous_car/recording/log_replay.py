import bisect
import csv
import json
import os
import struct
import zlib


class LogReplay:
    STREAMS = (
        "vehicle_state",
        "gnss",
        "imu",
        "steering",
        "arduino",
        "control",
        "lidar_summary",
        "route",
        "events",
        "camera_timestamps",
        "perception",
    )

    def __init__(self, session_path):
        self.session_path = session_path
        self.streams = {}
        self.timestamps = {}
        metadata_path = os.path.join(session_path, "metadata.json")
        with open(metadata_path, "r", encoding="utf-8") as file:
            self.metadata = json.load(file)
        for stream in self.STREAMS:
            path = os.path.join(session_path, f"{stream}.csv")
            if not os.path.exists(path):
                continue
            with open(path, "r", encoding="utf-8") as file:
                rows = list(csv.DictReader(file))
            valid_rows = []
            timestamps = []
            for row in rows:
                try:
                    timestamp = float(row["monotonic"])
                except (KeyError, TypeError, ValueError):
                    continue
                valid_rows.append(row)
                timestamps.append(timestamp)
            self.streams[stream] = valid_rows
            self.timestamps[stream] = timestamps

    @property
    def time_range(self):
        values = [timestamp for stream in self.timestamps.values() for timestamp in stream]
        return (min(values), max(values)) if values else (None, None)

    def state_at(self, monotonic_timestamp):
        state = {}
        for stream, timestamps in self.timestamps.items():
            index = bisect.bisect_right(timestamps, float(monotonic_timestamp)) - 1
            state[stream] = self.streams[stream][index] if index >= 0 else None
        return state

    def iter_lidar_raw(self):
        path = os.path.join(self.session_path, "lidar_raw.bin")
        if not os.path.exists(path):
            return
        with open(path, "rb") as file:
            while True:
                header = file.read(12)
                if not header:
                    return
                if len(header) != 12:
                    raise ValueError("Truncated lidar raw header")
                timestamp, size = struct.unpack("<dI", header)
                payload = file.read(size)
                if len(payload) != size:
                    raise ValueError("Truncated lidar raw payload")
                if self.metadata.get("lidar_raw_encoding") == "zlib_json_frames_v1":
                    payload = zlib.decompress(payload)
                yield timestamp, json.loads(payload)

    def summary(self):
        lidar_scan_count = 0
        first_lidar_timestamp = None
        last_lidar_timestamp = None
        for timestamp, _ in self.iter_lidar_raw() or ():
            if first_lidar_timestamp is None:
                first_lidar_timestamp = timestamp
            last_lidar_timestamp = timestamp
            lidar_scan_count += 1
        return {
            "session": os.path.basename(os.path.abspath(self.session_path)),
            "time_range": self.time_range,
            "streams": {name: len(rows) for name, rows in self.streams.items()},
            "lidar_scan_count": lidar_scan_count,
            "first_lidar_timestamp": first_lidar_timestamp,
            "last_lidar_timestamp": last_lidar_timestamp,
            "metadata": self.metadata,
        }
