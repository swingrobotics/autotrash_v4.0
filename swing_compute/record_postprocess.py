"""PC-side post-processing for finalized SWING RECORD sessions."""

from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import zlib


def _safe_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _camera_rows(session_path: Path):
    path = session_path / "camera_timestamps.csv"
    rows = []
    with open(path, "r", encoding="utf-8", newline="") as file:
        for row in csv.DictReader(file):
            value = dict(row)
            value["_monotonic"] = _safe_float(row.get("monotonic"))
            value["_wall_time"] = _safe_float(row.get("wall_time"))
            rows.append(value)
    return rows


def _frame_path(session_path: Path, row):
    raw = str(row.get("filename") or "").strip().replace("\\", "/")
    if not raw:
        return None
    relative = raw if raw.startswith("camera_frames/") else "camera_frames/" + raw.lstrip("/")
    candidate = (session_path / Path(relative)).resolve()
    try:
        candidate.relative_to(session_path.resolve())
    except ValueError:
        return None
    return candidate if candidate.is_file() else None


def _camera_entries(session_path: Path):
    rows = _camera_rows(session_path)
    valid = []
    first = next((row["_monotonic"] for row in rows if row["_monotonic"] is not None), None)
    for index, row in enumerate(rows):
        path = _frame_path(session_path, row)
        monotonic = row.get("_monotonic")
        if path is None or monotonic is None:
            continue
        valid.append(
            {
                "row": row,
                "path": path,
                "index": index,
                "offset": 0.0 if first is None else max(0.0, monotonic - first),
            }
        )
    return valid


def _ffmpeg_executable():
    configured = str(os.environ.get("SWING_WORKER_FFMPEG") or "").strip()
    if configured:
        return configured
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        value = shutil.which("ffmpeg")
        if value:
            return value
    raise RuntimeError("WORKER_FFMPEG_UNAVAILABLE")


def _concat_quote(path: Path):
    return str(path.resolve()).replace("'", "'\\''")


def build_h264(session_path, output_name="camera_browser_v2.mp4", progress=None):
    """Create browser H.264 with frame durations taken from Pi timestamps."""
    session = Path(session_path).resolve()
    entries = _camera_entries(session)
    if len(entries) < 2:
        raise ValueError("RECORD_CAMERA_FRAMES_INSUFFICIENT")
    progress = progress or (lambda *_args, **_kwargs: None)
    output = session / output_name
    temporary = Path(str(output) + ".part.mp4")
    concat = session / ".camera_concat_worker.txt"
    intervals = [
        b["offset"] - a["offset"]
        for a, b in zip(entries, entries[1:])
        if 0.001 < b["offset"] - a["offset"] < 5.0
    ]
    intervals_sorted = sorted(intervals)
    typical = (
        intervals_sorted[len(intervals_sorted) // 2]
        if intervals_sorted
        else 0.1
    )
    try:
        with open(concat, "w", encoding="utf-8", newline="\n") as file:
            for index, entry in enumerate(entries):
                duration = (
                    entries[index + 1]["offset"] - entry["offset"]
                    if index + 1 < len(entries)
                    else typical
                )
                duration = max(0.001, min(5.0, float(duration)))
                file.write(f"file '{_concat_quote(entry['path'])}'\n")
                file.write(f"duration {duration:.9f}\n")
            # ffmpeg concat demuxer applies the final duration only when the last
            # file is repeated.
            file.write(f"file '{_concat_quote(entries[-1]['path'])}'\n")
        progress(0.1, "H.264 변환 준비 완료")
        command = [
            _ffmpeg_executable(),
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "concat",
            "-safe",
            "0",
            "-i",
            str(concat),
            "-an",
            "-vf",
            "scale=trunc(iw/2)*2:trunc(ih/2)*2",
            "-vsync",
            "vfr",
            "-c:v",
            "libx264",
            "-preset",
            "fast",
            "-crf",
            "23",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(temporary),
        ]
        result = subprocess.run(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=max(300, int(len(entries) / 10 * 3)),
            check=False,
        )
        if result.returncode != 0:
            details = result.stderr.decode("utf-8", errors="replace").strip()
            raise RuntimeError(
                "WORKER_H264_TRANSCODE_FAILED"
                + (f":{details[-1600:]}" if details else "")
            )
        if not temporary.is_file() or temporary.stat().st_size <= 0:
            raise RuntimeError("WORKER_H264_TRANSCODE_EMPTY")
        os.replace(temporary, output)
        progress(1.0, "H.264 변환 완료")
        return {
            "path": str(output),
            "bytes": output.stat().st_size,
            "frames": len(entries),
            "duration_seconds": entries[-1]["offset"] + typical,
            "timeline": "camera_timestamps.csv",
        }
    finally:
        concat.unlink(missing_ok=True)
        temporary.unlink(missing_ok=True)


def _timebase_offset(session_path: Path):
    candidates = []
    for name in (
        "vehicle_state.csv",
        "control.csv",
        "steering.csv",
        "imu.csv",
        "gnss.csv",
        "camera_timestamps.csv",
    ):
        path = session_path / name
        if not path.is_file():
            continue
        try:
            with open(path, "r", encoding="utf-8", newline="") as file:
                for row in csv.DictReader(file):
                    monotonic = _safe_float(row.get("monotonic"))
                    wall = _safe_float(row.get("wall_time"))
                    if monotonic is not None and wall is not None:
                        candidates.append(wall - monotonic)
                        break
        except OSError:
            pass
    return candidates[0] if candidates else 0.0


def build_mcap(session_path, output_name="session.mcap", progress=None):
    """Materialize a portable indexed MCAP after the rover has finalized RECORD."""
    try:
        from mcap.writer import CompressionType, Writer
    except ImportError as error:
        raise RuntimeError("MCAP_PYTHON_UNAVAILABLE") from error

    session = Path(session_path).resolve()
    output = session / output_name
    temporary = Path(str(output) + ".part")
    progress = progress or (lambda *_args, **_kwargs: None)
    csv_files = [
        name
        for name in (
            "vehicle_state.csv",
            "gnss.csv",
            "imu.csv",
            "steering.csv",
            "arduino.csv",
            "control.csv",
            "lidar_summary.csv",
            "route.csv",
            "events.csv",
            "perception.csv",
        )
        if (session / name).is_file()
    ]
    entries = _camera_entries(session)
    monotonic_to_wall = _timebase_offset(session)

    def time_ns(monotonic, wall):
        wall_value = _safe_float(wall)
        if wall_value is None:
            mono = _safe_float(monotonic) or 0.0
            wall_value = mono + monotonic_to_wall
        return max(0, int(wall_value * 1_000_000_000))

    try:
        with open(temporary, "wb") as stream:
            writer = Writer(stream, compression=CompressionType.NONE)
            writer.start(profile="", library="SWING Compute Worker")
            json_schema = writer.register_schema(
                name="swing.record.json",
                encoding="jsonschema",
                data=json.dumps({"type": "object"}, separators=(",", ":")).encode("utf-8"),
            )
            channels = {}
            for filename in csv_files:
                topic = "/" + filename[:-4]
                channels[filename] = writer.register_channel(
                    topic=topic,
                    message_encoding="json",
                    schema_id=json_schema,
                )
            camera_channel = writer.register_channel(
                topic="/camera/jpeg",
                message_encoding="jpeg",
                schema_id=0,
                metadata={"timeline": "camera_timestamps.csv"},
            )
            lidar_channel = None
            if (session / "lidar_raw.bin").is_file():
                lidar_channel = writer.register_channel(
                    topic="/lidar/raw",
                    message_encoding="json",
                    schema_id=json_schema,
                )

            count = 0
            for filename in csv_files:
                with open(session / filename, "r", encoding="utf-8", newline="") as file:
                    for sequence, row in enumerate(csv.DictReader(file), 1):
                        timestamp = time_ns(row.get("monotonic"), row.get("wall_time"))
                        writer.add_message(
                            channels[filename],
                            log_time=timestamp,
                            publish_time=timestamp,
                            sequence=sequence,
                            data=json.dumps(row, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
                        )
                        count += 1

            for sequence, entry in enumerate(entries, 1):
                row = entry["row"]
                timestamp = time_ns(row.get("monotonic"), row.get("wall_time"))
                with open(entry["path"], "rb") as file:
                    jpeg = file.read()
                writer.add_message(
                    camera_channel,
                    log_time=timestamp,
                    publish_time=timestamp,
                    sequence=sequence,
                    data=jpeg,
                )
                count += 1

            if lidar_channel is not None:
                with open(session / "lidar_raw.bin", "rb") as file:
                    sequence = 0
                    while True:
                        header = file.read(12)
                        if not header:
                            break
                        if len(header) != 12:
                            raise OSError("LIDAR_RAW_TRUNCATED_HEADER")
                        monotonic, length = struct.unpack("<dI", header)
                        payload = file.read(length)
                        if len(payload) != length:
                            raise OSError("LIDAR_RAW_TRUNCATED_PAYLOAD")
                        sequence += 1
                        timestamp = time_ns(monotonic, None)
                        try:
                            data = zlib.decompress(payload)
                            json.loads(data.decode("utf-8"))
                        except Exception:
                            data = json.dumps(
                                {"decode_error": True, "compressed_bytes": length},
                                separators=(",", ":"),
                            ).encode("utf-8")
                        writer.add_message(
                            lidar_channel,
                            log_time=timestamp,
                            publish_time=timestamp,
                            sequence=sequence,
                            data=data,
                        )
                        count += 1
            writer.add_metadata(
                "swing.record",
                {
                    "session": session.name,
                    "source": "finalized_record",
                    "camera_storage": "segmented_jpeg_or_legacy",
                },
            )
            writer.finish()
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, output)
        progress(1.0, "MCAP 변환 완료")
        return {"path": str(output), "bytes": output.stat().st_size, "messages": count}
    finally:
        temporary.unlink(missing_ok=True)


def postprocess_session(session_path, *, make_h264=True, make_mcap=False, progress=None):
    session = Path(session_path).resolve()
    if not (session / "camera_timestamps.csv").is_file():
        raise FileNotFoundError("camera_timestamps.csv not found")
    progress = progress or (lambda *_args, **_kwargs: None)
    result = {"session": session.name, "h264": None, "mcap": None}
    if make_h264:
        result["h264"] = build_h264(
            session,
            progress=lambda value, message: progress(0.05 + value * 0.70, message),
        )
    if make_mcap:
        result["mcap"] = build_mcap(
            session,
            progress=lambda value, message: progress(0.78 + value * 0.20, message),
        )
    progress(1.0, "RECORD 후처리 완료")
    return result


__all__ = ["build_h264", "build_mcap", "postprocess_session"]
