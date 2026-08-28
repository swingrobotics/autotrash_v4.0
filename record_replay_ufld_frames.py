"""JPEG-first compatibility layer for finalized RECORD UFLD reanalysis."""

from __future__ import annotations

import csv
import math
import os
import threading
import time

import record_replay_media as media
import record_replay_ufld as ufld


_INSTALLED = False
_ORIGINAL_ANALYZE = ufld._analyze_session
_ORIGINAL_START = ufld.start_analysis


def _saved_frame(session_path, row):
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


def _has_segmented_frames(session_path):
    return any(_saved_frame(session_path, row) for row in media._camera_rows(session_path))


def _analyze_session(session_name, media_path, session_path):
    if not _has_segmented_frames(session_path):
        return _ORIGINAL_ANALYZE(session_name, media_path, session_path)

    period = ufld._period_seconds()
    output = ufld._result_path(session_path)
    temporary = output + ".tmp"
    row_count = 0
    started = time.time()
    try:
        controller = ufld._offline_controller()
        rows = media._camera_rows(session_path)
        timing = media._camera_timing(session_path)
        offsets = list(timing.get("offsets") or [])
        recorded_duration = ufld._safe_number(timing.get("duration_seconds"))
        total = (
            max(1, int(math.ceil(recorded_duration / period)) + 1)
            if recorded_duration and recorded_duration > 0
            else max(1, len(rows))
        )
        ufld._set_job(total=total)

        with open(temporary, "w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=ufld._RESULT_FIELDS)
            writer.writeheader()
            next_sample_offset = 0.0
            for frame_index, row in enumerate(rows):
                allowed, reason = ufld._analysis_allowed()
                if not allowed:
                    raise RuntimeError(f"UFLD_ANALYSIS_INTERRUPTED:{reason}")
                offset = (
                    float(offsets[frame_index])
                    if frame_index < len(offsets) and offsets[frame_index] is not None
                    else frame_index / 10.0
                )
                if offset + 1e-6 < next_sample_offset:
                    continue
                path = _saved_frame(session_path, row)
                if path is None:
                    next_sample_offset = offset + period
                    continue
                try:
                    with open(path, "rb") as image_file:
                        jpeg = image_file.read()
                    result = controller.analyze_neural_preview_jpeg(jpeg)
                    lane = result.as_dict()
                    diagnostics = dict(controller.preview_snapshot() or {})
                except Exception as error:
                    lane = {
                        "detected": False,
                        "confidence": 0.0,
                        "backend": "UFLD_ONNX",
                        "marking": "OFFLINE_REPLAY",
                        "error": f"RECORDED_FRAME_ANALYSIS_FAILED:{type(error).__name__}:{error}",
                    }
                    diagnostics = {}
                writer.writerow(
                    ufld._row_from_result(offset, frame_index, lane, diagnostics)
                )
                row_count += 1
                next_sample_offset = offset + period
                progress = min(1.0, row_count / total) if total > 0 else 0.0
                ufld._set_job(processed=row_count, progress=progress)
                if row_count % 20 == 0:
                    file.flush()
            file.flush()
            os.fsync(file.fileno())

        if row_count <= 0:
            raise RuntimeError("RECORDED_JPEG_SESSION_HAS_NO_DECODABLE_FRAMES")
        os.replace(temporary, output)
        ufld._RESULT_CACHE.pop(session_name, None)
        metadata = {
            "state": "completed",
            "session": session_name,
            "created_at": time.time(),
            "analysis_elapsed_seconds": max(0.0, time.time() - started),
            "recorded_duration_seconds": recorded_duration,
            "sample_period_seconds": period,
            "rows": row_count,
            "source": "segmented_jpeg_frames_v2",
            "timeline": "camera_timestamps.csv",
            "result": os.path.basename(output),
            "backend": "UFLD_ONNX",
            "execution": "WORKER_UFLD_OFFLINE_REPLAY",
            "control_authority": "NONE",
        }
        ufld._write_metadata(session_path, metadata)
        with ufld._JOB_LOCK:
            prior_total = int(ufld._JOB.get("total") or 0)
        ufld._set_job(
            state="completed",
            progress=1.0,
            processed=row_count,
            total=max(row_count, prior_total),
            error=None,
            finished_at=time.time(),
        )
    except Exception as error:
        try:
            if os.path.exists(temporary):
                os.remove(temporary)
        except OSError:
            pass
        ufld._set_job(
            state="failed",
            error=f"{type(error).__name__}: {error}",
            finished_at=time.time(),
        )


def _start_analysis(session_name, force=False):
    session_path = ufld._session_path(session_name)
    allowed, reason = ufld._analysis_allowed()
    if not allowed:
        raise RuntimeError(reason)

    output = ufld._result_path(session_path)
    if os.path.isfile(output) and not force:
        return ufld.analysis_status(session_name)

    if _has_segmented_frames(session_path):
        media_path = None
    else:
        media_path = ufld._finalized_media_path(session_name)

    with ufld._JOB_LOCK:
        if ufld._JOB.get("state") == "running":
            if ufld._JOB.get("session") == session_name:
                return dict(ufld._JOB)
            raise RuntimeError(f"UFLD_ANALYSIS_BUSY:{ufld._JOB.get('session')}")
        ufld._JOB.update(
            {
                "session": session_name,
                "state": "running",
                "progress": 0.0,
                "processed": 0,
                "total": 0,
                "error": None,
                "started_at": time.time(),
                "finished_at": None,
            }
        )

    thread = threading.Thread(
        target=_analyze_session,
        args=(session_name, media_path, session_path),
        name=f"ufld-replay-{session_name}",
        daemon=True,
    )
    thread.start()
    return ufld.analysis_status(session_name)


def install_frame_replay_ufld():
    global _INSTALLED
    if _INSTALLED:
        return True
    ufld._analyze_session = _analyze_session
    ufld.start_analysis = _start_analysis
    _INSTALLED = True
    return True


__all__ = ["install_frame_replay_ufld"]
