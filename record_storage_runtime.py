"""Runtime bridge for removable RECORD storage without changing vehicle safety code."""

from __future__ import annotations

import json
import os

from autonomous_car.recording import RecordStorageManager


_INSTALLED = False


def install_record_storage_runtime(legacy):
    """Route new RECORD sessions to USB while keeping legacy sessions readable."""
    global _INSTALLED
    if _INSTALLED:
        return getattr(legacy, "RECORD_STORAGE_MANAGER", None)

    manager = legacy.record_manager
    storage = RecordStorageManager.from_environment()
    manager.storage_manager = storage
    legacy.RECORD_STORAGE_MANAGER = storage
    legacy.LEGACY_RECORDINGS_PATH = os.path.abspath(legacy.RECORDINGS_PATH)

    def recording_roots():
        values = []

        def add(path, kind):
            if not path:
                return
            real = os.path.abspath(os.path.realpath(str(path)))
            if any(item["path"] == real for item in values):
                return
            if os.path.isdir(real):
                values.append({"path": real, "kind": kind})

        # Keep an already selected/active removable root visible even when the
        # device disappears after recording starts; status will still report the
        # storage failure separately.
        add(getattr(manager, "root_path", None), "USB")
        status = manager.storage_snapshot()
        add(status.get("recordings_root"), "USB")
        add(legacy.LEGACY_RECORDINGS_PATH, "LEGACY")
        return values

    def primary_recordings_root():
        status = manager.storage_snapshot()
        path = status.get("recordings_root")
        if path:
            return os.path.abspath(os.path.realpath(path))
        root = getattr(manager, "root_path", None)
        if root and os.path.abspath(root) != legacy.LEGACY_RECORDINGS_PATH:
            return os.path.abspath(os.path.realpath(root))
        return None

    def recording_session_path(session_name):
        raw = str(session_name or "").strip()
        name = os.path.basename(raw)
        if not raw or raw != name or name in {".", ".."}:
            raise ValueError("Invalid recording session")
        for root in recording_roots():
            candidate = os.path.abspath(os.path.join(root["path"], name))
            if os.path.commonpath([root["path"], candidate]) != root["path"]:
                continue
            if os.path.isdir(candidate):
                return candidate
        primary = primary_recordings_root()
        if primary:
            return os.path.join(primary, name)
        # Legacy path is returned only for compatibility with callers that will
        # subsequently perform their own existence check. New RECORD creation is
        # always handled by RecordManager and never falls back here.
        return os.path.join(legacy.LEGACY_RECORDINGS_PATH, name)

    def resolve_recording_path(session_name, filename):
        session = recording_session_path(session_name)
        safe_filename = os.path.basename(str(filename or ""))
        if not safe_filename or safe_filename != str(filename):
            raise ValueError("Invalid recording filename")
        path = os.path.abspath(os.path.join(session, safe_filename))
        if os.path.commonpath([os.path.abspath(session), path]) != os.path.abspath(session):
            raise ValueError("Invalid recording path")
        return path

    def list_recording_sessions():
        active_path = (
            os.path.abspath(manager.session_path)
            if manager.active and manager.session_path
            else None
        )
        sessions = {}
        for root in recording_roots():
            try:
                entries = list(os.scandir(root["path"]))
            except OSError:
                continue
            for entry in entries:
                if not entry.is_dir(follow_symlinks=False):
                    continue
                # Prefer USB if duplicate legacy/USB session names exist.
                existing = sessions.get(entry.name)
                if existing is not None and existing.get("storage_kind") == "USB":
                    continue
                metadata = {}
                manifest = {}
                try:
                    with open(os.path.join(entry.path, "metadata.json"), "r", encoding="utf-8") as file:
                        metadata = json.load(file)
                except (OSError, ValueError, TypeError, json.JSONDecodeError):
                    pass
                try:
                    with open(os.path.join(entry.path, "record_manifest.json"), "r", encoding="utf-8") as file:
                        manifest = json.load(file)
                except (OSError, ValueError, TypeError, json.JSONDecodeError):
                    pass
                sessions[entry.name] = {
                    "session": entry.name,
                    "label": str(metadata.get("label") or ""),
                    "started_wall_time": metadata.get("started_wall_time"),
                    "size_bytes": legacy.recording_directory_size(entry.path),
                    "active": active_path == os.path.abspath(entry.path),
                    "has_route": os.path.isfile(os.path.join(entry.path, "route.csv")),
                    "has_processed_route": os.path.isfile(
                        os.path.join(entry.path, "processed_route.json")
                    ),
                    "has_camera_frames": os.path.isfile(
                        os.path.join(entry.path, "camera_timestamps.csv")
                    ) and os.path.isdir(os.path.join(entry.path, "camera_frames")),
                    "has_camera_mp4": os.path.isfile(os.path.join(entry.path, "camera.mp4")),
                    "record_state": manifest.get("state"),
                    "storage_kind": root["kind"],
                    "storage_root": root["path"],
                }
        values = list(sessions.values())
        values.sort(key=lambda item: item.get("started_wall_time") or 0, reverse=True)
        return {"sessions": values, "storage": manager.storage_snapshot()}

    legacy.recording_roots = recording_roots
    legacy.recording_session_path = recording_session_path
    legacy.resolve_recording_path = resolve_recording_path
    legacy.list_recording_sessions = list_recording_sessions

    handler = legacy.CameraHandler
    original_do_get = handler.do_GET
    if not getattr(original_do_get, "_swing_record_storage", False):
        def do_get_with_record_storage(self):
            path = str(self.path or "").split("?", 1)[0]
            if path == "/api/recording/storage":
                self._send_json(manager.storage_snapshot())
                return
            return original_do_get(self)

        do_get_with_record_storage._swing_record_storage = True
        handler.do_GET = do_get_with_record_storage

    _INSTALLED = True
    return storage


__all__ = ["install_record_storage_runtime"]
