from __future__ import annotations

import os
from pathlib import Path, PurePosixPath


# Source artifacts that are allowed to leave the rover through the short-lived
# Compute Worker transfer capability. Derived Worker outputs such as H.264,
# MCAP and offline UFLD sidecars are deliberately excluded and can be rebuilt.
SOURCE_TOP_LEVEL_FILES = (
    "metadata.json",
    "record_manifest.json",
    "vehicle_state.csv",
    "camera_timestamps.csv",
    "camera.mp4",  # legacy RECORD compatibility
    "lidar_raw.bin",
    "imu.csv",
    "control.csv",
    "steering.csv",
    "perception.csv",
    "events.csv",
    "gnss.csv",
    "arduino.csv",
    "lidar_summary.csv",
    "route.csv",
)
_ALLOWED_COMPONENT_CHARACTERS = set(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_."
)


def normalize_record_relative_path(value):
    """Return one canonical POSIX-style source path or reject it.

    New RECORD sessions contain thousands of nested JPEG files. Keep the
    transfer namespace intentionally narrow: known top-level source artifacts
    plus JPEGs below camera_frames/. This also makes rover and Worker path
    validation identical on Linux and Windows.
    """

    text = str(value or "").strip()
    if not text or len(text) > 512 or "\\" in text or "\x00" in text:
        raise ValueError("INVALID_RECORD_FILE_PATH")
    path = PurePosixPath(text)
    if path.is_absolute() or not path.parts:
        raise ValueError("INVALID_RECORD_FILE_PATH")
    for component in path.parts:
        if (
            component in {"", ".", ".."}
            or len(component) > 160
            or any(character not in _ALLOWED_COMPONENT_CHARACTERS for character in component)
        ):
            raise ValueError("INVALID_RECORD_FILE_PATH")

    if len(path.parts) == 1:
        if path.name not in SOURCE_TOP_LEVEL_FILES:
            raise ValueError("RECORD_FILE_NOT_ALLOWED")
        return path.as_posix()

    if path.parts[0] != "camera_frames" or path.suffix.lower() not in {".jpg", ".jpeg"}:
        raise ValueError("RECORD_FILE_NOT_ALLOWED")
    return path.as_posix()


def record_source_path(session_path, relative_path, *, require_file=True):
    root = Path(session_path).resolve()
    relative = normalize_record_relative_path(relative_path)
    candidate = (root / Path(*PurePosixPath(relative).parts)).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as error:
        raise ValueError("RECORD_FILE_PATH_ESCAPE") from error
    if require_file and not candidate.is_file():
        raise FileNotFoundError(relative)
    return candidate


def iter_record_source_files(session_path):
    """Yield deterministic (relative_name, resolved_path) source artifacts."""

    root = Path(session_path).resolve()
    if not root.is_dir():
        raise FileNotFoundError(str(root))

    for name in SOURCE_TOP_LEVEL_FILES:
        candidate = root / name
        if not candidate.is_file() or candidate.is_symlink():
            continue
        yield name, record_source_path(root, name)

    camera_root = root / "camera_frames"
    if not camera_root.is_dir() or camera_root.is_symlink():
        return
    frames = sorted(
        path
        for path in camera_root.rglob("*")
        if path.is_file()
        and not path.is_symlink()
        and path.suffix.lower() in {".jpg", ".jpeg"}
        and not path.name.endswith(".part")
    )
    for path in frames:
        relative = path.relative_to(root).as_posix()
        normalized = normalize_record_relative_path(relative)
        resolved = record_source_path(root, normalized)
        # Guard against a symlinked ancestor that resolves outside the session.
        if os.path.commonpath([str(root), str(resolved)]) != str(root):
            raise ValueError("RECORD_FILE_PATH_ESCAPE")
        yield normalized, resolved


__all__ = [
    "SOURCE_TOP_LEVEL_FILES",
    "iter_record_source_files",
    "normalize_record_relative_path",
    "record_source_path",
]
