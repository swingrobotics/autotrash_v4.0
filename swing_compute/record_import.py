"""Discover and import SWING_DATA RECORD sessions on a Windows Worker."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import shutil

try:
    import psutil
except ImportError:  # pragma: no cover
    psutil = None


def _sha256(path: Path):
    digest = hashlib.sha256()
    with open(path, "rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def discover_record_sources():
    roots = []

    def add(value):
        if not value:
            return
        root = Path(value).expanduser().resolve()
        candidates = [root]
        if root.name.upper() != "SWING_DATA":
            candidates.append(root / "SWING_DATA")
        for candidate in candidates:
            recordings = candidate / "recordings"
            if recordings.is_dir() and recordings not in roots:
                roots.append(recordings)

    configured = str(os.environ.get("SWING_RECORD_IMPORT_ROOTS") or "")
    for value in configured.split(os.pathsep):
        if value.strip():
            add(value.strip())
    if psutil is not None:
        try:
            for partition in psutil.disk_partitions(all=False):
                add(partition.mountpoint)
        except Exception:
            pass
    return [str(path) for path in roots]


def list_usb_sessions():
    result = []
    for root_text in discover_record_sources():
        root = Path(root_text)
        try:
            entries = list(root.iterdir())
        except OSError:
            continue
        for entry in entries:
            if not entry.is_dir():
                continue
            manifest = entry / "record_manifest.json"
            camera = entry / "camera_timestamps.csv"
            if not camera.is_file():
                continue
            result.append(
                {
                    "session": entry.name,
                    "source_root": str(root),
                    "manifest": str(manifest) if manifest.is_file() else None,
                    "finalized": _manifest_finalized(manifest),
                }
            )
    result.sort(key=lambda item: (item["source_root"], item["session"]))
    return result


def _manifest_finalized(path: Path):
    if not path.is_file():
        return True  # legacy sessions predate the manifest
    try:
        import json

        document = json.loads(path.read_text(encoding="utf-8"))
        return str(document.get("state") or "").startswith("FINALIZED")
    except Exception:
        return False


def _copy_file_verified(source: Path, destination: Path):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_file() and destination.stat().st_size == source.stat().st_size:
        if _sha256(destination) == _sha256(source):
            return False
    temporary = Path(str(destination) + ".part")
    with open(source, "rb") as src, open(temporary, "wb") as dst:
        shutil.copyfileobj(src, dst, length=1024 * 1024)
        dst.flush()
        os.fsync(dst.fileno())
    if _sha256(temporary) != _sha256(source):
        temporary.unlink(missing_ok=True)
        raise OSError(f"RECORD_IMPORT_SHA256_MISMATCH:{source}")
    os.replace(temporary, destination)
    return True


def import_sessions(destination_root, sessions=None, source_root=None, progress=None, cancelled=None):
    destination_root = Path(destination_root).resolve()
    destination_root.mkdir(parents=True, exist_ok=True)
    progress = progress or (lambda *_args, **_kwargs: None)
    cancelled = cancelled or (lambda: False)
    requested = None if sessions is None else set(str(item) for item in sessions)
    sources = [Path(source_root).resolve()] if source_root else [Path(p) for p in discover_record_sources()]
    selected = []
    for root in sources:
        if not root.is_dir():
            continue
        for entry in root.iterdir():
            if not entry.is_dir() or (requested is not None and entry.name not in requested):
                continue
            if not (entry / "camera_timestamps.csv").is_file():
                continue
            if not _manifest_finalized(entry / "record_manifest.json"):
                continue
            selected.append((root, entry))
    if requested is not None:
        found = {entry.name for _, entry in selected}
        missing = sorted(requested - found)
        if missing:
            raise FileNotFoundError(f"USB_RECORD_SESSIONS_NOT_FOUND:{missing}")

    files = []
    for _, session in selected:
        for path in session.rglob("*"):
            if path.is_file() and not path.name.endswith(".part"):
                files.append((session, path))
    copied = 0
    reused = 0
    for index, (session, source) in enumerate(files, 1):
        if cancelled():
            raise RuntimeError("JOB_CANCELLED")
        relative = source.relative_to(session)
        destination = destination_root / session.name / relative
        if _copy_file_verified(source, destination):
            copied += 1
        else:
            reused += 1
        progress(index, len(files), f"{session.name}: {relative}")
    return {
        "sessions": sorted({session.name for _, session in selected}),
        "files": len(files),
        "copied_files": copied,
        "reused_files": reused,
        "destination": str(destination_root),
    }


__all__ = ["discover_record_sources", "import_sessions", "list_usb_sessions"]
