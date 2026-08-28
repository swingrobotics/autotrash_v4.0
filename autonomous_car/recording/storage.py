from __future__ import annotations

import os
import re
import shutil
from pathlib import Path


_MOUNT_ESCAPE = re.compile(r"\\([0-7]{3})")


def _decode_mount_field(value: str) -> str:
    return _MOUNT_ESCAPE.sub(lambda match: chr(int(match.group(1), 8)), value)


def _atomic_probe(path: Path) -> None:
    probe = path / ".swing-write-probe.tmp"
    with open(probe, "wb") as file:
        file.write(b"SWING\n")
        file.flush()
        os.fsync(file.fileno())
    probe.unlink(missing_ok=True)


def _base_block_name(device: str) -> str | None:
    if not str(device).startswith("/dev/"):
        return None
    name = os.path.basename(os.path.realpath(device))
    if re.fullmatch(r"nvme\d+n\d+p\d+", name):
        return re.sub(r"p\d+$", "", name)
    if re.fullmatch(r"mmcblk\d+p\d+", name):
        return re.sub(r"p\d+$", "", name)
    return re.sub(r"\d+$", "", name) or name


def _device_is_removable(device: str) -> bool:
    names = []
    raw = os.path.basename(os.path.realpath(device))
    if raw:
        names.append(raw)
    base = _base_block_name(device)
    if base and base not in names:
        names.append(base)
    for name in names:
        try:
            value = Path("/sys/class/block") / name / "removable"
            if value.is_file() and value.read_text(encoding="utf-8").strip() == "1":
                return True
        except OSError:
            pass
        try:
            real = (Path("/sys/class/block") / name).resolve()
            if "/usb" in str(real).lower():
                return True
        except OSError:
            pass
    return False


class RecordStorageManager:
    """Select and validate the removable SWING RECORD data root.

    The rover never silently falls back to the system microSD when removable
    storage is required.  Existing recordings may still be read from the legacy
    root, but new human RECORD sessions are admitted only after this preflight.
    """

    def __init__(
        self,
        configured_root: str | None = None,
        *,
        require_removable: bool = True,
        minimum_free_bytes: int = 2 * 1024 * 1024 * 1024,
        data_directory_name: str = "SWING_DATA",
    ):
        self.configured_root = str(configured_root or "").strip() or None
        self.require_removable = bool(require_removable)
        self.minimum_free_bytes = max(256 * 1024 * 1024, int(minimum_free_bytes))
        self.data_directory_name = str(data_directory_name or "SWING_DATA").strip() or "SWING_DATA"

    @classmethod
    def from_environment(cls):
        configured = os.environ.get("SWING_RECORD_STORAGE_ROOT")
        require = os.environ.get("SWING_RECORD_REQUIRE_REMOVABLE", "1").strip().lower()
        minimum_gib = float(os.environ.get("SWING_RECORD_MIN_FREE_GIB", "2"))
        return cls(
            configured,
            require_removable=require not in {"0", "false", "no"},
            minimum_free_bytes=int(max(0.25, minimum_gib) * 1024**3),
        )

    @staticmethod
    def _mounts():
        mounts = []
        try:
            with open("/proc/self/mounts", "r", encoding="utf-8") as file:
                for line in file:
                    parts = line.split()
                    if len(parts) < 4:
                        continue
                    device = _decode_mount_field(parts[0])
                    mountpoint = _decode_mount_field(parts[1])
                    mounts.append(
                        {
                            "device": device,
                            "mountpoint": mountpoint,
                            "fstype": parts[2],
                            "options": parts[3].split(","),
                        }
                    )
        except OSError:
            pass
        return mounts

    def _mount_for_path(self, path: Path):
        path = path.resolve()
        candidates = []
        for mount in self._mounts():
            try:
                mount_path = Path(mount["mountpoint"]).resolve()
                if os.path.commonpath([str(path), str(mount_path)]) == str(mount_path):
                    candidates.append((len(str(mount_path)), mount))
            except (OSError, ValueError):
                continue
        return max(candidates, default=(0, None), key=lambda item: item[0])[1]

    def _candidate_mounts(self):
        candidates = []
        for mount in self._mounts():
            device = mount["device"]
            mountpoint = Path(mount["mountpoint"])
            if not device.startswith("/dev/") or "ro" in mount["options"]:
                continue
            if str(mountpoint) in {"/", "/boot", "/boot/firmware"}:
                continue
            removable = _device_is_removable(device)
            conventional = str(mountpoint).startswith(("/media/", "/mnt/"))
            if removable or conventional:
                candidates.append((removable, str(mountpoint), mount))
        candidates.sort(key=lambda item: (not item[0], item[1]))
        return [item[2] for item in candidates]

    def _resolve_data_root(self, create=False):
        if self.configured_root:
            configured = Path(self.configured_root).expanduser()
            if configured.name == self.data_directory_name:
                data_root = configured
            elif configured.exists() and configured.is_dir():
                data_root = configured / self.data_directory_name
            else:
                data_root = configured
            mount = self._mount_for_path(data_root if data_root.exists() else data_root.parent)
            if create and mount is not None:
                data_root.mkdir(parents=True, exist_ok=True)
            return data_root, mount

        for mount in self._candidate_mounts():
            mountpoint = Path(mount["mountpoint"])
            data_root = mountpoint / self.data_directory_name
            if data_root.exists() or create:
                if create:
                    data_root.mkdir(parents=True, exist_ok=True)
                return data_root, mount
        return None, None

    def snapshot(self, *, create=False, write_probe=False):
        data_root, mount = self._resolve_data_root(create=create)
        result = {
            "ready": False,
            "data_root": None if data_root is None else str(data_root),
            "recordings_root": None,
            "device": None if mount is None else mount.get("device"),
            "mountpoint": None if mount is None else mount.get("mountpoint"),
            "filesystem": None if mount is None else mount.get("fstype"),
            "removable": False if mount is None else _device_is_removable(mount.get("device") or ""),
            "free_bytes": None,
            "total_bytes": None,
            "minimum_free_bytes": self.minimum_free_bytes,
            "require_removable": self.require_removable,
            "error": None,
        }
        if data_root is None or mount is None:
            result["error"] = "USB_RECORD_STORAGE_NOT_FOUND"
            return result
        if self.require_removable and not result["removable"]:
            result["error"] = "USB_RECORD_STORAGE_NOT_REMOVABLE"
            return result
        try:
            if create:
                data_root.mkdir(parents=True, exist_ok=True)
            if not data_root.is_dir():
                result["error"] = "USB_RECORD_STORAGE_DATA_ROOT_MISSING"
                return result
            if write_probe:
                _atomic_probe(data_root)
            usage = shutil.disk_usage(data_root)
            result["free_bytes"] = int(usage.free)
            result["total_bytes"] = int(usage.total)
            if usage.free < self.minimum_free_bytes:
                result["error"] = "USB_RECORD_STORAGE_LOW_SPACE"
                return result
            recordings = data_root / "recordings"
            if create:
                recordings.mkdir(parents=True, exist_ok=True)
            result["recordings_root"] = str(recordings)
            result["ready"] = True
            return result
        except OSError as error:
            result["error"] = f"USB_RECORD_STORAGE_IO_ERROR:{type(error).__name__}:{error}"
            return result

    def require_recordings_root(self):
        status = self.snapshot(create=True, write_probe=True)
        if not status.get("ready"):
            raise OSError(status.get("error") or "USB_RECORD_STORAGE_NOT_READY")
        return str(status["recordings_root"]), status


__all__ = ["RecordStorageManager"]
