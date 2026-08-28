import os
import resource
import time


def _read_text(path):
    try:
        with open(path, "r", encoding="utf-8") as file:
            return file.read().strip()
    except OSError:
        return None


def _temperature_celsius():
    raw = _read_text("/sys/class/thermal/thermal_zone0/temp")
    if raw is None:
        return None
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value / 1000.0 if value > 1000.0 else value


def _memory_snapshot():
    document = {}
    raw = _read_text("/proc/meminfo")
    if raw:
        for line in raw.splitlines():
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            parts = value.strip().split()
            if not parts:
                continue
            try:
                amount = float(parts[0])
            except ValueError:
                continue
            if len(parts) > 1 and parts[1].lower() == "kb":
                amount *= 1024.0
            document[key] = amount
    total = document.get("MemTotal")
    available = document.get("MemAvailable")
    return {
        "total_bytes": total,
        "available_bytes": available,
        "used_bytes": None if total is None or available is None else total - available,
    }


def _controller_timing(snapshot):
    if not isinstance(snapshot, dict):
        return None
    inference = snapshot.get("last_inference") or {}
    timing = inference.get("timing") if isinstance(inference, dict) else None
    return timing if isinstance(timing, dict) else None


def collect_runtime_metrics(legacy, full, gps_ai=None):
    usage = resource.getrusage(resource.RUSAGE_SELF)
    try:
        load1, load5, load15 = os.getloadavg()
    except (AttributeError, OSError):
        load1 = load5 = load15 = None

    ai_snapshot = full.ai.AUTO_AI_CONTROLLER.snapshot()
    local_snapshot = full.AUTO_LOCAL_CONTROLLER.snapshot()
    gps_snapshot = gps_ai.controller.snapshot() if gps_ai is not None else None
    mapping_snapshot = full.MAPPING_CONTROLLER.snapshot()
    recording = legacy.record_manager.snapshot()

    return {
        "timestamp": time.time(),
        "process": {
            "pid": os.getpid(),
            "cpu_user_seconds": usage.ru_utime,
            "cpu_system_seconds": usage.ru_stime,
            "maximum_rss_kib": usage.ru_maxrss,
        },
        "system": {
            "load_average_1m": load1,
            "load_average_5m": load5,
            "load_average_15m": load15,
            "temperature_celsius": _temperature_celsius(),
            "memory": _memory_snapshot(),
            "cpu_count": os.cpu_count(),
        },
        "recording": recording,
        "auto_ai": {
            "active": ai_snapshot.get("active"),
            "timing": _controller_timing(ai_snapshot),
        },
        "auto_gps": {
            "active": None if gps_snapshot is None else gps_snapshot.get("active"),
            "timing": _controller_timing(gps_snapshot),
        },
        "auto_local": {
            "active": local_snapshot.get("active"),
            "timing": (local_snapshot.get("last_command") or {}).get("timing"),
        },
        "mapping": {
            "active": mapping_snapshot.get("active"),
            "timing": mapping_snapshot.get("timing"),
        },
    }


__all__ = ["collect_runtime_metrics"]
