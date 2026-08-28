"""Validation helpers for user-facing V2 HTTP requests."""

DRIVE_MODE_REQUESTS = frozenset(
    {
        "MANUAL",
        "RECORD",
        "AUTO_AI",
        "AUTO_GPS",
        "AUTO_LOCAL",
        "AUTO",
        "DISARMED",
    }
)


def normalize_drive_mode_request(payload):
    """Return a canonical mode request or reject malformed UI/API input.

    Emergency stop intentionally has a dedicated endpoint and is not accepted
    here. This keeps an empty/malformed request from reaching ``DriveMode`` and
    leaking Enum implementation errors to the operator.
    """
    if not isinstance(payload, dict):
        raise ValueError("Drive mode request must be a JSON object")
    mode = str(payload.get("mode") or "").strip().upper()
    if not mode:
        raise ValueError("Drive mode is required")
    if mode not in DRIVE_MODE_REQUESTS:
        raise ValueError(f"Unsupported drive mode: {mode}")
    return mode, bool(payload.get("record_gps", True))


__all__ = ["DRIVE_MODE_REQUESTS", "normalize_drive_mode_request"]
