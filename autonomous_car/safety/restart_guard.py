import time


class RestartDelayGuard:
    def __init__(self, delay_seconds=1.5):
        self.delay_seconds = max(0.0, float(delay_seconds))
        self.reason = None
        self.clear_since = None

    def block(self, reason):
        self.reason = str(reason or "OBSTACLE_STOP")
        self.clear_since = None

    def reset(self):
        self.reason = None
        self.clear_since = None

    def remaining(self, now=None, advance=True):
        if self.reason is None:
            return 0.0
        current_time = time.monotonic() if now is None else float(now)
        if self.clear_since is None:
            if not advance:
                return self.delay_seconds
            self.clear_since = current_time
        remaining = max(0.0, self.delay_seconds - (current_time - self.clear_since))
        if remaining == 0.0 and advance:
            self.reset()
        return remaining

    def snapshot(self, now=None):
        return {
            "reason": self.reason,
            "remaining_seconds": self.remaining(now, advance=False),
        }
