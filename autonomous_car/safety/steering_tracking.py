import time


class SteeringTrackingGuard:
    def __init__(self, maximum_error_degrees=7.0, timeout_seconds=1.0):
        self.maximum_error_degrees = abs(float(maximum_error_degrees))
        self.timeout_seconds = max(0.0, float(timeout_seconds))
        self.exceeded_since = None
        self.last_error_degrees = None

    def reset(self):
        self.exceeded_since = None
        self.last_error_degrees = None

    def evaluate(self, target_degrees, actual_degrees, active=True, now=None):
        current_time = time.monotonic() if now is None else float(now)
        if not active or target_degrees is None or actual_degrees is None:
            self.reset()
            return None

        self.last_error_degrees = abs(float(target_degrees) - float(actual_degrees))
        if self.last_error_degrees <= self.maximum_error_degrees:
            self.exceeded_since = None
            return None

        if self.exceeded_since is None:
            self.exceeded_since = current_time
            return None
        if current_time - self.exceeded_since >= self.timeout_seconds:
            return "STEERING_TRACKING_ERROR"
        return None

    def snapshot(self, now=None):
        current_time = time.monotonic() if now is None else float(now)
        return {
            "maximum_error_degrees": self.maximum_error_degrees,
            "timeout_seconds": self.timeout_seconds,
            "last_error_degrees": self.last_error_degrees,
            "exceeded_seconds": (
                0.0
                if self.exceeded_since is None
                else max(0.0, current_time - self.exceeded_since)
            ),
        }
