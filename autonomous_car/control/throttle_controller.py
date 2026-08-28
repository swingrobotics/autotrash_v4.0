import json
import os


class ThrottleCalibration:
    def __init__(self, path, default_points=None):
        self.path = path
        self.points = default_points or [(0.0, 0.0), (0.25, 0.25)]
        self.load()

    def load(self):
        try:
            with open(self.path, "r", encoding="utf-8") as file:
                document = json.load(file)
            self.set_points(document.get("points", []), save=False)
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            pass
        return self.snapshot()

    def set_points(self, points, save=True):
        parsed = []
        for point in points:
            speed = float(point["speed_mps"] if isinstance(point, dict) else point[0])
            throttle = float(point["throttle"] if isinstance(point, dict) else point[1])
            if speed < 0 or not 0 <= throttle <= 1:
                raise ValueError("Speed must be non-negative and throttle must be between 0 and 1")
            parsed.append((speed, throttle))
        parsed.sort()
        if len(parsed) < 2 or parsed[0] != (0.0, 0.0):
            raise ValueError("Calibration requires at least two points starting with speed 0 and throttle 0")
        if any(parsed[index][0] == parsed[index - 1][0] for index in range(1, len(parsed))):
            raise ValueError("Calibration speeds must be unique")
        self.points = parsed
        if save:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            temporary = f"{self.path}.tmp"
            with open(temporary, "w", encoding="utf-8") as file:
                json.dump(self.snapshot(), file, ensure_ascii=False, indent=2)
            os.replace(temporary, self.path)
        return self.snapshot()

    def throttle_for_speed(self, speed_mps):
        speed = max(0.0, float(speed_mps))
        if speed <= self.points[0][0]:
            return self.points[0][1]
        for (speed_a, throttle_a), (speed_b, throttle_b) in zip(self.points, self.points[1:]):
            if speed <= speed_b:
                ratio = (speed - speed_a) / (speed_b - speed_a)
                return throttle_a + ratio * (throttle_b - throttle_a)
        return self.points[-1][1]

    def snapshot(self):
        return {
            "points": [
                {"speed_mps": speed, "throttle": throttle}
                for speed, throttle in self.points
            ],
            "path": self.path,
        }
