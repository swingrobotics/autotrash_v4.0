from dataclasses import dataclass
import csv
import json
import math
import os

from autonomous_car.control import PathPoint
from autonomous_car.localization import LocalENUConverter


@dataclass(frozen=True)
class ProcessedRoute:
    source_path: str
    origin: dict
    points: list[PathPoint]

    def as_dict(self):
        return {
            "source_path": self.source_path,
            "origin": self.origin,
            "points": [
                {"x": point.x, "y": point.y, "speed_mps": point.speed_mps}
                for point in self.points
            ],
        }


class RouteProcessor:
    def __init__(self, spacing_m=0.20, maximum_jump_m=2.0, smoothing_window=5):
        self.spacing_m = float(spacing_m)
        self.maximum_jump_m = float(maximum_jump_m)
        self.smoothing_window = max(1, int(smoothing_window))

    def process_csv(self, source_path, output_path=None):
        samples = self._read_fixed_samples(source_path)
        if len(samples) < 2:
            raise ValueError("At least two RTK FIXED route samples are required")
        converter = LocalENUConverter(samples[0][0], samples[0][1], samples[0][2])
        points = []
        for latitude, longitude, altitude, speed in samples:
            east, north, _ = converter.to_enu(latitude, longitude, altitude)
            point = PathPoint(east, north, speed)
            if points and math.hypot(point.x - points[-1].x, point.y - points[-1].y) > self.maximum_jump_m:
                continue
            points.append(point)
        points = self._smooth(points)
        points = self._resample(points)
        route = ProcessedRoute(source_path, converter.to_dict(), points)
        if output_path:
            with open(output_path, "w", encoding="utf-8") as file:
                json.dump(route.as_dict(), file, ensure_ascii=False, indent=2)
        return route

    @staticmethod
    def load_json(path):
        with open(path, "r", encoding="utf-8") as file:
            document = json.load(file)
        return ProcessedRoute(
            document.get("source_path", path),
            document["origin"],
            [PathPoint(point["x"], point["y"], point.get("speed_mps")) for point in document["points"]],
        )

    @staticmethod
    def _read_fixed_samples(path):
        samples = []
        with open(path, "r", encoding="utf-8") as file:
            for row in csv.DictReader(file):
                if row.get("rtk_status") != "RTK FIXED":
                    continue
                try:
                    samples.append(
                        (
                            float(row["latitude"]),
                            float(row["longitude"]),
                            float(row.get("altitude_m") or 0.0),
                            float(row.get("speed_mps") or 0.0),
                        )
                    )
                except (TypeError, ValueError):
                    continue
        return samples

    def _smooth(self, points):
        if len(points) < self.smoothing_window:
            return points
        radius = self.smoothing_window // 2
        smoothed = []
        for index, point in enumerate(points):
            section = points[max(0, index - radius): min(len(points), index + radius + 1)]
            smoothed.append(
                PathPoint(
                    sum(item.x for item in section) / len(section),
                    sum(item.y for item in section) / len(section),
                    point.speed_mps,
                )
            )
        return smoothed

    def _resample(self, points):
        if len(points) < 2:
            return points
        result = [points[0]]
        carried = 0.0
        for start, end in zip(points, points[1:]):
            segment = math.hypot(end.x - start.x, end.y - start.y)
            if segment == 0:
                continue
            distance = self.spacing_m - carried
            while distance <= segment:
                ratio = distance / segment
                result.append(
                    PathPoint(
                        start.x + (end.x - start.x) * ratio,
                        start.y + (end.y - start.y) * ratio,
                        end.speed_mps,
                    )
                )
                distance += self.spacing_m
            carried = max(0.0, segment - (distance - self.spacing_m))
        if math.hypot(result[-1].x - points[-1].x, result[-1].y - points[-1].y) > 0.05:
            result.append(points[-1])
        return result
