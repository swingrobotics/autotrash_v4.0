from dataclasses import dataclass
import math


SECTOR_DEFINITIONS = (
    ("far_right", -75.0),
    ("right", -50.0),
    ("front_right", -25.0),
    ("front", 0.0),
    ("front_left", 25.0),
    ("left", 50.0),
    ("far_left", 75.0),
)


@dataclass(frozen=True)
class LidarSectorFeatures:
    distances_m: dict[str, float]
    observed: dict[str, bool]

    def as_dict(self):
        return {
            "distances_m": dict(self.distances_m),
            "observed": dict(self.observed),
        }

    def as_vector(self, maximum_distance_m=8.0, include_observed_mask=True):
        maximum = max(0.01, float(maximum_distance_m))
        distance_vector = [
            max(0.0, min(maximum, float(self.distances_m[name]))) / maximum
            for name, _ in SECTOR_DEFINITIONS
        ]
        if not include_observed_mask:
            return distance_vector
        return distance_vector + [
            1.0 if self.observed[name] else 0.0
            for name, _ in SECTOR_DEFINITIONS
        ]


class LidarSectorizer:
    """Convert a LiDAR scan to a stable compact input for learned driving.

    Bearings follow the existing rover convention used by the LD06 monitor:
    positive bearings are vehicle-left and negative bearings are vehicle-right.
    Each sector stores the nearest observed point. Missing sectors are filled
    with `maximum_distance_m` and accompanied by an explicit observed mask so
    the model can distinguish "clear/far" from "not observed".
    """

    def __init__(
        self,
        maximum_distance_m=8.0,
        sector_half_width_degrees=12.5,
        minimum_confidence=0,
        lidar_to_front_bumper_m=0.254,
    ):
        self.maximum_distance_m = max(0.1, float(maximum_distance_m))
        self.sector_half_width_degrees = max(
            1.0,
            min(24.9, abs(float(sector_half_width_degrees))),
        )
        self.minimum_confidence = max(0.0, float(minimum_confidence))
        self.lidar_to_front_bumper_m = max(
            0.0,
            float(lidar_to_front_bumper_m),
        )

    def transform(self, points):
        distances = {
            name: self.maximum_distance_m
            for name, _ in SECTOR_DEFINITIONS
        }
        observed = {name: False for name, _ in SECTOR_DEFINITIONS}

        for point in points or ():
            try:
                distance_m = float(point["distance_mm"]) / 1000.0
                bearing = self._normalize_bearing(float(point["bearing_degrees"]))
                confidence = float(point.get("confidence", self.minimum_confidence))
            except (KeyError, TypeError, ValueError):
                continue
            if not math.isfinite(distance_m) or distance_m <= 0.0:
                continue
            if not math.isfinite(bearing) or confidence < self.minimum_confidence:
                continue

            bearing_radians = math.radians(bearing)
            forward_m = distance_m * math.cos(bearing_radians)
            if forward_m <= self.lidar_to_front_bumper_m:
                continue

            # Use radial distance for the learned feature but remove points
            # that are physically behind/inside the vehicle footprint above.
            clipped = min(self.maximum_distance_m, distance_m)
            for name, center in SECTOR_DEFINITIONS:
                if self._angular_distance(bearing, center) <= self.sector_half_width_degrees:
                    distances[name] = min(distances[name], clipped)
                    observed[name] = True
                    break

        return LidarSectorFeatures(distances, observed)

    @staticmethod
    def _normalize_bearing(value):
        return ((float(value) + 180.0) % 360.0) - 180.0

    @staticmethod
    def _angular_distance(left, right):
        return abs(((left - right + 180.0) % 360.0) - 180.0)
