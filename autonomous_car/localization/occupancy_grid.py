from __future__ import annotations

from dataclasses import dataclass
import gzip
import json
import math
import os
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class GridBounds:
    min_x: int
    min_y: int
    max_x: int
    max_y: int


class SparseOccupancyGrid:
    """Sparse log-odds 2D occupancy grid.

    The implementation intentionally uses only the Python standard library so
    AUTO_LOCAL mapping/localization can run in the existing Raspberry Pi
    service environment without ROS or a separate mapping daemon.
    """

    FORMAT = "sparse_logodds_grid_v1"

    def __init__(
        self,
        resolution_m=0.08,
        *,
        hit_log_odds=0.85,
        miss_log_odds=-0.35,
        minimum_log_odds=-3.5,
        maximum_log_odds=3.5,
        occupied_threshold=0.9,
        free_threshold=-0.35,
    ):
        self.resolution_m = max(0.02, float(resolution_m))
        self.hit_log_odds = float(hit_log_odds)
        self.miss_log_odds = float(miss_log_odds)
        self.minimum_log_odds = float(minimum_log_odds)
        self.maximum_log_odds = float(maximum_log_odds)
        self.occupied_threshold = float(occupied_threshold)
        self.free_threshold = float(free_threshold)
        self.cells: dict[tuple[int, int], float] = {}
        self.scan_count = 0

    def world_to_cell(self, x, y):
        return (
            int(round(float(x) / self.resolution_m)),
            int(round(float(y) / self.resolution_m)),
        )

    def cell_to_world(self, ix, iy):
        return float(ix) * self.resolution_m, float(iy) * self.resolution_m

    def log_odds(self, ix, iy):
        return self.cells.get((int(ix), int(iy)), 0.0)

    def is_known(self, ix, iy):
        return (int(ix), int(iy)) in self.cells

    def is_occupied_cell(self, ix, iy):
        return self.log_odds(ix, iy) >= self.occupied_threshold

    def is_free_cell(self, ix, iy):
        return self.is_known(ix, iy) and self.log_odds(ix, iy) <= self.free_threshold

    def occupied_probability(self, ix, iy):
        value = self.log_odds(ix, iy)
        try:
            return 1.0 - 1.0 / (1.0 + math.exp(value))
        except OverflowError:
            return 1.0 if value > 0 else 0.0

    def update_scan(
        self,
        pose,
        points,
        *,
        maximum_range_m=8.0,
        minimum_range_m=0.12,
        minimum_confidence=35,
        ray_stride=1,
    ):
        robot_cell = self.world_to_cell(pose.x, pose.y)
        used = 0
        for index, point in enumerate(points or ()):
            if ray_stride > 1 and index % ray_stride:
                continue
            parsed = self._parse_point(point)
            if parsed is None:
                continue
            bearing_degrees, distance_m, confidence = parsed
            if confidence is not None and confidence < minimum_confidence:
                continue
            if distance_m < minimum_range_m or distance_m > maximum_range_m:
                continue

            bearing = pose.yaw_radians + math.radians(bearing_degrees)
            endpoint_x = pose.x + math.cos(bearing) * distance_m
            endpoint_y = pose.y + math.sin(bearing) * distance_m
            endpoint_cell = self.world_to_cell(endpoint_x, endpoint_y)
            ray = self._bresenham(robot_cell[0], robot_cell[1], endpoint_cell[0], endpoint_cell[1])
            if not ray:
                continue
            for cell in ray[:-1]:
                self._add(cell, self.miss_log_odds)
            self._add(ray[-1], self.hit_log_odds)
            used += 1

        if used:
            self.scan_count += 1
        return used

    def score_scan(
        self,
        pose,
        points,
        *,
        maximum_range_m=8.0,
        minimum_range_m=0.12,
        minimum_confidence=35,
        point_stride=4,
        neighborhood_cells=1,
    ):
        total = 0
        score = 0.0
        for index, point in enumerate(points or ()):
            if point_stride > 1 and index % point_stride:
                continue
            parsed = self._parse_point(point)
            if parsed is None:
                continue
            bearing_degrees, distance_m, confidence = parsed
            if confidence is not None and confidence < minimum_confidence:
                continue
            if distance_m < minimum_range_m or distance_m > maximum_range_m:
                continue
            bearing = pose.yaw_radians + math.radians(bearing_degrees)
            x = pose.x + math.cos(bearing) * distance_m
            y = pose.y + math.sin(bearing) * distance_m
            ix, iy = self.world_to_cell(x, y)
            best = 0.0
            for dx in range(-neighborhood_cells, neighborhood_cells + 1):
                for dy in range(-neighborhood_cells, neighborhood_cells + 1):
                    if not self.is_known(ix + dx, iy + dy):
                        continue
                    best = max(best, self.occupied_probability(ix + dx, iy + dy))
            score += best
            total += 1
        return 0.0 if total == 0 else score / total

    def inflated_occupied(self, radius_m):
        radius_cells = max(0, int(math.ceil(float(radius_m) / self.resolution_m)))
        result = set()
        occupied = [cell for cell, value in self.cells.items() if value >= self.occupied_threshold]
        if radius_cells <= 0:
            return set(occupied)
        radius_sq = radius_cells * radius_cells
        for ix, iy in occupied:
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy <= radius_sq:
                        result.add((ix + dx, iy + dy))
        return result

    def bounds(self):
        if not self.cells:
            return None
        xs = [cell[0] for cell in self.cells]
        ys = [cell[1] for cell in self.cells]
        return GridBounds(min(xs), min(ys), max(xs), max(ys))

    def quality_snapshot(self):
        occupied = sum(1 for value in self.cells.values() if value >= self.occupied_threshold)
        free = sum(1 for value in self.cells.values() if value <= self.free_threshold)
        bounds = self.bounds()
        return {
            "resolution_m": self.resolution_m,
            "scan_count": self.scan_count,
            "known_cells": len(self.cells),
            "occupied_cells": occupied,
            "free_cells": free,
            "bounds": None if bounds is None else bounds.__dict__,
        }

    @staticmethod
    def _fsync_parent(path):
        parent = os.path.dirname(os.path.abspath(str(path))) or "."
        try:
            descriptor = os.open(parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        except OSError:
            return
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        document = {
            "format": self.FORMAT,
            "resolution_m": self.resolution_m,
            "hit_log_odds": self.hit_log_odds,
            "miss_log_odds": self.miss_log_odds,
            "minimum_log_odds": self.minimum_log_odds,
            "maximum_log_odds": self.maximum_log_odds,
            "occupied_threshold": self.occupied_threshold,
            "free_threshold": self.free_threshold,
            "scan_count": self.scan_count,
            "cells": [[ix, iy, round(value, 5)] for (ix, iy), value in self.cells.items()],
        }
        temporary = Path(str(path) + ".tmp")
        opener = gzip.open if str(path).endswith(".gz") else open
        with opener(temporary, "wt", encoding="utf-8") as file:
            json.dump(document, file, separators=(",", ":"))
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, path)
        self._fsync_parent(path)
        return self.quality_snapshot()

    @classmethod
    def load(cls, path):
        path = Path(path)
        opener = gzip.open if str(path).endswith(".gz") else open
        with opener(path, "rt", encoding="utf-8") as file:
            document = json.load(file)
        if document.get("format") != cls.FORMAT:
            raise ValueError(f"Unsupported occupancy map format: {document.get('format')}")
        grid = cls(
            document.get("resolution_m", 0.08),
            hit_log_odds=document.get("hit_log_odds", 0.85),
            miss_log_odds=document.get("miss_log_odds", -0.35),
            minimum_log_odds=document.get("minimum_log_odds", -3.5),
            maximum_log_odds=document.get("maximum_log_odds", 3.5),
            occupied_threshold=document.get("occupied_threshold", 0.9),
            free_threshold=document.get("free_threshold", -0.35),
        )
        for row in document.get("cells") or []:
            if not isinstance(row, list) or len(row) != 3:
                continue
            grid.cells[(int(row[0]), int(row[1]))] = float(row[2])
        grid.scan_count = int(document.get("scan_count") or 0)
        return grid

    def _add(self, cell, delta):
        value = self.cells.get(cell, 0.0) + delta
        self.cells[cell] = max(self.minimum_log_odds, min(self.maximum_log_odds, value))

    @staticmethod
    def _parse_point(point):
        try:
            bearing = float(point["bearing_degrees"])
            distance_m = float(point["distance_mm"]) / 1000.0
        except (KeyError, TypeError, ValueError):
            return None
        confidence = point.get("confidence")
        try:
            confidence = None if confidence is None else float(confidence)
        except (TypeError, ValueError):
            confidence = None
        if not math.isfinite(bearing) or not math.isfinite(distance_m):
            return None
        return bearing, distance_m, confidence

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            doubled = 2 * error
            if doubled >= dy:
                error += dy
                x0 += sx
            if doubled <= dx:
                error += dx
                y0 += sy
        return points
