from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .occupancy_grid import SparseOccupancyGrid


@dataclass(frozen=True)
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    yaw_radians: float = 0.0

    @property
    def heading_degrees(self):
        return math.degrees(self.yaw_radians) % 360.0

    def as_dict(self):
        return {
            "x": self.x,
            "y": self.y,
            "heading_degrees": self.heading_degrees,
        }


@dataclass(frozen=True)
class LocalizationResult:
    pose: Pose2D
    score: float
    localized: bool
    global_search: bool
    elapsed_seconds: float
    points_used: int

    def as_dict(self):
        return {
            "pose": self.pose.as_dict(),
            "score": self.score,
            "localized": self.localized,
            "global_search": self.global_search,
            "elapsed_seconds": self.elapsed_seconds,
            "points_used": self.points_used,
        }


class CorrelativeScanMatcher:
    """Small deterministic correlative scan matcher for the LD06 map.

    Local matching searches a tight pose window around the previous pose while
    global matching searches the explored map with a coarse-to-fine strategy.
    IMU yaw is used as a prediction/constraint, but the LiDAR map score remains
    the final localization evidence.
    """

    def __init__(
        self,
        grid: SparseOccupancyGrid,
        *,
        local_xy_window_m=0.32,
        local_yaw_window_degrees=10.0,
        minimum_local_score=0.22,
        minimum_global_score=0.18,
    ):
        self.grid = grid
        self.local_xy_window_m = abs(float(local_xy_window_m))
        self.local_yaw_window_radians = math.radians(abs(float(local_yaw_window_degrees)))
        self.minimum_local_score = float(minimum_local_score)
        self.minimum_global_score = float(minimum_global_score)

    def local_match(self, points, predicted_pose):
        started = time.monotonic()
        best_pose, best_score, used = self._search(
            points,
            predicted_pose,
            xy_window=self.local_xy_window_m,
            xy_step=max(self.grid.resolution_m, 0.08),
            yaw_window=self.local_yaw_window_radians,
            yaw_step=math.radians(4.0),
            point_stride=5,
        )
        refined_pose, refined_score, used_refined = self._search(
            points,
            best_pose,
            xy_window=max(self.grid.resolution_m * 1.5, 0.10),
            xy_step=max(self.grid.resolution_m * 0.5, 0.04),
            yaw_window=math.radians(4.0),
            yaw_step=math.radians(1.0),
            point_stride=3,
        )
        if refined_score >= best_score:
            best_pose, best_score, used = refined_pose, refined_score, used_refined
        return LocalizationResult(
            best_pose,
            best_score,
            best_score >= self.minimum_local_score,
            False,
            time.monotonic() - started,
            used,
        )

    def global_match(self, points, imu_heading_degrees=None, maximum_candidates=12000):
        started = time.monotonic()
        bounds = self.grid.bounds()
        if bounds is None:
            return LocalizationResult(Pose2D(), 0.0, False, True, 0.0, 0)

        margin = 2
        step_cells = max(2, int(round(0.40 / self.grid.resolution_m)))
        yaw_values = self._global_yaws(imu_heading_degrees)
        best_pose = Pose2D()
        best_score = -1.0
        evaluated = 0
        used = 0

        width = max(1, bounds.max_x - bounds.min_x + 1)
        height = max(1, bounds.max_y - bounds.min_y + 1)
        estimated = max(1, (width // step_cells + 1) * (height // step_cells + 1) * len(yaw_values))
        if estimated > maximum_candidates:
            scale = math.sqrt(estimated / maximum_candidates)
            step_cells = max(step_cells, int(math.ceil(step_cells * scale)))

        for ix in range(bounds.min_x - margin, bounds.max_x + margin + 1, step_cells):
            for iy in range(bounds.min_y - margin, bounds.max_y + margin + 1, step_cells):
                if not self._near_known_free(ix, iy, radius=2):
                    continue
                x, y = self.grid.cell_to_world(ix, iy)
                for yaw in yaw_values:
                    pose = Pose2D(x, y, yaw)
                    score = self.grid.score_scan(
                        pose,
                        points,
                        point_stride=8,
                        neighborhood_cells=1,
                    )
                    evaluated += 1
                    if score > best_score:
                        best_pose, best_score = pose, score
                if evaluated >= maximum_candidates:
                    break
            if evaluated >= maximum_candidates:
                break

        if best_score < 0:
            return LocalizationResult(Pose2D(), 0.0, False, True, time.monotonic() - started, 0)

        refined_pose, refined_score, used = self._search(
            points,
            best_pose,
            xy_window=0.55,
            xy_step=max(self.grid.resolution_m, 0.08),
            yaw_window=math.radians(15.0),
            yaw_step=math.radians(3.0),
            point_stride=4,
        )
        final_pose, final_score, used2 = self._search(
            points,
            refined_pose,
            xy_window=0.16,
            xy_step=max(self.grid.resolution_m * 0.5, 0.04),
            yaw_window=math.radians(4.0),
            yaw_step=math.radians(1.0),
            point_stride=3,
        )
        if final_score < refined_score:
            final_pose, final_score, used2 = refined_pose, refined_score, used
        return LocalizationResult(
            final_pose,
            final_score,
            final_score >= self.minimum_global_score,
            True,
            time.monotonic() - started,
            used2,
        )

    def _search(self, points, center, *, xy_window, xy_step, yaw_window, yaw_step, point_stride):
        best_pose = center
        best_score = self.grid.score_scan(center, points, point_stride=point_stride)
        values_xy = self._offset_values(xy_window, xy_step)
        values_yaw = self._offset_values(yaw_window, yaw_step)
        used = 0
        for dx in values_xy:
            for dy in values_xy:
                for dyaw in values_yaw:
                    pose = Pose2D(
                        center.x + dx,
                        center.y + dy,
                        self._normalize_radians(center.yaw_radians + dyaw),
                    )
                    score = self.grid.score_scan(
                        pose,
                        points,
                        point_stride=point_stride,
                        neighborhood_cells=1,
                    )
                    used += 1
                    if score > best_score:
                        best_pose, best_score = pose, score
        return best_pose, best_score, used

    def _global_yaws(self, imu_heading_degrees):
        if imu_heading_degrees is None:
            return [math.radians(value) for value in range(0, 360, 20)]
        try:
            heading = math.radians(float(imu_heading_degrees))
        except (TypeError, ValueError):
            return [math.radians(value) for value in range(0, 360, 20)]
        return [
            self._normalize_radians(heading + math.radians(offset))
            for offset in range(-30, 31, 10)
        ]

    def _near_known_free(self, ix, iy, radius):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if self.grid.is_free_cell(ix + dx, iy + dy):
                    return True
        return False

    @staticmethod
    def _offset_values(window, step):
        window = abs(float(window))
        step = max(1e-6, abs(float(step)))
        count = int(math.floor(window / step))
        values = [index * step for index in range(-count, count + 1)]
        if 0.0 not in values:
            values.append(0.0)
        return values

    @staticmethod
    def _normalize_radians(value):
        return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


class LidarImuSlam:
    """Incremental mapping/localization state for AUTO_LOCAL.

    During mapping, the first scan defines local (0,0,0). Sequential LiDAR scan
    matching estimates translation while IMU heading delta predicts rotation.
    During reuse, global_localize() finds the initial pose on a saved map and
    update_localization() tracks it with the local matcher.
    """

    def __init__(self, grid=None):
        self.grid = grid or SparseOccupancyGrid()
        self.matcher = CorrelativeScanMatcher(self.grid)
        self.pose = Pose2D()
        self.localized = False
        self.last_imu_heading_degrees = None
        self.last_result = None
        self.mapping_scans = 0
        self.localization_failures = 0
        self.trajectory = []

    def reset_mapping(self):
        self.pose = Pose2D()
        self.localized = True
        self.last_imu_heading_degrees = None
        self.last_result = LocalizationResult(self.pose, 1.0, True, False, 0.0, 0)
        self.mapping_scans = 0
        self.localization_failures = 0
        self.trajectory = []

    def process_mapping_scan(self, points, imu_heading_degrees=None):
        if not self.localized:
            self.reset_mapping()

        if self.mapping_scans == 0 or len(self.grid.cells) < 50:
            result = LocalizationResult(self.pose, 1.0, True, False, 0.0, 0)
        else:
            predicted = self._predicted_pose(imu_heading_degrees)
            result = self.matcher.local_match(points, predicted)
            if result.localized:
                self.pose = result.pose
            else:
                # Mapping must never jump to a weak match. Keep the predicted
                # pose and skip map insertion for that frame.
                self.pose = predicted
                self.localization_failures += 1
                self.last_result = result
                self._remember_imu(imu_heading_degrees)
                return result

        self.grid.update_scan(self.pose, points, ray_stride=2)
        self.mapping_scans += 1
        self.last_result = result
        self._remember_imu(imu_heading_degrees)
        if not self.trajectory or self._distance(self.pose, self.trajectory[-1]) >= 0.20:
            self.trajectory.append(self.pose)
        return result

    def global_localize(self, points, imu_heading_degrees=None):
        result = self.matcher.global_match(points, imu_heading_degrees)
        self.last_result = result
        self.localized = result.localized
        if result.localized:
            self.pose = result.pose
            self.localization_failures = 0
        else:
            self.localization_failures += 1
        self.last_imu_heading_degrees = imu_heading_degrees
        return result

    def update_localization(self, points, imu_heading_degrees=None):
        if not self.localized:
            return self.global_localize(points, imu_heading_degrees)
        predicted = self._predicted_pose(imu_heading_degrees)
        result = self.matcher.local_match(points, predicted)
        self.last_result = result
        if result.localized:
            self.pose = result.pose
            self.localization_failures = 0
        else:
            self.localization_failures += 1
            if self.localization_failures >= 3:
                self.localized = False
        self._remember_imu(imu_heading_degrees)
        return result

    def snapshot(self):
        return {
            "localized": self.localized,
            "pose": self.pose.as_dict(),
            "last_result": None if self.last_result is None else self.last_result.as_dict(),
            "mapping_scans": self.mapping_scans,
            "localization_failures": self.localization_failures,
            "trajectory_points": len(self.trajectory),
            "map_quality": self.grid.quality_snapshot(),
        }

    def _predicted_pose(self, imu_heading_degrees):
        yaw = self.pose.yaw_radians
        if imu_heading_degrees is not None and self.last_imu_heading_degrees is not None:
            try:
                delta = self._angle_delta_degrees(
                    float(imu_heading_degrees),
                    float(self.last_imu_heading_degrees),
                )
                yaw = CorrelativeScanMatcher._normalize_radians(yaw + math.radians(delta))
            except (TypeError, ValueError):
                pass
        return Pose2D(self.pose.x, self.pose.y, yaw)

    def _remember_imu(self, value):
        if value is None:
            return
        try:
            value = float(value)
        except (TypeError, ValueError):
            return
        if math.isfinite(value):
            self.last_imu_heading_degrees = value

    @staticmethod
    def _angle_delta_degrees(current, previous):
        return (current - previous + 180.0) % 360.0 - 180.0

    @staticmethod
    def _distance(left, right):
        return math.hypot(left.x - right.x, left.y - right.y)
