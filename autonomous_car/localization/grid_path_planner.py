from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

from autonomous_car.control import PathPoint


@dataclass(frozen=True)
class PlannedGridPath:
    points: list[PathPoint]
    raw_cells: list[tuple[int, int]]
    distance_m: float
    expanded_nodes: int

    def as_dict(self):
        return {
            "point_count": len(self.points),
            "distance_m": self.distance_m,
            "expanded_nodes": self.expanded_nodes,
            "points": [{"x": point.x, "y": point.y} for point in self.points],
        }


class GridPathPlanner:
    """A* over known-free occupancy cells with vehicle-radius inflation."""

    NEIGHBORS = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    )

    def __init__(
        self,
        grid,
        *,
        vehicle_width_m=0.4826,
        safety_margin_m=0.18,
        allow_unknown=False,
        maximum_expansions=150000,
    ):
        self.grid = grid
        self.inflation_radius_m = vehicle_width_m * 0.5 + safety_margin_m
        self.allow_unknown = bool(allow_unknown)
        self.maximum_expansions = int(maximum_expansions)

    def plan(self, start_x, start_y, goal_x, goal_y):
        start = self._nearest_traversable(self.grid.world_to_cell(start_x, start_y))
        goal = self._nearest_traversable(self.grid.world_to_cell(goal_x, goal_y))
        if start is None:
            raise ValueError("AUTO_LOCAL start pose is not on known free map space")
        if goal is None:
            raise ValueError("AUTO_LOCAL destination is not on known free map space")

        blocked = self.grid.inflated_occupied(self.inflation_radius_m)
        if start in blocked:
            blocked.discard(start)
        if goal in blocked:
            raise ValueError("AUTO_LOCAL destination is inside inflated obstacle space")

        frontier = [(0.0, 0.0, start)]
        parents = {start: None}
        costs = {start: 0.0}
        expanded = 0

        while frontier:
            _, current_cost, current = heapq.heappop(frontier)
            if current_cost != costs.get(current):
                continue
            if current == goal:
                cells = self._reconstruct(parents, goal)
                smoothed = self._smooth_cells(cells, blocked)
                points = [PathPoint(*self.grid.cell_to_world(ix, iy)) for ix, iy in smoothed]
                distance = sum(
                    math.hypot(points[index].x - points[index - 1].x, points[index].y - points[index - 1].y)
                    for index in range(1, len(points))
                )
                return PlannedGridPath(points, cells, distance, expanded)

            expanded += 1
            if expanded > self.maximum_expansions:
                raise ValueError("AUTO_LOCAL path planning exceeded search limit")

            for dx, dy, travel_cost in self.NEIGHBORS:
                neighbor = (current[0] + dx, current[1] + dy)
                if neighbor in blocked or not self._traversable(neighbor):
                    continue
                if dx and dy:
                    # Do not cut diagonally across an obstacle corner.
                    side_a = (current[0] + dx, current[1])
                    side_b = (current[0], current[1] + dy)
                    if side_a in blocked or side_b in blocked:
                        continue
                tentative = current_cost + travel_cost
                if tentative >= costs.get(neighbor, float("inf")):
                    continue
                costs[neighbor] = tentative
                parents[neighbor] = current
                heuristic = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                heapq.heappush(frontier, (tentative + heuristic, tentative, neighbor))

        raise ValueError("AUTO_LOCAL destination is unreachable on this map")

    def _traversable(self, cell):
        if self.grid.is_occupied_cell(*cell):
            return False
        if self.allow_unknown:
            return True
        return self.grid.is_free_cell(*cell)

    def _nearest_traversable(self, cell, maximum_radius_cells=12):
        if self._traversable(cell):
            return cell
        for radius in range(1, maximum_radius_cells + 1):
            candidates = []
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = (cell[0] + dx, cell[1] + dy)
                    if self._traversable(candidate):
                        candidates.append(candidate)
            if candidates:
                return min(candidates, key=lambda value: math.hypot(value[0] - cell[0], value[1] - cell[1]))
        return None

    @staticmethod
    def _reconstruct(parents, goal):
        result = []
        node = goal
        while node is not None:
            result.append(node)
            node = parents[node]
        result.reverse()
        return result

    def _smooth_cells(self, cells, blocked):
        if len(cells) <= 2:
            return cells
        result = [cells[0]]
        anchor = 0
        while anchor < len(cells) - 1:
            farthest = anchor + 1
            for candidate in range(len(cells) - 1, anchor, -1):
                if self._line_clear(cells[anchor], cells[candidate], blocked):
                    farthest = candidate
                    break
            result.append(cells[farthest])
            anchor = farthest
        return result

    def _line_clear(self, start, end, blocked):
        for cell in self._bresenham(start[0], start[1], end[0], end[1]):
            if cell in blocked or not self._traversable(cell):
                return False
        return True

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
