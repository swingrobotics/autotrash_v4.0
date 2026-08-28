import json
import math
import tempfile

from autonomous_car.localization import (
    GridPathPlanner,
    LidarImuSlam,
    Pose2D,
    SparseOccupancyGrid,
)
from autonomous_car.modes import AutoLocalPlanner


def room_grid():
    grid = SparseOccupancyGrid(resolution_m=0.10)
    # 6m x 4m known-free rectangular room with occupied boundary.
    for ix in range(0, 61):
        for iy in range(0, 41):
            boundary = ix in {0, 60} or iy in {0, 40}
            grid.cells[(ix, iy)] = 3.0 if boundary else -2.0
    grid.scan_count = 100
    return grid


def room_scan(pose, step_degrees=10):
    points = []
    min_x, max_x = 0.0, 6.0
    min_y, max_y = 0.0, 4.0
    for bearing_deg in range(-180, 180, step_degrees):
        angle = pose.yaw_radians + math.radians(bearing_deg)
        dx = math.cos(angle)
        dy = math.sin(angle)
        distances = []
        if abs(dx) > 1e-9:
            for wall_x in (min_x, max_x):
                t = (wall_x - pose.x) / dx
                y = pose.y + t * dy
                if t > 0 and min_y - 1e-6 <= y <= max_y + 1e-6:
                    distances.append(t)
        if abs(dy) > 1e-9:
            for wall_y in (min_y, max_y):
                t = (wall_y - pose.y) / dy
                x = pose.x + t * dx
                if t > 0 and min_x - 1e-6 <= x <= max_x + 1e-6:
                    distances.append(t)
        if not distances:
            continue
        distance = min(distances)
        points.append(
            {
                "bearing_degrees": float(bearing_deg),
                "distance_mm": distance * 1000.0,
                "confidence": 120,
            }
        )
    return points


def check_grid_persistence():
    grid = room_grid()
    with tempfile.TemporaryDirectory() as directory:
        path = f"{directory}/room.json.gz"
        before = grid.save(path)
        restored = SparseOccupancyGrid.load(path)
        after = restored.quality_snapshot()
    passed = (
        before["known_cells"] == after["known_cells"]
        and restored.is_occupied_cell(0, 0)
        and restored.is_free_cell(20, 20)
        and abs(restored.resolution_m - 0.10) < 1e-9
    )
    return passed, {"before": before, "after": after}


def check_global_and_local_localization():
    grid = room_grid()
    true_pose = Pose2D(2.0, 1.5, math.radians(20.0))
    slam = LidarImuSlam(grid)
    global_result = slam.global_localize(room_scan(true_pose), 20.0)

    moved_pose = Pose2D(2.12, 1.55, math.radians(22.0))
    local_result = slam.update_localization(room_scan(moved_pose), 22.0)

    global_error = math.hypot(global_result.pose.x - true_pose.x, global_result.pose.y - true_pose.y)
    local_error = math.hypot(local_result.pose.x - moved_pose.x, local_result.pose.y - moved_pose.y)
    passed = (
        global_result.localized
        and local_result.localized
        and global_error <= 0.25
        and local_error <= 0.20
        and abs(
            math.degrees(
                (local_result.pose.yaw_radians - moved_pose.yaw_radians + math.pi)
                % (2 * math.pi)
                - math.pi
            )
        ) <= 6.0
    )
    return passed, {
        "global": global_result.as_dict(),
        "local": local_result.as_dict(),
        "global_position_error_m": global_error,
        "local_position_error_m": local_error,
    }


def check_astar_path():
    grid = room_grid()
    planner = GridPathPlanner(grid)
    path = planner.plan(1.0, 1.0, 5.0, 3.0)
    passed = (
        len(path.points) >= 2
        and path.distance_m > 4.0
        and path.distance_m < 5.5
        and path.expanded_nodes > 0
    )
    return passed, path.as_dict()


def check_auto_local_commands():
    grid = room_grid()
    destination = {
        "destination_id": "goal",
        "name": "Goal",
        "x": 5.0,
        "y": 2.0,
        "heading_degrees": 0.0,
    }
    planner = AutoLocalPlanner(grid, destination)
    pose = Pose2D(1.0, 2.0, 0.0)
    planner.plan_from_pose(pose)
    clear = planner.update(pose, [])
    avoid = planner.update(
        pose,
        [
            {"bearing_degrees": 0.0, "distance_mm": 1200, "confidence": 120},
            {"bearing_degrees": 35.0, "distance_mm": 2600, "confidence": 120},
            {"bearing_degrees": -35.0, "distance_mm": 850, "confidence": 120},
        ],
    )
    stop = AutoLocalPlanner(grid, destination).update(
        pose,
        [
            {"bearing_degrees": 0.0, "distance_mm": 650, "confidence": 120},
            {"bearing_degrees": 35.0, "distance_mm": 2500, "confidence": 120},
        ],
    )
    passed = (
        clear.fault is None
        and clear.throttle > 0
        and avoid.avoidance_active
        and avoid.avoidance_side == "left"
        and 0 < avoid.throttle < clear.throttle
        and stop.avoidance_active
        and stop.throttle == 0.0
    )
    return passed, {
        "clear": clear.as_dict(),
        "avoid": avoid.as_dict(),
        "stop": stop.as_dict(),
    }


def main():
    checks = {
        "grid_persistence": check_grid_persistence(),
        "global_and_local_localization": check_global_and_local_localization(),
        "astar_path": check_astar_path(),
        "auto_local_commands": check_auto_local_commands(),
    }
    result = {
        name: {"passed": passed, "details": details}
        for name, (passed, details) in checks.items()
    }
    result["passed"] = all(item["passed"] for item in result.values())
    print(json.dumps(result, indent=2, default=str))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
