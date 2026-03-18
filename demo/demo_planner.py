from __future__ import annotations
import math
import sys
from pathlib import Path

from shapely.geometry import Polygon, box

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from warehouse_planner import Obstacle, OccupancyGridPlanner, Robot


def path_length(path):
    total_length = 0.0
    for i in range(len(path) - 1):
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        total_length += math.hypot(x2 - x1, y2 - y1)
    return total_length


def main():
    """Build a small map, run the planner, and print the path."""
    robot = Robot(radius=0.35)
    obstacles = [
    Obstacle(box(-6.5, 1.0, -5.3, 7.5)),
    Obstacle(box(-2.5, 1.0, -1.3, 7.5)),
    Obstacle(box(1.5, 1.0, 2.7, 7.5)),
    Obstacle(box(5.5, 1.0, 6.7, 7.5)),

    Obstacle(box(-6.5, -7.5, -5.3, -1.0)),
    Obstacle(box(-2.5, -7.5, -1.3, -1.0)),
    Obstacle(box(1.5, -7.5, 2.7, -1.0)),
    Obstacle(box(5.5, -7.5, 6.7, -1.0)),

    Obstacle(Polygon([
        (-0.8, -0.2),
        (0.7, 0.1),
        (1.0, 1.3),
        (-0.2, 1.8),
        (-1.1, 0.9),
    ])),
]

    planner = OccupancyGridPlanner(
        obstacles=obstacles,
        bounds=(-10.0, -10.0, 10.0, 10.0),
        robot=robot,
        resolution=0.1,
    )

    start = (-9.0, 8.5)
    goal = (1.8, 0.0)
    path = planner.plan(start=start, goal=goal)
    result = {
        "path": path,
        "runtime_ms": planner.last_runtime_ms,
        "nodes_explored": planner.last_nodes_explored,
        "path_length": path_length(path),
    }

    print("Planned path:")
    if not result["path"]:
        print("  No path found.")
        return

    for waypoint in result["path"]:
        print(f"  {waypoint}")
    print("Total length of path:", result["path_length"])
    print("Runtime (ms):", result["runtime_ms"])
    print("Nodes explored:", result["nodes_explored"])


    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return

    figure, axis = plt.subplots(figsize=(8, 6))
    for index, obstacle in enumerate(obstacles):
        x_coords, y_coords = obstacle.shape.exterior.xy
        label = "Obstacle" if index == 0 else None
        axis.fill(x_coords, y_coords, color="tab:gray", alpha=0.8, label=label)

    for index, inflated_obstacle in enumerate(planner.inflated_obstacles):
        x_coords, y_coords = inflated_obstacle.exterior.xy
        label = "Inflated obstacle" if index == 0 else None
        axis.fill(
            x_coords,
            y_coords,
            facecolor="none",
            edgecolor="tab:red",
            hatch="///",
            linewidth=1.2,
            alpha=0.5,
            label=label,
        )

    path_x = [point[0] for point in result["path"]]
    path_y = [point[1] for point in result["path"]]
    axis.plot(path_x, path_y, marker="o", color="tab:blue", linewidth=2, label="Path")
    axis.scatter([start[0]], [start[1]], color="tab:green", s=80, label="Start")
    axis.scatter([goal[0]], [goal[1]], color="tab:red", s=80, label="Goal")
    axis.set_xlim(planner.min_x, planner.max_x)
    axis.set_ylim(planner.min_y, planner.max_y)
    axis.set_aspect("equal", adjustable="box")
    axis.set_title("Warehouse Path Planner Demo")
    axis.legend()
    axis.grid(True, linestyle="--", alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()
