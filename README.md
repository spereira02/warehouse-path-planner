## Warehouse Path Planner

Small standalone Python package for 2D warehouse robot path planning on an occupancy grid.

This repository extracts the path planning component from a larger university project and keeps the core grid construction, collision inflation, and A* routing logic intact while packaging it as a reusable library.

### Project Layout

```text
warehouse-path-planner/
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   └── warehouse_planner/
│       ├── __init__.py
│       ├── occupancy_grid_planner.py
│       ├── robot.py
│       └── obstacle.py
├── demo/
│   └── demo_planner.py
└── media/
```

### Installation

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Quick Start

```python
from shapely.geometry import box

from warehouse_planner import Obstacle, OccupancyGridPlanner, Robot

robot = Robot(radius=0.25)
obstacles = [
    Obstacle(box(2.0, 2.0, 3.5, 6.0)),
    Obstacle(box(5.0, 0.5, 6.0, 4.0)),
]

planner = OccupancyGridPlanner(
    obstacles=obstacles,
    bounds=(0.0, 0.0, 10.0, 8.0),
    robot=robot,
    resolution=0.25,
    safety_margin=0.1,
)

path = planner.plan(start=(0.75, 0.75), goal=(8.5, 6.5))
print(path)
```

### Demo

Run the standalone demo:

```bash
PYTHONPATH=src python demo/demo_planner.py
```

If `matplotlib` is available, the demo also renders a simple map and path visualization.

### Notes

- Obstacles can be passed either as `Obstacle(shape=...)` instances or directly as shapely geometries.
- Temporary obstacles are supported via the `temporary_obstacles` argument and are inflated more conservatively to mimic other robots.

### Original System Demo

The following video comes from the original multi-robot warehouse project where this planner was used:

https://github.com/user-attachments/assets/7e8ee39e-a14b-46ac-9ff4-47c9e18d4fcd
