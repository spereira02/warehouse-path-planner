## Warehouse Path Planner

A small Python package for **2D path planning in warehouse-style environments** using an occupancy grid and A* search.

This repository extracts the path planning component from a larger university robotics project and packages it as a standalone module. The planner takes a continuous 2D map with polygonal obstacles, discretizes it into an occupancy grid, builds a graph over collision-free cells, and computes a path between a start and goal position.

---

## What the Planner Does

The planner is split into two main parts:

### 1. `OccupancyGridPlanner`

`OccupancyGridPlanner` is responsible for turning a continuous map into a search problem.

It takes:
- map bounds
- static obstacles
- optional temporary obstacles
- robot geometry
- grid resolution
- safety margins

From this, it:

- discretizes the continuous workspace into an **occupancy grid**
- inflates obstacles based on the robot radius and safety margin
- marks grid cells as free or occupied
- builds a graph over the free cells
- connects neighboring cells
- maps the continuous start and goal positions to valid graph nodes
- calls the A* planner to compute the shortest collision-free path

In other words, this class handles the **environment representation** and the conversion from continuous geometry to a graph search problem.

### 2. `AStar`

`AStar` solves the graph search problem created by `OccupancyGridPlanner`.

Given:
- a graph
- a start node
- a goal node

it searches for a lowest-cost path through the graph using the A* algorithm.

This part is responsible for:
- expanding nodes in a goal-directed way
- tracking the current best-known cost to each node
- using a heuristic to guide the search efficiently
- reconstructing the final shortest path once the goal is reached

So while `OccupancyGridPlanner` builds the navigable graph, `AStar` is the component that actually finds the route through it.

---

### Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Running the Demo

A simple demo is included to show the planner working on a small warehouse-style map.

```bash
PYTHONPATH=src python3 demo/demo_planner.py
```

The script visualizes the map, obstacles, and the computed path using **matplotlib**.

## Resolution Tradeoff

The planner represents the world as an **occupancy grid**, so the chosen grid resolution strongly affects both **path quality** and **runtime**.

- **Smaller resolution values (finer grid)**
  - more accurate obstacle representation
  - narrow passages may remain traversable
  - typically shorter or more direct paths
  - higher computational cost

- **Larger resolution values (coarser grid)**
  - faster planning
  - narrow gaps may disappear
  - paths can become longer or less direct

Using the same demo environment with two different resolutions produces different valid paths:

| Resolution | Path Length | Runtime |
|-----------|-------------|--------|
| `0.10` | `20.8797` | near-instant |
| `0.03` | `16.9215` | noticeably slower |

This is a standard tradeoff in grid-based planning: **better geometric fidelity usually costs more computation**.

### Coarser Grid (`resolution = 0.10`) vs. Finer Grid (`resolution = 0.03`)

<p align="center">
  <img src="media/res_0_1.png" width="450">
  <img src="media/res_0_03.png" width="450">
</p>

---

## Notes

- Obstacles can be passed either as `Obstacle(shape=...)` instances or directly as shapely geometries.
- Temporary obstacles are supported via the `temporary_obstacles` argument and can be inflated separately to mimic other robots or dynamic obstacles.

---

## Original System Demo

This planner was originally used inside a larger **multi-robot warehouse project**.

The video below shows the broader system context in which this planner was used:

https://github.com/user-attachments/assets/7e8ee39e-a14b-46ac-9ff4-47c9e18d4fcd
