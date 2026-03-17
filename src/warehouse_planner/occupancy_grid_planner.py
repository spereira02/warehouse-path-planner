from __future__ import annotations

import heapq
import math
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
from shapely import box
from shapely.geometry import LineString, Polygon
from shapely.geometry.base import BaseGeometry

from .obstacle import Obstacle
from .robot import Robot

Point2D = Tuple[float, float]
Bounds = Tuple[float, float, float, float]

def _as_geometry(obstacle: Obstacle | BaseGeometry) -> BaseGeometry:
    """Return a shapely geometry from either an obstacle wrapper or raw geometry."""
    if isinstance(obstacle, Obstacle):
        return obstacle.shape
    return obstacle


@dataclass(frozen=True)
class _GridGraph:
    """ Container for occupancy grid graph """

    coordinates: Dict[int, Point2D]             # key: node_id, value: (world_x, world_y)
    adjacency: Dict[int, List[int]]             # key: node_id, value: List[neighbor_node_ids]
    weights: Dict[Tuple[int, int], float]       # key: (from_node_id, to_node_id), value: distance between those nodes
    grid_to_id: Dict[Tuple[int, int], int]      # key: (row, col), val: node_id


class _AStarSolver:
    """Run A* on the occupancy-grid graph and returns the shortest path
    Input: Graph (built through OccupancyGridPlanner)
    """

    def __init__(self, planner: "OccupancyGridPlanner"):
        self.planner = planner

    def heuristic(self, u: int, v: int) -> float:
        "Input are 2 node ids (start,goal)"
        x1, y1 = self.planner.graph.coordinates[u]
        x2, y2 = self.planner.graph.coordinates[v]
        return math.hypot(x2 - x1, y2 - y1)

    def path(
        self,
        start_id: int,
        goal_id: int,
        start_pos: Point2D,
        goal_pos: Point2D,
    ) -> List[Point2D]:
        """Compute a shortest path, given start and goal id and greedily smooth it."""
        if start_id == goal_id:
            return [start_pos, goal_pos]

        counter = 0
        
        # cost for the priority queue (min_heap) : f(n) = g(n) + h(n)
        # total cost, g = cost so far, till node n, h = estimated cost to go (node->goal)
        # tie-break logic for heap: take lowest cost, if costs tie: take node which was first inserted(lower counter val)
        min_heap: List[Tuple[float, int, int]] = [(0.0, counter, start_id)]
        came_from: Dict[int, Optional[int]] = {start_id: None}  # val None is used in _reconstruct_path() to identify start node
        g_score: Dict[int, float] = {start_id: 0.0}
        path_indices: List[int] = []

        while min_heap:
            current_f, _, current = heapq.heappop(min_heap)
            if current == goal_id:
                path_indices = self._reconstruct_path(came_from, current)
                break

            if current_f > g_score[current] + self.heuristic(current, goal_id):
                continue

            for neighbor in self.planner.graph.adjacency.get(current, []):
                tentative_g = g_score[current] + self.planner.graph.weights[(current, neighbor)]
                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    counter += 1
                    f_score = tentative_g + self.heuristic(neighbor, goal_id)
                    heapq.heappush(min_heap, (f_score, counter, neighbor))

        if not path_indices:
            return []

        path_coords = [self.planner.graph.coordinates[node_id] for node_id in path_indices]
        full_path = [start_pos] + path_coords + [goal_pos]
        return self.smooth_path(full_path)

    def _reconstruct_path(
        self,
        came_from: Dict[int, Optional[int]],
        current: int,
    ) -> List[int]:
        path_indices: List[int] = []
        while current is not None:
            path_indices.append(current)
            current = came_from[current]
        path_indices.reverse()
        return path_indices

    def smooth_path(self, raw_path: Sequence[Point2D]) -> List[Point2D]:
        """start at the current waypoint, initially the start
        * try to connect it directly to the goal
        * if that collides, try the waypoint before the goal
        * keep moving backward until you find the farthest waypoint that can be reached in a straight collision-free line
        * add that waypoint to the smoothed path
        * move current_index to that waypoint
        * repeat until the current waypoint is the final one
        """
        if len(raw_path) <= 2:
            return list(raw_path)

        smoothed = [raw_path[0]]
        current_index = 0

        while current_index < len(raw_path) - 1:
            shortcut_found = False
            for check_index in range(len(raw_path) - 1, current_index, -1):
                if self.planner.check_collision_free(raw_path[current_index], raw_path[check_index]):
                    smoothed.append(raw_path[check_index])
                    current_index = check_index
                    shortcut_found = True
                    break

            if not shortcut_found:
                current_index += 1
                smoothed.append(raw_path[current_index])

        return smoothed


class OccupancyGridPlanner:
    """Plan collision-free 2D paths on an occupancy grid."""

    def __init__(
        self,
        obstacles: Iterable[Obstacle | BaseGeometry],
        bounds: Bounds,
        robot: Robot,
        resolution: float = 0.5,
        safety_margin: float | None =  None,
        temporary_obstacles: Optional[Iterable[Obstacle | BaseGeometry]] = None,
        temporary_safety_margin: Optional[float] = None,
    ):
        self.min_x, self.min_y, self.max_x, self.max_y = bounds
        self.resolution = resolution
        self.robot = robot
        self.safety_margin = 0.2 * robot.radius if safety_margin is None else safety_margin
        self.temporary_safety_margin = (
            self.safety_margin if temporary_safety_margin is None else temporary_safety_margin
        )

        self.obstacles = list(obstacles)
        self.temporary_obstacles = list(temporary_obstacles or [])

        self.robot_buffer = self.robot.radius + self.safety_margin
        self.temporary_robot_buffer = self.robot.radius + self.temporary_safety_margin + self.robot.radius
        self.inflated_obstacles: List[Polygon] = []

        self.width = int(np.ceil((self.max_x - self.min_x) / resolution))
        self.height = int(np.ceil((self.max_y - self.min_y) / resolution))
        self.grid = np.zeros((self.height, self.width), dtype=bool)

        self._discretize_grid_and_collision_checks()
        self.graph = self._build_graph()
        self._solver = _AStarSolver(self)

    def _discretize_grid_and_collision_checks(self) -> None:
        """Rasterize inflated obstacles into the occupancy grid. Function is also able to construct temporary obstacles for other robots in a multi-agent setting
        """
        # Inflate static obstacles by robot radius plus safety margin.
        for obstacle in self.obstacles:
            inflated = _as_geometry(obstacle).buffer(self.robot_buffer)
            self.inflated_obstacles.append(inflated)

        # incase of the multi-agent (robot) setting, add other robots than self to obstacles temporarily
        for obstacle in self.temporary_obstacles:
            inflated = _as_geometry(obstacle).buffer(self.temporary_robot_buffer)
            self.inflated_obstacles.append(inflated)

        for inflated_obstacle in self.inflated_obstacles:
            min_ox, min_oy, max_ox, max_oy = inflated_obstacle.bounds

            # */
            # map relevant region, in which obstacle could potentially be in to grid coordinates, by performing a shift
            # inflated shapes live in contninous world (for example spanning -x to x and -y to y)
            # -> we need to map the inflated shapes from cont world to our discrete grid (which is of geom mxn, m = 2*y/resolution, n = 2*x/resolution)
            # /*
            x_start = max(0, int(np.floor((min_ox - self.min_x) / self.resolution)))
            x_end = min(self.width - 1, int(np.floor((max_ox - self.min_x) / self.resolution)))
            y_start = max(0, int(np.floor((min_oy - self.min_y) / self.resolution)))
            y_end = min(self.height - 1, int(np.floor((max_oy - self.min_y) / self.resolution)))

            # scan the region of interest and check if current cell intersects obstacle
            for row in range(y_start, y_end + 1):
                cell_y_min = self.min_y + row * self.resolution
                cell_y_max = cell_y_min + self.resolution
                for col in range(x_start, x_end + 1):
                    if self.grid[row, col]:
                        continue
                    # */ to avoid collision we check if any part of the cell intesects the inflated obstacle, 
                    # if we only checked the cell center, we might miss an intersections and cause collision /*
                    cell_x_min = self.min_x + col * self.resolution
                    cell_x_max = cell_x_min + self.resolution
                    cell_box = box(cell_x_min, cell_y_min, cell_x_max, cell_y_max)

                    if inflated_obstacle.intersects(cell_box):
                        self.grid[row, col] = True

    def _build_graph(self) -> _GridGraph:
        """Build the connectivity graph for all free grid cells."""
        node_count = 0
        coordinates: Dict[int, Point2D] = {}
        adjacency: Dict[int, List[int]] = {}
        weights: Dict[Tuple[int, int], float] = {}
        grid_to_id: Dict[Tuple[int, int], int] = {}

        for row in range(self.height):
            for col in range(self.width):
                if self.grid[row, col]:         #cell is part of an obstacle -> skip
                    continue

                node_id = node_count
                node_count += 1
                grid_to_id[(row, col)] = node_id

                # revert back from grid indices to world coordinates
                world_x = self.min_x + (col + 0.5) * self.resolution
                world_y = self.min_y + (row + 0.5) * self.resolution
                coordinates[node_id] = (world_x, world_y)
                adjacency[node_id] = []

        # (move direction in x, move dir in y, distance (cost))
        moves = [
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (-1, -1, math.sqrt(2)),
        ]

        for (row, col), current_node_id in grid_to_id.items():
            for dr, dc, move_cost in moves:
                next_row = row + dr
                next_col = col + dc

                if (next_row, next_col) not in grid_to_id:
                    continue

                if dr != 0 and dc != 0:
                    if self.grid[row, next_col] or self.grid[next_row, col]:    # obstacle detected
                        continue

                neighbor_node_id = grid_to_id[(next_row, next_col)]
                adjacency[current_node_id].append(neighbor_node_id)
                weights[(current_node_id, neighbor_node_id)] = self.resolution * move_cost

        return _GridGraph(
            coordinates=coordinates,
            adjacency=adjacency,
            weights=weights,
            grid_to_id=grid_to_id,
        )

    def find_nearest_node(self, position: Point2D) -> Optional[int]:
        """Return the nearest reachable graph node for a world-space position."""
        #convert world_coordinates into grid indices
        col = int(np.clip(np.floor((position[0] - self.min_x) / self.resolution), 0, self.width - 1))
        row = int(np.clip(np.floor((position[1] - self.min_y) / self.resolution), 0, self.height - 1))
        if (row, col) in self.graph.grid_to_id:
            return self.graph.grid_to_id[(row, col)]

        queue = deque([(row, col)])
        visited = {(row, col)}
        max_search_radius = 1.2 / self.resolution

        while queue:
            current_row, current_col = queue.popleft()
            if abs(current_row - row) > max_search_radius or abs(current_col - col) > max_search_radius:
                continue

            for delta_row in [-1, 0, 1]:
                for delta_col in [-1, 0, 1]:
                    next_row = current_row + delta_row
                    next_col = current_col + delta_col
                    if (next_row, next_col) in visited:
                        continue
                    if (next_row, next_col) in self.graph.grid_to_id:
                        return self.graph.grid_to_id[(next_row, next_col)]
                    visited.add((next_row, next_col))
                    queue.append((next_row, next_col))

        return None

    def check_collision_free(self, point_a: Point2D, point_b: Point2D) -> bool:
        """Check whether the straight segment between two points is obstacle-free."""
        line = LineString([point_a, point_b])
        for obstacle in self.inflated_obstacles:
            if line.intersects(obstacle):
                return False
        return True
        

    def plan(self, start: Point2D, goal: Point2D) -> List[Point2D]:
        """Plan a path from start to goal in world coordinates."""
        start_id = self.find_nearest_node(start)
        goal_id = self.find_nearest_node(goal)

        if start_id is None or goal_id is None:
            return []

        return self._solver.path(start_id, goal_id, start, goal)


# Backward-compatible alias for the previous class name.
OccupancyGridGraph = OccupancyGridPlanner
