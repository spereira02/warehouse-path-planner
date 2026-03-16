import heapq
import math
import numpy as np
from typing import List, Tuple, Dict, Optional
from collections import deque
from shapely import box
from shapely.geometry import Point as ShapelyPoint, LineString, Polygon
from dg_commons.sim.models.diff_drive_structures import DiffDriveGeometry
from dg_commons.sim.models.obstacles import StaticObstacle


class OccupancyGridGraph:
    def __init__(
        self,
        static_obstacles: List[StaticObstacle],
        bounds: Tuple[float, float, float, float],
        ddg: DiffDriveGeometry,
        resolution: float = 0.5,
    ):
        self.min_x, self.min_y, self.max_x, self.max_y = bounds
        self.resolution = resolution

        self.robot_radius = ddg.radius
        self.safety_margin = 0.1

        # real, physically accurate buffer for collision check
        self.robot_buffer = self.robot_radius + self.safety_margin
        self.inflated_obstacles: List[Polygon] = []

        # creating a grid, to later do the occupancy checks
        self.width = int(np.ceil((self.max_x - self.min_x) / resolution))
        self.height = int(np.ceil((self.max_y - self.min_y) / resolution))
        self.grid = np.zeros((self.height, self.width), dtype=bool)

        # first discretize contionous world into grid world then inflate obstacles and conduct collision checks
        self._discretize_grid_and_collision_checks(static_obstacles)

        # 4. Build Graph
        self.coordinates: Dict[int, Tuple[float, float]] = {}
        self.adj_list: Dict[int, List[int]] = {}
        self.weights: Dict[Tuple[int, int], float] = {}
        self.grid_to_id: Dict[Tuple[int, int], int] = {}
        self._build_graph()

    def _discretize_grid_and_collision_checks(self, obstacles: List[StaticObstacle]):
        """
        Input: List with static obstacles
        Ouput: Occupancy grid
        """
        for obs in obstacles:
            # inflate obstacles with buffer (r = robot_radius + margin)
            inflated_obs = obs.shape.buffer(self.robot_buffer)
            self.inflated_obstacles.append(inflated_obs)

            min_ox, min_oy, max_ox, max_oy = (
                inflated_obs.bounds
            )  # min and max coordinate points for inflated grid geometries
            # first we do a coordinate transform (min_ox - min_x), since scene bounds are from -12 to 12. We want relative position to grid bounds. \
            # then scale the relative distance by /div self.resolution and finally discretize continous values to discrete ones.
            x_start = max(0, int(np.floor((min_ox - self.min_x) / self.resolution)))
            x_end = min(self.width - 1, int(np.floor((max_ox - self.min_x) / self.resolution)))
            y_start = max(0, int(np.floor((min_oy - self.min_y) / self.resolution)))
            y_end = min(self.height - 1, int(np.floor((max_oy - self.min_y) / self.resolution)))

            for row in range(y_start, y_end + 1):  # calculating the edges of the grid
                cell_y_min = self.min_y + row * self.resolution
                cell_y_max = cell_y_min + self.resolution
                for col in range(x_start, x_end + 1):
                    if self.grid[row, col]:  # if already occupied -> continue
                        continue
                    cell_x_min = self.min_x + col * self.resolution
                    cell_x_max = cell_x_min + self.resolution

                    # creating shapely polygon box to use for collision checking
                    cell_box = box(cell_x_min, cell_y_min, cell_x_max, cell_y_max)

                    # essentially we check if the grid cell intersects at all with our inflated obstacle
                    if inflated_obs.intersects(cell_box):
                        self.grid[row, col] = True

    def _build_graph(self):
        node_count = 0

        for row in range(self.height):
            for col in range(self.width):
                if not self.grid[row, col]:
                    node_id = node_count
                    node_count += 1
                    self.grid_to_id[(row, col)] = node_id  # each node has a node id, accessable through grid pos
                    # since we created a grid from [0,.., N] we need to shift it back to the original world frame
                    wrld_x = (
                        self.min_x + (col + 0.5) * self.resolution
                    )  # (col + 0.5) is to shift the index into the middle of the cell, else it would be
                    wrld_y = self.min_y + (row + 0.5) * self.resolution  # on the bottom left corner
                    self.coordinates[node_id] = (wrld_x, wrld_y)
                    self.adj_list[node_id] = []

        # standard moves: up, down, left, right and diagnoally; here I just introduced the distance as a measure of cost
        moves = [
            (0, 1, 1),
            (0, -1, 1),
            (1, 0, 1),
            (-1, 0, 1),
            (1, 1, np.sqrt(2)),
            (1, -1, np.sqrt(2)),
            (-1, 1, np.sqrt(2)),
            (-1, -1, np.sqrt(2)),
        ]
        for (row, col), curr_node_id in self.grid_to_id.items():
            for dr, dc, cost in moves:
                next_row, next_column = row + dr, col + dc

                # to check if the neighbor is inside gird and not an obstacle
                if (next_row, next_column) in self.grid_to_id:
                    if dr != 0 and dc != 0:  # diagonal move
                        if (
                            self.grid[row, next_column] or self.grid[next_row, col]
                        ):  # if diagonal cells are occupied, dont allow to move
                            continue
                    neighbor_node_id = self.grid_to_id[(next_row, next_column)]
                    # building the adj list
                    self.adj_list[curr_node_id].append(neighbor_node_id)
                    self.weights[(curr_node_id, neighbor_node_id)] = self.resolution * cost

    def get_node_coordinates(self, u: int) -> Tuple[float, float]:
        return self.coordinates[u]

    def get_weight(self, u: int, v: int) -> float:
        return self.weights.get((u, v), float("inf"))

    def find_nearest_node(self, pos: Tuple[float, float]) -> Optional[int]:
        "For a given position we assign the corresponding (closest) node_id"

        col = int(np.floor((pos[0] - self.min_x) / self.resolution))
        row = int(np.floor((pos[1] - self.min_y) / self.resolution))

        if (row, col) in self.grid_to_id:
            return self.grid_to_id[(row, col)]

        queue = deque([(row, col)])
        visited = {(row, col)}
        max_search_radius = 6
        while queue:
            current_row, current_column = queue.popleft()
            if abs(current_row - row) > max_search_radius or abs(current_column - col) > max_search_radius:
                continue
            for dr in [-1, 0, 1]:  # we allow looking around in x from -1 to 1
                for dc in [-1, 0, 1]:  # same for y, creating a 3x3 neighbor check around orignial postion
                    next_row, next_column = current_row + dr, current_column + dc
                    if (next_row, next_column) in visited:
                        continue
                    if (next_row, next_column) in self.grid_to_id:
                        return self.grid_to_id[(next_row, next_column)]
                    visited.add((next_row, next_column))
                    queue.append((next_row, next_column))
        return None

    def check_collision_free(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """
        Checks if a straight line between p1 and p2 intersects any obstacle.
        """
        line = LineString([p1, p2])
        for obs_geom in self.inflated_obstacles:
            if line.intersects(obs_geom):
                return False
        return True


class Astar:
    def __init__(self, graph: OccupancyGridGraph):
        self.graph = graph

    def heuristic(self, u: int, v: int) -> float:
        x1, y1 = self.graph.get_node_coordinates(u)
        x2, y2 = self.graph.get_node_coordinates(v)
        return math.hypot(x2 - x1, y2 - y1)

    def path(
        self, start_id: int, goal_id: int, start_pos: Tuple[float, float], goal_pos: Tuple[float, float]
    ) -> List[Tuple[float, float]]:
        """Takes in a start and goal position, the closest corresponding discrete grid node ids and computes shortest path.
        Outputs list of waypoints marking shortest path
        """
        if start_id == goal_id:
            return [start_pos, goal_pos]

        counter = 0
        queue = [(0.0, counter, start_id)]
        came_from = {start_id: None}
        g_score = {start_id: 0.0}

        path_indices: List = []
        found = False

        while queue:
            current_f, _, current = heapq.heappop(queue)
            if current == goal_id:
                path_indices = self._reconstruct_path(came_from, current)
                found = True
                break

            if current_f > g_score[current] + self.heuristic(current, goal_id):
                continue

            for neighbor in self.graph.adj_list.get(current, []):
                tentative_g = g_score[current] + self.graph.get_weight(
                    current, neighbor
                )  # cost from current to next node
                if tentative_g < g_score.get(
                    neighbor, float("inf")
                ):  # if new cost lower than previous one, update path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_id)
                    counter += 1
                    heapq.heappush(queue, (f_score, counter, neighbor))

        if not found:
            return []

        path_coords = [self.graph.get_node_coordinates(n_id) for n_id in path_indices]
        full_path = [start_pos] + path_coords + [goal_pos]

        return self.smooth_path(full_path)

    def _reconstruct_path(self, came_from: Dict[int, int], current: int) -> List[Tuple[float, float]]:
        path_indices = []
        while current is not None:
            path_indices.append(current)
            current = came_from[current]
        path_indices.reverse()
        return path_indices

    def smooth_path(self, raw_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Applies greedy shortcutting to remove zig-zags. Mainly fixing start/goal issues: Input coordinates are floats
        algo wants discrete nodes, therefore we need to somehow connect the discrete nodes with the continous pos of start/goal in a smooth manner
        """
        if len(raw_path) <= 2:
            return raw_path

        smoothed = [raw_path[0]]
        curr_idx = 0

        while curr_idx < len(raw_path) - 1:
            target_found = False
            # from the curr_idx we look backwards to start and check if path is collision free
            for check_idx in range(len(raw_path) - 1, curr_idx, -1):
                p1 = raw_path[curr_idx]
                p2 = raw_path[check_idx]

                if self.graph.check_collision_free(p1, p2):
                    # Found a shortcut!
                    smoothed.append(p2)
                    curr_idx = check_idx
                    target_found = True
                    break

            # Fallback (shouldn't happen in valid A* path, but for safety)
            if not target_found:
                curr_idx += 1
                smoothed.append(raw_path[curr_idx])

        return smoothed

