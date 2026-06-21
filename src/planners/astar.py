"""A* occupancy-grid planner baseline for ODA ground-plane trials."""

from __future__ import annotations

import heapq
from dataclasses import dataclass

import numpy as np

from src.planners.baselines import PlannedPath, resample_polyline


@dataclass(frozen=True)
class AStarConfig:
    resolution_m: float = 0.10
    margin_m: float = 1.00
    obstacle_radius_m: float = 0.20
    safety_distance_m: float = 0.50

    @property
    def inflated_radius_m(self) -> float:
        return self.obstacle_radius_m + self.safety_distance_m


def _make_bounds(start: np.ndarray, goal: np.ndarray, obstacles_xy: np.ndarray, margin_m: float):
    all_xy = np.vstack([start[None, :], goal[None, :], obstacles_xy])
    xy_min = all_xy.min(axis=0) - margin_m
    xy_max = all_xy.max(axis=0) + margin_m
    return xy_min, xy_max


def _xy_to_idx(xy: np.ndarray, xy_min: np.ndarray, resolution_m: float) -> tuple[int, int]:
    idx = np.round((xy - xy_min) / resolution_m).astype(int)
    return int(idx[0]), int(idx[1])


def _idx_to_xy(idx: tuple[int, int], xy_min: np.ndarray, resolution_m: float) -> np.ndarray:
    return xy_min + np.asarray(idx, dtype=float) * resolution_m


def _build_occupancy(
    xy_min: np.ndarray,
    xy_max: np.ndarray,
    obstacles_xy: np.ndarray,
    config: AStarConfig,
) -> np.ndarray:
    size = np.ceil((xy_max - xy_min) / config.resolution_m).astype(int) + 1
    xs = xy_min[0] + np.arange(size[0]) * config.resolution_m
    ys = xy_min[1] + np.arange(size[1]) * config.resolution_m
    grid_x, grid_y = np.meshgrid(xs, ys, indexing="ij")
    occupied = np.zeros(size, dtype=bool)
    for obstacle in obstacles_xy:
        dist = np.sqrt((grid_x - obstacle[0]) ** 2 + (grid_y - obstacle[1]) ** 2)
        occupied |= dist <= config.inflated_radius_m
    return occupied


def _nearest_free(occupied: np.ndarray, idx: tuple[int, int]) -> tuple[int, int]:
    nx, ny = occupied.shape
    x0 = min(max(idx[0], 0), nx - 1)
    y0 = min(max(idx[1], 0), ny - 1)
    if not occupied[x0, y0]:
        return x0, y0
    for radius in range(1, max(nx, ny)):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                x = x0 + dx
                y = y0 + dy
                if 0 <= x < nx and 0 <= y < ny and not occupied[x, y]:
                    return x, y
    raise ValueError("No free cell found in occupancy grid")


def _reconstruct(came_from: dict[tuple[int, int], tuple[int, int]], current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar_path(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    config: AStarConfig | None = None,
    num_points: int = 200,
) -> PlannedPath:
    """Plan an A* path around inflated circular obstacles."""

    config = config or AStarConfig()
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    obstacles_xy = np.asarray(obstacles_xy, dtype=float)
    xy_min, xy_max = _make_bounds(start, goal, obstacles_xy, config.margin_m)
    occupied = _build_occupancy(xy_min, xy_max, obstacles_xy, config)
    start_idx = _nearest_free(occupied, _xy_to_idx(start, xy_min, config.resolution_m))
    goal_idx = _nearest_free(occupied, _xy_to_idx(goal, xy_min, config.resolution_m))

    neighbors = [
        (-1, -1, np.sqrt(2.0)),
        (-1, 0, 1.0),
        (-1, 1, np.sqrt(2.0)),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (1, -1, np.sqrt(2.0)),
        (1, 0, 1.0),
        (1, 1, np.sqrt(2.0)),
    ]
    nx, ny = occupied.shape
    open_heap: list[tuple[float, tuple[int, int]]] = []
    heapq.heappush(open_heap, (0.0, start_idx))
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start_idx: 0.0}

    def heuristic(idx: tuple[int, int]) -> float:
        return float(np.linalg.norm(np.asarray(idx, dtype=float) - np.asarray(goal_idx, dtype=float)))

    visited: set[tuple[int, int]] = set()
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal_idx:
            idx_path = _reconstruct(came_from, current)
            waypoints = np.asarray(
                [_idx_to_xy(idx, xy_min, config.resolution_m) for idx in idx_path],
                dtype=float,
            )
            return PlannedPath(
                name="astar_grid",
                waypoints=waypoints,
                trajectory_xy=resample_polyline(waypoints, num_points),
            )

        for dx, dy, step_cost in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < nx and 0 <= neighbor[1] < ny):
                continue
            if occupied[neighbor]:
                continue
            tentative_g = g_score[current] + step_cost * config.resolution_m
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + heuristic(neighbor) * config.resolution_m
                heapq.heappush(open_heap, (f_score, neighbor))

    raise RuntimeError("A* failed to find a path")
