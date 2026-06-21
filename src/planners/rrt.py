"""Lightweight RRT planner baseline for ODA ground-plane trials."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.planners.baselines import PlannedPath, resample_polyline, segment_circle_intersects


@dataclass(frozen=True)
class RRTConfig:
    max_iterations: int = 1500
    step_size_m: float = 0.35
    goal_sample_rate: float = 0.20
    margin_m: float = 1.00
    obstacle_radius_m: float = 0.20
    safety_distance_m: float = 0.50
    seed: int = 7

    @property
    def inflated_radius_m(self) -> float:
        return self.obstacle_radius_m + self.safety_distance_m


def _make_bounds(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    margin_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    all_xy = np.vstack([start[None, :], goal[None, :], obstacles_xy])
    xy_min = all_xy.min(axis=0) - margin_m
    xy_max = all_xy.max(axis=0) + margin_m
    return xy_min, xy_max


def _point_is_free(point: np.ndarray, obstacles_xy: np.ndarray, inflated_radius_m: float) -> bool:
    if len(obstacles_xy) == 0:
        return True
    return bool(np.all(np.linalg.norm(obstacles_xy - point, axis=1) > inflated_radius_m))


def _segment_is_free(
    a: np.ndarray,
    b: np.ndarray,
    obstacles_xy: np.ndarray,
    inflated_radius_m: float,
) -> bool:
    for obstacle in obstacles_xy:
        if segment_circle_intersects(a, b, obstacle, inflated_radius_m):
            return False
    return True


def _steer(from_xy: np.ndarray, to_xy: np.ndarray, step_size_m: float) -> np.ndarray:
    delta = to_xy - from_xy
    dist = float(np.linalg.norm(delta))
    if dist <= step_size_m or dist == 0.0:
        return to_xy.copy()
    return from_xy + delta / dist * step_size_m


def _reconstruct(nodes: list[np.ndarray], parents: list[int], current_idx: int) -> np.ndarray:
    path = []
    while current_idx >= 0:
        path.append(nodes[current_idx])
        current_idx = parents[current_idx]
    path.reverse()
    return np.asarray(path, dtype=float)


def _shortcut_path(path: np.ndarray, obstacles_xy: np.ndarray, inflated_radius_m: float) -> np.ndarray:
    if len(path) <= 2:
        return path

    shortened = [path[0]]
    i = 0
    while i < len(path) - 1:
        next_i = i + 1
        for j in range(len(path) - 1, i, -1):
            if _segment_is_free(path[i], path[j], obstacles_xy, inflated_radius_m):
                next_i = j
                break
        shortened.append(path[next_i])
        i = next_i
    return np.asarray(shortened, dtype=float)


def rrt_path(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    config: RRTConfig | None = None,
    num_points: int = 200,
) -> PlannedPath:
    """Plan a deterministic RRT path around inflated circular obstacles."""

    config = config or RRTConfig()
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    obstacles_xy = np.asarray(obstacles_xy, dtype=float)

    if _segment_is_free(start, goal, obstacles_xy, config.inflated_radius_m):
        waypoints = np.asarray([start, goal], dtype=float)
        return PlannedPath(
            name="rrt",
            waypoints=waypoints,
            trajectory_xy=resample_polyline(waypoints, num_points),
        )

    xy_min, xy_max = _make_bounds(start, goal, obstacles_xy, config.margin_m)
    rng = np.random.default_rng(config.seed)
    nodes: list[np.ndarray] = [start]
    parents: list[int] = [-1]

    for _ in range(config.max_iterations):
        if rng.random() < config.goal_sample_rate:
            sample = goal
        else:
            sample = rng.uniform(xy_min, xy_max)

        node_arr = np.asarray(nodes)
        nearest_idx = int(np.argmin(np.linalg.norm(node_arr - sample, axis=1)))
        nearest = nodes[nearest_idx]
        new = _steer(nearest, sample, config.step_size_m)
        if not _point_is_free(new, obstacles_xy, config.inflated_radius_m):
            continue
        if not _segment_is_free(nearest, new, obstacles_xy, config.inflated_radius_m):
            continue

        nodes.append(new)
        parents.append(nearest_idx)
        new_idx = len(nodes) - 1

        if _segment_is_free(new, goal, obstacles_xy, config.inflated_radius_m):
            nodes.append(goal)
            parents.append(new_idx)
            waypoints = _reconstruct(nodes, parents, len(nodes) - 1)
            waypoints = _shortcut_path(waypoints, obstacles_xy, config.inflated_radius_m)
            return PlannedPath(
                name="rrt",
                waypoints=waypoints,
                trajectory_xy=resample_polyline(waypoints, num_points),
            )

    raise RuntimeError("RRT failed to find a path")
