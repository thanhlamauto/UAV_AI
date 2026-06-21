"""RRT* planner baseline for ODA ground-plane trials."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.planners.baselines import PlannedPath, resample_polyline
from src.planners.rrt import _make_bounds, _point_is_free, _segment_is_free, _steer


@dataclass(frozen=True)
class RRTStarConfig:
    max_iterations: int = 2500
    step_size_m: float = 0.30
    neighbor_radius_m: float = 0.75
    goal_sample_rate: float = 0.15
    margin_m: float = 1.00
    obstacle_radius_m: float = 0.20
    safety_distance_m: float = 0.50
    seed: int = 7

    @property
    def inflated_radius_m(self) -> float:
        return self.obstacle_radius_m + self.safety_distance_m


def _reconstruct(nodes: list[np.ndarray], parents: list[int], current_idx: int, goal: np.ndarray) -> np.ndarray:
    path = [goal]
    while current_idx >= 0:
        path.append(nodes[current_idx])
        current_idx = parents[current_idx]
    path.reverse()
    return np.asarray(path, dtype=float)


def rrt_star_path(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    config: RRTStarConfig | None = None,
    num_points: int = 200,
) -> PlannedPath:
    """Plan an RRT* path around inflated circular obstacles."""

    config = config or RRTStarConfig()
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    obstacles_xy = np.asarray(obstacles_xy, dtype=float)

    if _segment_is_free(start, goal, obstacles_xy, config.inflated_radius_m):
        waypoints = np.asarray([start, goal], dtype=float)
        return PlannedPath(
            name="rrt_star",
            waypoints=waypoints,
            trajectory_xy=resample_polyline(waypoints, num_points),
        )

    xy_min, xy_max = _make_bounds(start, goal, obstacles_xy, config.margin_m)
    rng = np.random.default_rng(config.seed)
    nodes: list[np.ndarray] = [start]
    parents: list[int] = [-1]
    costs: list[float] = [0.0]
    best_goal_parent: int | None = None
    best_goal_cost = float("inf")

    for _ in range(config.max_iterations):
        sample = goal if rng.random() < config.goal_sample_rate else rng.uniform(xy_min, xy_max)
        node_arr = np.asarray(nodes)
        distances = np.linalg.norm(node_arr - sample, axis=1)
        nearest_idx = int(np.argmin(distances))
        new = _steer(nodes[nearest_idx], sample, config.step_size_m)
        if not _point_is_free(new, obstacles_xy, config.inflated_radius_m):
            continue
        if not _segment_is_free(nodes[nearest_idx], new, obstacles_xy, config.inflated_radius_m):
            continue

        neighbor_indices = np.where(np.linalg.norm(node_arr - new, axis=1) <= config.neighbor_radius_m)[0].tolist()
        parent_idx = nearest_idx
        parent_cost = costs[nearest_idx] + float(np.linalg.norm(new - nodes[nearest_idx]))
        for idx in neighbor_indices:
            edge_cost = float(np.linalg.norm(new - nodes[idx]))
            candidate_cost = costs[idx] + edge_cost
            if candidate_cost < parent_cost and _segment_is_free(nodes[idx], new, obstacles_xy, config.inflated_radius_m):
                parent_idx = idx
                parent_cost = candidate_cost

        nodes.append(new)
        parents.append(parent_idx)
        costs.append(parent_cost)
        new_idx = len(nodes) - 1

        for idx in neighbor_indices:
            if idx == parent_idx:
                continue
            edge_cost = float(np.linalg.norm(nodes[idx] - new))
            candidate_cost = parent_cost + edge_cost
            if candidate_cost < costs[idx] and _segment_is_free(new, nodes[idx], obstacles_xy, config.inflated_radius_m):
                parents[idx] = new_idx
                costs[idx] = candidate_cost

        if _segment_is_free(new, goal, obstacles_xy, config.inflated_radius_m):
            goal_cost = parent_cost + float(np.linalg.norm(goal - new))
            if goal_cost < best_goal_cost:
                best_goal_cost = goal_cost
                best_goal_parent = new_idx

    if best_goal_parent is None:
        raise RuntimeError("RRT* failed to find a path")

    waypoints = _reconstruct(nodes, parents, best_goal_parent, goal)
    return PlannedPath(
        name="rrt_star",
        waypoints=waypoints,
        trajectory_xy=resample_polyline(waypoints, num_points),
    )
