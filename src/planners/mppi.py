"""Lightweight MPPI-style waypoint optimizer for ODA ground-plane trials."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.metrics import path_length, pairwise_ground_distances
from src.planners.baselines import PlannedPath, resample_polyline, select_best_geometric_bypass


@dataclass(frozen=True)
class MPPIConfig:
    num_rollouts: int = 512
    horizon_steps: int = 60
    dt: float = 0.10
    temperature: float = 1.0
    max_iterations: int = 10
    noise_sigma_m: float = 0.35
    obstacle_radius_m: float = 0.20
    safety_distance_m: float = 0.50
    path_length_weight: float = 1.0
    smoothness_weight: float = 0.50
    obstacle_weight: float = 80.0
    collision_weight: float = 10000.0
    seed: int = 7


def _initial_path(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    config: MPPIConfig,
) -> np.ndarray:
    geometric = select_best_geometric_bypass(
        start=start,
        goal=goal,
        obstacles_xy=obstacles_xy,
        obstacle_radius_m=config.obstacle_radius_m,
        safety_distance_m=config.safety_distance_m,
        num_points=config.horizon_steps,
    )
    return geometric.trajectory_xy


def _smoothness_cost(paths: np.ndarray) -> np.ndarray:
    if paths.shape[1] < 3:
        return np.zeros(paths.shape[0], dtype=float)
    second = paths[:, 2:, :] - 2.0 * paths[:, 1:-1, :] + paths[:, :-2, :]
    return np.sum(np.linalg.norm(second, axis=2) ** 2, axis=1)


def _rollout_cost(paths: np.ndarray, obstacles_xy: np.ndarray, config: MPPIConfig) -> np.ndarray:
    diffs = np.diff(paths, axis=1)
    lengths = np.sum(np.linalg.norm(diffs, axis=2), axis=1)
    smoothness = _smoothness_cost(paths)

    if len(obstacles_xy):
        distances = np.linalg.norm(paths[:, :, None, :] - obstacles_xy[None, None, :, :], axis=3)
        clearance = np.min(distances, axis=2) - config.obstacle_radius_m
        safety_deficit = np.maximum(config.safety_distance_m - clearance, 0.0)
        obstacle_cost = np.sum(safety_deficit**2, axis=1)
        collision_cost = np.sum(clearance <= 0.0, axis=1)
    else:
        obstacle_cost = np.zeros(paths.shape[0], dtype=float)
        collision_cost = np.zeros(paths.shape[0], dtype=float)

    return (
        config.path_length_weight * lengths
        + config.smoothness_weight * smoothness
        + config.obstacle_weight * obstacle_cost
        + config.collision_weight * collision_cost
    )


def _clip_endpoints(paths: np.ndarray, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
    paths[:, 0, :] = start
    paths[:, -1, :] = goal
    return paths


def mppi_path(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    config: MPPIConfig | None = None,
    num_points: int = 200,
) -> PlannedPath:
    """Optimize a 2D path using MPPI-style weighted rollout perturbations."""

    config = config or MPPIConfig()
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    obstacles_xy = np.asarray(obstacles_xy, dtype=float)
    if config.horizon_steps < 2:
        raise ValueError("MPPI horizon_steps must be at least 2")

    rng = np.random.default_rng(config.seed)
    mean_path = _initial_path(start, goal, obstacles_xy, config).astype(float)
    mean_path[0] = start
    mean_path[-1] = goal

    for _ in range(config.max_iterations):
        noise = rng.normal(0.0, config.noise_sigma_m, size=(config.num_rollouts, config.horizon_steps, 2))
        noise[:, 0, :] = 0.0
        noise[:, -1, :] = 0.0
        rollouts = mean_path[None, :, :] + noise
        rollouts = _clip_endpoints(rollouts, start, goal)
        costs = _rollout_cost(rollouts, obstacles_xy, config)
        shifted = costs - float(np.min(costs))
        weights = np.exp(-shifted / max(config.temperature, 1e-6))
        weight_sum = float(np.sum(weights))
        if weight_sum == 0.0 or not np.isfinite(weight_sum):
            break
        weights /= weight_sum
        mean_path = mean_path + np.sum(weights[:, None, None] * noise, axis=0)
        mean_path[0] = start
        mean_path[-1] = goal

    candidate_paths = np.concatenate([mean_path[None, :, :], _initial_path(start, goal, obstacles_xy, config)[None, :, :]], axis=0)
    costs = _rollout_cost(candidate_paths, obstacles_xy, config)
    waypoints = candidate_paths[int(np.argmin(costs))]
    if len(obstacles_xy):
        distances = pairwise_ground_distances(waypoints, obstacles_xy)
        if np.min(distances) - config.obstacle_radius_m < config.safety_distance_m:
            fallback = _initial_path(start, goal, obstacles_xy, config)
            if path_length(fallback) <= path_length(waypoints) * 1.5:
                waypoints = fallback

    return PlannedPath(
        name="mppi",
        waypoints=waypoints,
        trajectory_xy=resample_polyline(waypoints, num_points),
    )
