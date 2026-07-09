"""Continuous 3D MPPI-style trajectory optimizer over an ESDF."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from src.esdf3d import ESDF3D


@dataclass(frozen=True)
class MPPI3DConfig:
    num_rollouts: int = 768
    horizon_steps: int = 72
    max_iterations: int = 12
    temperature: float = 0.85
    noise_sigma_m: float = 0.34
    safety_radius_m: float = 0.42
    path_length_weight: float = 1.0
    smoothness_weight: float = 4.0
    clearance_weight: float = 140.0
    collision_weight: float = 5000.0
    bounds_weight: float = 5000.0
    altitude_weight: float = 80.0
    min_altitude_m: float = 0.45
    max_altitude_m: float = 2.55
    seed: int = 31


@dataclass(frozen=True)
class MPPI3DResult:
    trajectory_xyz: np.ndarray
    costs: list[float]
    compute_time_s: float
    min_esdf_distance_m: float
    path_length_m: float
    smoothness: float
    collision: bool
    safety_violation: bool


def _resample_polyline_3d(points: np.ndarray, num_points: int) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        raise ValueError("points cannot be empty")
    if len(points) == 1 or num_points <= 1:
        return np.repeat(points[:1], max(1, num_points), axis=0)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(lengths.sum())
    if total <= 1e-9:
        return np.repeat(points[:1], num_points, axis=0)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    targets = np.linspace(0.0, total, num_points)
    sampled = np.zeros((num_points, 3), dtype=float)
    seg = 0
    for i, target in enumerate(targets):
        while seg < len(lengths) - 1 and cumulative[seg + 1] < target:
            seg += 1
        denom = cumulative[seg + 1] - cumulative[seg]
        alpha = 0.0 if denom <= 0 else (target - cumulative[seg]) / denom
        sampled[i] = (1.0 - alpha) * points[seg] + alpha * points[seg + 1]
    return sampled


def _seed_paths(start: np.ndarray, goal: np.ndarray, config: MPPI3DConfig) -> list[np.ndarray]:
    t = np.linspace(0.0, 1.0, config.horizon_steps)
    direct = (1.0 - t)[:, None] * start + t[:, None] * goal
    xy = goal[:2] - start[:2]
    norm = float(np.linalg.norm(xy))
    normal = np.asarray([0.0, 1.0], dtype=float) if norm <= 1e-9 else np.asarray([-xy[1], xy[0]], dtype=float) / norm

    seeds = [direct]
    for lateral in (-1.15, 1.15, -1.75, 1.75):
        path = direct.copy()
        path[:, :2] += np.sin(np.pi * t)[:, None] * normal[None, :] * lateral
        seeds.append(path)
    for lift in (0.55, 0.95):
        path = direct.copy()
        path[:, 2] += np.sin(np.pi * t) * lift
        seeds.append(path)
    for lateral in (-1.05, 1.05, -1.75, 1.75):
        for lift in (0.45, 0.75):
            path = direct.copy()
            path[:, :2] += np.sin(np.pi * t)[:, None] * normal[None, :] * lateral
            path[:, 2] += np.sin(np.pi * t) * lift
            seeds.append(path)
    return seeds


def _path_length(paths: np.ndarray) -> np.ndarray:
    return np.sum(np.linalg.norm(np.diff(paths, axis=1), axis=2), axis=1)


def _smoothness(paths: np.ndarray) -> np.ndarray:
    if paths.shape[1] < 3:
        return np.zeros(paths.shape[0], dtype=float)
    second = paths[:, 2:, :] - 2.0 * paths[:, 1:-1, :] + paths[:, :-2, :]
    return np.sum(np.linalg.norm(second, axis=2) ** 2, axis=1)


def rollout_cost(paths: np.ndarray, esdf: ESDF3D, config: MPPI3DConfig) -> np.ndarray:
    flat = paths.reshape((-1, 3))
    distances = esdf.query_distance(flat).reshape(paths.shape[:2])
    inside = esdf.contains(flat).reshape(paths.shape[:2])

    safety_deficit = np.maximum(config.safety_radius_m - distances, 0.0)
    clearance_cost = np.sum(safety_deficit**2, axis=1)
    collision_cost = np.sum(distances <= 0.0, axis=1)
    bounds_cost = np.sum(~inside, axis=1)
    low = np.maximum(config.min_altitude_m - paths[:, :, 2], 0.0)
    high = np.maximum(paths[:, :, 2] - config.max_altitude_m, 0.0)
    altitude_cost = np.sum(low**2 + high**2, axis=1)

    return (
        config.path_length_weight * _path_length(paths)
        + config.smoothness_weight * _smoothness(paths)
        + config.clearance_weight * clearance_cost
        + config.collision_weight * collision_cost
        + config.bounds_weight * bounds_cost
        + config.altitude_weight * altitude_cost
    )


def mppi_3d_esdf_path(
    start_xyz: np.ndarray,
    goal_xyz: np.ndarray,
    esdf: ESDF3D,
    config: MPPI3DConfig | None = None,
) -> MPPI3DResult:
    """Optimize a continuous xyz trajectory using MPPI-style perturbations."""

    config = config or MPPI3DConfig()
    if config.horizon_steps < 3:
        raise ValueError("horizon_steps must be at least 3")

    start_time = time.perf_counter()
    start = np.asarray(start_xyz, dtype=float)
    goal = np.asarray(goal_xyz, dtype=float)
    rng = np.random.default_rng(config.seed)

    seed_candidates = np.stack(_seed_paths(start, goal, config), axis=0)
    seed_costs = rollout_cost(seed_candidates, esdf, config)
    mean_path = seed_candidates[int(np.argmin(seed_costs))].copy()
    mean_path[0] = start
    mean_path[-1] = goal
    best_costs = [float(np.min(seed_costs))]

    for _ in range(config.max_iterations):
        noise = rng.normal(0.0, config.noise_sigma_m, size=(config.num_rollouts, config.horizon_steps, 3))
        noise[:, 0, :] = 0.0
        noise[:, -1, :] = 0.0
        noise[:, :, 2] *= 0.65
        rollouts = mean_path[None, :, :] + noise
        rollouts[:, 0, :] = start
        rollouts[:, -1, :] = goal

        costs = rollout_cost(rollouts, esdf, config)
        shifted = costs - float(np.min(costs))
        weights = np.exp(-shifted / max(config.temperature, 1e-6))
        weight_sum = float(np.sum(weights))
        if not np.isfinite(weight_sum) or weight_sum <= 1e-12:
            break
        weights /= weight_sum
        mean_path = mean_path + np.sum(weights[:, None, None] * noise, axis=0)
        mean_path[0] = start
        mean_path[-1] = goal
        mean_path[:, 2] = np.clip(mean_path[:, 2], config.min_altitude_m, config.max_altitude_m)
        best_costs.append(float(rollout_cost(mean_path[None, :, :], esdf, config)[0]))

    final_candidates = np.concatenate([mean_path[None, :, :], seed_candidates], axis=0)
    final_costs = rollout_cost(final_candidates, esdf, config)
    trajectory = final_candidates[int(np.argmin(final_costs))]
    distances = esdf.query_distance(trajectory)
    compute_time_s = time.perf_counter() - start_time
    length = float(_path_length(trajectory[None, :, :])[0])
    smooth = float(_smoothness(trajectory[None, :, :])[0])
    min_dist = float(np.min(distances))
    return MPPI3DResult(
        trajectory_xyz=trajectory,
        costs=best_costs,
        compute_time_s=compute_time_s,
        min_esdf_distance_m=min_dist,
        path_length_m=length,
        smoothness=smooth,
        collision=bool(min_dist <= 0.0),
        safety_violation=bool(min_dist < config.safety_radius_m),
    )


def resample_result(result: MPPI3DResult, num_points: int) -> np.ndarray:
    return _resample_polyline_3d(result.trajectory_xyz, num_points)
