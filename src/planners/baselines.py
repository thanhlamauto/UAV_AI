"""Straight-line and geometric bypass planner baselines."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.metrics import path_length, pairwise_ground_distances


@dataclass(frozen=True)
class PlannedPath:
    name: str
    waypoints: np.ndarray
    trajectory_xy: np.ndarray


def resample_polyline(points: np.ndarray, num_points: int) -> np.ndarray:
    """Sample a polyline to a fixed number of points."""

    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        raise ValueError("points cannot be empty")
    if len(points) == 1 or num_points <= 1:
        return np.repeat(points[:1], max(1, num_points), axis=0)

    seg_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(seg_lengths.sum())
    if total == 0:
        return np.repeat(points[:1], num_points, axis=0)

    cumulative = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    targets = np.linspace(0.0, total, num_points)
    sampled = np.zeros((num_points, 2), dtype=float)
    seg_idx = 0
    for i, target in enumerate(targets):
        while seg_idx < len(seg_lengths) - 1 and cumulative[seg_idx + 1] < target:
            seg_idx += 1
        start = points[seg_idx]
        end = points[seg_idx + 1]
        denom = cumulative[seg_idx + 1] - cumulative[seg_idx]
        alpha = 0.0 if denom == 0 else (target - cumulative[seg_idx]) / denom
        sampled[i] = (1.0 - alpha) * start + alpha * end
    return sampled


def straight_line_path(start: np.ndarray, goal: np.ndarray, num_points: int) -> PlannedPath:
    waypoints = np.asarray([start, goal], dtype=float)
    return PlannedPath(
        name="straight_line",
        waypoints=waypoints,
        trajectory_xy=resample_polyline(waypoints, num_points),
    )


def segment_circle_intersects(
    a: np.ndarray,
    b: np.ndarray,
    center: np.ndarray,
    radius: float,
) -> bool:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom == 0:
        return float(np.linalg.norm(a - center)) <= radius
    t = float(np.dot(center - a, ab) / denom)
    t = min(1.0, max(0.0, t))
    closest = a + t * ab
    return float(np.linalg.norm(closest - center)) <= radius


def _ordered_intersecting_obstacles(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    inflated_radius_m: float,
) -> list[np.ndarray]:
    route = goal - start
    route_norm = float(np.linalg.norm(route))
    if route_norm == 0:
        return []
    route_unit = route / route_norm

    candidates: list[tuple[float, np.ndarray]] = []
    for obstacle in obstacles_xy:
        projection = float(np.dot(obstacle - start, route_unit))
        if projection < 0.0 or projection > route_norm:
            continue
        if segment_circle_intersects(start, goal, obstacle, inflated_radius_m):
            candidates.append((projection, obstacle))
    return [obstacle for _, obstacle in sorted(candidates, key=lambda item: item[0])]


def geometric_bypass_candidates(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    inflated_radius_m: float,
    bypass_margin_m: float = 0.25,
    num_points: int = 200,
) -> list[PlannedPath]:
    """Generate left/right geometric bypass paths.

    Obstacles intersecting the direct segment are sorted along the start-goal
    direction.  Each candidate inserts one lateral waypoint per intersecting
    obstacle on either the left or right side of the direct path.
    """

    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    obstacles_xy = np.asarray(obstacles_xy, dtype=float)
    direct = goal - start
    direct_norm = float(np.linalg.norm(direct))
    if direct_norm == 0:
        return [straight_line_path(start, goal, num_points)]

    route_unit = direct / direct_norm
    left_normal = np.asarray([-route_unit[1], route_unit[0]], dtype=float)
    obstacles = _ordered_intersecting_obstacles(start, goal, obstacles_xy, inflated_radius_m)
    if not obstacles:
        return [straight_line_path(start, goal, num_points)]

    candidates: list[PlannedPath] = []
    for side_name, side_sign in [("left", 1.0), ("right", -1.0)]:
        waypoints = [start]
        for obstacle in obstacles:
            waypoint = obstacle + side_sign * left_normal * (inflated_radius_m + bypass_margin_m)
            waypoints.append(waypoint)
        waypoints.append(goal)
        waypoint_arr = np.asarray(waypoints, dtype=float)
        candidates.append(
            PlannedPath(
                name=f"geometric_bypass_{side_name}",
                waypoints=waypoint_arr,
                trajectory_xy=resample_polyline(waypoint_arr, num_points),
            )
        )
    return candidates


def select_best_geometric_bypass(
    start: np.ndarray,
    goal: np.ndarray,
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float = 0.20,
    safety_distance_m: float = 0.50,
    bypass_margin_m: float = 0.25,
    num_points: int = 200,
) -> PlannedPath:
    inflated = obstacle_radius_m + safety_distance_m
    candidates = geometric_bypass_candidates(
        start=start,
        goal=goal,
        obstacles_xy=obstacles_xy,
        inflated_radius_m=inflated,
        bypass_margin_m=bypass_margin_m,
        num_points=num_points,
    )
    if len(candidates) == 1:
        candidate = candidates[0]
        if candidate.name == "straight_line":
            return PlannedPath(
                name="geometric_bypass_not_needed",
                waypoints=candidate.waypoints,
                trajectory_xy=candidate.trajectory_xy,
            )

    def score(path: PlannedPath) -> tuple[float, float]:
        distances = pairwise_ground_distances(path.trajectory_xy, obstacles_xy)
        min_clearance = float(distances.min() - obstacle_radius_m)
        return (min_clearance, -path_length(path.trajectory_xy))

    best = max(candidates, key=score)
    return PlannedPath(
        name="geometric_bypass",
        waypoints=best.waypoints,
        trajectory_xy=best.trajectory_xy,
    )
