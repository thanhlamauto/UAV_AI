"""Initial trajectory safety metrics for ODA obstacle-avoidance trials."""

from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import numpy as np


@dataclass(frozen=True)
class TrialMetrics:
    sequence: str
    obstacle_count: int
    min_center_distance_m: float
    min_boundary_clearance_m: float
    closest_time_s: float
    closest_index: int
    closest_obstacle_index: int
    collision: bool
    safety_violation: bool
    safety_distance_m: float
    path_length_m: float
    duration_s: float
    mean_speed_mps: float
    avoidance_label: str
    computation_time_ms: float

    def as_row(self) -> dict[str, object]:
        return {
            "sequence": self.sequence,
            "obstacles": self.obstacle_count,
            "min_center_distance_m": round(self.min_center_distance_m, 4),
            "min_boundary_clearance_m": round(self.min_boundary_clearance_m, 4),
            "closest_time_s": round(self.closest_time_s, 4),
            "closest_index": self.closest_index,
            "closest_obstacle_index": self.closest_obstacle_index,
            "collision": int(self.collision),
            "safety_violation": int(self.safety_violation),
            "safety_distance_m": round(self.safety_distance_m, 4),
            "path_length_m": round(self.path_length_m, 4),
            "duration_s": round(self.duration_s, 4),
            "mean_speed_mps": round(self.mean_speed_mps, 4),
            "avoidance_label": self.avoidance_label,
            "computation_time_ms": round(self.computation_time_ms, 4),
        }


def pairwise_ground_distances(trajectory_xy: np.ndarray, obstacles_xy: np.ndarray) -> np.ndarray:
    if trajectory_xy.ndim != 2 or trajectory_xy.shape[1] != 2:
        raise ValueError("trajectory_xy must have shape N x 2")
    if obstacles_xy.ndim != 2 or obstacles_xy.shape[1] != 2:
        raise ValueError("obstacles_xy must have shape M x 2")
    return np.linalg.norm(trajectory_xy[:, None, :] - obstacles_xy[None, :, :], axis=2)


def path_length(trajectory_xy: np.ndarray) -> float:
    if len(trajectory_xy) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(trajectory_xy, axis=0), axis=1).sum())


def classify_avoidance_side(
    trajectory_xy: np.ndarray,
    obstacle_xy: np.ndarray,
    closest_index: int,
    straight_tolerance_m: float = 0.25,
) -> str:
    """Heuristic left/right/straight label around the closest obstacle.

    The sign is relative to the line from the trajectory start to the closest
    obstacle.  This is only a first-pass label for mentor discussion, not a
    definitive behavior annotation.
    """

    start = trajectory_xy[0]
    closest = trajectory_xy[closest_index]
    ref = obstacle_xy - start
    side = closest - start
    ref_norm = np.linalg.norm(ref)
    if ref_norm == 0:
        return "unknown"

    signed_lateral = float(ref[0] * side[1] - ref[1] * side[0]) / ref_norm
    if abs(signed_lateral) < straight_tolerance_m:
        return "straight/near-center"
    return "left" if signed_lateral > 0 else "right"


def compute_trial_metrics(
    sequence: str,
    time_s: np.ndarray,
    trajectory_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float = 0.20,
    safety_distance_m: float = 0.50,
) -> TrialMetrics:
    started = perf_counter()
    if len(obstacles_xy) == 0:
        raise ValueError(f"Trial {sequence} has no obstacle coordinates in metadata")

    distances = pairwise_ground_distances(trajectory_xy, obstacles_xy)
    flat_index = int(np.argmin(distances))
    closest_index, closest_obstacle_index = np.unravel_index(flat_index, distances.shape)
    min_center_distance = float(distances[closest_index, closest_obstacle_index])
    min_clearance = min_center_distance - obstacle_radius_m
    duration = float(time_s[-1] - time_s[0]) if len(time_s) else 0.0
    length = path_length(trajectory_xy)
    closest_obstacle = obstacles_xy[closest_obstacle_index]
    label = classify_avoidance_side(trajectory_xy, closest_obstacle, int(closest_index))
    elapsed_ms = (perf_counter() - started) * 1000.0

    return TrialMetrics(
        sequence=str(sequence),
        obstacle_count=int(len(obstacles_xy)),
        min_center_distance_m=min_center_distance,
        min_boundary_clearance_m=min_clearance,
        closest_time_s=float(time_s[closest_index]),
        closest_index=int(closest_index),
        closest_obstacle_index=int(closest_obstacle_index),
        collision=bool(min_clearance <= 0.0),
        safety_violation=bool(min_clearance < safety_distance_m),
        safety_distance_m=safety_distance_m,
        path_length_m=length,
        duration_s=duration,
        mean_speed_mps=float(length / duration) if duration > 0 else 0.0,
        avoidance_label=label,
        computation_time_ms=elapsed_ms,
    )
