"""Risk labels derived from ground-truth obstacle clearance."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.metrics import pairwise_ground_distances


@dataclass(frozen=True)
class RiskLabelSummary:
    safe_count: int
    warning_count: int
    danger_count: int
    collision_count: int
    future_risk_count: int
    total_count: int

    def as_row(self, prefix: str = "") -> dict[str, object]:
        denom = max(1, self.total_count)
        return {
            f"{prefix}safe_count": self.safe_count,
            f"{prefix}warning_count": self.warning_count,
            f"{prefix}danger_count": self.danger_count,
            f"{prefix}collision_count": self.collision_count,
            f"{prefix}future_risk_count": self.future_risk_count,
            f"{prefix}danger_or_collision_rate": round(
                (self.danger_count + self.collision_count) / denom, 4
            ),
            f"{prefix}future_risk_rate": round(self.future_risk_count / denom, 4),
        }


def clearance_series(
    trajectory_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float = 0.20,
) -> np.ndarray:
    distances = pairwise_ground_distances(trajectory_xy, obstacles_xy)
    return distances.min(axis=1) - obstacle_radius_m


def risk_labels_from_clearance(
    clearance_m: np.ndarray,
    warning_clearance_m: float = 0.80,
    danger_clearance_m: float = 0.50,
) -> np.ndarray:
    labels = np.full(len(clearance_m), "safe", dtype=object)
    labels[clearance_m <= warning_clearance_m] = "warning"
    labels[clearance_m <= danger_clearance_m] = "danger"
    labels[clearance_m <= 0.0] = "collision"
    return labels


def future_risk_labels(
    time_s: np.ndarray,
    clearance_m: np.ndarray,
    horizon_s: float = 1.0,
    danger_clearance_m: float = 0.50,
) -> np.ndarray:
    labels = np.zeros(len(clearance_m), dtype=bool)
    end = 0
    for start, t in enumerate(time_s):
        while end < len(time_s) and time_s[end] <= t + horizon_s:
            end += 1
        labels[start] = bool(np.min(clearance_m[start:end]) <= danger_clearance_m)
    return labels


def summarize_risk_labels(labels: np.ndarray, future_risk: np.ndarray) -> RiskLabelSummary:
    return RiskLabelSummary(
        safe_count=int(np.sum(labels == "safe")),
        warning_count=int(np.sum(labels == "warning")),
        danger_count=int(np.sum(labels == "danger")),
        collision_count=int(np.sum(labels == "collision")),
        future_risk_count=int(np.sum(future_risk)),
        total_count=int(len(labels)),
    )
