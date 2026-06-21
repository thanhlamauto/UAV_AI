"""Matplotlib visualizations for ODA ground-truth trajectories."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from .metrics import TrialMetrics, pairwise_ground_distances


def plot_trial_ground_plane(
    sequence: str,
    time_s: np.ndarray,
    trajectory_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    metrics: TrialMetrics,
    output_path: str | Path,
    obstacle_radius_m: float = 0.20,
    safety_distance_m: float = 0.50,
) -> Path:
    """Save a trajectory plus obstacle plot with a distance trace."""

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    distances = pairwise_ground_distances(trajectory_xy, obstacles_xy)
    nearest_distance = distances.min(axis=1)
    nearest_clearance = nearest_distance - obstacle_radius_m

    fig = plt.figure(figsize=(10, 7), constrained_layout=True)
    grid = fig.add_gridspec(2, 1, height_ratios=[2.0, 1.0])

    ax = fig.add_subplot(grid[0])
    ax.plot(
        trajectory_xy[:, 0],
        trajectory_xy[:, 1],
        color="#1f77b4",
        linewidth=2.0,
        label="MAV ground-truth trajectory",
    )
    ax.scatter(
        trajectory_xy[0, 0],
        trajectory_xy[0, 1],
        color="#2ca02c",
        s=60,
        zorder=4,
        label="start",
    )
    ax.scatter(
        trajectory_xy[-1, 0],
        trajectory_xy[-1, 1],
        color="#111111",
        s=60,
        zorder=4,
        label="end",
    )
    ax.scatter(
        trajectory_xy[metrics.closest_index, 0],
        trajectory_xy[metrics.closest_index, 1],
        color="#ff7f0e",
        s=70,
        zorder=5,
        label="closest approach",
    )

    for idx, obstacle in enumerate(obstacles_xy):
        obstacle_circle = plt.Circle(
            obstacle,
            obstacle_radius_m,
            color="#d62728",
            alpha=0.35,
            label="obstacle radius" if idx == 0 else None,
        )
        safety_circle = plt.Circle(
            obstacle,
            obstacle_radius_m + safety_distance_m,
            fill=False,
            color="#d62728",
            linestyle="--",
            linewidth=1.4,
            alpha=0.85,
            label="safety boundary" if idx == 0 else None,
        )
        ax.add_patch(obstacle_circle)
        ax.add_patch(safety_circle)
        ax.text(
            obstacle[0],
            obstacle[1] + obstacle_radius_m + 0.08,
            f"obs {idx}",
            ha="center",
            va="bottom",
            fontsize=9,
            color="#7f1d1d",
        )

    ax.set_title(f"ODA sample {sequence}: MAV trajectory with obstacle position")
    ax.set_xlabel("OptiTrack x [m]")
    ax.set_ylabel("OptiTrack z [m] ground-plane axis")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=9)

    pad = 0.8
    all_xy = np.vstack([trajectory_xy, obstacles_xy])
    ax.set_xlim(all_xy[:, 0].min() - pad, all_xy[:, 0].max() + pad)
    ax.set_ylim(all_xy[:, 1].min() - pad, all_xy[:, 1].max() + pad)

    ax_dist = fig.add_subplot(grid[1])
    ax_dist.plot(time_s, nearest_clearance, color="#9467bd", linewidth=1.8)
    ax_dist.axhline(0.0, color="#d62728", linewidth=1.2, label="collision boundary")
    ax_dist.axhline(
        safety_distance_m,
        color="#ff7f0e",
        linestyle="--",
        linewidth=1.2,
        label=f"safety clearance {safety_distance_m:.2f} m",
    )
    ax_dist.scatter(
        [metrics.closest_time_s],
        [metrics.min_boundary_clearance_m],
        color="#ff7f0e",
        zorder=4,
    )
    ax_dist.set_title("Nearest obstacle boundary clearance over time")
    ax_dist.set_xlabel("time [s]")
    ax_dist.set_ylabel("clearance [m]")
    ax_dist.grid(True, alpha=0.25)
    ax_dist.legend(loc="best", fontsize=9)

    fig.savefig(output, dpi=180)
    plt.close(fig)
    return output
