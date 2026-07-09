#!/usr/bin/env python3
"""Generate a compact input/process/output example for planner explanation."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.metrics import pairwise_ground_distances, path_length
from src.planners.astar import AStarConfig, _build_occupancy, _make_bounds, astar_path
from src.planners.mppi import MPPIConfig, mppi_path
from src.planners.rrt import RRTConfig, rrt_path
from src.planners.rrt_star import RRTStarConfig, rrt_star_path


def _clearance(path_xy: np.ndarray, obstacles_xy: np.ndarray, obstacle_radius_m: float) -> float:
    return float(np.min(pairwise_ground_distances(path_xy, obstacles_xy)) - obstacle_radius_m)


def _short_points(points: np.ndarray, max_points: int = 8) -> str:
    if len(points) <= max_points:
        chosen = points
    else:
        head = max_points // 2
        tail = max_points - head
        chosen = np.vstack([points[:head], points[-tail:]])
    return " -> ".join(f"({x:.2f},{y:.2f})" for x, y in chosen)


def main() -> int:
    output_md = Path("outputs/planner_io_example.md")
    output_csv = Path("outputs/tables/planner_io_example.csv")
    output_md.parent.mkdir(parents=True, exist_ok=True)
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([6.4, 0.0], dtype=float)
    obstacles = np.asarray([[2.0, 1.0], [4.2, -0.8]], dtype=float)
    obstacle_radius_m = 0.35
    safety_distance_m = 0.50
    inflated_radius_m = obstacle_radius_m + safety_distance_m

    astar_config = AStarConfig(
        resolution_m=0.10,
        margin_m=1.00,
        obstacle_radius_m=obstacle_radius_m,
        safety_distance_m=safety_distance_m,
    )
    rrt_config = RRTConfig(
        max_iterations=2200,
        step_size_m=0.35,
        goal_sample_rate=0.20,
        margin_m=1.00,
        obstacle_radius_m=obstacle_radius_m,
        safety_distance_m=safety_distance_m,
        seed=23,
    )
    rrt_star_config = RRTStarConfig(
        max_iterations=2500,
        step_size_m=0.30,
        neighbor_radius_m=0.75,
        goal_sample_rate=0.15,
        margin_m=1.00,
        obstacle_radius_m=obstacle_radius_m,
        safety_distance_m=safety_distance_m,
        seed=23,
    )
    mppi_config = MPPIConfig(
        num_rollouts=512,
        horizon_steps=60,
        max_iterations=10,
        noise_sigma_m=0.30,
        obstacle_radius_m=obstacle_radius_m,
        safety_distance_m=safety_distance_m,
        seed=23,
    )

    planners = [
        (
            "A*",
            "Build occupancy grid, inflate obstacles by obstacle radius + safety distance, search 8-neighbor cells with g+h cost, reconstruct cell path.",
            astar_path(start, goal, obstacles, astar_config, num_points=200),
        ),
        (
            "RRT",
            "Sample continuous points, steer from nearest tree node, reject nodes/edges that intersect inflated obstacles, shortcut the first found path.",
            rrt_path(start, goal, obstacles, rrt_config, num_points=200),
        ),
        (
            "RRT*",
            "Sample continuous points like RRT, choose lower-cost parent from neighbors, rewire nearby nodes, keep the best connection to the goal.",
            rrt_star_path(start, goal, obstacles, rrt_star_config, num_points=200),
        ),
        (
            "MPPI",
            "Start from a geometric bypass path, sample noisy trajectory rollouts, score length/smoothness/clearance/collision, update the mean path by cost-weighted noise.",
            mppi_path(start, goal, obstacles, mppi_config, num_points=200),
        ),
    ]

    rows: list[dict[str, object]] = []
    for name, process, planned in planners:
        rows.append(
            {
                "planner": name,
                "input": f"start={start.tolist()}, goal={goal.tolist()}, obstacles={obstacles.tolist()}, obstacle_radius={obstacle_radius_m}, safety_distance={safety_distance_m}",
                "process": process,
                "waypoints": len(planned.waypoints),
                "trajectory_points": len(planned.trajectory_xy),
                "path_length_m": round(path_length(planned.trajectory_xy), 4),
                "min_clearance_m": round(_clearance(planned.trajectory_xy, obstacles, obstacle_radius_m), 4),
                "safety_violation": int(_clearance(planned.trajectory_xy, obstacles, obstacle_radius_m) < safety_distance_m),
                "output_preview": _short_points(planned.waypoints),
            }
        )

    xy_min, xy_max = _make_bounds(start, goal, obstacles, astar_config.margin_m)
    occupied = _build_occupancy(xy_min, xy_max, obstacles, astar_config)
    occupied_cells = int(occupied.sum())
    grid_shape = f"{occupied.shape[0]} x {occupied.shape[1]}"

    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# Planner Input/Output Example",
        "",
        "This example uses one simple 2D obstacle-avoidance scene so the planner inputs and outputs can be explained in a defense slide.",
        "",
        "## Common Input",
        "",
        f"- Start: `{start.tolist()}`",
        f"- Goal: `{goal.tolist()}`",
        f"- Obstacles: `{obstacles.tolist()}`",
        f"- Obstacle radius: `{obstacle_radius_m:.2f} m`",
        f"- Safety distance: `{safety_distance_m:.2f} m`",
        f"- Inflated obstacle radius used for collision checking: `{inflated_radius_m:.2f} m`",
        f"- A* occupancy grid for this example: `{grid_shape}` cells, `{occupied_cells}` occupied cells before safety inflation in the A* implementation.",
        "",
        "## Planner Outputs",
        "",
        "| Planner | Process | Waypoints | Path length m | Min clearance m | Safety violation | Output preview |",
        "|---|---|---:|---:|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            f"| {row['planner']} | {row['process']} | {row['waypoints']} | "
            f"{row['path_length_m']:.4f} | {row['min_clearance_m']:.4f} | "
            f"{row['safety_violation']} | `{row['output_preview']}` |"
        )
    lines.extend(
        [
            "",
            "## How To Explain It",
            "",
            "- A* output is a grid-cell path, so it is useful as a geometric baseline but can look angular.",
            "- RRT/RRT* output is a sampled continuous waypoint path; it still needs smoothing/control before real UAV flight.",
            "- MPPI output is an optimized trajectory-style path; in a full controller it would optimize control rollouts rather than only waypoints.",
            "- In all cases, the planner output is evaluated by the same safety metrics: path length, minimum clearance, collision and safety-distance violation.",
            "",
            f"CSV: `{output_csv}`",
            "",
        ]
    )
    output_md.write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {output_md} and {output_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
