#!/usr/bin/env python3
"""Export a lightweight 3D simulation scene for the UAV planner demo.

The generated JSON is consumed by `sim3d/index.html`.  It uses the same
pure-Python occupancy-grid planner helpers as the ROS2 costmap bridge, so the
browser demo stays aligned with the planner pipeline while remaining easy to
run on a laptop.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
ROS_PKG = REPO_ROOT / "ros2_ws" / "src" / "uav_oda_ros2_demo"
sys.path.insert(0, str(ROS_PKG))

from uav_oda_ros2_demo.grid_planners import GridSpec, PlannerConfig, inflate_grid, plan_path  # noqa: E402


OBSTACLES: list[dict[str, Any]] = [
    {
        "id": "pillar_1",
        "type": "cylinder",
        "label": "ODA-style pillar",
        "center": [2.0, 1.0],
        "radius": 0.35,
        "height": 2.4,
    },
    {
        "id": "box_1",
        "type": "box",
        "label": "LiDAR-detected bbox",
        "center": [4.2, -0.8],
        "size": [0.9, 0.75],
        "height": 1.9,
    },
]


def _mark_cylinder(grid: np.ndarray, spec: GridSpec, center: list[float], radius: float) -> None:
    cx, cy = center
    for row in range(grid.shape[0]):
        y = spec.origin_y + row * spec.resolution
        for col in range(grid.shape[1]):
            x = spec.origin_x + col * spec.resolution
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius**2:
                grid[row, col] = 100


def _mark_box(grid: np.ndarray, spec: GridSpec, center: list[float], size: list[float]) -> None:
    cx, cy = center
    sx, sy = size
    x0, x1 = cx - sx / 2.0, cx + sx / 2.0
    y0, y1 = cy - sy / 2.0, cy + sy / 2.0
    for row in range(grid.shape[0]):
        y = spec.origin_y + row * spec.resolution
        for col in range(grid.shape[1]):
            x = spec.origin_x + col * spec.resolution
            if x0 <= x <= x1 and y0 <= y <= y1:
                grid[row, col] = 100


def build_grid(spec: GridSpec) -> np.ndarray:
    grid = np.zeros((spec.height, spec.width), dtype=np.int8)
    for obstacle in OBSTACLES:
        if obstacle["type"] == "cylinder":
            _mark_cylinder(grid, spec, obstacle["center"], obstacle["radius"])
        elif obstacle["type"] == "box":
            _mark_box(grid, spec, obstacle["center"], obstacle["size"])
    return grid


def _cell_centers(mask: np.ndarray, spec: GridSpec, limit: int = 5000) -> list[list[float]]:
    rows, cols = np.where(mask)
    cells = [
        [round(float(spec.origin_x + col * spec.resolution), 3), round(float(spec.origin_y + row * spec.resolution), 3)]
        for row, col in zip(rows.tolist(), cols.tolist())
    ]
    if len(cells) <= limit:
        return cells
    step = max(1, math.ceil(len(cells) / limit))
    return cells[::step]


def _point_clearance(point: np.ndarray, obstacle: dict[str, Any]) -> float:
    cx, cy = obstacle["center"]
    if obstacle["type"] == "cylinder":
        return float(np.linalg.norm(point - np.asarray([cx, cy], dtype=float)) - obstacle["radius"])
    sx, sy = obstacle["size"]
    dx = abs(float(point[0]) - cx) - sx / 2.0
    dy = abs(float(point[1]) - cy) - sy / 2.0
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    inside = min(max(dx, dy), 0.0)
    return outside + inside


def _path_metrics(path: np.ndarray, safety_distance_m: float) -> dict[str, float | int | bool]:
    if len(path) <= 1:
        length = 0.0
    else:
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    clearances = [min(_point_clearance(point, obstacle) for obstacle in OBSTACLES) for point in path]
    min_clearance = float(min(clearances)) if clearances else float("nan")
    return {
        "waypoint_count": int(len(path)),
        "path_length_m": round(length, 4),
        "min_clearance_m": round(min_clearance, 4),
        "collision": bool(min_clearance < 0.0),
        "safety_violation": bool(min_clearance < safety_distance_m),
    }


def _benchmark_summary() -> dict[str, dict[str, Any]]:
    path = REPO_ROOT / "outputs" / "tables" / "planner_comparison_summary_300.csv"
    if not path.exists():
        return {}
    wanted = {"astar_grid": "astar", "rrt": "rrt", "mppi": "mppi", "rrt_star": "rrt_star", "human": "human"}
    out: dict[str, dict[str, Any]] = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            key = wanted.get(row.get("method", ""))
            if not key:
                continue
            out[key] = {
                "trials": int(float(row["trials"])),
                "collision_rate": float(row["collision_rate"]),
                "safety_violation_rate": float(row["safety_violation_rate"]),
                "mean_min_clearance_m": float(row["mean_min_clearance_m"]),
                "mean_path_length_m": float(row["mean_path_length_m"]),
                "mean_compute_ms": float(row["mean_planner_compute_time_ms"]),
            }
    return out


def export_scene(output: Path) -> None:
    spec = GridSpec(width=120, height=90, resolution=0.10, origin_x=-1.0, origin_y=-4.5)
    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([6.4, 0.0], dtype=float)
    safety_distance_m = 0.50
    config = PlannerConfig(
        robot_radius_m=0.18,
        safety_distance_m=safety_distance_m,
        rrt_max_iterations=2200,
        rrt_step_size_m=0.35,
        rrt_goal_sample_rate=0.20,
        mppi_rollouts=384,
        mppi_iterations=8,
        seed=23,
    )
    grid = build_grid(spec)
    inflated = inflate_grid(grid, spec, config.inflation_radius_m, config.occupied_threshold)

    planners: dict[str, dict[str, Any]] = {}
    for planner in ["astar", "rrt", "mppi"]:
        try:
            path = plan_path(planner, grid, spec, start, goal, config)
            planners[planner] = {
                "status": "ok",
                "path": [[round(float(x), 4), round(float(y), 4)] for x, y in path],
                "metrics": _path_metrics(path, safety_distance_m),
            }
        except Exception as exc:  # noqa: BLE001 - serialize demo failures.
            planners[planner] = {"status": "failed", "error": str(exc), "path": [], "metrics": {}}

    scene = {
        "title": "UAV 3D obstacle-avoidance simulation",
        "scope": "lightweight_browser_simulation_not_gazebo",
        "coordinate_frame": "x-z floor plane; y is altitude",
        "grid": {
            "width": spec.width,
            "height": spec.height,
            "resolution_m": spec.resolution,
            "origin": [spec.origin_x, spec.origin_y],
            "occupied_cells": _cell_centers(grid >= config.occupied_threshold, spec),
            "inflated_cells": _cell_centers((inflated >= config.occupied_threshold) & (grid < config.occupied_threshold), spec),
        },
        "uav": {"radius_m": config.robot_radius_m, "nominal_altitude_m": 1.2, "speed_mps": 0.85},
        "safety_distance_m": safety_distance_m,
        "start": start.tolist(),
        "goal": goal.tolist(),
        "obstacles": OBSTACLES,
        "planners": planners,
        "benchmark_summary_300": _benchmark_summary(),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(scene, indent=2), encoding="utf-8")
    print(f"Wrote {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("sim3d/uav_sim_data.json"))
    args = parser.parse_args()
    export_scene(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
