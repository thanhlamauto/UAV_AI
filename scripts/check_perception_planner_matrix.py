#!/usr/bin/env python3
"""Offline matrix check for perception-derived costmaps and planners.

This script avoids ROS imports. It verifies that LiDAR-bbox, depth-derived,
and fused occupancy grids can be consumed by every lightweight planner exposed
by the ROS2 demo: A*, RRT, and MPPI.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Scenario:
    source: str
    grid: np.ndarray
    spec: object
    start: np.ndarray
    goal: np.ndarray


@dataclass(frozen=True)
class MatrixResult:
    source: str
    planner: str
    occupied_cells: int
    width: int
    height: int
    path_waypoints: int
    path_length_m: float
    collision_free_after_inflation: int


def _read_bbox_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(f"Missing bbox CSV: {path}")
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _load_cached_depth_frame(path: Path, frame_index: int = 0) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"Missing cached depth NPZ: {path}")
    cache = np.load(path)
    if "depth_u8" not in cache:
        raise KeyError(f"{path} does not contain depth_u8")
    depth = np.asarray(cache["depth_u8"])
    if depth.ndim != 3:
        raise ValueError(f"Expected depth_u8 with shape [frames,h,w], got {depth.shape}")
    if not (0 <= frame_index < depth.shape[0]):
        raise IndexError(f"Frame index {frame_index} outside depth_u8 length {depth.shape[0]}")
    return depth[frame_index].astype(np.float32)


def _path_length(path: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0


def _path_collision_free(path: np.ndarray, mask: np.ndarray, spec: object) -> bool:
    from uav_oda_ros2_demo.grid_planners import world_to_cell

    if len(path) < 2:
        return False
    step_m = max(float(spec.resolution) * 0.5, 1e-6)
    for a, b in zip(path[:-1], path[1:]):
        dist = float(np.linalg.norm(b - a))
        steps = max(2, int(np.ceil(dist / step_m)))
        for alpha in np.linspace(0.0, 1.0, steps):
            cell = world_to_cell((1.0 - alpha) * a + alpha * b, spec)
            if not (0 <= cell[0] < mask.shape[0] and 0 <= cell[1] < mask.shape[1]):
                return False
            if not bool(mask[cell]):
                return False
    return True


def _build_scenarios(bbox_csv: Path, depth_cache: Path, depth_frame_index: int) -> list[Scenario]:
    from uav_oda_ros2_demo.costmap_converters import (
        DepthProjectionConfig,
        bbox_rows_to_grid,
        depth_image_to_grid,
        merge_occupancy_grids,
        select_bbox_rows,
    )

    rows = select_bbox_rows(_read_bbox_rows(bbox_csv), frame_offset=0, min_point_count=50)
    bbox_grid, bbox_spec = bbox_rows_to_grid(rows, resolution_m=0.20, margin_m=1.0)
    bbox_start = np.asarray([bbox_spec.origin_x + 0.4, 0.0], dtype=float)
    bbox_goal = np.asarray([bbox_spec.origin_x + (bbox_spec.width - 2) * bbox_spec.resolution, 4.0], dtype=float)

    depth_config = DepthProjectionConfig(resolution_m=0.10, sample_stride_px=3, hit_dilation_cells=2)
    metric_depth = np.full((96, 160), 7.5, dtype=np.float32)
    metric_depth[38:72, 64:91] = 2.0
    metric_depth[44:78, 105:132] = 3.6
    metric_grid, metric_spec, _ = depth_image_to_grid(metric_depth, "32FC1", depth_config)
    depth_start = np.asarray([0.0, 0.0], dtype=float)
    depth_goal = np.asarray([6.0, 0.0], dtype=float)

    relative_depth = np.full((96, 160), 20, dtype=np.float32)
    relative_depth[38:72, 64:91] = 250
    relative_depth[44:78, 105:132] = 210
    relative_grid, relative_spec, _ = depth_image_to_grid(relative_depth, "mono8", depth_config)

    mux_grid, mux_spec = merge_occupancy_grids(
        [(bbox_grid, bbox_spec), (relative_grid, relative_spec)],
        occupied_threshold=50,
        resolution_m=0.20,
        padding_m=0.25,
    )
    mux_start = np.asarray([0.0, 0.0], dtype=float)
    mux_goal = bbox_goal

    cached_depth = _load_cached_depth_frame(depth_cache, depth_frame_index)
    cached_grid, cached_spec, _ = depth_image_to_grid(cached_depth, "mono8", depth_config)
    cached_mux_grid, cached_mux_spec = merge_occupancy_grids(
        [(bbox_grid, bbox_spec), (cached_grid, cached_spec)],
        occupied_threshold=50,
        resolution_m=0.20,
        padding_m=0.25,
    )

    return [
        Scenario("lidar_bbox_csv", bbox_grid, bbox_spec, bbox_start, bbox_goal),
        Scenario("metric_depth_image", metric_grid, metric_spec, depth_start, depth_goal),
        Scenario("relative_predicted_depth_proxy", relative_grid, relative_spec, depth_start, depth_goal),
        Scenario("lidar_bbox_plus_relative_depth_mux", mux_grid, mux_spec, mux_start, mux_goal),
        Scenario("lidar_bbox_plus_cached_depth_mux", cached_mux_grid, cached_mux_spec, mux_start, mux_goal),
    ]


def _write_csv(results: list[MatrixResult], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(MatrixResult.__dataclass_fields__))
        writer.writeheader()
        for result in results:
            writer.writerow(result.__dict__)


def _write_markdown(results: list[MatrixResult], output_md: Path, output_csv: Path) -> None:
    output_md.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Perception Planner Matrix",
        "",
        "This offline check verifies that every perception-derived obstacle map used by the ROS2 demo can feed every lightweight planner.",
        "",
        f"CSV: `{output_csv}`",
        "",
        "| Source | Planner | Grid | Occupied | Waypoints | Length m | Collision-free |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for result in results:
        lines.append(
            f"| {result.source} | {result.planner} | {result.width}x{result.height} | "
            f"{result.occupied_cells} | {result.path_waypoints} | {result.path_length_m:.2f} | "
            f"{result.collision_free_after_inflation} |"
        )
    lines.extend(
        [
            "",
            "`Collision-free` is checked after applying the same robot-radius plus safety-distance inflation used by the planner.",
            "This does not replace ROS2/Gazebo runtime evidence; it is a local contract check before server execution.",
            "",
        ]
    )
    output_md.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bbox-csv",
        type=Path,
        default=Path("outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv"),
    )
    parser.add_argument("--depth-cache", type=Path, default=Path("data/processed/depth_sample_3_5fps.npz"))
    parser.add_argument("--depth-frame-index", type=int, default=0)
    parser.add_argument("--output-csv", type=Path, default=Path("outputs/tables/perception_planner_matrix.csv"))
    parser.add_argument("--output-md", type=Path, default=Path("outputs/perception_planner_matrix.md"))
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    ros_pkg = repo_root / "ros2_ws" / "src" / "uav_oda_ros2_demo"
    sys.path.insert(0, str(ros_pkg))

    from uav_oda_ros2_demo.grid_planners import PlannerConfig, free_mask, inflate_grid, plan_path

    config = PlannerConfig(
        robot_radius_m=0.10,
        safety_distance_m=0.15,
        rrt_max_iterations=900,
        rrt_step_size_m=0.35,
        rrt_goal_sample_rate=0.25,
        mppi_rollouts=48,
        mppi_iterations=2,
        seed=23,
    )
    planners = ["astar", "rrt", "mppi"]
    results: list[MatrixResult] = []

    for scenario in _build_scenarios(args.bbox_csv, args.depth_cache, args.depth_frame_index):
        inflated = inflate_grid(scenario.grid, scenario.spec, config.inflation_radius_m, config.occupied_threshold)
        mask = free_mask(inflated, config)
        for planner in planners:
            path = plan_path(planner, scenario.grid, scenario.spec, scenario.start, scenario.goal, config)
            collision_free = _path_collision_free(path, mask, scenario.spec)
            result = MatrixResult(
                source=scenario.source,
                planner=planner,
                occupied_cells=int((scenario.grid >= config.occupied_threshold).sum()),
                width=int(scenario.spec.width),
                height=int(scenario.spec.height),
                path_waypoints=len(path),
                path_length_m=_path_length(path),
                collision_free_after_inflation=int(collision_free),
            )
            if result.occupied_cells <= 0:
                raise RuntimeError(f"{scenario.source} produced no occupied cells")
            if result.path_waypoints < 2:
                raise RuntimeError(f"{scenario.source}/{planner} produced invalid path")
            if not collision_free:
                raise RuntimeError(f"{scenario.source}/{planner} path intersects inflated occupied cells")
            results.append(result)

    _write_csv(results, args.output_csv)
    _write_markdown(results, args.output_md, args.output_csv)
    for result in results:
        print(
            f"{result.source}/{result.planner}: grid={result.width}x{result.height} "
            f"occupied={result.occupied_cells} path_waypoints={result.path_waypoints} "
            f"path_length_m={result.path_length_m:.2f} collision_free={result.collision_free_after_inflation}"
        )
    print(f"Wrote {args.output_csv} and {args.output_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
