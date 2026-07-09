#!/usr/bin/env python3
"""Offline contract check for perception output -> costmap -> planner.

This intentionally avoids ROS imports.  It verifies that the same pure
converters used by ROS2 nodes can turn LiDAR bbox and depth-derived obstacles
into occupancy grids consumed by the A* planner.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class ContractResult:
    source: str
    occupied_cells: int
    width: int
    height: int
    start_x: float
    start_y: float
    goal_x: float
    goal_y: float
    path_waypoints: int
    path_length_m: float


def _path_length(path: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0


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


def _write_csv(results: list[ContractResult], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(ContractResult.__dataclass_fields__))
        writer.writeheader()
        for result in results:
            writer.writerow(result.__dict__)


def _write_markdown(results: list[ContractResult], output_md: Path, output_csv: Path) -> None:
    output_md.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Perception-to-Planner Contract Check",
        "",
        "This is an offline, non-ROS check. It proves that the same converter helpers used by ROS2 nodes can produce a non-empty `OccupancyGrid`-style array and that the planner can consume it.",
        "",
        f"CSV: `{output_csv}`",
        "Figure: `outputs/figures/perception_to_planner_contract.svg`",
        "",
        "| Source | Grid | Occupied cells | Path waypoints | Path length m |",
        "|---|---:|---:|---:|---:|",
    ]
    for result in results:
        lines.append(
            f"| {result.source} | {result.width}x{result.height} | "
            f"{result.occupied_cells} | {result.path_waypoints} | {result.path_length_m:.2f} |"
        )
    lines.extend(
        [
            "",
            "Passing this check does not replace ROS2/Gazebo runtime verification. It only verifies the local conversion/planning contract before server execution.",
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
    parser.add_argument("--output-csv", type=Path, default=Path("outputs/tables/perception_to_planner_contract.csv"))
    parser.add_argument("--output-md", type=Path, default=Path("outputs/perception_to_planner_contract.md"))
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    ros_pkg = repo_root / "ros2_ws" / "src" / "uav_oda_ros2_demo"
    sys.path.insert(0, str(ros_pkg))

    from uav_oda_ros2_demo.costmap_converters import (
        DepthProjectionConfig,
        bbox_rows_to_grid,
        depth_image_to_grid,
        merge_occupancy_grids,
        select_bbox_rows,
    )
    from uav_oda_ros2_demo.grid_planners import PlannerConfig, plan_path

    planner_config = PlannerConfig(
        robot_radius_m=0.10,
        safety_distance_m=0.15,
        rrt_max_iterations=800,
        mppi_rollouts=48,
        mppi_iterations=2,
        seed=23,
    )
    results: list[ContractResult] = []

    rows = select_bbox_rows(_read_bbox_rows(args.bbox_csv), frame_offset=0, min_point_count=50)
    bbox_grid, bbox_spec = bbox_rows_to_grid(rows, resolution_m=0.20, margin_m=1.0)
    bbox_start = np.asarray([bbox_spec.origin_x + 0.4, 0.0], dtype=float)
    bbox_goal = np.asarray([bbox_spec.origin_x + (bbox_spec.width - 2) * bbox_spec.resolution, 4.0], dtype=float)
    bbox_path = plan_path("astar", bbox_grid, bbox_spec, bbox_start, bbox_goal, planner_config)
    results.append(
        ContractResult(
            source="lidar_bbox_csv",
            occupied_cells=int((bbox_grid >= 50).sum()),
            width=bbox_spec.width,
            height=bbox_spec.height,
            start_x=float(bbox_start[0]),
            start_y=float(bbox_start[1]),
            goal_x=float(bbox_goal[0]),
            goal_y=float(bbox_goal[1]),
            path_waypoints=len(bbox_path),
            path_length_m=_path_length(bbox_path),
        )
    )

    depth_config = DepthProjectionConfig(resolution_m=0.10, sample_stride_px=3, hit_dilation_cells=2)
    metric_depth = np.full((96, 160), 7.5, dtype=np.float32)
    metric_depth[38:72, 64:91] = 2.0
    metric_depth[44:78, 105:132] = 3.6
    metric_grid, metric_spec, _ = depth_image_to_grid(metric_depth, "32FC1", depth_config)
    metric_start = np.asarray([0.0, 0.0], dtype=float)
    metric_goal = np.asarray([6.0, 0.0], dtype=float)
    metric_path = plan_path("astar", metric_grid, metric_spec, metric_start, metric_goal, planner_config)
    results.append(
        ContractResult(
            source="metric_depth_image",
            occupied_cells=int((metric_grid >= 50).sum()),
            width=metric_spec.width,
            height=metric_spec.height,
            start_x=float(metric_start[0]),
            start_y=float(metric_start[1]),
            goal_x=float(metric_goal[0]),
            goal_y=float(metric_goal[1]),
            path_waypoints=len(metric_path),
            path_length_m=_path_length(metric_path),
        )
    )

    relative_depth = np.full((96, 160), 20, dtype=np.float32)
    relative_depth[38:72, 64:91] = 250
    relative_depth[44:78, 105:132] = 210
    relative_grid, relative_spec, _ = depth_image_to_grid(relative_depth, "mono8", depth_config)
    relative_path = plan_path("astar", relative_grid, relative_spec, metric_start, metric_goal, planner_config)
    results.append(
        ContractResult(
            source="relative_predicted_depth_proxy",
            occupied_cells=int((relative_grid >= 50).sum()),
            width=relative_spec.width,
            height=relative_spec.height,
            start_x=float(metric_start[0]),
            start_y=float(metric_start[1]),
            goal_x=float(metric_goal[0]),
            goal_y=float(metric_goal[1]),
            path_waypoints=len(relative_path),
            path_length_m=_path_length(relative_path),
        )
    )

    mux_grid, mux_spec = merge_occupancy_grids(
        [(bbox_grid, bbox_spec), (relative_grid, relative_spec)],
        occupied_threshold=50,
        resolution_m=0.20,
        padding_m=0.25,
    )
    mux_start = np.asarray([0.0, 0.0], dtype=float)
    mux_goal = bbox_goal
    mux_path = plan_path("astar", mux_grid, mux_spec, mux_start, mux_goal, planner_config)
    results.append(
        ContractResult(
            source="lidar_bbox_plus_relative_depth_mux",
            occupied_cells=int((mux_grid >= 50).sum()),
            width=mux_spec.width,
            height=mux_spec.height,
            start_x=float(mux_start[0]),
            start_y=float(mux_start[1]),
            goal_x=float(mux_goal[0]),
            goal_y=float(mux_goal[1]),
            path_waypoints=len(mux_path),
            path_length_m=_path_length(mux_path),
        )
    )

    cached_depth = _load_cached_depth_frame(args.depth_cache, args.depth_frame_index)
    cached_grid, cached_spec, _ = depth_image_to_grid(cached_depth, "mono8", depth_config)
    cached_mux_grid, cached_mux_spec = merge_occupancy_grids(
        [(bbox_grid, bbox_spec), (cached_grid, cached_spec)],
        occupied_threshold=50,
        resolution_m=0.20,
        padding_m=0.25,
    )
    cached_mux_path = plan_path("astar", cached_mux_grid, cached_mux_spec, mux_start, mux_goal, planner_config)
    results.append(
        ContractResult(
            source="lidar_bbox_plus_cached_depth_mux",
            occupied_cells=int((cached_mux_grid >= 50).sum()),
            width=cached_mux_spec.width,
            height=cached_mux_spec.height,
            start_x=float(mux_start[0]),
            start_y=float(mux_start[1]),
            goal_x=float(mux_goal[0]),
            goal_y=float(mux_goal[1]),
            path_waypoints=len(cached_mux_path),
            path_length_m=_path_length(cached_mux_path),
        )
    )

    for result in results:
        if result.occupied_cells <= 0:
            raise RuntimeError(f"{result.source} produced no occupied cells")
        if result.path_waypoints < 2:
            raise RuntimeError(f"{result.source} produced invalid path")

    _write_csv(results, args.output_csv)
    _write_markdown(results, args.output_md, args.output_csv)
    for result in results:
        print(
            f"{result.source}: grid={result.width}x{result.height} "
            f"occupied={result.occupied_cells} path_waypoints={result.path_waypoints} "
            f"path_length_m={result.path_length_m:.2f}"
        )
    print(f"Wrote {args.output_csv} and {args.output_md}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
