#!/usr/bin/env python3
"""Render an SVG proof figure for perception -> costmap -> planner."""

from __future__ import annotations

import argparse
import csv
import html
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Scenario:
    title: str
    source: str
    grid: np.ndarray
    origin_x: float
    origin_y: float
    resolution: float
    start: np.ndarray
    goal: np.ndarray
    path: np.ndarray


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


def _build_scenarios(repo_root: Path, bbox_csv: Path, depth_cache: Path, depth_frame_index: int) -> list[Scenario]:
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

    scenarios: list[Scenario] = []

    rows = select_bbox_rows(_read_bbox_rows(bbox_csv), frame_offset=0, min_point_count=50)
    bbox_grid, bbox_spec = bbox_rows_to_grid(rows, resolution_m=0.20, margin_m=1.0)
    bbox_start = np.asarray([bbox_spec.origin_x + 0.4, 0.0], dtype=float)
    bbox_goal = np.asarray([bbox_spec.origin_x + (bbox_spec.width - 2) * bbox_spec.resolution, 4.0], dtype=float)
    bbox_path = plan_path("astar", bbox_grid, bbox_spec, bbox_start, bbox_goal, planner_config)
    scenarios.append(
        Scenario(
            "LiDAR bbox CSV",
            "Multi-LiDAR 3D boxes",
            bbox_grid,
            bbox_spec.origin_x,
            bbox_spec.origin_y,
            bbox_spec.resolution,
            bbox_start,
            bbox_goal,
            bbox_path,
        )
    )

    depth_config = DepthProjectionConfig(resolution_m=0.10, sample_stride_px=3, hit_dilation_cells=2)
    metric_depth = np.full((96, 160), 7.5, dtype=np.float32)
    metric_depth[38:72, 64:91] = 2.0
    metric_depth[44:78, 105:132] = 3.6
    metric_grid, metric_spec, _ = depth_image_to_grid(metric_depth, "32FC1", depth_config)
    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([6.0, 0.0], dtype=float)
    metric_path = plan_path("astar", metric_grid, metric_spec, start, goal, planner_config)
    scenarios.append(
        Scenario(
            "Metric depth image",
            "32FC1 near-depth projection",
            metric_grid,
            metric_spec.origin_x,
            metric_spec.origin_y,
            metric_spec.resolution,
            start,
            goal,
            metric_path,
        )
    )

    relative_depth = np.full((96, 160), 20, dtype=np.float32)
    relative_depth[38:72, 64:91] = 250
    relative_depth[44:78, 105:132] = 210
    relative_grid, relative_spec, _ = depth_image_to_grid(relative_depth, "mono8", depth_config)
    relative_path = plan_path("astar", relative_grid, relative_spec, start, goal, planner_config)
    scenarios.append(
        Scenario(
            "Relative predicted-depth proxy",
            "mono8 inverse-depth projection",
            relative_grid,
            relative_spec.origin_x,
            relative_spec.origin_y,
            relative_spec.resolution,
            start,
            goal,
            relative_path,
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
    scenarios.append(
        Scenario(
            "LiDAR bbox + depth mux",
            "proxy depth + bbox merged costmap",
            mux_grid,
            mux_spec.origin_x,
            mux_spec.origin_y,
            mux_spec.resolution,
            mux_start,
            mux_goal,
            mux_path,
        )
    )

    cached_depth = _load_cached_depth_frame(depth_cache, depth_frame_index)
    cached_grid, cached_spec, _ = depth_image_to_grid(cached_depth, "mono8", depth_config)
    cached_mux_grid, cached_mux_spec = merge_occupancy_grids(
        [(bbox_grid, bbox_spec), (cached_grid, cached_spec)],
        occupied_threshold=50,
        resolution_m=0.20,
        padding_m=0.25,
    )
    cached_mux_path = plan_path("astar", cached_mux_grid, cached_mux_spec, mux_start, mux_goal, planner_config)
    scenarios.append(
        Scenario(
            "LiDAR bbox + cached depth",
            f"depth_sample frame {depth_frame_index} + bbox mux",
            cached_mux_grid,
            cached_mux_spec.origin_x,
            cached_mux_spec.origin_y,
            cached_mux_spec.resolution,
            mux_start,
            mux_goal,
            cached_mux_path,
        )
    )
    return scenarios


def _path_length(path: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0


def _render_panel(scenario: Scenario, x0: int, y0: int, panel_w: int, panel_h: int) -> str:
    title_h = 46
    legend_h = 30
    plot_x = x0 + 18
    plot_y = y0 + title_h
    plot_w = panel_w - 36
    plot_h = panel_h - title_h - legend_h - 16

    grid_h, grid_w = scenario.grid.shape
    world_w = grid_w * scenario.resolution
    world_h = grid_h * scenario.resolution
    scale = min(plot_w / world_w, plot_h / world_h)
    draw_w = world_w * scale
    draw_h = world_h * scale
    ox = plot_x + (plot_w - draw_w) / 2.0
    oy = plot_y + (plot_h - draw_h) / 2.0

    def xy(point: np.ndarray | tuple[float, float]) -> tuple[float, float]:
        px = float(point[0])
        py = float(point[1])
        sx = ox + (px - scenario.origin_x) * scale
        sy = oy + draw_h - (py - scenario.origin_y) * scale
        return sx, sy

    parts = [
        f'<g transform="translate(0,0)">',
        f'<rect x="{x0}" y="{y0}" width="{panel_w}" height="{panel_h}" rx="4" fill="#ffffff" stroke="#d0d7de"/>',
        f'<text x="{x0 + 18}" y="{y0 + 24}" font-size="15" font-weight="700" fill="#1f2328">{html.escape(scenario.title)}</text>',
        f'<text x="{x0 + 18}" y="{y0 + 41}" font-size="11" fill="#57606a">{html.escape(scenario.source)}</text>',
        f'<rect x="{ox:.2f}" y="{oy:.2f}" width="{draw_w:.2f}" height="{draw_h:.2f}" fill="#f6f8fa" stroke="#8c959f" stroke-width="1"/>',
    ]

    occupied = np.argwhere(scenario.grid >= 50)
    cell = scenario.resolution * scale
    for row, col in occupied:
        rx = ox + col * cell
        ry = oy + draw_h - (row + 1) * cell
        parts.append(f'<rect x="{rx:.2f}" y="{ry:.2f}" width="{cell + 0.2:.2f}" height="{cell + 0.2:.2f}" fill="#d1242f" opacity="0.72"/>')

    if len(scenario.path) >= 2:
        pts = " ".join(f"{px:.2f},{py:.2f}" for px, py in (xy(p) for p in scenario.path))
        parts.append(f'<polyline points="{pts}" fill="none" stroke="#0969da" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>')

    sx, sy = xy(scenario.start)
    gx, gy = xy(scenario.goal)
    parts.extend(
        [
            f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="5" fill="#1a7f37" stroke="#ffffff" stroke-width="1.5"/>',
            f'<circle cx="{gx:.2f}" cy="{gy:.2f}" r="5" fill="#8250df" stroke="#ffffff" stroke-width="1.5"/>',
            f'<text x="{x0 + 18}" y="{y0 + panel_h - 24}" font-size="11" fill="#24292f">occupied={(scenario.grid >= 50).sum()} cells, path={len(scenario.path)} wp, length={_path_length(scenario.path):.2f} m</text>',
            f'<text x="{x0 + 18}" y="{y0 + panel_h - 8}" font-size="10" fill="#57606a">green=start, purple=goal, red=occupied, blue=A* path</text>',
            "</g>",
        ]
    )
    return "\n".join(parts)


def render_svg(scenarios: list[Scenario], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    panel_w = 360
    panel_h = 310
    gap = 18
    margin = 24
    width = margin * 2 + panel_w * len(scenarios) + gap * (len(scenarios) - 1)
    height = 390
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f6f8fa"/>',
        f'<text x="{margin}" y="32" font-size="20" font-weight="700" fill="#1f2328">Perception output to obstacle map to planner</text>',
        f'<text x="{margin}" y="54" font-size="12" fill="#57606a">Same converter helpers used by ROS2 nodes; offline proof before ROS2/Gazebo runtime verification.</text>',
    ]
    y0 = 72
    for idx, scenario in enumerate(scenarios):
        x0 = margin + idx * (panel_w + gap)
        parts.append(_render_panel(scenario, x0, y0, panel_w, panel_h))
    parts.append("</svg>")
    output.write_text("\n".join(parts), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bbox-csv", type=Path, default=Path("outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv"))
    parser.add_argument("--depth-cache", type=Path, default=Path("data/processed/depth_sample_3_5fps.npz"))
    parser.add_argument("--depth-frame-index", type=int, default=0)
    parser.add_argument("--output", type=Path, default=Path("outputs/figures/perception_to_planner_contract.svg"))
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    scenarios = _build_scenarios(repo_root, args.bbox_csv, args.depth_cache, args.depth_frame_index)
    render_svg(scenarios, args.output)
    print(f"Wrote {args.output} with {len(scenarios)} panel(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
