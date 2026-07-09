#!/usr/bin/env python3
"""Benchmark sensor-to-occupancy frontend latency against MPPI feasibility.

This complements `benchmark_online_latency_feasibility.py`.  Instead of assuming
an ideal occupancy/ESDF frontend, it measures or reuses measured compute time for
the repository's available sensor-to-occupancy adapters, then plugs that frontend
latency into the same kinematic-delay feasibility check.

Scope:
- ODA RGB/depth: measured Depth Anything V2 Small replay timing + depth-to-grid.
- ODA cached depth: depth-to-grid only, useful for separating postprocess cost.
- External Multi-LiDAR bbox: bbox-to-grid only, because ODA has no LiDAR.
- Radar/IMU are not included because the current ODA pipeline extracts risk
  features from them; it does not produce an occupancy grid from radar/IMU.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROS_PKG = REPO_ROOT / "ros2_ws" / "src" / "uav_oda_ros2_demo"
if str(ROS_PKG) not in sys.path:
    sys.path.insert(0, str(ROS_PKG))

from experiments.benchmark_online_latency_feasibility import (  # noqa: E402
    Scenario,
    _combined_trajectory,
    _first_range_below_threshold,
    _map_update_samples,
    _run_planner,
)
from uav_oda_ros2_demo.costmap_converters import (  # noqa: E402
    DepthProjectionConfig,
    bbox_rows_to_grid,
    depth_image_to_grid,
    merge_occupancy_grids,
    select_bbox_rows,
)


@dataclass(frozen=True)
class FrontendProfile:
    name: str
    source: str
    algorithm: str
    sensor_period_ms: float
    frontend_ms: float
    occupied_cells: int
    grid_shape: str
    note: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--resolution", type=float, default=0.15)
    parser.add_argument("--prefer-scipy", action="store_true")
    parser.add_argument("--control-publish-ms", type=float, default=10.0)
    parser.add_argument("--body-radius", type=float, default=0.20)
    parser.add_argument("--safety-radius", type=float, default=0.45)
    parser.add_argument("--map-repeats", type=int, default=7)
    parser.add_argument("--timing-repeats", type=int, default=9)
    parser.add_argument("--depth-cache", type=Path, default=Path("data/processed/depth_sample_3_5fps.npz"))
    parser.add_argument(
        "--depth-timing-csv",
        type=Path,
        default=Path("outputs/tables/depth_batch_timing_depth_anything_v2_small_50.csv"),
    )
    parser.add_argument(
        "--bbox-csv",
        type=Path,
        default=Path("outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv"),
    )
    return parser.parse_args()


def _read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _weighted_depth_wall_ms(path: Path) -> float:
    rows = _read_csv_rows(path)
    frames = sum(int(float(row["frames"])) for row in rows)
    wall_s = sum(float(row["wall_seconds"]) for row in rows)
    if frames <= 0:
        raise ValueError(f"No frames in {path}")
    return wall_s / frames * 1000.0


def _load_depth_frames(path: Path, max_frames: int = 12) -> np.ndarray:
    data = np.load(path)
    depth = np.asarray(data["depth_u8"], dtype=np.float32)
    if depth.ndim != 3:
        raise ValueError(f"Expected [frames,h,w] depth_u8 in {path}, got {depth.shape}")
    return depth[: min(max_frames, depth.shape[0])]


def _median_ms(fn, repeats: int) -> tuple[float, object]:
    samples: list[float] = []
    last = None
    for _ in range(max(1, repeats)):
        start = time.perf_counter()
        last = fn()
        samples.append((time.perf_counter() - start) * 1000.0)
    return float(statistics.median(samples)), last


def _build_frontends(args: argparse.Namespace, ideal_esdf_ms: float) -> list[FrontendProfile]:
    depth_config = DepthProjectionConfig(resolution_m=0.10, sample_stride_px=3, hit_dilation_cells=2)
    depth_frames = _load_depth_frames(args.depth_cache)
    bbox_rows = select_bbox_rows(_read_csv_rows(args.bbox_csv), frame_offset=0, min_point_count=50)

    frame_cursor = {"idx": 0}

    def depth_grid_once():
        idx = frame_cursor["idx"] % len(depth_frames)
        frame_cursor["idx"] += 1
        return depth_image_to_grid(depth_frames[idx], "mono8", depth_config)

    cached_depth_grid_ms, depth_result = _median_ms(depth_grid_once, args.timing_repeats)
    depth_grid, depth_spec, _ = depth_result
    depth_occ = int((depth_grid >= 50).sum())
    depth_shape = f"{int(depth_spec.width)}x{int(depth_spec.height)}"
    depth_wall_ms = _weighted_depth_wall_ms(args.depth_timing_csv)

    def bbox_grid_once():
        return bbox_rows_to_grid(bbox_rows, resolution_m=0.20, margin_m=1.0)

    bbox_grid_ms, bbox_result = _median_ms(bbox_grid_once, args.timing_repeats)
    bbox_grid, bbox_spec = bbox_result
    bbox_occ = int((bbox_grid >= 50).sum())
    bbox_shape = f"{int(bbox_spec.width)}x{int(bbox_spec.height)}"

    def mux_once():
        d_grid, d_spec, _ = depth_image_to_grid(depth_frames[0], "mono8", depth_config)
        b_grid, b_spec = bbox_rows_to_grid(bbox_rows, resolution_m=0.20, margin_m=1.0)
        return merge_occupancy_grids([(b_grid, b_spec), (d_grid, d_spec)], occupied_threshold=50, resolution_m=0.20, padding_m=0.25)

    mux_ms, mux_result = _median_ms(mux_once, args.timing_repeats)
    mux_grid, mux_spec = mux_result

    return [
        FrontendProfile(
            name="ideal_occupancy_esdf",
            source="synthetic_gt",
            algorithm="ground-truth occupancy -> ESDF distance transform",
            sensor_period_ms=100.0,
            frontend_ms=ideal_esdf_ms,
            occupied_cells=3084,
            grid_shape="3D 43680 voxels",
            note="upper-bound map frontend; no raw sensor processing",
        ),
        FrontendProfile(
            name="oda_cached_depth_to_grid",
            source="ODA RGB cached depth",
            algorithm="relative depth image -> 2D occupancy projection",
            sensor_period_ms=200.0,
            frontend_ms=cached_depth_grid_ms,
            occupied_cells=depth_occ,
            grid_shape=depth_shape,
            note="postprocess only; depth prediction already cached",
        ),
        FrontendProfile(
            name="oda_depth_anything_to_grid",
            source="ODA RGB",
            algorithm="Depth Anything V2 Small wall-time + depth-to-grid",
            sensor_period_ms=200.0,
            frontend_ms=depth_wall_ms + cached_depth_grid_ms,
            occupied_cells=depth_occ,
            grid_shape=depth_shape,
            note="dataset replay compute; not camera hardware latency",
        ),
        FrontendProfile(
            name="external_lidar_bbox_to_grid",
            source="Multi-LiDAR bbox CSV",
            algorithm="3D bbox footprints -> 2D occupancy raster",
            sensor_period_ms=100.0,
            frontend_ms=bbox_grid_ms,
            occupied_cells=bbox_occ,
            grid_shape=bbox_shape,
            note="not ODA; starts after point-cloud clustering/bbox extraction",
        ),
        FrontendProfile(
            name="bbox_cached_depth_mux_to_grid",
            source="Multi-LiDAR bbox + ODA cached depth",
            algorithm="bbox grid + cached depth grid -> merged occupancy",
            sensor_period_ms=200.0,
            frontend_ms=mux_ms,
            occupied_cells=int((mux_grid >= 50).sum()),
            grid_shape=f"{int(mux_spec.width)}x{int(mux_spec.height)}",
            note="fusion contract timing; mixed-source proxy",
        ),
    ]


def _speed_scenarios() -> list[Scenario]:
    return [
        Scenario("slow_1mps", 1.0, 384, 8, 56, 51),
        Scenario("nominal_2mps", 2.0, 768, 10, 64, 52),
        Scenario("fast_3mps", 3.0, 1152, 12, 72, 53),
        Scenario("very_fast_4mps", 4.0, 1152, 12, 72, 54),
        Scenario("extreme_5mps", 5.0, 1152, 12, 72, 55),
        Scenario("failure_6mps", 6.0, 1152, 12, 72, 56),
    ]


def _row_for_profile(args: argparse.Namespace, profile: FrontendProfile, scenario: Scenario, esdf) -> dict[str, object]:
    start_xyz = np.asarray([0.0, 0.0, 1.15], dtype=float)
    goal_xyz = np.asarray([6.55, 0.0, 1.15], dtype=float)
    base_delay_ms = profile.sensor_period_ms + profile.frontend_ms + float(args.control_publish_ms)
    result, total_delay_ms, delayed_start = _run_planner(
        scenario=scenario,
        esdf=esdf,
        start_xyz=start_xyz,
        goal_xyz=goal_xyz,
        speed_mps=scenario.speed_mps,
        base_delay_ms=base_delay_ms,
        safety_radius_m=float(args.safety_radius),
    )
    combined = _combined_trajectory(start_xyz, delayed_start, result.trajectory_xyz)
    distances = esdf.query_distance(combined)
    min_esdf = float(np.min(distances))
    min_body_clearance = min_esdf - float(args.body_radius)
    min_safety_margin = min_esdf - float(args.safety_radius)
    delay_distance = scenario.speed_mps * total_delay_ms / 1000.0
    first_safety_breach = _first_range_below_threshold(esdf, start_xyz, goal_xyz, float(args.safety_radius))
    first_body_contact = _first_range_below_threshold(esdf, start_xyz, goal_xyz, float(args.body_radius))
    return {
        "frontend": profile.name,
        "source": profile.source,
        "algorithm": profile.algorithm,
        "sensor_period_ms": round(profile.sensor_period_ms, 3),
        "frontend_to_occupancy_ms": round(profile.frontend_ms, 3),
        "mppi_compute_ms": round(result.compute_time_s * 1000.0, 3),
        "control_publish_ms": round(float(args.control_publish_ms), 3),
        "uav_speed_mps": round(scenario.speed_mps, 3),
        "total_delay_ms": round(total_delay_ms, 3),
        "distance_during_delay_m": round(delay_distance, 4),
        "latency_buffer_to_safety_breach_m": round(first_safety_breach - delay_distance, 4),
        "latency_buffer_to_body_contact_m": round(first_body_contact - delay_distance, 4),
        "min_body_clearance_m": round(min_body_clearance, 4),
        "min_safety_margin_m": round(min_safety_margin, 4),
        "collision": int(min_body_clearance <= 0.0),
        "safety_violation": int(min_safety_margin < 0.0),
        "occupied_cells": profile.occupied_cells,
        "grid_shape": profile.grid_shape,
        "note": profile.note,
    }


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _profile_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for frontend in dict.fromkeys(str(row["frontend"]) for row in rows):
        group = [row for row in rows if row["frontend"] == frontend]
        safe = [float(row["uav_speed_mps"]) for row in group if int(row["safety_violation"]) == 0]
        collisions = [float(row["uav_speed_mps"]) for row in group if int(row["collision"]) == 1]
        violations = [float(row["uav_speed_mps"]) for row in group if int(row["safety_violation"]) == 1]
        at_2 = next(row for row in group if float(row["uav_speed_mps"]) == 2.0)
        out.append(
            {
                "frontend": frontend,
                "source": at_2["source"],
                "sensor_period_ms": at_2["sensor_period_ms"],
                "frontend_to_occupancy_ms": at_2["frontend_to_occupancy_ms"],
                "total_delay_at_2mps_ms": at_2["total_delay_ms"],
                "delay_distance_at_2mps_m": at_2["distance_during_delay_m"],
                "max_speed_no_violation_mps": max(safe) if safe else 0.0,
                "first_safety_violation_mps": min(violations) if violations else "",
                "first_collision_mps": min(collisions) if collisions else "",
                "note": at_2["note"],
            }
        )
    return out


def _write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    summary = _profile_summary(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Sensor Frontend Latency Feasibility",
        "",
        "This table plugs measured/reused sensor-to-occupancy compute time into the same MPPI kinematic-delay check.",
        "Radar/IMU are excluded because the current ODA pipeline turns them into risk features, not occupancy grids.",
        "",
        "| Frontend | Source | Sensor ms | Frontend ms | Delay at 2 m/s ms | Max no-violation speed | First violation | First collision |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in summary:
        lines.append(
            f"| {row['frontend']} | {row['source']} | {float(row['sensor_period_ms']):.1f} | "
            f"{float(row['frontend_to_occupancy_ms']):.1f} | {float(row['total_delay_at_2mps_ms']):.1f} | "
            f"{float(row['max_speed_no_violation_mps']):.1f} | {row['first_safety_violation_mps']} | {row['first_collision_mps']} |"
        )
    lines.extend(["", "Full per-speed rows are in `outputs/tables/sensor_frontend_latency_feasibility.csv`.", ""])
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_plot(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9.2, 5.2))
    for frontend in dict.fromkeys(str(row["frontend"]) for row in rows):
        group = [row for row in rows if row["frontend"] == frontend]
        speeds = [float(row["uav_speed_mps"]) for row in group]
        clearances = [float(row["min_body_clearance_m"]) for row in group]
        ax.plot(speeds, clearances, marker="o", linewidth=1.8, label=frontend)
    ax.axhline(0.0, color="#991b1b", linestyle="--", linewidth=1.0, label="collision boundary")
    ax.axhline(0.25, color="#f97316", linestyle=":", linewidth=1.0, label="low body-clearance guide")
    ax.set_xlabel("UAV speed [m/s]")
    ax.set_ylabel("minimum body clearance after frontend+MPPI delay [m]")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    _, _, esdf, map_samples_ms = _map_update_samples(args.resolution, args.prefer_scipy, args.map_repeats)
    ideal_esdf_ms = float(statistics.median(map_samples_ms))
    profiles = _build_frontends(args, ideal_esdf_ms)
    scenarios = _speed_scenarios()
    rows = [_row_for_profile(args, profile, scenario, esdf) for profile in profiles for scenario in scenarios]

    table_path = args.output_dir / "tables" / "sensor_frontend_latency_feasibility.csv"
    summary_path = args.output_dir / "sensor_frontend_latency_feasibility_summary.md"
    figure_path = args.output_dir / "figures" / "sensor_frontend_latency_feasibility.png"
    _write_csv(table_path, rows)
    _write_summary(summary_path, rows)
    _write_plot(figure_path, rows)
    print(f"Wrote {table_path}")
    print(f"Wrote {summary_path}")
    print(f"Wrote {figure_path}")
    for row in _profile_summary(rows):
        print(row)


if __name__ == "__main__":
    main()
