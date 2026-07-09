#!/usr/bin/env python3
"""ODA replay online-latency feasibility using actual OptiTrack delayed poses.

This variant scores replay safety in 3D.  ODA OptiTrack is represented as
``(x, z, y_height)`` and obstacle metadata is represented as finite cylinders.
The planners still operate on the ground-plane footprint; their paths are
lifted between delayed and goal heights before 3D clearance is evaluated.
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

from src.metrics import pairwise_ground_distances
from src.oda_io import Obstacle, available_trial_ids, dataset_root, load_optitrack, obstacle_array, read_trial_overview
from src.planners.mppi import MPPIConfig, mppi_path
from src.planners.rrt import RRTConfig, rrt_path
from src.planners.rrt_star import RRTStarConfig, rrt_star_path


UAV_PROFILES = {
    "local_measured": {
        "label": "Local measured Python timing",
        "sensor_hz": None,
        "sensor_pipeline_ms": 0.0,
        "costmap_scale": 1.0,
        "costmap_extra_ms": 0.0,
        "planner_scale": 1.0,
        "planner_extra_ms": 0.0,
        "control_publish_ms": None,
    },
    "jetson_orin_nano": {
        "label": "Emulated Jetson Orin Nano-class companion computer",
        "sensor_hz": 10.0,
        "sensor_pipeline_ms": 20.0,
        "costmap_scale": 1.5,
        "costmap_extra_ms": 3.0,
        "planner_scale": 2.0,
        "planner_extra_ms": 5.0,
        "control_publish_ms": 15.0,
    },
    "voxl2": {
        "label": "Emulated ModalAI VOXL2-class companion computer",
        "sensor_hz": 10.0,
        "sensor_pipeline_ms": 25.0,
        "costmap_scale": 2.0,
        "costmap_extra_ms": 5.0,
        "planner_scale": 2.5,
        "planner_extra_ms": 8.0,
        "control_publish_ms": 15.0,
    },
    "rpi5": {
        "label": "Emulated Raspberry Pi 5-class companion computer",
        "sensor_hz": 10.0,
        "sensor_pipeline_ms": 30.0,
        "costmap_scale": 3.0,
        "costmap_extra_ms": 8.0,
        "planner_scale": 4.0,
        "planner_extra_ms": 10.0,
        "control_publish_ms": 20.0,
    },
}


@dataclass(frozen=True)
class ReplayCase:
    sequence: str
    case_id: int
    time_s: float
    speed_mps: float
    clearance_m: float


@dataclass(frozen=True)
class TimedPlan:
    trajectory_xy: np.ndarray
    compute_ms: float
    failed: bool = False
    failure_reason: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=Path("data/raw/ODA_Dataset/dataset"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--trial-ids", nargs="*", default=None)
    parser.add_argument("--cases-per-trial", type=int, default=8)
    parser.add_argument("--sensor-hz", type=float, default=10.0)
    parser.add_argument("--uav-profile", choices=sorted(UAV_PROFILES), default="local_measured")
    parser.add_argument("--sensor-pipeline-ms", type=float, default=None)
    parser.add_argument("--costmap-scale", type=float, default=None)
    parser.add_argument("--costmap-extra-ms", type=float, default=None)
    parser.add_argument("--planner-scale", type=float, default=None)
    parser.add_argument("--planner-extra-ms", type=float, default=None)
    parser.add_argument("--control-publish-ms", type=float, default=10.0)
    parser.add_argument("--costmap-resolution", type=float, default=0.10)
    parser.add_argument("--costmap-margin", type=float, default=1.20)
    parser.add_argument("--goal-lookahead-s", type=float, default=1.50)
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument("--seed-base", type=int, default=3100)
    parser.add_argument("--rrt-iterations", type=int, default=900)
    parser.add_argument("--rrt-star-iterations", type=int, default=700)
    args = parser.parse_args()
    _apply_uav_profile(args)
    return args


def _apply_uav_profile(args: argparse.Namespace) -> None:
    profile = UAV_PROFILES[args.uav_profile]
    if profile["sensor_hz"] is not None:
        args.sensor_hz = float(profile["sensor_hz"])
    if profile["control_publish_ms"] is not None:
        args.control_publish_ms = float(profile["control_publish_ms"])
    args.uav_profile_label = str(profile["label"])
    for attr, profile_key in [
        ("sensor_pipeline_ms", "sensor_pipeline_ms"),
        ("costmap_scale", "costmap_scale"),
        ("costmap_extra_ms", "costmap_extra_ms"),
        ("planner_scale", "planner_scale"),
        ("planner_extra_ms", "planner_extra_ms"),
    ]:
        if getattr(args, attr) is None:
            setattr(args, attr, float(profile[profile_key]))


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _interp_xy(time_s: np.ndarray, xy: np.ndarray, query_s: float) -> np.ndarray:
    q = float(np.clip(query_s, time_s[0], time_s[-1]))
    return np.asarray([np.interp(q, time_s, xy[:, 0]), np.interp(q, time_s, xy[:, 1])], dtype=float)


def _interp_xyz(time_s: np.ndarray, xyz: np.ndarray, query_s: float) -> np.ndarray:
    q = float(np.clip(query_s, time_s[0], time_s[-1]))
    return np.asarray(
        [np.interp(q, time_s, xyz[:, 0]), np.interp(q, time_s, xyz[:, 1]), np.interp(q, time_s, xyz[:, 2])],
        dtype=float,
    )


def _actual_segment(time_s: np.ndarray, xy: np.ndarray, start_s: float, end_s: float) -> np.ndarray:
    if end_s <= start_s:
        return _interp_xy(time_s, xy, start_s)[None, :]
    mask = (time_s > start_s) & (time_s < end_s)
    points = [_interp_xy(time_s, xy, start_s)]
    points.extend(xy[mask])
    points.append(_interp_xy(time_s, xy, end_s))
    return np.asarray(points, dtype=float)


def _actual_segment_xyz(time_s: np.ndarray, xyz: np.ndarray, start_s: float, end_s: float) -> np.ndarray:
    if end_s <= start_s:
        return _interp_xyz(time_s, xyz, start_s)[None, :]
    mask = (time_s > start_s) & (time_s < end_s)
    points = [_interp_xyz(time_s, xyz, start_s)]
    points.extend(xyz[mask])
    points.append(_interp_xyz(time_s, xyz, end_s))
    return np.asarray(points, dtype=float)


def _speed_series(time_s: np.ndarray, xyz: np.ndarray, window_s: float = 0.25) -> np.ndarray:
    if len(time_s) < 2:
        return np.zeros(len(time_s), dtype=float)
    out = np.zeros(len(time_s), dtype=float)
    half = window_s / 2.0
    for idx, t in enumerate(time_s):
        t0 = max(float(time_s[0]), float(t - half))
        t1 = min(float(time_s[-1]), float(t + half))
        if t1 <= t0:
            out[idx] = 0.0
            continue
        p0 = _interp_xyz(time_s, xyz, t0)
        p1 = _interp_xyz(time_s, xyz, t1)
        out[idx] = float(np.linalg.norm(p1 - p0) / (t1 - t0))
    return out


def _cylinder_clearance_3d(points_xyz: np.ndarray, obstacles: tuple[Obstacle, ...], obstacle_radius_m: float) -> np.ndarray:
    """Signed clearance from points to finite vertical obstacle cylinders.

    Points are in ODA plotting coordinates ``(x, z, y_height)``.  Positive
    clearance means outside every cylinder; negative means inside a cylinder.
    """

    points = np.asarray(points_xyz, dtype=float)
    all_clearances: list[np.ndarray] = []
    for obstacle in obstacles:
        horizontal = np.hypot(points[:, 0] - obstacle.x, points[:, 1] - obstacle.ground_y)
        below = np.maximum(0.0, -points[:, 2])
        above = np.maximum(0.0, points[:, 2] - obstacle.height_y)
        outside_horizontal = np.maximum(0.0, horizontal - obstacle_radius_m)
        outside_distance = np.sqrt(outside_horizontal**2 + below**2 + above**2)

        inside = (horizontal <= obstacle_radius_m) & (points[:, 2] >= 0.0) & (points[:, 2] <= obstacle.height_y)
        inward_distance = np.minimum.reduce(
            [
                obstacle_radius_m - horizontal,
                points[:, 2],
                obstacle.height_y - points[:, 2],
            ]
        )
        signed = outside_distance.copy()
        signed[inside] = -inward_distance[inside]
        all_clearances.append(signed)
    return np.vstack(all_clearances).min(axis=0)


def _lift_path_to_3d(path_xy: np.ndarray, start_height_m: float, goal_height_m: float) -> np.ndarray:
    if len(path_xy) == 0:
        return np.empty((0, 3), dtype=float)
    if len(path_xy) == 1:
        return np.column_stack([path_xy, np.asarray([start_height_m])])
    segment_lengths = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    if s[-1] <= 1e-9:
        alpha = np.linspace(0.0, 1.0, len(path_xy))
    else:
        alpha = s / s[-1]
    heights = (1.0 - alpha) * start_height_m + alpha * goal_height_m
    return np.column_stack([path_xy, heights])


def _select_cases(
    sequence: str,
    time_s: np.ndarray,
    xyz: np.ndarray,
    obstacles: tuple[Obstacle, ...],
    args: argparse.Namespace,
) -> list[ReplayCase]:
    clearance = _cylinder_clearance_3d(xyz, obstacles, args.obstacle_radius)
    speeds = _speed_series(time_s, xyz)
    valid = np.flatnonzero(time_s <= time_s[-1] - args.goal_lookahead_s - 0.65)
    valid = valid[valid >= 2]
    if len(valid) == 0:
        return []

    selected: list[int] = []
    min_spacing_s = 0.35
    for idx in valid[np.argsort(clearance[valid])]:
        if all(abs(float(time_s[idx] - time_s[prev])) >= min_spacing_s for prev in selected):
            selected.append(int(idx))
        if len(selected) >= args.cases_per_trial:
            break

    if len(selected) < args.cases_per_trial:
        quantile_indices = np.linspace(0, len(valid) - 1, args.cases_per_trial, dtype=int)
        for idx in valid[quantile_indices]:
            if idx not in selected:
                selected.append(int(idx))
            if len(selected) >= args.cases_per_trial:
                break

    selected = sorted(selected, key=lambda idx: time_s[idx])
    return [
        ReplayCase(
            sequence=sequence,
            case_id=case_id,
            time_s=float(time_s[idx]),
            speed_mps=float(speeds[idx]),
            clearance_m=float(clearance[idx]),
        )
        for case_id, idx in enumerate(selected[: args.cases_per_trial])
    ]


def _build_metadata_costmap(
    start_xyz: np.ndarray,
    delayed_xyz: np.ndarray,
    goal_xyz: np.ndarray,
    obstacles: tuple[Obstacle, ...],
    args: argparse.Namespace,
) -> tuple[float, int, tuple[int, int, int]]:
    started = time.perf_counter()
    obstacle_points = np.asarray([(obs.x, obs.ground_y, obs.height_y) for obs in obstacles], dtype=float)
    floor_points = np.asarray([(obs.x, obs.ground_y, 0.0) for obs in obstacles], dtype=float)
    all_xyz = np.vstack([start_xyz[None, :], delayed_xyz[None, :], goal_xyz[None, :], obstacle_points, floor_points])
    xyz_min = all_xyz.min(axis=0) - args.costmap_margin
    xyz_max = all_xyz.max(axis=0) + args.costmap_margin
    xyz_min[2] = min(0.0, xyz_min[2])
    size = np.ceil((xyz_max - xyz_min) / args.costmap_resolution).astype(int) + 1
    xs = xyz_min[0] + np.arange(size[0]) * args.costmap_resolution
    ys = xyz_min[1] + np.arange(size[1]) * args.costmap_resolution
    zs = xyz_min[2] + np.arange(size[2]) * args.costmap_resolution
    grid_x, grid_y, grid_z = np.meshgrid(xs, ys, zs, indexing="ij")
    occupied = np.zeros(tuple(size), dtype=bool)
    inflated = args.obstacle_radius + args.safety_distance
    for obstacle in obstacles:
        radial = np.sqrt((grid_x - obstacle.x) ** 2 + (grid_y - obstacle.ground_y) ** 2)
        vertical = (grid_z >= -args.safety_distance) & (grid_z <= obstacle.height_y + args.safety_distance)
        occupied |= (radial <= inflated) & vertical
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    return elapsed_ms, int(np.sum(occupied)), (int(size[0]), int(size[1]), int(size[2]))


def _straight_fallback(start_xy: np.ndarray, goal_xy: np.ndarray, samples: int = 120) -> np.ndarray:
    return np.linspace(start_xy, goal_xy, samples)


def _time_planner(
    planner: str,
    start_xy: np.ndarray,
    goal_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    args: argparse.Namespace,
    seed: int,
) -> TimedPlan:
    started = time.perf_counter()
    try:
        if planner == "rrt":
            config = RRTConfig(
                max_iterations=args.rrt_iterations,
                step_size_m=0.25,
                goal_sample_rate=0.20,
                margin_m=args.costmap_margin,
                obstacle_radius_m=args.obstacle_radius,
                safety_distance_m=args.safety_distance,
                seed=seed,
            )
            path = rrt_path(start_xy, goal_xy, obstacles_xy, config, num_points=160)
        elif planner == "rrt_star":
            config = RRTStarConfig(
                max_iterations=args.rrt_star_iterations,
                step_size_m=0.25,
                neighbor_radius_m=0.65,
                goal_sample_rate=0.18,
                margin_m=args.costmap_margin,
                obstacle_radius_m=args.obstacle_radius,
                safety_distance_m=args.safety_distance,
                seed=seed,
            )
            path = rrt_star_path(start_xy, goal_xy, obstacles_xy, config, num_points=160)
        elif planner == "mppi":
            config = MPPIConfig(
                num_rollouts=512,
                horizon_steps=60,
                max_iterations=8,
                obstacle_radius_m=args.obstacle_radius,
                safety_distance_m=args.safety_distance,
                seed=seed,
            )
            path = mppi_path(start_xy, goal_xy, obstacles_xy, config, num_points=160)
        else:
            raise ValueError(f"Unknown planner {planner}")
        return TimedPlan(path.trajectory_xy, (time.perf_counter() - started) * 1000.0)
    except Exception as exc:  # noqa: BLE001 - planner failure is part of the benchmark.
        return TimedPlan(
            _straight_fallback(start_xy, goal_xy),
            (time.perf_counter() - started) * 1000.0,
            failed=True,
            failure_reason=str(exc),
        )


def _profile_costmap_ms(args: argparse.Namespace, measured_ms: float) -> float:
    return float(measured_ms) * float(args.costmap_scale) + float(args.costmap_extra_ms)


def _profile_planner_ms(args: argparse.Namespace, measured_ms: float) -> float:
    return float(measured_ms) * float(args.planner_scale) + float(args.planner_extra_ms)


def _status(min_clearance_m: float, failed: bool, safety_distance_m: float) -> str:
    if failed or min_clearance_m <= 0.0:
        return "collision"
    if min_clearance_m < safety_distance_m:
        return "violation"
    return "safe"


def _speed_bin(speed_mps: float) -> str:
    if speed_mps < 0.5:
        return "<0.5"
    if speed_mps < 1.0:
        return "0.5-1.0"
    if speed_mps < 1.5:
        return "1.0-1.5"
    return ">=1.5"


def _row_for_case(
    args: argparse.Namespace,
    planner: str,
    case: ReplayCase,
    time_s: np.ndarray,
    xyz: np.ndarray,
    obstacles: tuple[Obstacle, ...],
    seed: int,
) -> dict[str, object]:
    start_xyz = _interp_xyz(time_s, xyz, case.time_s)
    start_xy = start_xyz[:2]
    obstacles_xy = obstacle_array(obstacles)
    lidar_period_ms = 1000.0 / args.sensor_hz
    base_goal_xyz = _interp_xyz(time_s, xyz, case.time_s + args.goal_lookahead_s)
    base_goal_xy = base_goal_xyz[:2]
    raw_costmap_ms, occupied_cells, costmap_shape = _build_metadata_costmap(
        start_xyz, start_xyz, base_goal_xyz, obstacles, args
    )
    costmap_ms = _profile_costmap_ms(args, raw_costmap_ms)
    base_delay_ms = lidar_period_ms + args.sensor_pipeline_ms + costmap_ms + args.control_publish_ms

    first_delay_s = case.time_s + base_delay_ms / 1000.0
    first_start_xyz = _interp_xyz(time_s, xyz, first_delay_s)
    first_goal_xyz = _interp_xyz(time_s, xyz, first_delay_s + args.goal_lookahead_s)
    first_start_xy = first_start_xyz[:2]
    first_goal_xy = first_goal_xyz[:2]
    first = _time_planner(planner, first_start_xy, first_goal_xy, obstacles_xy, args, seed)
    first_planner_ms = _profile_planner_ms(args, first.compute_ms)

    total_delay_ms = base_delay_ms + first_planner_ms
    delay_time_s = case.time_s + total_delay_ms / 1000.0
    delayed_xyz = _interp_xyz(time_s, xyz, delay_time_s)
    delayed_xy = delayed_xyz[:2]
    goal_xyz = _interp_xyz(time_s, xyz, delay_time_s + args.goal_lookahead_s)
    goal_xy = goal_xyz[:2]
    raw_costmap_ms, occupied_cells, costmap_shape = _build_metadata_costmap(start_xyz, delayed_xyz, goal_xyz, obstacles, args)
    costmap_ms = _profile_costmap_ms(args, raw_costmap_ms)
    base_delay_ms = lidar_period_ms + args.sensor_pipeline_ms + costmap_ms + args.control_publish_ms
    final = _time_planner(planner, delayed_xy, goal_xy, obstacles_xy, args, seed)
    planner_ms = _profile_planner_ms(args, final.compute_ms)
    total_delay_ms = base_delay_ms + planner_ms
    delay_time_s = case.time_s + total_delay_ms / 1000.0
    delayed_xyz = _interp_xyz(time_s, xyz, delay_time_s)

    actual_delay = _actual_segment_xyz(time_s, xyz, case.time_s, delay_time_s)
    planned_xyz = _lift_path_to_3d(final.trajectory_xy, delayed_xyz[2], goal_xyz[2])
    combined = np.vstack([actual_delay, planned_xyz[1:]])
    clearances = _cylinder_clearance_3d(combined, obstacles, args.obstacle_radius)
    min_clearance = float(np.min(clearances))
    status = _status(min_clearance, final.failed, args.safety_distance)

    return {
        "sequence": case.sequence,
        "case_id": case.case_id,
        "planner": planner,
        "time_s": round(case.time_s, 4),
        "speed_mps": round(case.speed_mps, 4),
        "speed_bin": _speed_bin(case.speed_mps),
        "metric_frame": "3d_optitrack_xzy_cylinder",
        "uav_profile": args.uav_profile,
        "uav_profile_label": args.uav_profile_label,
        "recorded_clearance_at_t_m": round(case.clearance_m, 4),
        "sensor_hz": round(args.sensor_hz, 3),
        "sensor_period_ms": round(lidar_period_ms, 3),
        "sensor_pipeline_ms": round(args.sensor_pipeline_ms, 3),
        "raw_costmap_update_ms": round(raw_costmap_ms, 3),
        "costmap_update_ms": round(costmap_ms, 3),
        "raw_planner_compute_ms": round(final.compute_ms, 3),
        "planner_compute_ms": round(planner_ms, 3),
        "control_publish_ms": round(args.control_publish_ms, 3),
        "costmap_scale": round(args.costmap_scale, 3),
        "planner_scale": round(args.planner_scale, 3),
        "total_delay_ms": round(total_delay_ms, 3),
        "delay_distance_m": round(float(np.linalg.norm(delayed_xyz - start_xyz)), 4),
        "goal_lookahead_s": round(args.goal_lookahead_s, 3),
        "min_clearance_m": round(min_clearance, 4),
        "collision": int(status == "collision"),
        "safety_violation": int(status in {"violation", "collision"}),
        "status": status,
        "planner_failed": int(final.failed),
        "failure_reason": final.failure_reason,
        "occupied_costmap_cells": occupied_cells,
        "costmap_width": costmap_shape[0],
        "costmap_height": costmap_shape[1],
        "costmap_layers": costmap_shape[2],
    }


def _aggregate(rows: list[dict[str, object]], keys: list[str]) -> list[dict[str, object]]:
    groups = sorted({tuple(str(row[key]) for key in keys) for row in rows})
    out: list[dict[str, object]] = []
    for group_key in groups:
        group = [row for row in rows if tuple(str(row[key]) for key in keys) == group_key]
        n = len(group)
        safe = sum(1 for row in group if row["status"] == "safe")
        violation = sum(1 for row in group if row["status"] == "violation")
        collision = sum(1 for row in group if row["status"] == "collision")
        failures = sum(int(row["planner_failed"]) for row in group)
        result = {key: group_key[idx] for idx, key in enumerate(keys)}
        result.update(
            {
                "cases": n,
                "safe_count": safe,
                "violation_count": violation,
                "collision_count": collision,
                "failure_count": failures,
                "safe_rate_pct": round(safe / n * 100.0, 2),
                "violation_rate_pct": round(violation / n * 100.0, 2),
                "collision_rate_pct": round(collision / n * 100.0, 2),
                "mean_speed_mps": round(statistics.mean(float(row["speed_mps"]) for row in group), 4),
                "mean_total_delay_ms": round(statistics.mean(float(row["total_delay_ms"]) for row in group), 3),
                "mean_delay_distance_m": round(statistics.mean(float(row["delay_distance_m"]) for row in group), 4),
                "mean_planner_compute_ms": round(statistics.mean(float(row["planner_compute_ms"]) for row in group), 3),
                "mean_costmap_update_ms": round(statistics.mean(float(row["costmap_update_ms"]) for row in group), 3),
                "mean_min_clearance_m": round(statistics.mean(float(row["min_clearance_m"]) for row in group), 4),
            }
        )
        out.append(result)
    return out


def _write_summary(path: Path, rows: list[dict[str, object]], by_planner: list[dict[str, object]], by_speed: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    first_row = rows[0]
    lines = [
        "# ODA Replay Online Feasibility",
        "",
        f"UAV timing profile: `{first_row['uav_profile']}` - {first_row['uav_profile_label']}.",
        "",
        "This is a timing emulation/profile applied to local Python measurements, not a real onboard hardware benchmark. It is intended to stress the replay with representative companion-computer latency assumptions.",
        "",
        "Each case is an ODA OptiTrack timestamp near an obstacle. The delayed pose is obtained by interpolating the recorded OptiTrack trajectory at `t + total_delay`, then the planner replans from that delayed pose.",
        "",
        "Safety is scored in 3D using OptiTrack `(x, z, y_height)` points and finite cylinder obstacles from ODA metadata. RRT, RRT*, and MPPI plan in the ground-plane footprint and their paths are lifted between delayed and goal heights for 3D clearance scoring.",
        "",
        "## Planner Summary",
        "",
        "| Planner | Cases | Safe % | Violation % | Collision % | Mean speed m/s | Mean delay ms | Mean delay m | Mean compute ms | Mean clearance m |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in by_planner:
        lines.append(
            f"| {row['planner']} | {row['cases']} | {float(row['safe_rate_pct']):.1f} | "
            f"{float(row['violation_rate_pct']):.1f} | {float(row['collision_rate_pct']):.1f} | "
            f"{float(row['mean_speed_mps']):.3f} | {float(row['mean_total_delay_ms']):.1f} | "
            f"{float(row['mean_delay_distance_m']):.3f} | {float(row['mean_planner_compute_ms']):.1f} | "
            f"{float(row['mean_min_clearance_m']):.3f} |"
        )
    lines.extend(
        [
            "",
            "## Planner x Speed Bin",
            "",
            "| Planner | Speed bin | Cases | Safe % | Violation % | Collision % | Mean delay ms | Mean clearance m |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in by_speed:
        lines.append(
            f"| {row['planner']} | {row['speed_bin']} | {row['cases']} | {float(row['safe_rate_pct']):.1f} | "
            f"{float(row['violation_rate_pct']):.1f} | {float(row['collision_rate_pct']):.1f} | "
            f"{float(row['mean_total_delay_ms']):.1f} | {float(row['mean_min_clearance_m']):.3f} |"
        )
    lines.extend(
        [
            "",
            f"Total detailed cases: {len(rows)}.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_plot(path: Path, by_speed: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    order = ["<0.5", "0.5-1.0", "1.0-1.5", ">=1.5"]
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    for planner in ["rrt", "rrt_star", "mppi"]:
        group = [row for row in by_speed if row["planner"] == planner]
        xs = [idx for idx, label in enumerate(order) if any(row["speed_bin"] == label for row in group)]
        ys = [float(next(row for row in group if row["speed_bin"] == order[idx])["safe_rate_pct"]) for idx in xs]
        if xs:
            ax.plot(xs, ys, marker="o", linewidth=1.8, label=planner)
    ax.set_xticks(range(len(order)))
    ax.set_xticklabels(order)
    ax.set_xlabel("ODA speed bin [m/s]")
    ax.set_ylabel("safe cases [%]")
    ax.set_ylim(-3, 103)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    trial_infos = read_trial_overview(dataset_dir)
    trial_ids = [str(item) for item in args.trial_ids] if args.trial_ids else available_trial_ids(dataset_dir)
    planners = ["rrt", "rrt_star", "mppi"]

    detail_rows: list[dict[str, object]] = []
    for sequence in trial_ids:
        if sequence not in trial_infos or trial_infos[sequence].obstacle_count == 0:
            continue
        opt = load_optitrack(dataset_dir, sequence)
        time_s = opt["time_s"]
        xyz = np.column_stack([opt["ground_x_m"], opt["ground_y_m"], opt["height_m"]])
        obstacles = trial_infos[sequence].obstacles
        cases = _select_cases(sequence, time_s, xyz, obstacles, args)
        for planner_idx, planner in enumerate(planners):
            for case in cases:
                seed = args.seed_base + planner_idx * 10_000 + int(sequence) * 100 + case.case_id
                detail_rows.append(_row_for_case(args, planner, case, time_s, xyz, obstacles, seed))

    if not detail_rows:
        raise SystemExit("No ODA replay feasibility rows were produced.")

    by_planner = _aggregate(detail_rows, ["planner"])
    by_speed = _aggregate(detail_rows, ["planner", "speed_bin"])

    detail_path = args.output_dir / "tables" / "oda_replay_online_feasibility_detail.csv"
    planner_path = args.output_dir / "tables" / "oda_replay_online_feasibility_by_planner.csv"
    speed_path = args.output_dir / "tables" / "oda_replay_online_feasibility_by_speed_bin.csv"
    summary_path = args.output_dir / "oda_replay_online_feasibility_summary.md"
    figure_path = args.output_dir / "figures" / "oda_replay_online_feasibility.png"
    _write_csv(detail_path, detail_rows)
    _write_csv(planner_path, by_planner)
    _write_csv(speed_path, by_speed)
    _write_summary(summary_path, detail_rows, by_planner, by_speed)
    _write_plot(figure_path, by_speed)

    print(f"Wrote {detail_path}")
    print(f"Wrote {planner_path}")
    print(f"Wrote {speed_path}")
    print(f"Wrote {summary_path}")
    print(f"Wrote {figure_path}")
    for row in by_planner:
        print(row)


if __name__ == "__main__":
    main()
