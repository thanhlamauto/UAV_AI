#!/usr/bin/env python3
"""Benchmark ODA depth-derived occupancy against metadata GT occupancy and planners.

Depth occupancy is built from metric depth cache in camera local coordinates
(`x` lateral, `z` forward).  GT occupancy is an approximate local projection of
ODA obstacle metadata using OptiTrack position and motion heading.  This is a
useful ODA-derived perception-to-planner proxy, but not a full calibrated camera
extrinsics benchmark.
"""

from __future__ import annotations

import argparse
import csv
import platform
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from src.depth_metric import CameraIntrinsics, depth_to_point_cloud, intrinsics_from_fov, load_cached_metric_depth
from src.metrics import pairwise_ground_distances
from src.oda_io import Obstacle, dataset_root, load_optitrack, obstacle_array, read_trial_overview
from src.planners.mppi import MPPIConfig, mppi_path
from src.planners.rrt import RRTConfig, rrt_path
from src.planners.rrt_star import RRTStarConfig, rrt_star_path


@dataclass(frozen=True)
class Case:
    sequence: str
    case_id: int
    time_s: float
    depth_index: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--depth-cache-dir", default="data/processed/metric_depth")
    parser.add_argument("--timing-csv", default="outputs/tables/metric_depth_timing_depth_anything_v2_metric_indoor_small.csv")
    parser.add_argument("--output-dir", default="outputs")
    parser.add_argument("--trial-ids", nargs="*", default=[str(i) for i in range(1, 16)])
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--cases-per-trial", type=int, default=4)
    parser.add_argument("--depth-min-m", type=float, default=0.20)
    parser.add_argument("--depth-max-m", type=float, default=8.0)
    parser.add_argument("--horizontal-fov-deg", type=float, default=70.0)
    parser.add_argument("--voxel-size", type=float, default=0.15)
    parser.add_argument("--safety-radius", type=float, default=0.50)
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--lookahead-m", type=float, default=4.0)
    parser.add_argument("--sensor-period-ms", type=float, default=200.0)
    parser.add_argument("--command-ms", type=float, default=10.0)
    return parser.parse_args()


def _interp_xy(time_s: np.ndarray, xy: np.ndarray, query_s: float) -> np.ndarray:
    q = float(np.clip(query_s, time_s[0], time_s[-1]))
    return np.asarray([np.interp(q, time_s, xy[:, 0]), np.interp(q, time_s, xy[:, 1])], dtype=float)


def _heading_at(time_s: np.ndarray, xy: np.ndarray, query_s: float) -> np.ndarray:
    p0 = _interp_xy(time_s, xy, query_s - 0.15)
    p1 = _interp_xy(time_s, xy, query_s + 0.15)
    heading = p1 - p0
    norm = float(np.linalg.norm(heading))
    if norm <= 1e-6:
        return np.asarray([1.0, 0.0], dtype=float)
    return heading / norm


def _global_to_local_forward_lateral(
    points_xy: np.ndarray,
    origin_xy: np.ndarray,
    forward_xy: np.ndarray,
) -> np.ndarray:
    rel = np.asarray(points_xy, dtype=float) - origin_xy[None, :]
    lateral = np.asarray([-forward_xy[1], forward_xy[0]], dtype=float)
    forward = rel @ forward_xy
    side = rel @ lateral
    return np.column_stack([side, forward])


def _depth_topdown_occupancy(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    args: argparse.Namespace,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    started = time.perf_counter()
    points = depth_to_point_cloud(
        depth_m,
        intrinsics=intrinsics,
        depth_min_m=args.depth_min_m,
        depth_max_m=args.depth_max_m,
        stride=4,
    )
    xs = np.arange(-3.0, 3.0 + args.voxel_size, args.voxel_size)
    zs = np.arange(0.0, args.depth_max_m + args.voxel_size, args.voxel_size)
    grid = np.zeros((len(xs), len(zs)), dtype=bool)
    if len(points):
        valid = (points[:, 0] >= xs[0]) & (points[:, 0] <= xs[-1]) & (points[:, 2] >= zs[0]) & (points[:, 2] <= zs[-1])
        idx_x = np.floor((points[valid, 0] - xs[0]) / args.voxel_size).astype(int)
        idx_z = np.floor((points[valid, 2] - zs[0]) / args.voxel_size).astype(int)
        idx_x = np.clip(idx_x, 0, len(xs) - 1)
        idx_z = np.clip(idx_z, 0, len(zs) - 1)
        grid[idx_x, idx_z] = True
    try:
        from scipy.ndimage import binary_dilation

        inflate = int(np.ceil(args.safety_radius / args.voxel_size))
        grid = binary_dilation(grid, iterations=max(1, inflate))
    except Exception:
        pass
    return grid, xs, zs, (time.perf_counter() - started) * 1000.0


def _gt_topdown_occupancy(
    obstacles: tuple[Obstacle, ...],
    local_obstacles_xz: np.ndarray,
    xs: np.ndarray,
    zs: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    grid_x, grid_z = np.meshgrid(xs, zs, indexing="ij")
    gt = np.zeros_like(grid_x, dtype=bool)
    inflated = args.obstacle_radius + args.safety_radius
    for local in local_obstacles_xz:
        gt |= (grid_x - local[0]) ** 2 + (grid_z - local[1]) ** 2 <= inflated**2
    return gt


def _obstacle_points_from_depth_grid(grid: np.ndarray, xs: np.ndarray, zs: np.ndarray) -> np.ndarray:
    try:
        from scipy.ndimage import label

        labels, n = label(grid)
        points: list[tuple[float, float]] = []
        for idx in range(1, int(n) + 1):
            coords = np.argwhere(labels == idx)
            if len(coords) < 3:
                continue
            cx = float(np.mean(xs[coords[:, 0]]))
            cz = float(np.mean(zs[coords[:, 1]]))
            points.append((cx, cz))
        return np.asarray(points, dtype=float) if points else np.empty((0, 2), dtype=float)
    except Exception:
        coords = np.argwhere(grid)
        if len(coords) == 0:
            return np.empty((0, 2), dtype=float)
        stride = max(1, len(coords) // 80)
        sampled = coords[::stride]
        return np.column_stack([xs[sampled[:, 0]], zs[sampled[:, 1]]]).astype(float)


def _select_cases(sequence: str, times: np.ndarray, count: int) -> list[Case]:
    if len(times) == 0:
        return []
    indices = np.linspace(0, len(times) - 1, min(count, len(times)), dtype=int)
    return [Case(sequence=sequence, case_id=i, time_s=float(times[idx]), depth_index=int(idx)) for i, idx in enumerate(indices)]


def _timing_lookup(path: Path) -> dict[str, float]:
    if not path.exists() or path.stat().st_size == 0:
        return {}
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return {row["sequence"]: float(row["inference_seconds_per_frame"]) * 1000.0 for row in rows if row.get("sequence")}


def _run_planner(planner: str, obstacles_xz: np.ndarray, args: argparse.Namespace, seed: int) -> tuple[np.ndarray, float, bool, str]:
    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([0.0, args.lookahead_m], dtype=float)
    started = time.perf_counter()
    try:
        if planner == "rrt":
            result = rrt_path(
                start,
                goal,
                obstacles_xz,
                RRTConfig(max_iterations=900, step_size_m=0.25, obstacle_radius_m=args.voxel_size, safety_distance_m=args.safety_radius, seed=seed),
                num_points=120,
            )
        elif planner == "rrt_star":
            result = rrt_star_path(
                start,
                goal,
                obstacles_xz,
                RRTStarConfig(max_iterations=700, step_size_m=0.25, obstacle_radius_m=args.voxel_size, safety_distance_m=args.safety_radius, seed=seed),
                num_points=120,
            )
        elif planner == "mppi":
            result = mppi_path(
                start,
                goal,
                obstacles_xz,
                MPPIConfig(num_rollouts=384, horizon_steps=48, max_iterations=6, obstacle_radius_m=args.voxel_size, safety_distance_m=args.safety_radius, seed=seed),
                num_points=120,
            )
        else:
            raise ValueError(planner)
        return result.trajectory_xy, (time.perf_counter() - started) * 1000.0, False, ""
    except Exception as exc:  # planner failure is part of the benchmark
        fallback = np.linspace(start, goal, 120)
        return fallback, (time.perf_counter() - started) * 1000.0, True, str(exc)


def _score_path(path_xz: np.ndarray, gt_obstacles_xz: np.ndarray, args: argparse.Namespace) -> tuple[float, str]:
    if len(gt_obstacles_xz) == 0:
        return float("inf"), "safe"
    clearance = pairwise_ground_distances(path_xz, gt_obstacles_xz).min(axis=1) - args.obstacle_radius
    min_clearance = float(np.min(clearance))
    if min_clearance <= 0.0:
        return min_clearance, "collision"
    if min_clearance < args.safety_radius:
        return min_clearance, "violation"
    return min_clearance, "safe"


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _aggregate(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    groups = sorted({str(row["planner"]) for row in rows})
    out = []
    for planner in groups:
        group = [row for row in rows if row["planner"] == planner]
        n = len(group)
        out.append(
            {
                "planner": planner,
                "cases": n,
                "safe_pct": round(sum(row["status"] == "safe" for row in group) / n * 100.0, 2),
                "violation_pct": round(sum(row["status"] == "violation" for row in group) / n * 100.0, 2),
                "collision_pct": round(sum(row["status"] == "collision" for row in group) / n * 100.0, 2),
                "mean_iou": round(float(np.mean([float(row["occupancy_iou"]) for row in group])), 4),
                "mean_total_ms": round(float(np.mean([float(row["total_perception_planning_ms"]) for row in group])), 3),
                "mean_planner_ms": round(float(np.mean([float(row["planner_time_ms"]) for row in group])), 3),
                "mean_min_clearance_m": round(float(np.mean([float(row["min_clearance_m"]) for row in group])), 4),
            }
        )
    return out


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    trial_infos = read_trial_overview(dataset_dir)
    timing = _timing_lookup(Path(args.timing_csv))
    detail_rows: list[dict[str, object]] = []
    for sequence in [str(item) for item in args.trial_ids]:
        cache_path = Path(args.depth_cache_dir) / sequence / f"metric_depth_{args.fps:g}fps.npz"
        if not cache_path.exists() or sequence not in trial_infos:
            print(f"Skip {sequence}: missing metric depth cache or metadata")
            continue
        times, depth_stack, meta = load_cached_metric_depth(cache_path)
        optitrack = load_optitrack(dataset_dir, sequence)
        xy = np.column_stack([optitrack["ground_x_m"], optitrack["ground_y_m"]])
        cases = _select_cases(sequence, times, args.cases_per_trial)
        if not cases:
            continue
        intrinsics = intrinsics_from_fov(depth_stack.shape[2], depth_stack.shape[1], args.horizontal_fov_deg)
        for case in cases:
            pose = _interp_xy(optitrack["time_s"], xy, case.time_s)
            heading = _heading_at(optitrack["time_s"], xy, case.time_s)
            gt_local = _global_to_local_forward_lateral(obstacle_array(trial_infos[sequence].obstacles), pose, heading)
            depth_grid, xs, zs, occ_ms = _depth_topdown_occupancy(depth_stack[case.depth_index], intrinsics, args)
            gt_grid = _gt_topdown_occupancy(trial_infos[sequence].obstacles, gt_local, xs, zs, args)
            union = int(np.sum(depth_grid | gt_grid))
            inter = int(np.sum(depth_grid & gt_grid))
            iou = 1.0 if union == 0 else inter / union
            depth_obstacles = _obstacle_points_from_depth_grid(depth_grid, xs, zs)
            depth_inference_ms = float(timing.get(sequence, 0.0))
            for planner in ("rrt", "rrt_star", "mppi"):
                path, planner_ms, failed, reason = _run_planner(planner, depth_obstacles, args, seed=1000 + case.case_id)
                min_clearance, status = _score_path(path, gt_local, args)
                if failed:
                    status = "collision"
                total_ms = args.sensor_period_ms + depth_inference_ms + occ_ms + planner_ms + args.command_ms
                detail_rows.append(
                    {
                        "sequence": sequence,
                        "case_id": case.case_id,
                        "time_s": round(case.time_s, 4),
                        "planner": planner,
                        "map_source": "metric_depth_cache",
                        "model_id": str(meta.get("model_id", "")),
                        "device": str(meta.get("device", "")),
                        "platform": platform.platform(),
                        "machine": platform.machine(),
                        "occupancy_iou": round(float(iou), 4),
                        "depth_obstacle_components": int(len(depth_obstacles)),
                        "gt_obstacles": int(len(gt_local)),
                        "depth_inference_ms": round(depth_inference_ms, 3),
                        "occupancy_update_ms": round(occ_ms, 3),
                        "planner_time_ms": round(planner_ms, 3),
                        "total_perception_planning_ms": round(total_ms, 3),
                        "min_clearance_m": round(min_clearance, 4),
                        "collision_count": int(status == "collision"),
                        "violation_count": int(status in {"violation", "collision"}),
                        "status": status,
                        "planner_failed": int(failed),
                        "failure_reason": reason,
                    }
                )
    if not detail_rows:
        raise SystemExit("No depth occupancy planner rows produced.")
    output_dir = Path(args.output_dir)
    detail_path = output_dir / "tables" / "depth_occupancy_planner_detail.csv"
    summary_path = output_dir / "tables" / "depth_occupancy_planner_summary.csv"
    _write_csv(detail_path, detail_rows)
    _write_csv(summary_path, _aggregate(detail_rows))
    print(f"Wrote {detail_path}")
    print(f"Wrote {summary_path}")


if __name__ == "__main__":
    main()

