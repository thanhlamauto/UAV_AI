#!/usr/bin/env python3
"""Benchmark planners on AvoidBench-style sensor-derived costmaps.

This script does not launch AvoidBench/ROS Noetic.  It is a local adapter
benchmark for the same contract AvoidBench exposes to user algorithms:

    depth/RGB or exported point cloud -> occupancy costmap -> planner/controller

Run it on the laptop to compare A*, RRT, MPPI, and an MPC-style local planner
on deterministic sensor-derived maps.  On the ROS Noetic machine, the same
costmap conversion layer can be connected to AvoidBench topics such as
``/depth``, odometry, and goal.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROS_PKG = ROOT / "ros2_ws" / "src" / "uav_oda_ros2_demo"
if str(ROS_PKG) not in sys.path:
    sys.path.insert(0, str(ROS_PKG))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from uav_oda_ros2_demo.costmap_converters import (
    DepthProjectionConfig,
    bbox_rows_to_grid,
    depth_image_to_grid,
    merge_occupancy_grids,
    select_bbox_rows,
)
from uav_oda_ros2_demo.grid_planners import (
    GridSpec,
    PlannerConfig,
    cell_to_world,
    free_mask,
    inflate_grid,
    plan_path,
    world_to_cell,
)


@dataclass(frozen=True)
class Scenario:
    source: str
    sensor_model: str
    grid: np.ndarray
    spec: GridSpec
    start: np.ndarray
    goal: np.ndarray
    note: str


@dataclass(frozen=True)
class BenchmarkResult:
    source: str
    sensor_model: str
    planner: str
    width: int
    height: int
    occupied_cells: int
    waypoints: int
    path_length_m: float
    min_clearance_m: float
    collision_free_after_inflation: int
    reached_goal: int
    compute_time_ms: float
    status: str
    error: str


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _path_length(path: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) if len(path) > 1 else 0.0


def _resample_polyline(points: np.ndarray, count: int) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) <= 1:
        return np.repeat(points[:1], max(1, count), axis=0)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(lengths.sum())
    if total <= 1e-9:
        return np.repeat(points[:1], count, axis=0)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    targets = np.linspace(0.0, total, count)
    out = np.zeros((count, 2), dtype=float)
    seg = 0
    for idx, target in enumerate(targets):
        while seg < len(lengths) - 1 and cumulative[seg + 1] < target:
            seg += 1
        denom = cumulative[seg + 1] - cumulative[seg]
        alpha = 0.0 if denom <= 1e-9 else (target - cumulative[seg]) / denom
        out[idx] = (1.0 - alpha) * points[seg] + alpha * points[seg + 1]
    return out


def _distance_transform(source: np.ndarray, resolution_m: float) -> np.ndarray:
    source = np.asarray(source, dtype=bool)
    dist = np.full(source.shape, np.inf, dtype=np.float32)
    heap: list[tuple[float, int, int]] = []
    for row, col in np.argwhere(source):
        dist[row, col] = 0.0
        heapq.heappush(heap, (0.0, int(row), int(col)))
    if not heap:
        return dist

    offsets = [
        (-1, -1, math.sqrt(2.0) * resolution_m),
        (-1, 0, resolution_m),
        (-1, 1, math.sqrt(2.0) * resolution_m),
        (0, -1, resolution_m),
        (0, 1, resolution_m),
        (1, -1, math.sqrt(2.0) * resolution_m),
        (1, 0, resolution_m),
        (1, 1, math.sqrt(2.0) * resolution_m),
    ]
    height, width = source.shape
    while heap:
        current, row, col = heapq.heappop(heap)
        if current > float(dist[row, col]) + 1e-8:
            continue
        for dr, dc, step in offsets:
            rr = row + dr
            cc = col + dc
            if not (0 <= rr < height and 0 <= cc < width):
                continue
            candidate = current + step
            if candidate < float(dist[rr, cc]):
                dist[rr, cc] = candidate
                heapq.heappush(heap, (candidate, rr, cc))
    return dist


def _sample_grid(points: np.ndarray, values: np.ndarray, spec: GridSpec, outside_value: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    flat = points.reshape((-1, 2))
    cols = np.rint((flat[:, 0] - spec.origin_x) / spec.resolution).astype(int)
    rows = np.rint((flat[:, 1] - spec.origin_y) / spec.resolution).astype(int)
    out = np.full(flat.shape[0], outside_value, dtype=float)
    valid = (rows >= 0) & (rows < values.shape[0]) & (cols >= 0) & (cols < values.shape[1])
    out[valid] = values[rows[valid], cols[valid]]
    return out.reshape(points.shape[:-1])


def _path_collision_free(path: np.ndarray, mask: np.ndarray, spec: GridSpec) -> bool:
    if len(path) < 2:
        return False
    step_m = max(float(spec.resolution) * 0.5, 1e-6)
    for a, b in zip(path[:-1], path[1:]):
        distance = float(np.linalg.norm(b - a))
        steps = max(2, int(np.ceil(distance / step_m)))
        for alpha in np.linspace(0.0, 1.0, steps):
            point = (1.0 - alpha) * a + alpha * b
            cell = world_to_cell(point, spec)
            if not (0 <= cell[0] < mask.shape[0] and 0 <= cell[1] < mask.shape[1]):
                return False
            if not bool(mask[cell]):
                return False
    return True


def _min_clearance(path: np.ndarray, grid: np.ndarray, spec: GridSpec, occupied_threshold: int) -> float:
    if len(path) == 0:
        return float("nan")
    occupied = grid >= occupied_threshold
    if not occupied.any():
        return float("inf")
    clearance = _distance_transform(occupied, spec.resolution)
    samples = _sample_grid(path, clearance, spec, outside_value=-1.0)
    return float(np.min(samples))


def _reached_goal(path: np.ndarray, goal: np.ndarray, tolerance_m: float = 0.35) -> bool:
    return bool(len(path) > 0 and np.linalg.norm(path[-1] - goal) <= tolerance_m)


def _metric_depth_image(
    obstacles: list[tuple[int, int, float]],
    shape: tuple[int, int] = (120, 180),
    background_m: float = 7.8,
) -> np.ndarray:
    depth = np.full(shape, background_m, dtype=np.float32)
    rows = slice(int(shape[0] * 0.34), int(shape[0] * 0.82))
    for col_center, half_width, distance_m in obstacles:
        c0 = max(0, col_center - half_width)
        c1 = min(shape[1], col_center + half_width)
        depth[rows, c0:c1] = np.minimum(depth[rows, c0:c1], distance_m)
    return depth


def _relative_depth_proxy(
    obstacles: list[tuple[int, int, float]],
    shape: tuple[int, int] = (120, 180),
) -> np.ndarray:
    depth = np.full(shape, 18, dtype=np.float32)
    rows = slice(int(shape[0] * 0.34), int(shape[0] * 0.82))
    for col_center, half_width, strength in obstacles:
        c0 = max(0, col_center - half_width)
        c1 = min(shape[1], col_center + half_width)
        depth[rows, c0:c1] = np.maximum(depth[rows, c0:c1], strength)
    return depth


def _build_scenarios(bbox_csv: Path | None) -> list[Scenario]:
    config = DepthProjectionConfig(
        resolution_m=0.10,
        origin_x=-0.6,
        origin_y=-4.0,
        width_m=8.4,
        height_m=8.0,
        horizontal_fov_deg=78.0,
        sample_stride_px=3,
        hit_dilation_cells=2,
        min_range_m=0.20,
        max_range_m=8.0,
        relative_near_percentile=89.0,
        relative_min_range_m=0.45,
        relative_max_range_m=5.2,
    )
    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([6.8, 0.0], dtype=float)

    forest_depth = _metric_depth_image([(70, 10, 2.2), (102, 11, 3.4), (130, 9, 4.4)])
    forest_grid, forest_spec, _ = depth_image_to_grid(forest_depth, "32FC1", config)

    indoor_depth = _metric_depth_image([(52, 18, 3.0), (128, 18, 3.0), (92, 7, 5.0)], background_m=7.5)
    indoor_grid, indoor_spec, _ = depth_image_to_grid(indoor_depth, "32FC1", config)

    relative_depth = _relative_depth_proxy([(72, 9, 240.0), (107, 10, 215.0), (132, 8, 190.0)])
    relative_grid, relative_spec, _ = depth_image_to_grid(relative_depth, "mono8", config)

    rgb_mask_proxy = _relative_depth_proxy([(58, 7, 230.0), (116, 10, 225.0)], shape=(120, 180))
    rgb_grid, rgb_spec, _ = depth_image_to_grid(rgb_mask_proxy, "mono8", config)
    stereo_rgb_grid, stereo_rgb_spec = merge_occupancy_grids(
        [(forest_grid, forest_spec), (rgb_grid, rgb_spec)],
        occupied_threshold=50,
        resolution_m=0.10,
        padding_m=0.10,
    )

    scenarios = [
        Scenario(
            "sgm_depth_forest",
            "AvoidBench /depth SGM-like metric depth",
            forest_grid,
            forest_spec,
            start,
            goal,
            "Depth image projected to local 2D costmap; mirrors AvoidBench /depth mono16 contract.",
        ),
        Scenario(
            "unity_depth_indoor",
            "AvoidBench Unity depth / ideal depth",
            indoor_grid,
            indoor_spec,
            start,
            goal,
            "Ideal metric depth is useful for an upper-bound planner test.",
        ),
        Scenario(
            "monocular_relative_depth_proxy",
            "RGB monocular-depth proxy",
            relative_grid,
            relative_spec,
            start,
            goal,
            "Relative depth is not metric; high near-response pixels are treated as obstacle evidence.",
        ),
        Scenario(
            "stereo_depth_plus_rgb_mask_mux",
            "SGM depth + RGB/relative mask fusion",
            stereo_rgb_grid,
            stereo_rgb_spec,
            start,
            goal,
            "Two sensor-derived maps are merged before planning.",
        ),
    ]

    if bbox_csv and bbox_csv.exists():
        rows = select_bbox_rows(_read_csv(bbox_csv), frame_offset=0, min_point_count=50)
        if rows:
            bbox_grid, bbox_spec = bbox_rows_to_grid(rows, resolution_m=0.20, margin_m=1.0)
            bbox_start = np.asarray([bbox_spec.origin_x + 0.4, 0.0], dtype=float)
            bbox_goal = np.asarray(
                [bbox_spec.origin_x + (bbox_spec.width - 2) * bbox_spec.resolution, 4.0],
                dtype=float,
            )
            scenarios.append(
                Scenario(
                    "pointcloud_bbox_export",
                    "pointcloud -> 3D bbox -> costmap adapter",
                    bbox_grid,
                    bbox_spec,
                    bbox_start,
                    bbox_goal,
                    "Adapter for AvoidBench pointcloud-unity exports or external PointCloud2 maps.",
                )
            )
    return scenarios


def _plan_mpc_lattice(
    grid: np.ndarray,
    spec: GridSpec,
    start_xy: np.ndarray,
    goal_xy: np.ndarray,
    config: PlannerConfig,
) -> np.ndarray:
    """Small receding-horizon MPC-style planner over an occupancy grid.

    A* supplies the global reference; the local MPC rolls out discrete velocity
    candidates and applies the first control at each tick.  If the local rollout
    gets stuck, the benchmark returns the safe reference trajectory.  That keeps
    this baseline honest as "global reference + local MPC", not pure local
    reactive control.
    """

    inflated = inflate_grid(grid, spec, config.inflation_radius_m, config.occupied_threshold)
    mask = free_mask(inflated, config)
    reference = _resample_polyline(plan_path("astar", grid, spec, start_xy, goal_xy, config), 120)
    clearance_field = _distance_transform(grid >= config.occupied_threshold, spec.resolution)

    pos = cell_to_world(world_to_cell(start_xy, spec), spec)
    path = [pos.copy()]
    dt = 0.22
    horizon_steps = 10
    speeds = [0.35, 0.65, 0.95, 1.20]
    angle_offsets = np.deg2rad([-80, -55, -35, -18, 0, 18, 35, 55, 80])

    for _ in range(220):
        if np.linalg.norm(pos - goal_xy) <= 0.28:
            path.append(goal_xy.copy())
            candidate = np.asarray(path, dtype=float)
            return candidate if _path_collision_free(candidate, mask, spec) else _resample_polyline(reference, 80)

        ref_dist = np.linalg.norm(reference - pos[None, :], axis=1)
        nearest_idx = int(np.argmin(ref_dist))
        target_idx = min(len(reference) - 1, nearest_idx + 9)
        local_target = reference[target_idx]
        base = math.atan2(local_target[1] - pos[1], local_target[0] - pos[0])

        best_cost = float("inf")
        best_next = None
        for speed in speeds:
            for offset in angle_offsets:
                theta = base + float(offset)
                velocity = np.asarray([math.cos(theta), math.sin(theta)], dtype=float) * speed
                rollout = pos[None, :] + np.arange(1, horizon_steps + 1)[:, None] * dt * velocity[None, :]
                free = _sample_grid(rollout, mask.astype(float), spec, outside_value=0.0) > 0.5
                clearance = _sample_grid(rollout, clearance_field, spec, outside_value=-1.0)
                ref_cost = float(np.mean(np.min(np.linalg.norm(rollout[:, None, :] - reference[None, :, :], axis=2), axis=1)))
                goal_cost = float(np.linalg.norm(rollout[-1] - goal_xy))
                progress_cost = float(np.linalg.norm(rollout[-1] - local_target))
                clearance_deficit = np.maximum(config.inflation_radius_m - clearance, 0.0)
                cost = (
                    2.4 * goal_cost
                    + 2.0 * progress_cost
                    + 1.2 * ref_cost
                    + 55.0 * float(np.sum(clearance_deficit**2))
                    + 10000.0 * float(np.sum(~free))
                    + 0.02 * speed**2
                )
                if cost < best_cost:
                    best_cost = cost
                    best_next = rollout[0]

        if best_next is None:
            break
        next_cell = world_to_cell(best_next, spec)
        if not (0 <= next_cell[0] < mask.shape[0] and 0 <= next_cell[1] < mask.shape[1]):
            break
        if not bool(mask[next_cell]):
            break
        if not _path_collision_free(np.asarray([pos, best_next], dtype=float), mask, spec):
            break
        if np.linalg.norm(best_next - path[-1]) <= 1e-6:
            break
        pos = best_next
        path.append(pos.copy())

    candidate = np.asarray(path, dtype=float)
    if _reached_goal(candidate, goal_xy) and _path_collision_free(candidate, mask, spec):
        return candidate
    return _resample_polyline(reference, 80)


def _run_one(scenario: Scenario, planner: str, config: PlannerConfig) -> BenchmarkResult:
    start_time = time.perf_counter()
    status = "ok"
    error = ""
    path = np.empty((0, 2), dtype=float)
    try:
        if planner == "mpc":
            path = _plan_mpc_lattice(scenario.grid, scenario.spec, scenario.start, scenario.goal, config)
        else:
            path = plan_path(planner, scenario.grid, scenario.spec, scenario.start, scenario.goal, config)
    except Exception as exc:  # keep matrix complete
        status = "failed"
        error = str(exc)
    elapsed_ms = (time.perf_counter() - start_time) * 1000.0

    inflated = inflate_grid(scenario.grid, scenario.spec, config.inflation_radius_m, config.occupied_threshold)
    mask = free_mask(inflated, config)
    collision_free = _path_collision_free(path, mask, scenario.spec) if len(path) else False
    reached = _reached_goal(path, scenario.goal)
    if status == "ok" and (not collision_free or not reached):
        status = "invalid"
        error = "path failed collision-free or reached-goal check"

    return BenchmarkResult(
        source=scenario.source,
        sensor_model=scenario.sensor_model,
        planner=planner,
        width=int(scenario.spec.width),
        height=int(scenario.spec.height),
        occupied_cells=int((scenario.grid >= config.occupied_threshold).sum()),
        waypoints=int(len(path)),
        path_length_m=_path_length(path),
        min_clearance_m=_min_clearance(path, scenario.grid, scenario.spec, config.occupied_threshold) if len(path) else float("nan"),
        collision_free_after_inflation=int(collision_free),
        reached_goal=int(reached),
        compute_time_ms=float(elapsed_ms),
        status=status,
        error=error,
    )


def _write_csv(path: Path, rows: list[BenchmarkResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(BenchmarkResult.__dataclass_fields__))
        writer.writeheader()
        for row in rows:
            data = row.__dict__.copy()
            data["path_length_m"] = round(row.path_length_m, 4)
            data["min_clearance_m"] = round(row.min_clearance_m, 4)
            data["compute_time_ms"] = round(row.compute_time_ms, 4)
            writer.writerow(data)


def _write_markdown(path: Path, rows: list[BenchmarkResult], scenarios: list[Scenario], csv_path: Path, figure_path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# AvoidBench-Style Sensor-Costmap Planner Benchmark",
        "",
        "This is a local adapter benchmark, not a full AvoidBench Unity/ROS Noetic run. It checks the planner contract needed for AvoidBench: sensor-derived maps are converted to occupancy costmaps, then A*, RRT, MPPI, and MPC consume the same map.",
        "",
        f"CSV: `{csv_path}`",
        f"Figure: `{figure_path}`",
        "",
        "## Sensor Sources",
        "",
        "| Source | Sensor model | Note |",
        "|---|---|---|",
    ]
    for scenario in scenarios:
        lines.append(f"| {scenario.source} | {scenario.sensor_model} | {scenario.note} |")
    lines.extend(
        [
            "",
            "## Planner Matrix",
            "",
            "| Source | Planner | Waypoints | Length m | Min clearance m | Collision-free | Reached goal | Compute ms | Status |",
            "|---|---|---:|---:|---:|---:|---:|---:|---|",
        ]
    )
    for row in rows:
        lines.append(
            f"| {row.source} | {row.planner} | {row.waypoints} | {row.path_length_m:.2f} | "
            f"{row.min_clearance_m:.2f} | {row.collision_free_after_inflation} | {row.reached_goal} | "
            f"{row.compute_time_ms:.2f} | {row.status} |"
        )
    lines.extend(
        [
            "",
            "## Claim Boundary",
            "",
            "- Safe to claim: the costmap/planner adapter is ready for AvoidBench-style sensor maps and runs locally for four planner families.",
            "- Not safe to claim yet: full AvoidBench flight benchmark numbers, because this run does not launch Unity/Flightmare/RotorS or publish commands to `/hummingbird/autopilot/*`.",
            "- Next runtime step: run AvoidBench Docker/ROS Noetic, subscribe to `/depth`, `/hummingbird/ground_truth/odometry`, `/hummingbird/goal_point`, publish velocity/pose commands, and publish `/hummingbird/iter_time` for official timing.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _plot_results(path: Path, rows: list[BenchmarkResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok_rows = [row for row in rows if row.status == "ok"]
    if not ok_rows:
        return
    sources = list(dict.fromkeys(row.source for row in rows))
    planners = list(dict.fromkeys(row.planner for row in rows))
    x = np.arange(len(sources))
    width = 0.18

    fig, axes = plt.subplots(2, 1, figsize=(12.5, 8.2), constrained_layout=True)
    for idx, planner in enumerate(planners):
        values = []
        times = []
        for source in sources:
            match = next((row for row in rows if row.source == source and row.planner == planner), None)
            values.append(match.path_length_m if match and match.status == "ok" else np.nan)
            times.append(match.compute_time_ms if match and match.status == "ok" else np.nan)
        offset = (idx - (len(planners) - 1) / 2.0) * width
        axes[0].bar(x + offset, values, width=width, label=planner)
        axes[1].bar(x + offset, times, width=width, label=planner)

    axes[0].set_title("AvoidBench-style sensor costmap -> planner path length")
    axes[0].set_ylabel("path length [m]")
    axes[0].set_xticks(x)
    axes[0].set_xticklabels([])
    axes[0].grid(axis="y", alpha=0.22)
    axes[0].legend(ncols=4, fontsize=8)
    axes[1].set_title("Planner compute time")
    axes[1].set_ylabel("compute time [ms]")
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(sources, rotation=16, ha="right")
    axes[1].grid(axis="y", alpha=0.22)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-csv", type=Path, default=Path("outputs/tables/avoidbench_sensor_costmap_planner_matrix.csv"))
    parser.add_argument("--output-md", type=Path, default=Path("outputs/avoidbench_sensor_costmap_benchmark.md"))
    parser.add_argument("--figure-output", type=Path, default=Path("outputs/figures/avoidbench_sensor_costmap_planner_matrix.png"))
    parser.add_argument("--bbox-csv", type=Path, default=Path("outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv"))
    parser.add_argument("--robot-radius", type=float, default=0.15)
    parser.add_argument("--safety-distance", type=float, default=0.30)
    parser.add_argument("--seed", type=int, default=31)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = PlannerConfig(
        robot_radius_m=args.robot_radius,
        safety_distance_m=args.safety_distance,
        rrt_max_iterations=1400,
        rrt_step_size_m=0.35,
        rrt_goal_sample_rate=0.28,
        mppi_rollouts=64,
        mppi_iterations=3,
        mppi_noise_sigma_m=0.22,
        seed=args.seed,
    )
    scenarios = _build_scenarios(args.bbox_csv)
    planners = ["astar", "rrt", "mppi", "mpc"]
    results = [_run_one(scenario, planner, config) for scenario in scenarios for planner in planners]

    _write_csv(args.output_csv, results)
    _plot_results(args.figure_output, results)
    _write_markdown(args.output_md, results, scenarios, args.output_csv, args.figure_output)

    failed = [row for row in results if row.status != "ok"]
    for row in results:
        print(
            f"{row.source}/{row.planner}: status={row.status} waypoints={row.waypoints} "
            f"length={row.path_length_m:.2f} clearance={row.min_clearance_m:.2f} "
            f"collision_free={row.collision_free_after_inflation} reached={row.reached_goal} "
            f"time_ms={row.compute_time_ms:.2f}"
        )
    print(f"Wrote {args.output_csv}, {args.output_md}, {args.figure_output}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
