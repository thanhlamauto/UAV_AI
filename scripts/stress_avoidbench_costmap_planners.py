#!/usr/bin/env python3
"""Stress-test planners with corrupted AvoidBench-style sensor costmaps.

Unlike the clean adapter benchmark, this script plans on a sensor-estimated map
but evaluates collision and clearance on a separate ground-truth map.  This
exposes the failure mode that matters in a real AvoidBench run: perception can
miss or shift obstacles even when the planner interface itself works.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROS_PKG = ROOT / "ros2_ws" / "src" / "uav_oda_ros2_demo"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROS_PKG) not in sys.path:
    sys.path.insert(0, str(ROS_PKG))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from scripts.benchmark_avoidbench_costmap_planners import (
    _min_clearance,
    _path_collision_free,
    _path_length,
    _plan_mpc_lattice,
    _reached_goal,
)
from uav_oda_ros2_demo.grid_planners import (
    GridSpec,
    PlannerConfig,
    cell_to_world,
    free_mask,
    inflate_grid,
    nearest_free_cell,
    plan_path,
    world_to_cell,
)


@dataclass(frozen=True)
class StressCase:
    name: str
    sensor_model: str
    gt_grid: np.ndarray
    sensor_grid: np.ndarray
    spec: GridSpec
    start: np.ndarray
    goal: np.ndarray
    note: str


@dataclass(frozen=True)
class StressResult:
    stress_case: str
    sensor_model: str
    planner: str
    sensor_occupied_cells: int
    gt_occupied_cells: int
    waypoints: int
    path_length_m: float
    gt_min_clearance_m: float
    gt_collision_free_after_inflation: int
    reached_goal: int
    compute_time_ms: float
    status: str
    note: str
    error: str


def _blank_grid(spec: GridSpec) -> np.ndarray:
    return np.zeros((spec.height, spec.width), dtype=np.int8)


def _fill_rect(grid: np.ndarray, spec: GridSpec, x0: float, x1: float, y0: float, y1: float, value: int = 100) -> None:
    c0 = max(0, int(np.floor((min(x0, x1) - spec.origin_x) / spec.resolution)))
    c1 = min(spec.width - 1, int(np.ceil((max(x0, x1) - spec.origin_x) / spec.resolution)))
    r0 = max(0, int(np.floor((min(y0, y1) - spec.origin_y) / spec.resolution)))
    r1 = min(spec.height - 1, int(np.ceil((max(y0, y1) - spec.origin_y) / spec.resolution)))
    grid[r0 : r1 + 1, c0 : c1 + 1] = value


def _base_gt_map(spec: GridSpec) -> np.ndarray:
    grid = _blank_grid(spec)
    _fill_rect(grid, spec, 2.40, 3.00, -1.15, 1.15)
    _fill_rect(grid, spec, 5.35, 5.95, -0.78, 0.78)
    _fill_rect(grid, spec, 4.00, 4.65, 1.35, 2.75)
    _fill_rect(grid, spec, 4.05, 4.70, -2.75, -1.35)
    _fill_rect(grid, spec, 6.75, 7.20, 1.00, 2.20)
    return grid


def _narrow_gate_gt_map(spec: GridSpec) -> np.ndarray:
    grid = _blank_grid(spec)
    _fill_rect(grid, spec, 2.25, 2.85, -3.30, -0.55)
    _fill_rect(grid, spec, 2.25, 2.85, 0.55, 3.30)
    _fill_rect(grid, spec, 4.45, 5.05, -3.30, -0.65)
    _fill_rect(grid, spec, 4.45, 5.05, 0.65, 3.30)
    _fill_rect(grid, spec, 6.15, 6.65, -0.62, 0.62)
    return grid


def _remove_rect(grid: np.ndarray, spec: GridSpec, x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
    out = grid.copy()
    _fill_rect(out, spec, x0, x1, y0, y1, value=0)
    return out


def _shift_grid(grid: np.ndarray, rows: int, cols: int) -> np.ndarray:
    shifted = np.zeros_like(grid)
    src_r0 = max(0, -rows)
    src_r1 = min(grid.shape[0], grid.shape[0] - rows)
    src_c0 = max(0, -cols)
    src_c1 = min(grid.shape[1], grid.shape[1] - cols)
    dst_r0 = max(0, rows)
    dst_r1 = dst_r0 + max(0, src_r1 - src_r0)
    dst_c0 = max(0, cols)
    dst_c1 = dst_c0 + max(0, src_c1 - src_c0)
    if dst_r1 > dst_r0 and dst_c1 > dst_c0:
        shifted[dst_r0:dst_r1, dst_c0:dst_c1] = grid[src_r0:src_r1, src_c0:src_c1]
    return shifted


def _dropout_and_speckle(grid: np.ndarray, seed: int, dropout_rate: float, speckle_rate: float) -> np.ndarray:
    rng = np.random.default_rng(seed)
    out = grid.copy()
    occupied = out >= 50
    drop = occupied & (rng.random(out.shape) < dropout_rate)
    out[drop] = 0
    free = out < 50
    speckle = free & (rng.random(out.shape) < speckle_rate)
    out[speckle] = 100
    return out


def _range_limit(grid: np.ndarray, spec: GridSpec, max_x_m: float) -> np.ndarray:
    out = grid.copy()
    xs = spec.origin_x + np.arange(spec.width) * spec.resolution
    out[:, xs > max_x_m] = 0
    return out


def _add_false_positive_clutter(grid: np.ndarray, spec: GridSpec) -> np.ndarray:
    out = grid.copy()
    _fill_rect(out, spec, 1.15, 1.55, 1.55, 2.95)
    _fill_rect(out, spec, 3.45, 3.85, -3.20, -1.85)
    _fill_rect(out, spec, 6.15, 6.75, -2.75, -1.25)
    _fill_rect(out, spec, 7.20, 7.55, 0.85, 2.65)
    return out


def _build_cases(seed: int) -> list[StressCase]:
    spec = GridSpec(width=100, height=80, resolution=0.10, origin_x=-1.0, origin_y=-4.0)
    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([8.0, 0.0], dtype=float)
    gt = _base_gt_map(spec)
    narrow_gt = _narrow_gate_gt_map(spec)
    return [
        StressCase(
            "clean_depth_gt",
            "metric depth costmap, no corruption",
            gt,
            gt.copy(),
            spec,
            start,
            goal,
            "Control case; planner sees the same map used for evaluation.",
        ),
        StressCase(
            "depth_dropout_speckle",
            "SGM depth with false negatives and speckle",
            gt,
            _dropout_and_speckle(gt, seed=seed, dropout_rate=0.58, speckle_rate=0.004),
            spec,
            start,
            goal,
            "58% occupied-cell dropout plus sparse false positives.",
        ),
        StressCase(
            "blind_central_obstacle",
            "sensor miss: central obstacle absent from costmap",
            gt,
            _remove_rect(gt, spec, 2.25, 3.20, -1.40, 1.40),
            spec,
            start,
            goal,
            "Planner map misses the first blocking obstacle; evaluated against ground truth.",
        ),
        StressCase(
            "limited_range_fov",
            "depth range limit; far obstacle unseen",
            gt,
            _range_limit(gt, spec, max_x_m=4.75),
            spec,
            start,
            goal,
            "Only near-depth obstacles are visible; later obstacle is hidden.",
        ),
        StressCase(
            "pose_shift_40cm",
            "costmap shifted by pose/calibration error",
            gt,
            _shift_grid(gt, rows=4, cols=0),
            spec,
            start,
            goal,
            "Obstacle evidence is shifted +0.40 m in y before planning.",
        ),
        StressCase(
            "false_positive_clutter",
            "extra RGB/depth false-positive obstacles",
            gt,
            _add_false_positive_clutter(gt, spec),
            spec,
            start,
            goal,
            "Planner sees extra obstacles that do not exist in ground truth.",
        ),
        StressCase(
            "narrow_gate_clean",
            "clean depth but narrow gates",
            narrow_gt,
            narrow_gt.copy(),
            spec,
            start,
            goal,
            "Hard geometry with gate-like passages; tests inflation sensitivity.",
        ),
    ]


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


def _plan_straight_line(start_xy: np.ndarray, goal_xy: np.ndarray) -> np.ndarray:
    return _resample_polyline(np.asarray([start_xy, goal_xy], dtype=float), 80)


def _segment_free(mask: np.ndarray, spec: GridSpec, a: np.ndarray, b: np.ndarray) -> bool:
    return _path_collision_free(np.asarray([a, b], dtype=float), mask, spec)


def _point_free(mask: np.ndarray, spec: GridSpec, point: np.ndarray) -> bool:
    cell = world_to_cell(point, spec)
    return 0 <= cell[0] < mask.shape[0] and 0 <= cell[1] < mask.shape[1] and bool(mask[cell])


def _grid_bounds(spec: GridSpec) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray([spec.origin_x, spec.origin_y], dtype=float),
        np.asarray(
            [
                spec.origin_x + (spec.width - 1) * spec.resolution,
                spec.origin_y + (spec.height - 1) * spec.resolution,
            ],
            dtype=float,
        ),
    )


def _steer(from_xy: np.ndarray, to_xy: np.ndarray, step_size_m: float) -> np.ndarray:
    delta = to_xy - from_xy
    distance = float(np.linalg.norm(delta))
    if distance <= step_size_m or distance <= 1e-9:
        return to_xy.copy()
    return from_xy + delta / distance * step_size_m


def _plan_grid_rrt_star(
    grid: np.ndarray,
    spec: GridSpec,
    start_xy: np.ndarray,
    goal_xy: np.ndarray,
    config: PlannerConfig,
) -> np.ndarray:
    inflated = inflate_grid(grid, spec, config.inflation_radius_m, config.occupied_threshold)
    mask = free_mask(inflated, config)
    start = cell_to_world(nearest_free_cell(mask, world_to_cell(start_xy, spec)), spec)
    goal = cell_to_world(nearest_free_cell(mask, world_to_cell(goal_xy, spec)), spec)
    if _segment_free(mask, spec, start, goal):
        return np.asarray([start, goal], dtype=float)

    rng = np.random.default_rng(config.seed + 101)
    xy_min, xy_max = _grid_bounds(spec)
    nodes: list[np.ndarray] = [start]
    parents: list[int] = [-1]
    costs: list[float] = [0.0]
    best_goal_parent: int | None = None
    best_goal_cost = float("inf")
    neighbor_radius = max(0.75, config.rrt_step_size_m * 2.8)

    for _ in range(config.rrt_max_iterations):
        sample = goal if rng.random() < config.rrt_goal_sample_rate else rng.uniform(xy_min, xy_max)
        node_arr = np.asarray(nodes)
        nearest_idx = int(np.argmin(np.linalg.norm(node_arr - sample, axis=1)))
        new = _steer(nodes[nearest_idx], sample, config.rrt_step_size_m)
        if not _point_free(mask, spec, new):
            continue
        if not _segment_free(mask, spec, nodes[nearest_idx], new):
            continue

        neighbor_indices = np.where(np.linalg.norm(node_arr - new, axis=1) <= neighbor_radius)[0].tolist()
        parent_idx = nearest_idx
        parent_cost = costs[nearest_idx] + float(np.linalg.norm(new - nodes[nearest_idx]))
        for idx in neighbor_indices:
            edge_cost = float(np.linalg.norm(new - nodes[idx]))
            candidate_cost = costs[idx] + edge_cost
            if candidate_cost < parent_cost and _segment_free(mask, spec, nodes[idx], new):
                parent_idx = idx
                parent_cost = candidate_cost

        nodes.append(new)
        parents.append(parent_idx)
        costs.append(parent_cost)
        new_idx = len(nodes) - 1

        for idx in neighbor_indices:
            if idx == parent_idx:
                continue
            edge_cost = float(np.linalg.norm(nodes[idx] - new))
            candidate_cost = parent_cost + edge_cost
            if candidate_cost < costs[idx] and _segment_free(mask, spec, new, nodes[idx]):
                parents[idx] = new_idx
                costs[idx] = candidate_cost

        if _segment_free(mask, spec, new, goal):
            goal_cost = parent_cost + float(np.linalg.norm(goal - new))
            if goal_cost < best_goal_cost:
                best_goal_cost = goal_cost
                best_goal_parent = new_idx

    if best_goal_parent is None:
        raise RuntimeError("RRT* failed to find a path")

    path = [goal]
    idx = best_goal_parent
    while idx >= 0:
        path.append(nodes[idx])
        idx = parents[idx]
    path.reverse()
    return np.asarray(path, dtype=float)


def _plan_greedy_reactive(
    grid: np.ndarray,
    spec: GridSpec,
    start_xy: np.ndarray,
    goal_xy: np.ndarray,
    config: PlannerConfig,
) -> np.ndarray:
    inflated = inflate_grid(grid, spec, config.inflation_radius_m, config.occupied_threshold)
    mask = free_mask(inflated, config)
    pos = cell_to_world(nearest_free_cell(mask, world_to_cell(start_xy, spec)), spec)
    goal = cell_to_world(nearest_free_cell(mask, world_to_cell(goal_xy, spec)), spec)
    path = [pos.copy()]
    step = 0.28
    angle_offsets = np.deg2rad([0, -20, 20, -40, 40, -65, 65, -95, 95, 140, -140])
    visited: dict[tuple[int, int], int] = {}

    for _ in range(420):
        if np.linalg.norm(pos - goal) <= 0.30:
            path.append(goal.copy())
            return np.asarray(path, dtype=float)
        base = float(np.arctan2(goal[1] - pos[1], goal[0] - pos[0]))
        best_cost = float("inf")
        best_next: np.ndarray | None = None
        for offset in angle_offsets:
            theta = base + float(offset)
            candidate = pos + step * np.asarray([np.cos(theta), np.sin(theta)], dtype=float)
            if not _point_free(mask, spec, candidate):
                continue
            if not _segment_free(mask, spec, pos, candidate):
                continue
            cell = world_to_cell(candidate, spec)
            visit_penalty = 0.12 * visited.get(cell, 0)
            progress = float(np.linalg.norm(candidate - goal))
            heading_penalty = abs(float(offset)) * 0.08
            cost = progress + heading_penalty + visit_penalty
            if cost < best_cost:
                best_cost = cost
                best_next = candidate
        if best_next is None:
            break
        pos = best_next
        path.append(pos.copy())
        visited[world_to_cell(pos, spec)] = visited.get(world_to_cell(pos, spec), 0) + 1
    return np.asarray(path, dtype=float)


def _run_one(case: StressCase, planner: str, config: PlannerConfig) -> StressResult:
    start_time = time.perf_counter()
    path = np.empty((0, 2), dtype=float)
    error = ""
    try:
        if planner == "straight_line":
            path = _plan_straight_line(case.start, case.goal)
        elif planner == "greedy_reactive":
            path = _plan_greedy_reactive(case.sensor_grid, case.spec, case.start, case.goal, config)
        elif planner == "rrt_star":
            path = _plan_grid_rrt_star(case.sensor_grid, case.spec, case.start, case.goal, config)
        elif planner == "mpc":
            path = _plan_mpc_lattice(case.sensor_grid, case.spec, case.start, case.goal, config)
        else:
            path = plan_path(planner, case.sensor_grid, case.spec, case.start, case.goal, config)
    except Exception as exc:
        error = str(exc)
    compute_ms = (time.perf_counter() - start_time) * 1000.0

    gt_inflated = inflate_grid(case.gt_grid, case.spec, config.inflation_radius_m, config.occupied_threshold)
    gt_mask = free_mask(gt_inflated, config)
    collision_free = _path_collision_free(path, gt_mask, case.spec) if len(path) else False
    reached_goal = _reached_goal(path, case.goal)
    if error:
        status = "planner_failed"
    elif not reached_goal:
        status = "invalid_goal"
    elif not collision_free:
        status = "unsafe_gt"
    else:
        status = "safe"

    return StressResult(
        stress_case=case.name,
        sensor_model=case.sensor_model,
        planner=planner,
        sensor_occupied_cells=int((case.sensor_grid >= config.occupied_threshold).sum()),
        gt_occupied_cells=int((case.gt_grid >= config.occupied_threshold).sum()),
        waypoints=int(len(path)),
        path_length_m=_path_length(path),
        gt_min_clearance_m=_min_clearance(path, case.gt_grid, case.spec, config.occupied_threshold) if len(path) else float("nan"),
        gt_collision_free_after_inflation=int(collision_free),
        reached_goal=int(reached_goal),
        compute_time_ms=float(compute_ms),
        status=status,
        note=case.note,
        error=error,
    )


def _write_csv(path: Path, rows: list[StressResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(StressResult.__dataclass_fields__))
        writer.writeheader()
        for row in rows:
            data = row.__dict__.copy()
            data["path_length_m"] = round(row.path_length_m, 4)
            data["gt_min_clearance_m"] = round(row.gt_min_clearance_m, 4)
            data["compute_time_ms"] = round(row.compute_time_ms, 4)
            writer.writerow(data)


def _write_markdown(path: Path, rows: list[StressResult], csv_path: Path, figure_path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    total = len(rows)
    safe = sum(1 for row in rows if row.status == "safe")
    lines = [
        "# AvoidBench-Style Stress Benchmark",
        "",
        "This stress test plans on corrupted sensor-estimated costmaps but evaluates safety on a separate ground-truth costmap.",
        "",
        f"Safe cases: `{safe}/{total}`",
        f"CSV: `{csv_path}`",
        f"Figure: `{figure_path}`",
        "",
        "| Stress case | Planner | Status | Length m | GT clearance m | GT collision-free | Reached goal | Compute ms |",
        "|---|---|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row.stress_case} | {row.planner} | {row.status} | {row.path_length_m:.2f} | "
            f"{row.gt_min_clearance_m:.2f} | {row.gt_collision_free_after_inflation} | "
            f"{row.reached_goal} | {row.compute_time_ms:.2f} |"
        )
    lines.extend(
        [
            "",
            "Interpretation:",
            "",
            "- `safe` means the path planned on the sensor map also clears the ground-truth inflated map.",
            "- `unsafe_gt` means the planner found a path, but the path is unsafe when checked against the true map.",
            "- Sensor miss, limited range, and pose shift are expected to expose failures; those failures are useful evidence that clean costmap results were optimistic.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def _plot(path: Path, rows: list[StressResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    cases = list(dict.fromkeys(row.stress_case for row in rows))
    planners = list(dict.fromkeys(row.planner for row in rows))
    status_score = {"safe": 1.0, "unsafe_gt": 0.0, "invalid_goal": -0.5, "planner_failed": -1.0}
    matrix = np.zeros((len(cases), len(planners)), dtype=float)
    for r, case in enumerate(cases):
        for c, planner in enumerate(planners):
            row = next(item for item in rows if item.stress_case == case and item.planner == planner)
            matrix[r, c] = status_score.get(row.status, -1.0)

    fig, axes = plt.subplots(1, 2, figsize=(13.6, 5.4), constrained_layout=True)
    image = axes[0].imshow(matrix, vmin=-1.0, vmax=1.0, cmap="RdYlGn")
    axes[0].set_title("GT safety after sensor-map planning")
    axes[0].set_xticks(np.arange(len(planners)))
    axes[0].set_xticklabels(planners, rotation=25, ha="right")
    axes[0].set_yticks(np.arange(len(cases)))
    axes[0].set_yticklabels(cases)
    for r in range(len(cases)):
        for c in range(len(planners)):
            row = next(item for item in rows if item.stress_case == cases[r] and item.planner == planners[c])
            label = {"safe": "S", "unsafe_gt": "U", "invalid_goal": "G", "planner_failed": "F"}.get(row.status, "?")
            axes[0].text(c, r, label, ha="center", va="center", color="black", fontweight="bold")
    fig.colorbar(image, ax=axes[0], fraction=0.046, pad=0.04)

    safe_rate = [
        sum(1 for row in rows if row.planner == planner and row.status == "safe")
        / max(1, sum(1 for row in rows if row.planner == planner))
        for planner in planners
    ]
    colors = ["#64748b", "#14b8a6", "#2563eb", "#f97316", "#8b5cf6", "#16a34a", "#dc2626"]
    axes[1].bar(planners, safe_rate, color=colors[: len(planners)])
    axes[1].set_ylim(0.0, 1.0)
    axes[1].set_title("Safe rate across stress cases")
    axes[1].set_ylabel("safe fraction")
    axes[1].tick_params(axis="x", rotation=25)
    for label in axes[1].get_xticklabels():
        label.set_horizontalalignment("right")
    axes[1].grid(axis="y", alpha=0.25)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-csv", type=Path, default=Path("outputs/tables/avoidbench_stress_cpu_baseline_matrix.csv"))
    parser.add_argument("--output-md", type=Path, default=Path("outputs/avoidbench_stress_cpu_baseline_benchmark.md"))
    parser.add_argument("--figure-output", type=Path, default=Path("outputs/figures/avoidbench_stress_cpu_baseline_matrix.png"))
    parser.add_argument("--robot-radius", type=float, default=0.12)
    parser.add_argument("--safety-distance", type=float, default=0.28)
    parser.add_argument("--seed", type=int, default=47)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = PlannerConfig(
        robot_radius_m=args.robot_radius,
        safety_distance_m=args.safety_distance,
        rrt_max_iterations=1600,
        rrt_step_size_m=0.35,
        rrt_goal_sample_rate=0.28,
        mppi_rollouts=72,
        mppi_iterations=3,
        mppi_noise_sigma_m=0.24,
        seed=args.seed,
    )
    cases = _build_cases(args.seed)
    planners = ["straight_line", "greedy_reactive", "astar", "rrt", "rrt_star", "mppi", "mpc"]
    results = [_run_one(case, planner, config) for case in cases for planner in planners]

    _write_csv(args.output_csv, results)
    _write_markdown(args.output_md, results, args.output_csv, args.figure_output)
    _plot(args.figure_output, results)

    for row in results:
        print(
            f"{row.stress_case}/{row.planner}: status={row.status} "
            f"length={row.path_length_m:.2f} gt_clearance={row.gt_min_clearance_m:.2f} "
            f"gt_free={row.gt_collision_free_after_inflation} reached={row.reached_goal} "
            f"time_ms={row.compute_time_ms:.2f}"
        )
    print(f"Wrote {args.output_csv}, {args.output_md}, {args.figure_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
