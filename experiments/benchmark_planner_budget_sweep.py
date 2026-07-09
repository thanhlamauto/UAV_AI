#!/usr/bin/env python3
"""Run a small reviewer-facing planner budget sweep on ODA trials.

The full 300-trial benchmark remains in ``batch_planner_metrics_300.csv``.
This script writes separate reviewer tables and does not overwrite the main
benchmark artifacts.
"""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from experiments.batch_benchmark_planners import evaluate_path, make_time_like_human
from src.oda_bench_downstream import read_csv, write_csv
from src.oda_io import dataset_root, load_optitrack, obstacle_array, read_trial_overview
from src.planners.mppi import MPPIConfig, mppi_path
from src.planners.rrt import RRTConfig, rrt_path
from src.planners.rrt_star import RRTStarConfig, rrt_star_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--manifest", default="outputs/tables/target_300_trials_manifest.csv")
    parser.add_argument("--outputs-dir", default="outputs")
    parser.add_argument("--limit", type=int, default=45)
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument("--warning-clearance", type=float, default=0.80)
    parser.add_argument("--future-risk-horizon", type=float, default=1.0)
    parser.add_argument("--rrt-iterations", nargs="*", type=int, default=[300, 600, 900])
    parser.add_argument("--rrt-star-iterations", nargs="*", type=int, default=[300, 600, 900])
    parser.add_argument("--mppi-rollouts", nargs="*", type=int, default=[96, 192, 384])
    parser.add_argument("--mppi-iterations", nargs="*", type=int, default=[3, 5, 8])
    parser.add_argument("--seed", type=int, default=7)
    return parser.parse_args()


def trial_ids_from_manifest(path: str | Path, limit: int) -> list[str]:
    return [str(row["sequence"]) for row in read_csv(path)[:limit]]


def aggregate(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[tuple[str, str], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[(str(row["method"]), str(row["budget_label"]))].append(row)
    out: list[dict[str, object]] = []
    for (method, budget), items in sorted(grouped.items()):
        n = len(items)
        out.append(
            {
                "method": method,
                "budget_label": budget,
                "trials": n,
                "collision_rate": round(float(np.mean([int(r["collision"]) for r in items])), 6),
                "safety_violation_rate": round(float(np.mean([int(r["safety_violation"]) for r in items])), 6),
                "mean_min_clearance_m": round(float(np.mean([float(r["min_boundary_clearance_m"]) for r in items])), 6),
                "mean_path_length_m": round(float(np.mean([float(r["path_length_m"]) for r in items])), 6),
                "mean_smoothness": round(float(np.mean([float(r["smoothness_heading_change"]) for r in items])), 6),
                "mean_compute_time_ms": round(float(np.mean([float(r["planner_compute_time_ms"]) for r in items])), 6),
                "p95_compute_time_ms": round(float(np.percentile([float(r["planner_compute_time_ms"]) for r in items], 95)), 6),
            }
        )
    return out


def main() -> None:
    args = parse_args()
    root = dataset_root(args.dataset_root)
    trials = read_trial_overview(root)
    trial_ids = trial_ids_from_manifest(args.manifest, args.limit)
    rows: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []

    for sequence in trial_ids:
        trial = trials[str(sequence)]
        opt = load_optitrack(root, sequence)
        human_xy = np.column_stack([opt["ground_x_m"], opt["ground_y_m"]])
        human_time = opt["time_s"]
        time_s = make_time_like_human(human_time, len(human_xy))
        obstacles_xy = obstacle_array(trial.obstacles)
        start = human_xy[0]
        goal = human_xy[-1]
        num_points = len(human_xy)

        for iterations in args.rrt_iterations:
            label = f"iter={iterations}"
            started = perf_counter()
            try:
                path = rrt_path(
                    start,
                    goal,
                    obstacles_xy,
                    config=RRTConfig(
                        max_iterations=iterations,
                        step_size_m=0.35,
                        obstacle_radius_m=args.obstacle_radius,
                        safety_distance_m=args.safety_distance,
                        seed=args.seed + int(sequence) + iterations,
                    ),
                    num_points=num_points,
                )
                compute_ms = (perf_counter() - started) * 1000.0
                row = evaluate_path(
                    sequence,
                    "rrt",
                    time_s,
                    path.trajectory_xy,
                    obstacles_xy,
                    args.obstacle_radius,
                    args.safety_distance,
                    args.warning_clearance,
                    args.future_risk_horizon,
                    compute_ms,
                    len(path.waypoints),
                )
                row["budget_label"] = label
                rows.append(row)
            except Exception as exc:
                failures.append({"sequence": sequence, "method": "rrt", "budget_label": label, "reason": str(exc)})

        for iterations in args.rrt_star_iterations:
            label = f"iter={iterations}"
            started = perf_counter()
            try:
                path = rrt_star_path(
                    start,
                    goal,
                    obstacles_xy,
                    config=RRTStarConfig(
                        max_iterations=iterations,
                        step_size_m=0.35,
                        neighbor_radius_m=0.75,
                        obstacle_radius_m=args.obstacle_radius,
                        safety_distance_m=args.safety_distance,
                        seed=args.seed + int(sequence) + iterations,
                    ),
                    num_points=num_points,
                )
                compute_ms = (perf_counter() - started) * 1000.0
                row = evaluate_path(
                    sequence,
                    "rrt_star",
                    time_s,
                    path.trajectory_xy,
                    obstacles_xy,
                    args.obstacle_radius,
                    args.safety_distance,
                    args.warning_clearance,
                    args.future_risk_horizon,
                    compute_ms,
                    len(path.waypoints),
                )
                row["budget_label"] = label
                rows.append(row)
            except Exception as exc:
                failures.append({"sequence": sequence, "method": "rrt_star", "budget_label": label, "reason": str(exc)})

        for rollouts, iterations in zip(args.mppi_rollouts, args.mppi_iterations):
            label = f"rollouts={rollouts},iter={iterations}"
            started = perf_counter()
            try:
                path = mppi_path(
                    start,
                    goal,
                    obstacles_xy,
                    config=MPPIConfig(
                        num_rollouts=rollouts,
                        horizon_steps=50,
                        max_iterations=iterations,
                        obstacle_radius_m=args.obstacle_radius,
                        safety_distance_m=args.safety_distance,
                        seed=args.seed + int(sequence) + rollouts + iterations,
                    ),
                    num_points=num_points,
                )
                compute_ms = (perf_counter() - started) * 1000.0
                row = evaluate_path(
                    sequence,
                    "mppi",
                    time_s,
                    path.trajectory_xy,
                    obstacles_xy,
                    args.obstacle_radius,
                    args.safety_distance,
                    args.warning_clearance,
                    args.future_risk_horizon,
                    compute_ms,
                    len(path.waypoints),
                )
                row["budget_label"] = label
                rows.append(row)
            except Exception as exc:
                failures.append({"sequence": sequence, "method": "mppi", "budget_label": label, "reason": str(exc)})

    tables = Path(args.outputs_dir) / "tables"
    write_csv(tables / "reviewer_planner_budget_sweep_detail.csv", rows)
    write_csv(tables / "reviewer_planner_budget_sweep_summary.csv", aggregate(rows))
    write_csv(tables / "reviewer_planner_budget_sweep_failures.csv", failures)
    print(f"Wrote {tables / 'reviewer_planner_budget_sweep_summary.csv'}")
    print(f"Wrote {tables / 'reviewer_planner_budget_sweep_detail.csv'}")
    print(f"Wrote {tables / 'reviewer_planner_budget_sweep_failures.csv'}")


if __name__ == "__main__":
    main()
