#!/usr/bin/env python3
"""Batch benchmark human ODA trajectories against simple planner baselines."""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from src.metrics import compute_trial_metrics, path_length
from src.oda_io import available_trial_ids, dataset_root, load_optitrack, obstacle_array, read_trial_overview
from src.planners.astar import AStarConfig, astar_path
from src.planners.baselines import PlannedPath, select_best_geometric_bypass, straight_line_path
from src.planners.mppi import MPPIConfig, mppi_path
from src.planners.rrt import RRTConfig, rrt_path
from src.planners.rrt_star import RRTStarConfig, rrt_star_path
from src.risk import (
    clearance_series,
    future_risk_labels,
    risk_labels_from_clearance,
    summarize_risk_labels,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--trial-ids", nargs="*", default=None)
    parser.add_argument(
        "--manifest",
        default=None,
        help="CSV manifest with a sequence column. Used when --trial-ids is omitted.",
    )
    parser.add_argument(
        "--readiness",
        default=None,
        help="Optional readiness CSV from check_oda_trial_readiness.py.",
    )
    parser.add_argument(
        "--ready-only",
        action="store_true",
        help="When --readiness is provided, run only rows where ready=1.",
    )
    parser.add_argument("--outputs-dir", default="outputs")
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument("--warning-clearance", type=float, default=0.80)
    parser.add_argument("--future-risk-horizon", type=float, default=1.0)
    parser.add_argument("--astar-resolution", type=float, default=0.10)
    parser.add_argument("--skip-astar", action="store_true")
    parser.add_argument("--skip-rrt", action="store_true")
    parser.add_argument("--rrt-iterations", type=int, default=1500)
    parser.add_argument("--rrt-step-size", type=float, default=0.35)
    parser.add_argument("--rrt-seed", type=int, default=7)
    parser.add_argument("--planner-seed", type=int, default=7)
    parser.add_argument("--skip-rrt-star", action="store_true")
    parser.add_argument("--rrt-star-iterations", type=int, default=2500)
    parser.add_argument("--rrt-star-step-size", type=float, default=0.30)
    parser.add_argument("--rrt-star-neighbor-radius", type=float, default=0.75)
    parser.add_argument("--skip-mppi", action="store_true")
    parser.add_argument("--mppi-rollouts", type=int, default=512)
    parser.add_argument("--mppi-horizon-steps", type=int, default=60)
    parser.add_argument("--mppi-iterations", type=int, default=10)
    return parser.parse_args()


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def read_sequence_column(path: str | Path, only_ready: bool = False) -> list[str]:
    sequences: list[str] = []
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            if only_ready and str(row.get("ready", "0")) != "1":
                continue
            sequences.append(str(row["sequence"]))
    return sequences


def resolve_trial_ids(args: argparse.Namespace, dataset_dir: Path) -> list[str]:
    if args.trial_ids:
        return [str(item) for item in args.trial_ids]
    if args.ready_only:
        if not args.readiness:
            raise ValueError("--ready-only requires --readiness")
        return read_sequence_column(args.readiness, only_ready=True)
    if args.manifest:
        return read_sequence_column(args.manifest, only_ready=False)
    if args.readiness:
        return read_sequence_column(args.readiness, only_ready=False)
    return available_trial_ids(dataset_dir)


def smoothness_score(trajectory_xy: np.ndarray) -> float:
    """Simple lower-is-smoother score from squared heading changes."""

    if len(trajectory_xy) < 3:
        return 0.0
    diffs = np.diff(trajectory_xy, axis=0)
    headings = np.unwrap(np.arctan2(diffs[:, 1], diffs[:, 0]))
    heading_delta = np.diff(headings)
    return float(np.mean(heading_delta**2))


def evaluate_path(
    sequence: str,
    method: str,
    time_s: np.ndarray,
    trajectory_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float,
    safety_distance_m: float,
    warning_clearance_m: float,
    future_risk_horizon_s: float,
    planner_compute_time_ms: float,
    waypoint_count: int,
) -> dict[str, object]:
    metrics = compute_trial_metrics(
        sequence=sequence,
        time_s=time_s,
        trajectory_xy=trajectory_xy,
        obstacles_xy=obstacles_xy,
        obstacle_radius_m=obstacle_radius_m,
        safety_distance_m=safety_distance_m,
    )
    clearance = clearance_series(trajectory_xy, obstacles_xy, obstacle_radius_m)
    labels = risk_labels_from_clearance(
        clearance,
        warning_clearance_m=warning_clearance_m,
        danger_clearance_m=safety_distance_m,
    )
    future = future_risk_labels(
        time_s,
        clearance,
        horizon_s=future_risk_horizon_s,
        danger_clearance_m=safety_distance_m,
    )
    risk = summarize_risk_labels(labels, future)
    row = metrics.as_row()
    row.update(
        {
            "method": method,
            "planner_compute_time_ms": round(planner_compute_time_ms, 4),
            "waypoint_count": waypoint_count,
            "smoothness_heading_change": round(smoothness_score(trajectory_xy), 6),
        }
    )
    row.update(risk.as_row())
    return row


def make_time_like_human(human_time_s: np.ndarray, num_points: int) -> np.ndarray:
    duration = float(human_time_s[-1] - human_time_s[0]) if len(human_time_s) else 0.0
    return np.linspace(0.0, duration, num_points)


def plan_baselines(
    sequence: str,
    human_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float,
    safety_distance_m: float,
    astar_resolution_m: float,
    skip_astar: bool,
    skip_rrt: bool,
    rrt_iterations: int,
    rrt_step_size_m: float,
    rrt_seed: int,
    planner_seed: int,
    skip_rrt_star: bool,
    rrt_star_iterations: int,
    rrt_star_step_size_m: float,
    rrt_star_neighbor_radius_m: float,
    skip_mppi: bool,
    mppi_rollouts: int,
    mppi_horizon_steps: int,
    mppi_iterations: int,
) -> tuple[list[tuple[PlannedPath, float]], list[dict[str, object]]]:
    start = human_xy[0]
    goal = human_xy[-1]
    num_points = len(human_xy)
    planned: list[tuple[PlannedPath, float]] = []
    failures: list[dict[str, object]] = []

    started = perf_counter()
    planned.append((straight_line_path(start, goal, num_points), (perf_counter() - started) * 1000.0))

    started = perf_counter()
    planned.append(
        (
            select_best_geometric_bypass(
                start=start,
                goal=goal,
                obstacles_xy=obstacles_xy,
                obstacle_radius_m=obstacle_radius_m,
                safety_distance_m=safety_distance_m,
                num_points=num_points,
            ),
            (perf_counter() - started) * 1000.0,
        )
    )

    if not skip_astar:
        started = perf_counter()
        try:
            planned.append(
                (
                    astar_path(
                        start=start,
                        goal=goal,
                        obstacles_xy=obstacles_xy,
                        config=AStarConfig(
                            resolution_m=astar_resolution_m,
                            obstacle_radius_m=obstacle_radius_m,
                            safety_distance_m=safety_distance_m,
                        ),
                        num_points=num_points,
                    ),
                    (perf_counter() - started) * 1000.0,
                )
            )
        except Exception as exc:  # keep batch benchmark moving
            failures.append({"sequence": sequence, "method": "astar_grid", "reason": str(exc)})
            print(f"Warning: A* failed for trial {sequence}: {exc}", file=sys.stderr)
    if not skip_rrt:
        started = perf_counter()
        try:
            planned.append(
                (
                    rrt_path(
                        start=start,
                        goal=goal,
                        obstacles_xy=obstacles_xy,
                        config=RRTConfig(
                            max_iterations=rrt_iterations,
                            step_size_m=rrt_step_size_m,
                            obstacle_radius_m=obstacle_radius_m,
                            safety_distance_m=safety_distance_m,
                            seed=rrt_seed,
                        ),
                        num_points=num_points,
                    ),
                    (perf_counter() - started) * 1000.0,
                )
            )
        except Exception as exc:  # keep batch benchmark moving
            failures.append({"sequence": sequence, "method": "rrt", "reason": str(exc)})
            print(f"Warning: RRT failed for trial {sequence}: {exc}", file=sys.stderr)

    if not skip_rrt_star:
        started = perf_counter()
        try:
            planned.append(
                (
                    rrt_star_path(
                        start=start,
                        goal=goal,
                        obstacles_xy=obstacles_xy,
                        config=RRTStarConfig(
                            max_iterations=rrt_star_iterations,
                            step_size_m=rrt_star_step_size_m,
                            neighbor_radius_m=rrt_star_neighbor_radius_m,
                            obstacle_radius_m=obstacle_radius_m,
                            safety_distance_m=safety_distance_m,
                            seed=planner_seed + int(sequence),
                        ),
                        num_points=num_points,
                    ),
                    (perf_counter() - started) * 1000.0,
                )
            )
        except Exception as exc:  # keep batch benchmark moving
            failures.append({"sequence": sequence, "method": "rrt_star", "reason": str(exc)})
            print(f"Warning: RRT* failed for trial {sequence}: {exc}", file=sys.stderr)

    if not skip_mppi:
        started = perf_counter()
        try:
            planned.append(
                (
                    mppi_path(
                        start=start,
                        goal=goal,
                        obstacles_xy=obstacles_xy,
                        config=MPPIConfig(
                            num_rollouts=mppi_rollouts,
                            horizon_steps=mppi_horizon_steps,
                            max_iterations=mppi_iterations,
                            obstacle_radius_m=obstacle_radius_m,
                            safety_distance_m=safety_distance_m,
                            seed=planner_seed + int(sequence),
                        ),
                        num_points=num_points,
                    ),
                    (perf_counter() - started) * 1000.0,
                )
            )
        except Exception as exc:  # keep batch benchmark moving
            failures.append({"sequence": sequence, "method": "mppi", "reason": str(exc)})
            print(f"Warning: MPPI failed for trial {sequence}: {exc}", file=sys.stderr)

    return planned, failures


def plot_comparison(
    sequence: str,
    human_xy: np.ndarray,
    paths: list[PlannedPath],
    obstacles_xy: np.ndarray,
    output_path: Path,
    obstacle_radius_m: float,
    safety_distance_m: float,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.0, 6.5), constrained_layout=True)
    colors = {
        "human": "#1f77b4",
        "straight_line": "#7f7f7f",
        "geometric_bypass": "#2ca02c",
        "geometric_bypass_not_needed": "#2ca02c",
        "astar_grid": "#9467bd",
        "rrt": "#ff7f0e",
        "rrt_star": "#8c564b",
        "mppi": "#e377c2",
    }
    ax.plot(human_xy[:, 0], human_xy[:, 1], label="human/OptiTrack", color=colors["human"], linewidth=2.5)
    for path in paths:
        ax.plot(
            path.trajectory_xy[:, 0],
            path.trajectory_xy[:, 1],
            label=path.name,
            color=colors.get(path.name, None),
            linewidth=1.8,
            linestyle="--",
        )
        ax.scatter(path.waypoints[:, 0], path.waypoints[:, 1], s=14, alpha=0.45)

    for idx, obstacle in enumerate(obstacles_xy):
        obstacle_circle = plt.Circle(obstacle, obstacle_radius_m, color="#d62728", alpha=0.35)
        safety_circle = plt.Circle(
            obstacle,
            obstacle_radius_m + safety_distance_m,
            fill=False,
            color="#d62728",
            linestyle="--",
            linewidth=1.4,
        )
        ax.add_patch(obstacle_circle)
        ax.add_patch(safety_circle)
        ax.text(obstacle[0], obstacle[1], f"obs {idx}", ha="center", va="center", fontsize=8)

    ax.scatter(human_xy[0, 0], human_xy[0, 1], color="#2ca02c", s=50, label="start")
    ax.scatter(human_xy[-1, 0], human_xy[-1, 1], color="#111111", s=50, label="goal")
    ax.set_title(f"ODA trial {sequence}: human vs planner baselines")
    ax.set_xlabel("OptiTrack x [m]")
    ax.set_ylabel("OptiTrack z [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best", fontsize=8)
    all_xy = np.vstack([human_xy, obstacles_xy, *[p.trajectory_xy for p in paths]])
    pad = 0.8
    ax.set_xlim(all_xy[:, 0].min() - pad, all_xy[:, 0].max() + pad)
    ax.set_ylim(all_xy[:, 1].min() - pad, all_xy[:, 1].max() + pad)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def aggregate_summary(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    if not rows:
        return []
    grouped: dict[str, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        grouped[str(row["method"])].append(row)

    summary = []
    for method, method_rows in sorted(grouped.items()):
        n = len(method_rows)
        summary.append(
            {
                "method": method,
                "trials": n,
                "collision_rate": round(sum(int(r["collision"]) for r in method_rows) / n, 4),
                "safety_violation_rate": round(sum(int(r["safety_violation"]) for r in method_rows) / n, 4),
                "mean_min_clearance_m": round(
                    float(np.mean([float(r["min_boundary_clearance_m"]) for r in method_rows])), 4
                ),
                "mean_path_length_m": round(float(np.mean([float(r["path_length_m"]) for r in method_rows])), 4),
                "mean_smoothness": round(
                    float(np.mean([float(r["smoothness_heading_change"]) for r in method_rows])), 6
                ),
                "mean_planner_compute_time_ms": round(
                    float(np.mean([float(r["planner_compute_time_ms"]) for r in method_rows])), 4
                ),
            }
        )
    return summary


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    outputs_dir = Path(args.outputs_dir)
    tables_dir = outputs_dir / "tables"
    figures_dir = outputs_dir / "figures"

    trials = read_trial_overview(dataset_dir)
    trial_ids = resolve_trial_ids(args, dataset_dir)

    rows: list[dict[str, object]] = []
    skipped: list[dict[str, object]] = []
    planner_failures: list[dict[str, object]] = []
    for sequence in trial_ids:
        sequence = str(sequence)
        try:
            trial = trials[sequence]
            if trial.obstacle_count == 0:
                raise ValueError("missing obstacle coordinates")
            optitrack = load_optitrack(dataset_dir, sequence)
            human_xy = np.column_stack([optitrack["ground_x_m"], optitrack["ground_y_m"]])
            obstacles_xy = obstacle_array(trial.obstacles)
            human_time = optitrack["time_s"]
            rows.append(
                evaluate_path(
                    sequence=sequence,
                    method="human",
                    time_s=human_time,
                    trajectory_xy=human_xy,
                    obstacles_xy=obstacles_xy,
                    obstacle_radius_m=args.obstacle_radius,
                    safety_distance_m=args.safety_distance,
                    warning_clearance_m=args.warning_clearance,
                    future_risk_horizon_s=args.future_risk_horizon,
                    planner_compute_time_ms=0.0,
                    waypoint_count=len(human_xy),
                )
            )

            planned, failures = plan_baselines(
                sequence=sequence,
                human_xy=human_xy,
                obstacles_xy=obstacles_xy,
                obstacle_radius_m=args.obstacle_radius,
                safety_distance_m=args.safety_distance,
                astar_resolution_m=args.astar_resolution,
                skip_astar=args.skip_astar,
                skip_rrt=args.skip_rrt,
                rrt_iterations=args.rrt_iterations,
                rrt_step_size_m=args.rrt_step_size,
                rrt_seed=args.rrt_seed + int(sequence),
                planner_seed=args.planner_seed,
                skip_rrt_star=args.skip_rrt_star,
                rrt_star_iterations=args.rrt_star_iterations,
                rrt_star_step_size_m=args.rrt_star_step_size,
                rrt_star_neighbor_radius_m=args.rrt_star_neighbor_radius,
                skip_mppi=args.skip_mppi,
                mppi_rollouts=args.mppi_rollouts,
                mppi_horizon_steps=args.mppi_horizon_steps,
                mppi_iterations=args.mppi_iterations,
            )
            planner_failures.extend(failures)
            synthetic_time = make_time_like_human(human_time, len(human_xy))
            for path, compute_ms in planned:
                rows.append(
                    evaluate_path(
                        sequence=sequence,
                        method=path.name,
                        time_s=synthetic_time,
                        trajectory_xy=path.trajectory_xy,
                        obstacles_xy=obstacles_xy,
                        obstacle_radius_m=args.obstacle_radius,
                        safety_distance_m=args.safety_distance,
                        warning_clearance_m=args.warning_clearance,
                        future_risk_horizon_s=args.future_risk_horizon,
                        planner_compute_time_ms=compute_ms,
                        waypoint_count=len(path.waypoints),
                    )
                )

            plot_comparison(
                sequence=sequence,
                human_xy=human_xy,
                paths=[path for path, _ in planned],
                obstacles_xy=obstacles_xy,
                output_path=figures_dir / f"planner_comparison_sample_{sequence}.png",
                obstacle_radius_m=args.obstacle_radius,
                safety_distance_m=args.safety_distance,
            )
        except Exception as exc:
            skipped.append({"sequence": sequence, "reason": str(exc)})
            print(f"Skipped trial {sequence}: {exc}", file=sys.stderr)

    write_csv(tables_dir / "batch_planner_metrics.csv", rows)
    write_csv(tables_dir / "planner_comparison_summary.csv", aggregate_summary(rows))
    write_csv(tables_dir / "batch_skipped_trials.csv", skipped)
    write_csv(tables_dir / "planner_failures.csv", planner_failures)
    print(f"Wrote {tables_dir / 'batch_planner_metrics.csv'}")
    print(f"Wrote {tables_dir / 'planner_comparison_summary.csv'}")
    print(f"Wrote {tables_dir / 'batch_skipped_trials.csv'}")
    print(f"Wrote {tables_dir / 'planner_failures.csv'}")
    print(f"Benchmarked {len(set(row['sequence'] for row in rows))} trials, skipped {len(skipped)}")


if __name__ == "__main__":
    main()
