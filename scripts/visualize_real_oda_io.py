#!/usr/bin/env python3
"""Visualize one real ODA trial input/output example.

The figure is intended for defense discussion: it shows the real ODA input
signals, the planner outputs, and the ground-truth safety labels on one page.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from experiments.batch_benchmark_planners import plan_baselines
from src.oda_io import dataset_root, load_optitrack, obstacle_array, read_trial_overview


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--trial-id", default="3")
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument(
        "--risk-features",
        default="outputs/tables/perception_risk_features_depth_anything_v2_small_50.csv",
    )
    parser.add_argument("--planner-metrics", default="outputs/tables/batch_planner_metrics_300.csv")
    parser.add_argument("--output", default=None)
    return parser.parse_args()


def read_rows(path: Path, sequence: str) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return [row for row in csv.DictReader(f) if str(row.get("sequence")) == str(sequence)]


def as_float(rows: list[dict[str, str]], column: str) -> np.ndarray:
    values: list[float] = []
    for row in rows:
        try:
            values.append(float(row[column]))
        except (KeyError, TypeError, ValueError):
            values.append(np.nan)
    return np.asarray(values, dtype=float)


def normalized(values: np.ndarray) -> np.ndarray:
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return np.zeros_like(values)
    lo = float(np.nanmin(finite))
    hi = float(np.nanmax(finite))
    if hi <= lo:
        return np.zeros_like(values)
    return (values - lo) / (hi - lo)


def draw_ground_plane(
    ax: plt.Axes,
    sequence: str,
    human_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    paths: list[object],
    obstacle_radius_m: float,
    safety_distance_m: float,
) -> None:
    colors = {
        "human": "#1f77b4",
        "astar_grid": "#6f42c1",
        "rrt": "#f97316",
        "rrt_star": "#8c564b",
        "mppi": "#db2777",
    }
    ax.plot(human_xy[:, 0], human_xy[:, 1], color=colors["human"], linewidth=2.5, label="human OptiTrack")
    for path in paths:
        if path.name not in {"astar_grid", "rrt", "rrt_star", "mppi"}:
            continue
        ax.plot(
            path.trajectory_xy[:, 0],
            path.trajectory_xy[:, 1],
            color=colors.get(path.name, "#444444"),
            linewidth=1.9,
            linestyle="--",
            label=path.name,
        )
        ax.scatter(path.waypoints[:, 0], path.waypoints[:, 1], s=12, alpha=0.42, color=colors.get(path.name, "#444444"))

    for idx, obstacle in enumerate(obstacles_xy):
        ax.add_patch(plt.Circle(obstacle, obstacle_radius_m, color="#ef4444", alpha=0.35))
        ax.add_patch(
            plt.Circle(
                obstacle,
                obstacle_radius_m + safety_distance_m,
                fill=False,
                color="#ef4444",
                linestyle="--",
                linewidth=1.4,
            )
        )
        ax.text(obstacle[0], obstacle[1], f"obs {idx}", ha="center", va="center", fontsize=8)

    ax.scatter(human_xy[0, 0], human_xy[0, 1], color="#16a34a", s=52, zorder=5, label="start")
    ax.scatter(human_xy[-1, 0], human_xy[-1, 1], color="#111827", s=52, zorder=5, label="goal")
    ax.set_title(f"Real ODA trial {sequence}: planner output vs ground truth")
    ax.set_xlabel("OptiTrack x [m]")
    ax.set_ylabel("OptiTrack z [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.22)
    ax.legend(fontsize=7, loc="best")

    all_xy = np.vstack([human_xy, obstacles_xy, *[path.trajectory_xy for path in paths if path.name in {"astar_grid", "rrt", "rrt_star", "mppi"}]])
    pad = 0.75
    ax.set_xlim(float(all_xy[:, 0].min()) - pad, float(all_xy[:, 0].max()) + pad)
    ax.set_ylim(float(all_xy[:, 1].min()) - pad, float(all_xy[:, 1].max()) + pad)


def draw_sensor_timeline(ax: plt.Axes, risk_rows: list[dict[str, str]]) -> None:
    if not risk_rows:
        ax.text(0.5, 0.5, "No risk feature CSV rows found", ha="center", va="center")
        ax.axis("off")
        return
    time_s = as_float(risk_rows, "time_s")
    depth = normalized(as_float(risk_rows, "depth_median"))
    radar = normalized(as_float(risk_rows, "radar_peak"))
    imu = normalized(as_float(risk_rows, "imu_acc_norm"))
    ax.plot(time_s, depth, label="depth median norm", color="#7c3aed", linewidth=1.8)
    ax.plot(time_s, radar, label="radar peak norm", color="#0891b2", linewidth=1.8)
    ax.plot(time_s, imu, label="IMU accel norm", color="#f97316", linewidth=1.8)
    ax.set_title("Real sensor-derived features")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("normalized value")
    ax.grid(True, alpha=0.22)
    ax.legend(fontsize=8, loc="best")


def draw_clearance(ax: plt.Axes, risk_rows: list[dict[str, str]], safety_distance_m: float) -> None:
    if not risk_rows:
        ax.text(0.5, 0.5, "No risk feature CSV rows found", ha="center", va="center")
        ax.axis("off")
        return
    time_s = as_float(risk_rows, "time_s")
    clearance = as_float(risk_rows, "clearance_m")
    ax.plot(time_s, clearance, color="#2563eb", linewidth=2.0, label="ground-truth clearance")
    ax.axhline(safety_distance_m, color="#dc2626", linestyle="--", linewidth=1.4, label="safety threshold")
    future_times = np.asarray(
        [float(row["time_s"]) for row in risk_rows if row.get("future_risk_label") == "future_risk"],
        dtype=float,
    )
    if future_times.size:
        future_y = np.interp(future_times, time_s, clearance)
        ax.scatter(future_times, future_y, color="#dc2626", s=28, label="future_risk frames", zorder=5)
    ax.set_title("Ground-truth safety label over time")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("clearance [m]")
    ax.grid(True, alpha=0.22)
    ax.legend(fontsize=8, loc="best")


def draw_metric_table(ax: plt.Axes, rows: list[dict[str, str]], sequence: str) -> None:
    ax.axis("off")
    wanted = ["human", "astar_grid", "rrt", "rrt_star", "mppi"]
    by_method = {row.get("method"): row for row in rows}
    table_rows = []
    for method in wanted:
        row = by_method.get(method)
        if not row:
            continue
        table_rows.append(
            [
                method,
                row.get("waypoint_count", ""),
                row.get("path_length_m", ""),
                row.get("min_boundary_clearance_m", ""),
                row.get("collision", ""),
                row.get("safety_violation", ""),
            ]
        )

    ax.set_title(f"Output metrics from batch_planner_metrics_300.csv, trial {sequence}", loc="left", pad=10)
    if not table_rows:
        ax.text(0.02, 0.88, "No planner metric CSV rows found.", transform=ax.transAxes)
        return

    table = ax.table(
        cellText=table_rows,
        colLabels=["method", "waypoints", "length m", "min clearance m", "coll.", "viol."],
        loc="center",
        cellLoc="center",
        colLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table.scale(1.0, 1.4)
    for (row_idx, _), cell in table.get_celld().items():
        if row_idx == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#e5e7eb")
        else:
            cell.set_facecolor("#ffffff" if row_idx % 2 else "#f8fafc")


def main() -> int:
    args = parse_args()
    sequence = str(args.trial_id)
    dataset_dir = dataset_root(args.dataset_root)
    output = Path(args.output or f"outputs/figures/real_oda_io_trial_{sequence}.png")

    trial_info = read_trial_overview(dataset_dir)[sequence]
    optitrack = load_optitrack(dataset_dir, sequence)
    human_xy = np.column_stack([optitrack["ground_x_m"], optitrack["ground_y_m"]])
    obstacles_xy = obstacle_array(trial_info.obstacles)

    planned, failures = plan_baselines(
        sequence=sequence,
        human_xy=human_xy,
        obstacles_xy=obstacles_xy,
        obstacle_radius_m=args.obstacle_radius,
        safety_distance_m=args.safety_distance,
        astar_resolution_m=0.10,
        skip_astar=False,
        skip_rrt=False,
        rrt_iterations=1500,
        rrt_step_size_m=0.35,
        rrt_seed=7 + int(sequence),
        planner_seed=7,
        skip_rrt_star=False,
        rrt_star_iterations=2500,
        rrt_star_step_size_m=0.30,
        rrt_star_neighbor_radius_m=0.75,
        skip_mppi=False,
        mppi_rollouts=512,
        mppi_horizon_steps=60,
        mppi_iterations=10,
    )
    if failures:
        print(f"Planner failures: {failures}", file=sys.stderr)
    paths = [path for path, _ in planned]

    risk_rows = read_rows(Path(args.risk_features), sequence)
    metric_rows = read_rows(Path(args.planner_metrics), sequence)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(14.2, 11.0), constrained_layout=True)
    grid = fig.add_gridspec(3, 2, height_ratios=[1.15, 1.0, 0.56])
    draw_ground_plane(fig.add_subplot(grid[:2, 0]), sequence, human_xy, obstacles_xy, paths, args.obstacle_radius, args.safety_distance)
    draw_sensor_timeline(fig.add_subplot(grid[0, 1]), risk_rows)
    draw_clearance(fig.add_subplot(grid[1, 1]), risk_rows, args.safety_distance)
    draw_metric_table(fig.add_subplot(grid[2, :]), metric_rows, sequence)

    table_fig = plt.figure(figsize=(9.8, 2.6), constrained_layout=True)
    draw_metric_table(table_fig.add_subplot(111), metric_rows, sequence)
    table_output = output.with_name(output.stem + "_metrics.png")

    fig.suptitle(
        "Real-data input/output example: ODA metadata + OptiTrack + sensor features -> planner/risk outputs",
        fontsize=13,
        fontweight="bold",
    )
    fig.savefig(output, dpi=180)
    table_fig.savefig(table_output, dpi=180)
    plt.close(fig)
    plt.close(table_fig)
    print(f"Wrote {output}")
    print(f"Wrote {table_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
