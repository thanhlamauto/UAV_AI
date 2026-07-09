#!/usr/bin/env python3
"""Compute 3D trajectory feasibility proxies on local ODA trials.

The ODA plotting frame is `(x, z, y_height)`: OptiTrack `x` and `z` are the
ground plane, while OptiTrack `y` is height.  RRT/RRT*/MPPI still plan in the
ground-plane footprint; this script lifts those planned paths between the
recorded start and goal heights before computing 3D path geometry and
smoothness.  It intentionally does not report velocity because the planners do
not output time-parameterized velocity commands.

The smoothness numbers are geometry proxies, not full quadrotor dynamics:
smooth UAV flight would require bounded velocity, acceleration, jerk, yaw-rate,
and control-input variation.  Paths are resampled by arclength before scoring so
sharp corners are not hidden by sparse planner waypoints.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from experiments.batch_benchmark_planners import plan_baselines
from src.oda_io import available_trial_ids, dataset_root, load_optitrack, obstacle_array, read_trial_overview


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--trial-ids", nargs="*", default=None)
    parser.add_argument("--output", default="outputs/tables/trajectory_feasibility_3d_summary.csv")
    parser.add_argument("--detail-output", default="outputs/tables/trajectory_feasibility_3d_detail.csv")
    parser.add_argument("--figure-output", default="outputs/figures/trajectory_feasibility_3d_trial_345.png")
    parser.add_argument("--figure-trial", default="345")
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument("--rrt-iterations", type=int, default=1500)
    parser.add_argument("--rrt-star-iterations", type=int, default=2500)
    parser.add_argument("--mppi-rollouts", type=int, default=1024)
    parser.add_argument("--mppi-horizon-steps", type=int, default=96)
    parser.add_argument("--mppi-iterations", type=int, default=16)
    parser.add_argument("--trajectory-samples", type=int, default=240)
    parser.add_argument("--smoothing-passes", type=int, default=3)
    return parser.parse_args()


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"No rows to write to {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def path_length_3d(path_xyz: np.ndarray) -> float:
    if len(path_xyz) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(path_xyz, axis=0), axis=1).sum())


def resample_polyline_nd(points: np.ndarray, count: int) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        return np.empty((0, points.shape[1] if points.ndim == 2 else 0), dtype=float)
    if len(points) == 1 or count <= 1:
        return np.repeat(points[:1], max(1, count), axis=0)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(lengths.sum())
    if total <= 1e-9:
        return np.repeat(points[:1], count, axis=0)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    targets = np.linspace(0.0, total, count)
    out = np.empty((count, points.shape[1]), dtype=float)
    seg = 0
    for idx, target in enumerate(targets):
        while seg < len(lengths) - 1 and cumulative[seg + 1] < target:
            seg += 1
        span = cumulative[seg + 1] - cumulative[seg]
        alpha = 0.0 if span <= 1e-9 else (target - cumulative[seg]) / span
        out[idx] = (1.0 - alpha) * points[seg] + alpha * points[seg + 1]
    return out


def chaikin_corner_cut(points: np.ndarray, passes: int) -> np.ndarray:
    out = np.asarray(points, dtype=float)
    for _ in range(max(0, passes)):
        if len(out) <= 2:
            break
        refined = [out[0]]
        for a, b in zip(out[:-1], out[1:]):
            refined.append(0.75 * a + 0.25 * b)
            refined.append(0.25 * a + 0.75 * b)
        refined.append(out[-1])
        out = np.asarray(refined, dtype=float)
    return out


def min_obstacle_clearance_xy(path_xy: np.ndarray, obstacles_xy: np.ndarray, obstacle_radius_m: float) -> float:
    if len(path_xy) == 0 or len(obstacles_xy) == 0:
        return float("inf")
    distances = np.linalg.norm(path_xy[:, None, :] - obstacles_xy[None, :, :], axis=2)
    return float(np.min(distances) - obstacle_radius_m)


def smooth_path_xy_safe(
    path_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float,
    safety_distance_m: float,
    passes: int,
    samples: int,
) -> np.ndarray:
    for current_passes in range(max(0, passes), -1, -1):
        candidate = chaikin_corner_cut(path_xy, current_passes)
        candidate = resample_polyline_nd(candidate, samples)
        if min_obstacle_clearance_xy(candidate, obstacles_xy, obstacle_radius_m) >= safety_distance_m:
            return candidate
    return resample_polyline_nd(path_xy, samples)


def turn_angles(path_xyz: np.ndarray) -> np.ndarray:
    if len(path_xyz) < 3:
        return np.empty(0, dtype=float)
    diffs = np.diff(path_xyz, axis=0)
    norms = np.linalg.norm(diffs, axis=1)
    valid = norms > 1e-9
    if np.count_nonzero(valid) < 2:
        return np.empty(0, dtype=float)
    unit = diffs[valid] / norms[valid, None]
    dots = np.sum(unit[:-1] * unit[1:], axis=1)
    return np.arccos(np.clip(dots, -1.0, 1.0))


def smoothness_metrics_3d(path_xyz: np.ndarray) -> dict[str, float]:
    length = path_length_3d(path_xyz)
    if len(path_xyz) < 4 or length <= 1e-9:
        return {
            "turn_energy": 0.0,
            "max_turn_rad": 0.0,
            "curvature_energy": 0.0,
            "accel_energy": 0.0,
            "jerk_energy": 0.0,
        }

    angles = turn_angles(path_xyz)
    ds = length / max(len(path_xyz) - 1, 1)
    second = path_xyz[2:] - 2.0 * path_xyz[1:-1] + path_xyz[:-2]
    third = path_xyz[3:] - 3.0 * path_xyz[2:-1] + 3.0 * path_xyz[1:-2] - path_xyz[:-3]
    accel = second / max(ds**2, 1e-9)
    jerk = third / max(ds**3, 1e-9)
    curvature = angles / max(ds, 1e-9)
    return {
        "turn_energy": float(np.mean(angles**2)) if len(angles) else 0.0,
        "max_turn_rad": float(np.max(angles)) if len(angles) else 0.0,
        "curvature_energy": float(np.mean(curvature**2)) if len(curvature) else 0.0,
        "accel_energy": float(np.mean(np.linalg.norm(accel, axis=1) ** 2)) if len(accel) else 0.0,
        "jerk_energy": float(np.mean(np.linalg.norm(jerk, axis=1) ** 2)) if len(jerk) else 0.0,
    }


def lift_path_to_3d(path_xy: np.ndarray, start_h: float, goal_h: float) -> np.ndarray:
    if len(path_xy) == 0:
        return np.empty((0, 3), dtype=float)
    if len(path_xy) == 1:
        return np.column_stack([path_xy, np.asarray([start_h])])
    segment_lengths = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    alpha = np.linspace(0.0, 1.0, len(path_xy)) if s[-1] <= 1e-9 else s / s[-1]
    heights = (1.0 - alpha) * start_h + alpha * goal_h
    return np.column_stack([path_xy, heights])


def method_label(method: str) -> str:
    return {"human": "Human", "rrt": "RRT", "rrt_star": "RRT*", "mppi": "MPPI"}.get(method, method)


def evaluate(method: str, sequence: str, path_xyz: np.ndarray, duration_s: float, compute_ms: float) -> dict[str, object]:
    length = path_length_3d(path_xyz)
    smooth = smoothness_metrics_3d(path_xyz)
    return {
        "sequence": sequence,
        "method": method,
        "path_length_3d_m": round(length, 4),
        "duration_s": round(duration_s, 4),
        "smoothness_3d_turn_angle": round(smooth["turn_energy"], 6),
        "max_turn_angle_rad": round(smooth["max_turn_rad"], 6),
        "curvature_energy": round(smooth["curvature_energy"], 6),
        "accel_energy": round(smooth["accel_energy"], 6),
        "jerk_energy": round(smooth["jerk_energy"], 6),
        "planner_compute_time_ms": round(compute_ms, 4),
        "trajectory_points": len(path_xyz),
        "metric_scope": "3d_arclength_resampled_path_geometry_no_velocity_command",
    }


def summarize(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for method in ["human", "rrt", "rrt_star", "mppi"]:
        group = [row for row in rows if row["method"] == method]
        if not group:
            continue
        lengths = [float(row["path_length_3d_m"]) for row in group]
        smooth = [float(row["smoothness_3d_turn_angle"]) for row in group]
        max_turn = [float(row["max_turn_angle_rad"]) for row in group]
        curvature = [float(row["curvature_energy"]) for row in group]
        jerk = [float(row["jerk_energy"]) for row in group]
        compute = [float(row["planner_compute_time_ms"]) for row in group]
        out.append(
            {
                "method": method,
                "trials": len(group),
                "mean_path_length_3d_m": round(float(np.mean(lengths)), 4),
                "p95_path_length_3d_m": round(float(np.percentile(lengths, 95)), 4),
                "mean_smoothness_3d": round(float(np.mean(smooth)), 6),
                "p95_smoothness_3d": round(float(np.percentile(smooth, 95)), 6),
                "mean_max_turn_angle_rad": round(float(np.mean(max_turn)), 6),
                "mean_curvature_energy": round(float(np.mean(curvature)), 6),
                "mean_jerk_energy": round(float(np.mean(jerk)), 6),
                "mean_planner_compute_time_ms": round(float(np.mean(compute)), 4),
                "metric_scope": "3d_arclength_resampled_path_geometry_no_velocity_command",
            }
        )
    return out


def draw_cylinder(ax, center_x: float, center_z: float, radius: float, height: float, color: str, alpha: float) -> None:
    theta = np.linspace(0.0, 2.0 * np.pi, 48)
    y = np.array([0.0, height])
    tt, yy = np.meshgrid(theta, y)
    xx = center_x + radius * np.cos(tt)
    zz = center_z + radius * np.sin(tt)
    ax.plot_surface(xx, zz, yy, color=color, alpha=alpha, linewidth=0, shade=True)


def plot_trial_3d(path: Path, sequence: str, human_xyz: np.ndarray, planned_xyz: dict[str, np.ndarray], trial, obstacle_radius: float, safety_distance: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(10.2, 7.2), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")
    colors = {"human": "#2563eb", "rrt": "#f97316", "rrt_star": "#8b5cf6", "mppi": "#db2777"}
    labels = {"human": "Human/OptiTrack", "rrt": "RRT", "rrt_star": "RRT*", "mppi": "MPPI"}

    ax.plot(human_xyz[:, 0], human_xyz[:, 1], human_xyz[:, 2], color=colors["human"], linewidth=2.5, label=labels["human"])
    for method in ["rrt", "rrt_star", "mppi"]:
        traj = planned_xyz.get(method)
        if traj is None:
            continue
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color=colors[method], linewidth=1.9, linestyle="--", label=labels[method])

    for idx, obs in enumerate(trial.obstacles, start=1):
        draw_cylinder(ax, obs.x, obs.ground_y, obstacle_radius, obs.height_y, "#ef4444", 0.30)
        draw_cylinder(ax, obs.x, obs.ground_y, obstacle_radius + safety_distance, obs.height_y, "#f59e0b", 0.08)
        ax.text(obs.x, obs.ground_y, obs.height_y + 0.08, f"obs {idx}", color="#991b1b", ha="center")

    ax.scatter(human_xyz[0, 0], human_xyz[0, 1], human_xyz[0, 2], color="#16a34a", s=70, label="start")
    ax.scatter(human_xyz[-1, 0], human_xyz[-1, 1], human_xyz[-1, 2], color="#111827", s=70, marker="s", label="goal")
    ax.set_title(f"ODA trial {sequence}: 3D path feasibility proxy")
    ax.set_xlabel("OptiTrack x [m]")
    ax.set_ylabel("OptiTrack z [m]")
    ax.set_zlabel("height y [m]")
    all_xyz = np.vstack([human_xyz, *planned_xyz.values()])
    ax.set_xlim(float(all_xyz[:, 0].min()) - 0.8, float(all_xyz[:, 0].max()) + 0.8)
    ax.set_ylim(float(all_xyz[:, 1].min()) - 0.8, float(all_xyz[:, 1].max()) + 0.8)
    max_h = max(float(all_xyz[:, 2].max()), max(obs.height_y for obs in trial.obstacles))
    ax.set_zlim(0.0, max_h + 0.35)
    ax.view_init(elev=25, azim=-58)
    ax.legend(loc="upper right", fontsize=8)
    try:
        ax.set_box_aspect((np.ptp(ax.get_xlim()), np.ptp(ax.get_ylim()), np.ptp(ax.get_zlim())))
    except Exception:
        pass
    fig.savefig(path, dpi=190, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    trial_infos = read_trial_overview(dataset_dir)
    trial_ids = [str(item) for item in args.trial_ids] if args.trial_ids else available_trial_ids(dataset_dir)

    rows: list[dict[str, object]] = []
    figure_payload: tuple[np.ndarray, dict[str, np.ndarray], object] | None = None
    for sequence in trial_ids:
        if sequence not in trial_infos or trial_infos[sequence].obstacle_count == 0:
            continue
        opt = load_optitrack(dataset_dir, sequence)
        time_s = opt["time_s"]
        duration_s = float(time_s[-1] - time_s[0])
        human_xy = np.column_stack([opt["ground_x_m"], opt["ground_y_m"]])
        human_xyz = np.column_stack([opt["ground_x_m"], opt["ground_y_m"], opt["height_m"]])
        human_xyz = resample_polyline_nd(human_xyz, args.trajectory_samples)
        obstacles_xy = obstacle_array(trial_infos[sequence].obstacles)

        rows.append(evaluate("human", sequence, human_xyz, duration_s, 0.0))
        planned, _failures = plan_baselines(
            sequence=sequence,
            human_xy=human_xy,
            obstacles_xy=obstacles_xy,
            obstacle_radius_m=args.obstacle_radius,
            safety_distance_m=args.safety_distance,
            astar_resolution_m=0.10,
            skip_astar=True,
            skip_rrt=False,
            rrt_iterations=args.rrt_iterations,
            rrt_step_size_m=0.35,
            rrt_seed=7 + int(sequence),
            planner_seed=7,
            skip_rrt_star=False,
            rrt_star_iterations=args.rrt_star_iterations,
            rrt_star_step_size_m=0.25,
            rrt_star_neighbor_radius_m=0.75,
            skip_mppi=False,
            mppi_rollouts=args.mppi_rollouts,
            mppi_horizon_steps=args.mppi_horizon_steps,
            mppi_iterations=args.mppi_iterations,
        )

        planned_xyz_for_plot: dict[str, np.ndarray] = {}
        for path, compute_ms in planned:
            if path.name not in {"rrt", "rrt_star", "mppi"}:
                continue
            smoothed_xy = smooth_path_xy_safe(
                path.trajectory_xy,
                obstacles_xy,
                args.obstacle_radius,
                args.safety_distance,
                args.smoothing_passes,
                args.trajectory_samples,
            )
            xyz = lift_path_to_3d(smoothed_xy, float(human_xyz[0, 2]), float(human_xyz[-1, 2]))
            rows.append(evaluate(path.name, sequence, xyz, duration_s, compute_ms))
            planned_xyz_for_plot[path.name] = xyz

        if sequence == str(args.figure_trial):
            figure_payload = (human_xyz, planned_xyz_for_plot, trial_infos[sequence])

    summary = summarize(rows)
    write_csv(Path(args.detail_output), rows)
    write_csv(Path(args.output), summary)

    if figure_payload is not None:
        human_xyz, planned_xyz, trial = figure_payload
        plot_trial_3d(
            Path(args.figure_output),
            str(args.figure_trial),
            human_xyz,
            planned_xyz,
            trial,
            args.obstacle_radius,
            args.safety_distance,
        )

    print(f"Wrote {args.detail_output} ({len(rows)} rows)")
    print(f"Wrote {args.output} ({len(summary)} rows)")
    print(f"Wrote {args.figure_output}")
    for row in summary:
        print(row)


if __name__ == "__main__":
    main()
