#!/usr/bin/env python3
"""Compare online 10 Hz planner feasibility over multiple seeds.

This extends the narrow mentor-facing latency check beyond MPPI.  A 10 Hz 3D
LiDAR period and an ESDF map-update latency are included before each planner is
allowed to publish a new command.  During that delay the UAV keeps flying along
the previous straight command, then the planner replans from the delayed pose.

The RRT and RRT* baselines are fixed-altitude 2D planners already present in
the project.  Their trajectories are lifted to a constant z and evaluated
against the same 3D ESDF as MPPI.
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

from experiments.benchmark_online_latency_feasibility import (
    _advance_along_goal,
    _combined_trajectory,
    _first_range_below_threshold,
    _map_update_samples,
)
from src.planners.rrt import RRTConfig, rrt_path
from src.planners.rrt_star import RRTStarConfig, rrt_star_path
from src.planners.mppi_3d_esdf import MPPI3DConfig, mppi_3d_esdf_path


@dataclass(frozen=True)
class PlannerCase:
    planner: str
    speed_mps: float
    case_id: int
    seed: int


@dataclass(frozen=True)
class TimedPlan:
    trajectory_xyz: np.ndarray
    compute_ms: float
    failed: bool = False
    failure_reason: str = ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--resolution", type=float, default=0.15)
    parser.add_argument("--sensor-hz", type=float, default=10.0)
    parser.add_argument("--control-publish-ms", type=float, default=10.0)
    parser.add_argument("--body-radius", type=float, default=0.20)
    parser.add_argument("--safety-radius", type=float, default=0.45)
    parser.add_argument("--fixed-altitude-m", type=float, default=1.50)
    parser.add_argument("--map-repeats", type=int, default=7)
    parser.add_argument("--cases-per-speed", type=int, default=8)
    parser.add_argument("--seed-base", type=int, default=1200)
    parser.add_argument("--prefer-scipy", action="store_true")
    parser.add_argument("--rrt-iterations", type=int, default=900)
    parser.add_argument("--rrt-star-iterations", type=int, default=700)
    return parser.parse_args()


def _planner_obstacles_xy() -> np.ndarray:
    # Footprints of the 3D scene obstacles that intersect the nominal flight
    # altitude.  Corridor walls remain enforced by the final ESDF evaluation.
    return np.asarray(
        [
            [2.05, -0.38],  # pillar_low_lateral
            [4.25, 0.44],  # stacked_box_mid
        ],
        dtype=float,
    )


def _scenario_mppi_config(speed_mps: float, seed: int, safety_radius_m: float) -> MPPI3DConfig:
    if speed_mps <= 1.0:
        rollouts, iterations, horizon = 384, 8, 56
    elif speed_mps <= 2.0:
        rollouts, iterations, horizon = 768, 10, 64
    else:
        rollouts, iterations, horizon = 1152, 12, 72
    return MPPI3DConfig(
        num_rollouts=rollouts,
        horizon_steps=horizon,
        max_iterations=iterations,
        safety_radius_m=safety_radius_m,
        seed=seed,
    )


def _lift_xy(path_xy: np.ndarray, altitude_m: float) -> np.ndarray:
    return np.column_stack([path_xy, np.full(len(path_xy), altitude_m, dtype=float)])


def _straight_fallback(start_xyz: np.ndarray, goal_xyz: np.ndarray, samples: int = 96) -> np.ndarray:
    return np.linspace(start_xyz, goal_xyz, samples)


def _time_2d_planner(
    fn,
    start_xyz: np.ndarray,
    goal_xyz: np.ndarray,
    obstacles_xy: np.ndarray,
    config: object,
    fixed_altitude_m: float,
) -> TimedPlan:
    start_time = time.perf_counter()
    try:
        path = fn(start_xyz[:2], goal_xyz[:2], obstacles_xy, config, num_points=180)
        compute_ms = (time.perf_counter() - start_time) * 1000.0
        return TimedPlan(_lift_xy(path.trajectory_xy, fixed_altitude_m), compute_ms)
    except Exception as exc:  # noqa: BLE001 - planner failure is an output metric here.
        compute_ms = (time.perf_counter() - start_time) * 1000.0
        return TimedPlan(
            _straight_fallback(start_xyz, goal_xyz),
            compute_ms,
            failed=True,
            failure_reason=str(exc),
        )


def _time_planner(
    args: argparse.Namespace,
    case: PlannerCase,
    esdf,
    start_xyz: np.ndarray,
    goal_xyz: np.ndarray,
) -> TimedPlan:
    obstacles_xy = _planner_obstacles_xy()
    if case.planner == "rrt":
        config = RRTConfig(
            max_iterations=args.rrt_iterations,
            step_size_m=0.35,
            goal_sample_rate=0.20,
            margin_m=1.40,
            obstacle_radius_m=0.45,
            safety_distance_m=args.safety_radius,
            seed=case.seed,
        )
        return _time_2d_planner(rrt_path, start_xyz, goal_xyz, obstacles_xy, config, args.fixed_altitude_m)

    if case.planner == "rrt_star":
        config = RRTStarConfig(
            max_iterations=args.rrt_star_iterations,
            step_size_m=0.35,
            neighbor_radius_m=0.85,
            goal_sample_rate=0.18,
            margin_m=1.40,
            obstacle_radius_m=0.45,
            safety_distance_m=args.safety_radius,
            seed=case.seed,
        )
        return _time_2d_planner(rrt_star_path, start_xyz, goal_xyz, obstacles_xy, config, args.fixed_altitude_m)

    if case.planner == "mppi_3d_esdf":
        config = _scenario_mppi_config(case.speed_mps, case.seed, args.safety_radius)
        start_time = time.perf_counter()
        result = mppi_3d_esdf_path(start_xyz, goal_xyz, esdf, config)
        compute_ms = (time.perf_counter() - start_time) * 1000.0
        return TimedPlan(result.trajectory_xyz, compute_ms)

    raise ValueError(f"Unknown planner: {case.planner}")


def _run_online_case(args: argparse.Namespace, case: PlannerCase, esdf, map_update_ms: float) -> dict[str, object]:
    start_xyz = np.asarray([0.0, 0.0, args.fixed_altitude_m], dtype=float)
    goal_xyz = np.asarray([6.55, 0.0, args.fixed_altitude_m], dtype=float)
    lidar_period_ms = 1000.0 / args.sensor_hz
    base_delay_ms = lidar_period_ms + map_update_ms + args.control_publish_ms

    estimated_start = _advance_along_goal(start_xyz, goal_xyz, case.speed_mps * base_delay_ms / 1000.0)
    first_plan = _time_planner(args, case, esdf, estimated_start, goal_xyz)
    total_delay_ms = base_delay_ms + first_plan.compute_ms
    delayed_start = _advance_along_goal(start_xyz, goal_xyz, case.speed_mps * total_delay_ms / 1000.0)
    final_plan = _time_planner(args, case, esdf, delayed_start, goal_xyz)
    total_delay_ms = base_delay_ms + final_plan.compute_ms
    delayed_start = _advance_along_goal(start_xyz, goal_xyz, case.speed_mps * total_delay_ms / 1000.0)

    combined = _combined_trajectory(start_xyz, delayed_start, final_plan.trajectory_xyz)
    distances = esdf.query_distance(combined)
    min_esdf = float(np.min(distances))
    min_body_clearance = min_esdf - args.body_radius
    min_safety_margin = min_esdf - args.safety_radius
    delay_distance = case.speed_mps * total_delay_ms / 1000.0
    first_safety_breach = _first_range_below_threshold(esdf, start_xyz, goal_xyz, args.safety_radius)
    first_body_contact = _first_range_below_threshold(esdf, start_xyz, goal_xyz, args.body_radius)

    collision = int(min_body_clearance <= 0.0)
    violation = int(min_safety_margin < 0.0)
    if final_plan.failed:
        collision = 1
        violation = 1

    status = "collision" if collision else "violation" if violation else "safe"
    return {
        "planner": case.planner,
        "sensor_hz": round(args.sensor_hz, 3),
        "uav_speed_mps": round(case.speed_mps, 3),
        "case_id": case.case_id,
        "seed": case.seed,
        "fixed_altitude_m": round(args.fixed_altitude_m, 3),
        "lidar_period_ms": round(lidar_period_ms, 3),
        "map_update_ms": round(map_update_ms, 3),
        "planner_compute_ms": round(final_plan.compute_ms, 3),
        "control_publish_ms": round(args.control_publish_ms, 3),
        "total_delay_ms": round(total_delay_ms, 3),
        "distance_during_delay_m": round(delay_distance, 4),
        "direct_safety_breach_range_m": round(first_safety_breach, 4),
        "direct_body_contact_range_m": round(first_body_contact, 4),
        "latency_buffer_to_safety_breach_m": round(first_safety_breach - delay_distance, 4),
        "latency_buffer_to_body_contact_m": round(first_body_contact - delay_distance, 4),
        "min_esdf_distance_m": round(min_esdf, 4),
        "min_body_clearance_m": round(min_body_clearance, 4),
        "min_safety_margin_m": round(min_safety_margin, 4),
        "collision": collision,
        "safety_violation": violation,
        "status": status,
        "planner_failed": int(final_plan.failed),
        "failure_reason": final_plan.failure_reason,
    }


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _aggregate(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    keys = sorted({(str(row["planner"]), float(row["uav_speed_mps"])) for row in rows}, key=lambda item: (item[0], item[1]))
    out: list[dict[str, object]] = []
    for planner, speed in keys:
        group = [row for row in rows if row["planner"] == planner and float(row["uav_speed_mps"]) == speed]
        n = len(group)
        safe = sum(1 for row in group if row["status"] == "safe")
        violation = sum(1 for row in group if row["status"] == "violation")
        collision = sum(1 for row in group if row["status"] == "collision")
        failures = sum(int(row["planner_failed"]) for row in group)
        out.append(
            {
                "planner": planner,
                "uav_speed_mps": speed,
                "cases": n,
                "safe_count": safe,
                "violation_count": violation,
                "collision_count": collision,
                "failure_count": failures,
                "safe_rate_pct": round(safe / n * 100.0, 2),
                "violation_rate_pct": round(violation / n * 100.0, 2),
                "collision_rate_pct": round(collision / n * 100.0, 2),
                "failure_rate_pct": round(failures / n * 100.0, 2),
                "mean_planner_compute_ms": round(statistics.mean(float(row["planner_compute_ms"]) for row in group), 3),
                "mean_total_delay_ms": round(statistics.mean(float(row["total_delay_ms"]) for row in group), 3),
                "mean_delay_distance_m": round(statistics.mean(float(row["distance_during_delay_m"]) for row in group), 4),
                "mean_min_body_clearance_m": round(statistics.mean(float(row["min_body_clearance_m"]) for row in group), 4),
            }
        )
    return out


def _overall(agg_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for planner in dict.fromkeys(str(row["planner"]) for row in agg_rows):
        group = [row for row in agg_rows if row["planner"] == planner]
        safe_speed_rows = [row for row in group if float(row["safe_rate_pct"]) >= 50.0]
        first_unsafe = [row for row in group if float(row["violation_rate_pct"]) + float(row["collision_rate_pct"]) > 0.0]
        first_collision = [row for row in group if float(row["collision_rate_pct"]) > 0.0]
        out.append(
            {
                "planner": planner,
                "max_speed_safe_rate_ge_50_mps": max((float(row["uav_speed_mps"]) for row in safe_speed_rows), default=0.0),
                "first_any_unsafe_speed_mps": min((float(row["uav_speed_mps"]) for row in first_unsafe), default=""),
                "first_collision_speed_mps": min((float(row["uav_speed_mps"]) for row in first_collision), default=""),
                "mean_compute_ms_all_cases": round(statistics.mean(float(row["mean_planner_compute_ms"]) for row in group), 3),
                "mean_total_delay_ms_all_cases": round(statistics.mean(float(row["mean_total_delay_ms"]) for row in group), 3),
            }
        )
    return out


def _write_summary(path: Path, agg_rows: list[dict[str, object]], overall_rows: list[dict[str, object]], map_samples_ms: list[float]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Online 10 Hz Planner Feasibility Rates",
        "",
        "Rates are computed over 8 seeds per planner and speed with a 10 Hz sensor period.",
        "RRT, RRT*, and MPPI use seeded sampling.",
        "RRT and RRT* are fixed-altitude 2D baselines lifted into the 3D ESDF for clearance evaluation.",
        "`violation` means safety-radius violation without body collision; `collision` is counted separately.",
        "",
        f"Map/ESDF update samples ms: min={min(map_samples_ms):.3f}, median={statistics.median(map_samples_ms):.3f}, max={max(map_samples_ms):.3f}.",
        "",
        "## Overall Thresholds",
        "",
        "| Planner | Max speed with safe rate >= 50% | First unsafe speed | First collision speed | Mean compute ms | Mean total delay ms |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in overall_rows:
        lines.append(
            f"| {row['planner']} | {row['max_speed_safe_rate_ge_50_mps']} | {row['first_any_unsafe_speed_mps']} | "
            f"{row['first_collision_speed_mps']} | {float(row['mean_compute_ms_all_cases']):.1f} | "
            f"{float(row['mean_total_delay_ms_all_cases']):.1f} |"
        )
    lines.extend(
        [
            "",
            "## Per-Speed Rates",
            "",
            "| Planner | Speed | Cases | Safe % | Violation % | Collision % | Mean compute ms | Mean delay ms | Mean clearance m |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in agg_rows:
        lines.append(
            f"| {row['planner']} | {float(row['uav_speed_mps']):.1f} | {row['cases']} | "
            f"{float(row['safe_rate_pct']):.1f} | {float(row['violation_rate_pct']):.1f} | "
            f"{float(row['collision_rate_pct']):.1f} | {float(row['mean_planner_compute_ms']):.1f} | "
            f"{float(row['mean_total_delay_ms']):.1f} | {float(row['mean_min_body_clearance_m']):.3f} |"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_plot(path: Path, agg_rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    for planner in dict.fromkeys(str(row["planner"]) for row in agg_rows):
        group = [row for row in agg_rows if row["planner"] == planner]
        speeds = [float(row["uav_speed_mps"]) for row in group]
        safe_rates = [float(row["safe_rate_pct"]) for row in group]
        ax.plot(speeds, safe_rates, marker="o", linewidth=1.8, label=planner)
    ax.axhline(50.0, color="#991b1b", linestyle="--", linewidth=1.0, label="50% safe-rate")
    ax.set_xlabel("UAV speed [m/s]")
    ax.set_ylabel("safe cases [%]")
    ax.set_ylim(-3, 103)
    ax.grid(True, alpha=0.25)
    ax.legend(loc="lower left", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    _, _, esdf, map_samples_ms = _map_update_samples(args.resolution, args.prefer_scipy, args.map_repeats)
    map_update_ms = float(statistics.median(map_samples_ms))
    speeds = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    planners = ["rrt", "rrt_star", "mppi_3d_esdf"]

    detail_rows: list[dict[str, object]] = []
    for planner_idx, planner in enumerate(planners):
        for speed in speeds:
            for case_id in range(int(args.cases_per_speed)):
                seed = int(args.seed_base) + planner_idx * 10_000 + int(speed * 100) + case_id
                case = PlannerCase(planner=planner, speed_mps=speed, case_id=case_id, seed=seed)
                detail_rows.append(_run_online_case(args, case, esdf, map_update_ms))

    agg_rows = _aggregate(detail_rows)
    overall_rows = _overall(agg_rows)

    detail_path = args.output_dir / "tables" / "online_10hz_planner_feasibility_rates_detail.csv"
    agg_path = args.output_dir / "tables" / "online_10hz_planner_feasibility_rates.csv"
    overall_path = args.output_dir / "tables" / "online_10hz_planner_feasibility_rates_overall.csv"
    summary_path = args.output_dir / "online_10hz_planner_feasibility_rates_summary.md"
    figure_path = args.output_dir / "figures" / "online_10hz_planner_feasibility_rates.png"
    _write_csv(detail_path, detail_rows)
    _write_csv(agg_path, agg_rows)
    _write_csv(overall_path, overall_rows)
    _write_summary(summary_path, agg_rows, overall_rows, map_samples_ms)
    _write_plot(figure_path, agg_rows)

    print(f"Wrote {detail_path}")
    print(f"Wrote {agg_path}")
    print(f"Wrote {overall_path}")
    print(f"Wrote {summary_path}")
    print(f"Wrote {figure_path}")
    for row in overall_rows:
        print(row)


if __name__ == "__main__":
    main()
