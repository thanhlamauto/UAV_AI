#!/usr/bin/env python3
"""Aggregate safe / violation / collision rates over many MPPI cases."""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from experiments.benchmark_online_latency_feasibility import Scenario, _map_update_samples
from experiments.benchmark_sensor_frontend_latency_feasibility import _build_frontends, _row_for_profile


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
    parser.add_argument("--cases-per-speed", type=int, default=8)
    parser.add_argument("--seed-base", type=int, default=700)
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


def _scenario_for(speed_mps: float, case_idx: int, seed: int) -> Scenario:
    if speed_mps <= 1.0:
        rollouts, iterations, horizon = 384, 8, 56
    elif speed_mps <= 2.0:
        rollouts, iterations, horizon = 768, 10, 64
    else:
        rollouts, iterations, horizon = 1152, 12, 72
    return Scenario(
        label=f"{speed_mps:.1f}mps_case_{case_idx:02d}",
        speed_mps=speed_mps,
        rollouts=rollouts,
        iterations=iterations,
        horizon_steps=horizon,
        seed=seed,
    )


def _status(row: dict[str, object]) -> str:
    if int(row["collision"]) == 1:
        return "collision"
    if int(row["safety_violation"]) == 1:
        return "violation"
    return "safe"


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _aggregate(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    keys = sorted({(str(row["frontend"]), float(row["uav_speed_mps"])) for row in rows}, key=lambda item: (item[0], item[1]))
    out: list[dict[str, object]] = []
    for frontend, speed in keys:
        group = [row for row in rows if row["frontend"] == frontend and float(row["uav_speed_mps"]) == speed]
        n = len(group)
        safe = sum(1 for row in group if _status(row) == "safe")
        violation = sum(1 for row in group if _status(row) == "violation")
        collision = sum(1 for row in group if _status(row) == "collision")
        out.append(
            {
                "frontend": frontend,
                "source": group[0]["source"],
                "uav_speed_mps": speed,
                "cases": n,
                "safe_count": safe,
                "violation_count": violation,
                "collision_count": collision,
                "safe_rate_pct": round(safe / n * 100.0, 2),
                "violation_rate_pct": round(violation / n * 100.0, 2),
                "collision_rate_pct": round(collision / n * 100.0, 2),
                "mean_total_delay_ms": round(statistics.mean(float(row["total_delay_ms"]) for row in group), 3),
                "mean_delay_distance_m": round(statistics.mean(float(row["distance_during_delay_m"]) for row in group), 4),
                "mean_min_body_clearance_m": round(statistics.mean(float(row["min_body_clearance_m"]) for row in group), 4),
                "frontend_to_occupancy_ms": group[0]["frontend_to_occupancy_ms"],
                "sensor_period_ms": group[0]["sensor_period_ms"],
            }
        )
    return out


def _frontend_overall(agg_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for frontend in dict.fromkeys(str(row["frontend"]) for row in agg_rows):
        group = [row for row in agg_rows if row["frontend"] == frontend]
        safe_speed_rows = [row for row in group if float(row["safe_rate_pct"]) >= 50.0]
        first_violation = [row for row in group if float(row["violation_rate_pct"]) + float(row["collision_rate_pct"]) > 0.0]
        first_collision = [row for row in group if float(row["collision_rate_pct"]) > 0.0]
        out.append(
            {
                "frontend": frontend,
                "source": group[0]["source"],
                "frontend_to_occupancy_ms": group[0]["frontend_to_occupancy_ms"],
                "sensor_period_ms": group[0]["sensor_period_ms"],
                "max_speed_safe_rate_ge_50_mps": max((float(row["uav_speed_mps"]) for row in safe_speed_rows), default=0.0),
                "first_any_unsafe_speed_mps": min((float(row["uav_speed_mps"]) for row in first_violation), default=""),
                "first_collision_speed_mps": min((float(row["uav_speed_mps"]) for row in first_collision), default=""),
            }
        )
    return out


def _write_summary(path: Path, agg_rows: list[dict[str, object]], overall_rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Sensor Frontend Feasibility Rates",
        "",
        "Rates are computed over multiple MPPI seeds per frontend and speed.",
        "`violation` means safety-radius violation without body collision; `collision` is counted separately.",
        "",
        "## Overall Thresholds",
        "",
        "| Frontend | Sensor ms | Frontend ms | Max speed with safe rate >= 50% | First unsafe speed | First collision speed |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in overall_rows:
        lines.append(
            f"| {row['frontend']} | {float(row['sensor_period_ms']):.1f} | "
            f"{float(row['frontend_to_occupancy_ms']):.1f} | "
            f"{row['max_speed_safe_rate_ge_50_mps']} | {row['first_any_unsafe_speed_mps']} | {row['first_collision_speed_mps']} |"
        )
    lines.extend(
        [
            "",
            "## Per-Speed Rates",
            "",
            "| Frontend | Speed | Cases | Safe % | Violation % | Collision % | Mean delay ms | Mean clearance m |",
            "|---|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in agg_rows:
        lines.append(
            f"| {row['frontend']} | {float(row['uav_speed_mps']):.1f} | {row['cases']} | "
            f"{float(row['safe_rate_pct']):.1f} | {float(row['violation_rate_pct']):.1f} | "
            f"{float(row['collision_rate_pct']):.1f} | {float(row['mean_total_delay_ms']):.1f} | "
            f"{float(row['mean_min_body_clearance_m']):.3f} |"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def _write_plot(path: Path, agg_rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9.2, 5.0))
    for frontend in dict.fromkeys(str(row["frontend"]) for row in agg_rows):
        group = [row for row in agg_rows if row["frontend"] == frontend]
        speeds = [float(row["uav_speed_mps"]) for row in group]
        safe_rates = [float(row["safe_rate_pct"]) for row in group]
        ax.plot(speeds, safe_rates, marker="o", linewidth=1.8, label=frontend)
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
    ideal_esdf_ms = float(statistics.median(map_samples_ms))
    profiles = _build_frontends(args, ideal_esdf_ms)
    speeds = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    detail_rows: list[dict[str, object]] = []
    for profile_idx, profile in enumerate(profiles):
        for speed in speeds:
            for case_idx in range(int(args.cases_per_speed)):
                seed = int(args.seed_base) + profile_idx * 10_000 + int(speed * 100) + case_idx
                scenario = _scenario_for(speed, case_idx, seed)
                row = _row_for_profile(args, profile, scenario, esdf)
                row["case_id"] = case_idx
                row["mppi_seed"] = seed
                row["status"] = _status(row)
                detail_rows.append(row)

    agg_rows = _aggregate(detail_rows)
    overall_rows = _frontend_overall(agg_rows)
    detail_path = args.output_dir / "tables" / "sensor_frontend_feasibility_rates_detail.csv"
    agg_path = args.output_dir / "tables" / "sensor_frontend_feasibility_rates.csv"
    overall_path = args.output_dir / "tables" / "sensor_frontend_feasibility_rates_overall.csv"
    summary_path = args.output_dir / "sensor_frontend_feasibility_rates_summary.md"
    figure_path = args.output_dir / "figures" / "sensor_frontend_feasibility_rates.png"
    _write_csv(detail_path, detail_rows)
    _write_csv(agg_path, agg_rows)
    _write_csv(overall_path, overall_rows)
    _write_summary(summary_path, agg_rows, overall_rows)
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
