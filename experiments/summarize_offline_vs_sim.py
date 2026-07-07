#!/usr/bin/env python3
"""Summarize offline ODA-Bench ranking versus lightweight simulator ranking."""

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

from src.oda_bench_downstream import read_csv, write_csv


METHOD_MAP = {
    "rrt_star": "rrt_star",
    "mppi": "mppi",
    "plain_bc_mppi": "plain_bc_mppi",
    "filtered_bc_mppi": "filtered_bc_mppi",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner-summary", default="outputs/tables/planner_comparison_summary_300.csv")
    parser.add_argument("--bc-results", default="outputs/tables/oda_bc_results.csv")
    parser.add_argument("--sim-results", default="outputs/tables/pybullet_validation_results.csv")
    parser.add_argument("--outputs-dir", default="outputs")
    return parser.parse_args()


def offline_score(row: dict[str, str]) -> float:
    success = float(row.get("success_rate", 1.0))
    collision = float(row.get("collision_rate", 0.0))
    violation = float(row.get("safety_violation_rate", 0.0))
    clearance = float(row.get("mean_min_clearance_m", 0.0))
    smoothness = float(row.get("mean_smoothness", 0.0))
    compute_ms = float(row.get("mean_compute_time_ms", row.get("mean_planner_compute_time_ms", 0.0)))
    return (
        3.0 * success
        + 0.5 * clearance
        - 4.0 * collision
        - 1.5 * violation
        - 0.05 * smoothness
        - 0.0005 * compute_ms
    )


def sim_score(row: dict[str, str]) -> float:
    success = float(row.get("success_rate", 0.0))
    collision = float(row.get("collision_rate", 0.0))
    violation = float(row.get("safety_violation_rate", 0.0))
    latency = float(row.get("latency_violation_rate", 0.0))
    clearance = float(row.get("mean_min_clearance_m", 0.0))
    compute_ms = float(row.get("mean_compute_time_ms", 0.0))
    return (
        3.0 * success
        + 0.5 * clearance
        - 4.0 * collision
        - 1.5 * violation
        - 1.0 * latency
        - 0.0005 * compute_ms
    )


def rank_scores(scores: dict[str, float]) -> dict[str, int]:
    ordered = sorted(scores, key=lambda m: scores[m], reverse=True)
    return {method: idx + 1 for idx, method in enumerate(ordered)}


def spearman_from_ranks(a: list[int], b: list[int]) -> float:
    if len(a) < 2:
        return 0.0
    x = np.asarray(a, dtype=float)
    y = np.asarray(b, dtype=float)
    x -= x.mean()
    y -= y.mean()
    denom = float(np.linalg.norm(x) * np.linalg.norm(y))
    return float(np.dot(x, y) / denom) if denom > 0 else 0.0


def plot_rank_correlation(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(5.8, 4.8), constrained_layout=True)
    for row in rows:
        x = float(row["offline_rank"])
        y = float(row["sim_rank"])
        ax.scatter(x, y, s=90, color="#ee0033")
        ha = "right" if x >= 0.9 * max([float(r["offline_rank"]) for r in rows]) else "left"
        dx = -0.06 if ha == "right" else 0.04
        ax.text(x + dx, y + 0.04, str(row["method"]), fontsize=9, ha=ha)
    max_rank = max([float(r["offline_rank"]) for r in rows] + [float(r["sim_rank"]) for r in rows])
    ax.plot([0.8, max_rank + 0.2], [0.8, max_rank + 0.2], color="#64748b", linestyle="--")
    ax.set_xlabel("Offline ODA rank (1 is best)")
    ax.set_ylabel("Simulator rank (1 is best)")
    ax.set_title("Offline ODA-Bench rank transfer")
    ax.set_xticks(range(1, int(max_rank) + 1))
    ax.set_yticks(range(1, int(max_rank) + 1))
    ax.set_xlim(0.6, max_rank + 0.35)
    ax.set_ylim(0.6, max_rank + 0.35)
    ax.grid(True, alpha=0.25)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    outputs = Path(args.outputs_dir)
    tables = outputs / "tables"
    figures = outputs / "figures"

    planner = {row["method"]: row for row in read_csv(args.planner_summary)}
    bc = {row["method"]: row for row in read_csv(args.bc_results)}
    sim = {row["method"]: row for row in read_csv(args.sim_results)}

    offline_rows: dict[str, dict[str, str]] = {}
    for method in ["rrt_star", "mppi"]:
        if method in planner:
            offline_rows[method] = planner[method]
    for method in ["plain_bc_mppi", "filtered_bc_mppi"]:
        if method in bc:
            offline_rows[method] = bc[method]

    common = [m for m in METHOD_MAP if m in offline_rows and m in sim]
    offline_scores = {m: offline_score(offline_rows[m]) for m in common}
    sim_scores = {m: sim_score(sim[m]) for m in common}
    offline_ranks = rank_scores(offline_scores)
    sim_ranks = rank_scores(sim_scores)
    rho = spearman_from_ranks([offline_ranks[m] for m in common], [sim_ranks[m] for m in common])
    offline_top2 = set(sorted(common, key=lambda m: offline_ranks[m])[: min(2, len(common))])
    sim_top2 = set(sorted(common, key=lambda m: sim_ranks[m])[: min(2, len(common))])
    top2_agreement = len(offline_top2 & sim_top2) / max(1, len(offline_top2 | sim_top2))

    rows: list[dict[str, object]] = []
    for method in common:
        rows.append(
            {
                "method": method,
                "offline_score": round(offline_scores[method], 6),
                "sim_score": round(sim_scores[method], 6),
                "offline_rank": offline_ranks[method],
                "sim_rank": sim_ranks[method],
                "spearman_rho": round(rho, 6),
                "top2_agreement": round(top2_agreement, 6),
            }
        )

    write_csv(tables / "offline_vs_sim_rank_correlation.csv", rows)
    plot_rank_correlation(figures / "offline_vs_sim_rank_correlation.png", rows)
    print(f"Wrote {tables / 'offline_vs_sim_rank_correlation.csv'}")
    print(f"Wrote {figures / 'offline_vs_sim_rank_correlation.png'}")


if __name__ == "__main__":
    main()
