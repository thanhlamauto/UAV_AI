#!/usr/bin/env python3
"""Build a compact trajectory feasibility proxy summary from planner metrics."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metrics", default="outputs/tables/batch_planner_metrics_300.csv")
    parser.add_argument("--output", default="outputs/tables/trajectory_feasibility_summary.csv")
    parser.add_argument("--figure-output", default="outputs/figures/trajectory_feasibility_summary.png")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def fvalues(rows: list[dict[str, str]], field: str) -> np.ndarray:
    return np.asarray([float(row[field]) for row in rows], dtype=float)


def summarize(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    groups: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        groups[row["method"]].append(row)

    out: list[dict[str, object]] = []
    for method in sorted(groups):
        items = groups[method]
        smoothness = fvalues(items, "smoothness_heading_change")
        speed = fvalues(items, "mean_speed_mps")
        path_length = fvalues(items, "path_length_m")
        compute = fvalues(items, "planner_compute_time_ms")
        out.append(
            {
                "method": method,
                "trials": len(items),
                "collision_rate": round(float(np.mean(fvalues(items, "collision"))), 4),
                "safety_violation_rate": round(float(np.mean(fvalues(items, "safety_violation"))), 4),
                "mean_speed_mps": round(float(np.mean(speed)), 4),
                "p95_mean_speed_mps": round(float(np.percentile(speed, 95)), 4),
                "mean_path_length_m": round(float(np.mean(path_length)), 4),
                "mean_smoothness_heading_change": round(float(np.mean(smoothness)), 6),
                "p95_smoothness_heading_change": round(float(np.percentile(smoothness, 95)), 6),
                "max_smoothness_heading_change": round(float(np.max(smoothness)), 6),
                "mean_planner_compute_time_ms": round(float(np.mean(compute)), 4),
                "p95_planner_compute_time_ms": round(float(np.percentile(compute, 95)), 4),
                "feasibility_scope": "proxy_from_2d_path_speed_and_heading_change",
            }
        )
    return out


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"No rows to write to {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def short_name(method: str) -> str:
    return {
        "geometric_bypass_not_needed": "geo-not-needed",
        "geometric_bypass": "geo-bypass",
        "straight_line": "straight",
        "astar_grid": "A*",
        "rrt_star": "RRT*",
    }.get(method, method)


def plot_summary(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    labels = [short_name(str(row["method"])) for row in rows]
    x = np.arange(len(labels))
    violation = [float(row["safety_violation_rate"]) for row in rows]
    smooth = [float(row["mean_smoothness_heading_change"]) for row in rows]
    compute = [float(row["mean_planner_compute_time_ms"]) for row in rows]

    fig, axes = plt.subplots(1, 3, figsize=(11.2, 3.6), constrained_layout=True)
    axes[0].bar(x, violation, color="#e45756")
    axes[0].set_title("Safety violations")
    axes[0].set_ylabel("Rate")
    axes[0].set_xticks(x, labels, rotation=35, ha="right")
    axes[0].grid(axis="y", alpha=0.25)

    axes[1].bar(x, smooth, color="#4c78a8")
    axes[1].set_title("Heading-change smoothness")
    axes[1].set_ylabel("Mean squared heading change")
    axes[1].set_xticks(x, labels, rotation=35, ha="right")
    axes[1].grid(axis="y", alpha=0.25)

    axes[2].bar(x, compute, color="#f58518")
    axes[2].set_title("Planner compute time")
    axes[2].set_ylabel("Mean ms/trial")
    axes[2].set_xticks(x, labels, rotation=35, ha="right")
    axes[2].grid(axis="y", alpha=0.25)

    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    rows = read_rows(Path(args.metrics))
    summary = summarize(rows)
    write_csv(Path(args.output), summary)
    plot_summary(Path(args.figure_output), summary)
    print(f"Wrote {args.output} ({len(summary)} rows)")
    print(f"Wrote {args.figure_output}")


if __name__ == "__main__":
    main()
