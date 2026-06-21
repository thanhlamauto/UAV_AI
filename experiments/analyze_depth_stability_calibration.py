#!/usr/bin/env python3
"""Analyze relative-depth stability and weak calibration against clearance/radar."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--features",
        nargs="+",
        required=True,
        help="Feature CSV(s), optionally label=path.",
    )
    parser.add_argument("--output", default="outputs/tables/depth_stability_calibration.csv")
    parser.add_argument("--figure-output", default="outputs/figures/depth_stability_calibration.png")
    return parser.parse_args()


def parse_feature_arg(value: str) -> tuple[str, Path]:
    if "=" in value:
        label, path = value.split("=", 1)
        return label, Path(path)
    path = Path(value)
    return path.stem, path


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        raise FileNotFoundError(f"Missing or empty features: {path}")
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def pearson(x: np.ndarray, y: np.ndarray) -> float:
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    if len(x) < 2 or float(np.std(x)) == 0.0 or float(np.std(y)) == 0.0:
        return 0.0
    return float(np.corrcoef(x, y)[0, 1])


def rankdata(values: np.ndarray) -> np.ndarray:
    order = np.argsort(values, kind="mergesort")
    ranks = np.empty(len(values), dtype=float)
    i = 0
    while i < len(values):
        j = i + 1
        while j < len(values) and values[order[j]] == values[order[i]]:
            j += 1
        ranks[order[i:j]] = 0.5 * (i + j - 1)
        i = j
    return ranks


def spearman(x: np.ndarray, y: np.ndarray) -> float:
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    if len(x) < 2:
        return 0.0
    return pearson(rankdata(x), rankdata(y))


def linear_rmse(x: np.ndarray, y: np.ndarray) -> float:
    mask = np.isfinite(x) & np.isfinite(y)
    x = x[mask]
    y = y[mask]
    if len(x) < 2 or float(np.std(x)) == 0.0:
        return float("nan")
    a, b = np.polyfit(x, y, 1)
    pred = a * x + b
    return float(np.sqrt(np.mean((pred - y) ** 2)))


def summarize(label: str, rows: list[dict[str, str]]) -> dict[str, object]:
    by_seq: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        by_seq[row["sequence"]].append(row)

    stability_values = []
    stability_p95 = []
    for seq_rows in by_seq.values():
        seq_rows = sorted(seq_rows, key=lambda row: float(row["time_s"]))
        depth = np.asarray([float(row["depth_median"]) for row in seq_rows], dtype=float)
        if len(depth) < 2:
            continue
        delta = np.abs(np.diff(depth))
        stability_values.append(float(np.mean(delta)))
        stability_p95.append(float(np.percentile(delta, 95.0)))

    depth_median = np.asarray([float(row["depth_median"]) for row in rows], dtype=float)
    depth_p10 = np.asarray([float(row["depth_p10"]) for row in rows], dtype=float)
    radar_peak = np.asarray([float(row["radar_peak"]) for row in rows], dtype=float)
    radar_energy = np.asarray([float(row["radar_energy"]) for row in rows], dtype=float)
    clearance = np.asarray([float(row["clearance_m"]) for row in rows], dtype=float)

    return {
        "model": label,
        "rows": len(rows),
        "trials": len(by_seq),
        "mean_abs_depth_median_delta": round(float(np.mean(stability_values)), 4) if stability_values else 0.0,
        "p95_abs_depth_median_delta": round(float(np.mean(stability_p95)), 4) if stability_p95 else 0.0,
        "pearson_depth_median_clearance": round(pearson(depth_median, clearance), 4),
        "spearman_depth_median_clearance": round(spearman(depth_median, clearance), 4),
        "pearson_depth_p10_clearance": round(pearson(depth_p10, clearance), 4),
        "spearman_depth_p10_clearance": round(spearman(depth_p10, clearance), 4),
        "depth_median_linear_clearance_rmse": round(linear_rmse(depth_median, clearance), 4),
        "pearson_radar_peak_clearance": round(pearson(radar_peak, clearance), 4),
        "spearman_radar_peak_clearance": round(spearman(radar_peak, clearance), 4),
        "pearson_radar_energy_clearance": round(pearson(radar_energy, clearance), 4),
        "spearman_radar_energy_clearance": round(spearman(radar_energy, clearance), 4),
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    labels = [str(row["model"]) for row in rows]
    x = np.arange(len(labels))
    stability = [float(row["mean_abs_depth_median_delta"]) for row in rows]
    corr = [float(row["spearman_depth_median_clearance"]) for row in rows]
    rmse = [float(row["depth_median_linear_clearance_rmse"]) for row in rows]
    rmse = [0.0 if math.isnan(v) else v for v in rmse]

    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.6), constrained_layout=True)
    axes[0].bar(x, stability, color="#4c78a8")
    axes[0].set_title("Temporal stability")
    axes[0].set_ylabel("Mean |delta depth median|")
    axes[1].bar(x, corr, color="#f58518")
    axes[1].set_title("Depth-clearance Spearman")
    axes[1].set_ylim(-1.0, 1.0)
    axes[2].bar(x, rmse, color="#54a24b")
    axes[2].set_title("Linear clearance RMSE")
    axes[2].set_ylabel("m")
    for ax in axes:
        ax.set_xticks(x, labels=labels, rotation=25, ha="right")
        ax.grid(axis="y", alpha=0.25)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    output_rows = []
    for item in args.features:
        label, path = parse_feature_arg(item)
        output_rows.append(summarize(label, read_rows(path)))
    write_csv(Path(args.output), output_rows)
    plot_summary(Path(args.figure_output), output_rows)
    print(f"Wrote {args.output}")
    print(f"Wrote {args.figure_output}")
    for row in output_rows:
        print(row)


if __name__ == "__main__":
    main()
