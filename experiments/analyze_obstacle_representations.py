#!/usr/bin/env python3
"""Summarize lightweight obstacle representations from ODA metadata.

The ODA metadata gives cylindrical obstacle centers on the ground plane, so this
script compares representations that can be derived without a point cloud:
circles, inflated safety circles, axis-aligned bounding boxes, and a 2D
occupancy footprint.  It does not claim 3D voxel or point-cloud segmentation.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from src.oda_io import available_trial_ids, dataset_root, obstacle_array, read_trial_overview


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--readiness", default="outputs/tables/target_300_trials_readiness.csv")
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument("--grid-resolution", type=float, default=0.10)
    parser.add_argument("--output", default="outputs/tables/obstacle_representation_summary.csv")
    parser.add_argument("--aggregate-output", default="outputs/tables/obstacle_representation_aggregate.csv")
    parser.add_argument("--figure-output", default="outputs/figures/obstacle_representation_summary.png")
    return parser.parse_args()


def ready_sequences(path: Path, dataset_dir: Path) -> list[str]:
    if path.exists() and path.stat().st_size > 0:
        with path.open(newline="") as f:
            return [
                row["sequence"]
                for row in csv.DictReader(f)
                if row.get("ready", "0") in {"1", "true", "True"}
            ]
    return available_trial_ids(dataset_dir)


def occupancy_stats(
    obstacles_xy: np.ndarray,
    inflated_radius_m: float,
    resolution_m: float,
) -> dict[str, float]:
    if len(obstacles_xy) == 0:
        return {
            "bbox_min_x_m": 0.0,
            "bbox_max_x_m": 0.0,
            "bbox_min_y_m": 0.0,
            "bbox_max_y_m": 0.0,
            "bounding_box_area_m2": 0.0,
            "occupancy_grid_cells": 0,
            "occupancy_blocked_cells": 0,
            "occupancy_area_m2": 0.0,
            "occupancy_fill_ratio": 0.0,
            "bbox_over_occupancy_ratio": 0.0,
        }

    min_x = float(np.min(obstacles_xy[:, 0]) - inflated_radius_m)
    max_x = float(np.max(obstacles_xy[:, 0]) + inflated_radius_m)
    min_y = float(np.min(obstacles_xy[:, 1]) - inflated_radius_m)
    max_y = float(np.max(obstacles_xy[:, 1]) + inflated_radius_m)
    width = max(max_x - min_x, resolution_m)
    height = max(max_y - min_y, resolution_m)
    cols = max(1, int(math.ceil(width / resolution_m)))
    rows = max(1, int(math.ceil(height / resolution_m)))

    xs = min_x + (np.arange(cols, dtype=float) + 0.5) * resolution_m
    ys = min_y + (np.arange(rows, dtype=float) + 0.5) * resolution_m
    xx, yy = np.meshgrid(xs, ys)
    grid_points = np.column_stack([xx.ravel(), yy.ravel()])
    distances = np.linalg.norm(grid_points[:, None, :] - obstacles_xy[None, :, :], axis=2)
    blocked = np.any(distances <= inflated_radius_m, axis=1)
    blocked_cells = int(np.sum(blocked))
    grid_cells = int(rows * cols)
    occupancy_area = blocked_cells * resolution_m * resolution_m
    bbox_area = width * height
    fill_ratio = occupancy_area / bbox_area if bbox_area > 0 else 0.0
    ratio = bbox_area / occupancy_area if occupancy_area > 0 else 0.0

    return {
        "bbox_min_x_m": round(min_x, 4),
        "bbox_max_x_m": round(max_x, 4),
        "bbox_min_y_m": round(min_y, 4),
        "bbox_max_y_m": round(max_y, 4),
        "bounding_box_area_m2": round(float(bbox_area), 4),
        "occupancy_grid_cells": grid_cells,
        "occupancy_blocked_cells": blocked_cells,
        "occupancy_area_m2": round(float(occupancy_area), 4),
        "occupancy_fill_ratio": round(float(fill_ratio), 4),
        "bbox_over_occupancy_ratio": round(float(ratio), 4),
    }


def summarize_sequence(
    sequence: str,
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float,
    safety_distance_m: float,
    resolution_m: float,
) -> dict[str, object]:
    obstacle_count = int(len(obstacles_xy))
    inflated_radius = obstacle_radius_m + safety_distance_m
    obstacle_area = obstacle_count * math.pi * obstacle_radius_m**2
    safety_area = obstacle_count * math.pi * inflated_radius**2
    row: dict[str, object] = {
        "sequence": sequence,
        "obstacle_count": obstacle_count,
        "representation_scope": "2d_ground_plane",
        "obstacle_radius_m": round(obstacle_radius_m, 4),
        "safety_distance_m": round(safety_distance_m, 4),
        "inflated_radius_m": round(inflated_radius, 4),
        "circle_area_m2": round(float(obstacle_area), 4),
        "inflated_circle_area_m2": round(float(safety_area), 4),
        "occupancy_resolution_m": round(resolution_m, 4),
    }
    row.update(occupancy_stats(obstacles_xy, inflated_radius, resolution_m))
    return row


def aggregate_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    groups: dict[int, list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        groups[int(row["obstacle_count"])].append(row)

    numeric_fields = [
        "circle_area_m2",
        "inflated_circle_area_m2",
        "bounding_box_area_m2",
        "occupancy_area_m2",
        "occupancy_fill_ratio",
        "bbox_over_occupancy_ratio",
        "occupancy_grid_cells",
        "occupancy_blocked_cells",
    ]
    aggregate: list[dict[str, object]] = []
    for obstacle_count in sorted(groups):
        items = groups[obstacle_count]
        out: dict[str, object] = {"obstacle_count": obstacle_count, "trials": len(items)}
        for field in numeric_fields:
            values = np.asarray([float(item[field]) for item in items], dtype=float)
            out[f"mean_{field}"] = round(float(np.mean(values)), 4)
        aggregate.append(out)
    return aggregate


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"No rows to write to {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(path: Path, aggregate: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    labels = [str(row["obstacle_count"]) for row in aggregate]
    x = np.arange(len(labels))
    bbox = [float(row["mean_bounding_box_area_m2"]) for row in aggregate]
    occ = [float(row["mean_occupancy_area_m2"]) for row in aggregate]
    circle = [float(row["mean_inflated_circle_area_m2"]) for row in aggregate]
    fill = [float(row["mean_occupancy_fill_ratio"]) for row in aggregate]

    fig, axes = plt.subplots(1, 2, figsize=(9.8, 3.8), constrained_layout=True)
    width = 0.26
    axes[0].bar(x - width, circle, width, label="Inflated circles", color="#4c78a8")
    axes[0].bar(x, bbox, width, label="Bounding box", color="#f58518")
    axes[0].bar(x + width, occ, width, label="Occupancy footprint", color="#54a24b")
    axes[0].set_xticks(x, labels)
    axes[0].set_xlabel("Obstacle count")
    axes[0].set_ylabel("Mean area (m^2)")
    axes[0].set_title("2D obstacle representation area")
    axes[0].legend(fontsize=8)
    axes[0].grid(axis="y", alpha=0.25)

    axes[1].bar(x, fill, color="#b279a2")
    axes[1].set_xticks(x, labels)
    axes[1].set_xlabel("Obstacle count")
    axes[1].set_ylabel("Occupancy / bounding box")
    axes[1].set_ylim(0, max(1.0, max(fill) * 1.15 if fill else 1.0))
    axes[1].set_title("Bounding-box fill ratio")
    axes[1].grid(axis="y", alpha=0.25)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    trials = read_trial_overview(dataset_dir)
    sequences = ready_sequences(Path(args.readiness), dataset_dir)
    rows: list[dict[str, object]] = []
    for sequence in sequences:
        trial = trials.get(sequence)
        if trial is None:
            print(f"Warning: metadata missing for trial {sequence}", file=sys.stderr)
            continue
        rows.append(
            summarize_sequence(
                sequence,
                obstacle_array(trial.obstacles),
                args.obstacle_radius,
                args.safety_distance,
                args.grid_resolution,
            )
        )
    aggregate = aggregate_rows(rows)
    write_csv(Path(args.output), rows)
    write_csv(Path(args.aggregate_output), aggregate)
    plot_summary(Path(args.figure_output), aggregate)
    print(f"Wrote {args.output} ({len(rows)} rows)")
    print(f"Wrote {args.aggregate_output} ({len(aggregate)} rows)")
    print(f"Wrote {args.figure_output}")


if __name__ == "__main__":
    main()
