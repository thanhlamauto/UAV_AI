#!/usr/bin/env python3
"""Run a lightweight ODA CSV benchmark and produce mentor-ready outputs."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from src.metrics import TrialMetrics, compute_trial_metrics
from src.oda_io import (
    available_trial_ids,
    dataset_root,
    load_optitrack,
    obstacle_array,
    read_trial_overview,
)
from src.visualize import plot_trial_ground_plane


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dataset-root",
        default="data/raw/ODA_Dataset/dataset",
        help="Path to ODA dataset folder or ODA repo root.",
    )
    parser.add_argument(
        "--trial-ids",
        nargs="*",
        default=None,
        help="Trial IDs to benchmark. Defaults to all local sample folders.",
    )
    parser.add_argument(
        "--outputs-dir",
        default="outputs",
        help="Directory for generated figures, tables, and summary.",
    )
    parser.add_argument(
        "--obstacle-radius",
        type=float,
        default=0.20,
        help="Obstacle cylinder radius in meters. ODA sample scripts use 0.20 m.",
    )
    parser.add_argument(
        "--safety-distance",
        type=float,
        default=0.50,
        help="Required clearance from obstacle boundary in meters.",
    )
    return parser.parse_args()


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def markdown_table(rows: list[dict[str, object]], columns: list[str]) -> str:
    if not rows:
        return "_No rows._"
    header = "| " + " | ".join(columns) + " |"
    separator = "| " + " | ".join("---" for _ in columns) + " |"
    body = [
        "| " + " | ".join(str(row.get(column, "")) for column in columns) + " |"
        for row in rows
    ]
    return "\n".join([header, separator, *body])


def dataset_summary_rows(dataset_dir: Path) -> list[dict[str, object]]:
    trials = read_trial_overview(dataset_dir)
    local_ids = available_trial_ids(dataset_dir)
    obstacle_counts = [trial.obstacle_count for trial in trials.values()]

    return [
        {"item": "Unique trial IDs in metadata", "value": len(trials)},
        {"item": "Rows in trial_overview.csv", "value": sum(max(1, c) for c in obstacle_counts)},
        {"item": "Local CSV sample trials", "value": ", ".join(local_ids)},
        {"item": "Trials with 0 obstacle coordinates", "value": obstacle_counts.count(0)},
        {"item": "Trials with 1 obstacle coordinate", "value": obstacle_counts.count(1)},
        {"item": "Trials with 2 obstacle coordinates", "value": obstacle_counts.count(2)},
        {"item": "Full-light trials in metadata", "value": sum(1 for t in trials.values() if t.lux == "100")},
        {"item": "Dim-light trials in metadata", "value": sum(1 for t in trials.values() if t.lux == "1")},
        {"item": "Trials with RGB video flag", "value": sum(1 for t in trials.values() if t.has_video)},
        {"item": "Frame convention used here", "value": "ground plane = OptiTrack x vs z; height = OptiTrack y"},
    ]


def benchmark_trial(
    dataset_dir: Path,
    sequence: str,
    obstacle_radius_m: float,
    safety_distance_m: float,
) -> tuple[TrialMetrics, dict[str, np.ndarray], np.ndarray]:
    trials = read_trial_overview(dataset_dir)
    if sequence not in trials:
        raise KeyError(f"Trial {sequence} not found in trial_overview.csv")

    trial = trials[sequence]
    optitrack = load_optitrack(dataset_dir, sequence)
    trajectory_xy = np.column_stack(
        [optitrack["ground_x_m"], optitrack["ground_y_m"]]
    )
    obstacles_xy = obstacle_array(trial.obstacles)

    metrics = compute_trial_metrics(
        sequence=sequence,
        time_s=optitrack["time_s"],
        trajectory_xy=trajectory_xy,
        obstacles_xy=obstacles_xy,
        obstacle_radius_m=obstacle_radius_m,
        safety_distance_m=safety_distance_m,
    )
    return metrics, optitrack, obstacles_xy


def write_summary(
    path: Path,
    summary_rows: list[dict[str, object]],
    metric_rows: list[dict[str, object]],
    figure_paths: list[Path],
    obstacle_radius_m: float,
    safety_distance_m: float,
) -> None:
    columns = [
        "sequence",
        "obstacles",
        "min_center_distance_m",
        "min_boundary_clearance_m",
        "closest_time_s",
        "collision",
        "safety_violation",
        "avoidance_label",
        "computation_time_ms",
    ]
    lines = [
        "# ODA UAV Obstacle Avoidance: First Results",
        "",
        "## Dataset Structure",
        "",
        markdown_table(summary_rows, ["item", "value"]),
        "",
        "## Initial Benchmark Metrics",
        "",
        (
            f"Obstacle radius is set to {obstacle_radius_m:.2f} m, matching the "
            f"upstream sample scripts. Safety violation means boundary clearance "
            f"is below {safety_distance_m:.2f} m."
        ),
        "",
        markdown_table(metric_rows, columns),
        "",
        "## Generated Visualizations",
        "",
        *[f"- `{path}`" for path in figure_paths],
        "",
        "## Notes",
        "",
        "- The full 4TU archive is about 98 GB, so this first pass uses the GitHub-bundled CSV samples and full metadata file.",
        "- Samples 593-629 have missing obstacle coordinates in metadata and should be skipped or repaired before obstacle-distance benchmarking.",
        "- Avoidance labels are heuristic side labels computed from closest approach relative to the start-to-obstacle line.",
        "",
        "## Source Links",
        "",
        "- ODA Dataset GitHub: https://github.com/JuSquare/ODA_Dataset",
        "- Full 4TU dataset record: https://data.4tu.nl/articles/dataset/The_Obstacle_Detection_and_Avoidance_Dataset_for_Drones/14214236/1",
        "",
        "## Next Planner Comparison Plan",
        "",
        "1. Freeze this metrics contract: min clearance, collision, safety violation, closest-approach time, path length, and compute time.",
        "2. Add a straight-line baseline between the same start/end points and compare its clearance against the human-flown trajectory.",
        "3. Add a geometric left/right bypass baseline that inserts one waypoint around the obstacle safety circle.",
        "4. Add A* on a 2D occupancy grid using the same obstacle cylinders and safety radius.",
        "5. Add RRT/RRT* for continuous-space comparison; reserve MPPI/MPC until these classical baselines are stable.",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    outputs_dir = Path(args.outputs_dir)
    figures_dir = outputs_dir / "figures"
    tables_dir = outputs_dir / "tables"

    trial_ids = args.trial_ids or available_trial_ids(dataset_dir)
    summary_rows = dataset_summary_rows(dataset_dir)
    write_csv(tables_dir / "dataset_structure_summary.csv", summary_rows)

    metric_rows: list[dict[str, object]] = []
    figure_paths: list[Path] = []
    for sequence in trial_ids:
        metrics, optitrack, obstacles_xy = benchmark_trial(
            dataset_dir=dataset_dir,
            sequence=str(sequence),
            obstacle_radius_m=args.obstacle_radius,
            safety_distance_m=args.safety_distance,
        )
        metric_rows.append(metrics.as_row())
        trajectory_xy = np.column_stack(
            [optitrack["ground_x_m"], optitrack["ground_y_m"]]
        )
        figure_path = figures_dir / f"trajectory_sample_{sequence}.png"
        plot_trial_ground_plane(
            sequence=str(sequence),
            time_s=optitrack["time_s"],
            trajectory_xy=trajectory_xy,
            obstacles_xy=obstacles_xy,
            metrics=metrics,
            output_path=figure_path,
            obstacle_radius_m=args.obstacle_radius,
            safety_distance_m=args.safety_distance,
        )
        figure_paths.append(figure_path)

    write_csv(tables_dir / "benchmark_metrics.csv", metric_rows)
    write_summary(
        outputs_dir / "mentor_summary.md",
        summary_rows=summary_rows,
        metric_rows=metric_rows,
        figure_paths=figure_paths,
        obstacle_radius_m=args.obstacle_radius,
        safety_distance_m=args.safety_distance,
    )

    print(f"Wrote {tables_dir / 'dataset_structure_summary.csv'}")
    print(f"Wrote {tables_dir / 'benchmark_metrics.csv'}")
    print(f"Wrote {outputs_dir / 'mentor_summary.md'}")
    for path in figure_paths:
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
