#!/usr/bin/env python3
"""Build a low-dimensional ODA-Bench policy dataset from MPPI expert paths."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from src.oda_bench_downstream import (
    ACTION_COLUMNS,
    FEATURE_COLUMNS,
    load_trial_spec,
    observations_actions_from_path,
    read_trial_ids,
    save_trial_specs,
    split_trial_ids,
    write_csv,
)
from src.planners.mppi import MPPIConfig, mppi_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--readiness", default="outputs/tables/target_300_trials_readiness.csv")
    parser.add_argument("--outputs-dir", default="outputs")
    parser.add_argument("--limit-trials", type=int, default=120)
    parser.add_argument(
        "--local-only",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Filter readiness rows to trials with local optitrack.csv before splitting.",
    )
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--expert", default="mppi", choices=["mppi"])
    parser.add_argument("--samples-per-trial", type=int, default=80)
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument("--mppi-rollouts", type=int, default=192)
    parser.add_argument("--mppi-iterations", type=int, default=5)
    parser.add_argument("--mppi-horizon-steps", type=int, default=50)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    outputs = Path(args.outputs_dir)
    tables = outputs / "tables"
    data_dir = outputs / "datasets"
    data_dir.mkdir(parents=True, exist_ok=True)

    trial_ids = read_trial_ids(args.readiness, ready_only=True, limit=None)
    if args.local_only:
        root = Path(args.dataset_root)
        if (root / "dataset").is_dir():
            root = root / "dataset"
        trial_ids = [seq for seq in trial_ids if (root / str(seq) / "optitrack.csv").exists()]
    if args.limit_trials is not None:
        trial_ids = trial_ids[: args.limit_trials]
    splits = split_trial_ids(trial_ids, seed=args.seed)
    split_for_trial = {seq: split for split, ids in splits.items() for seq in ids}

    x_parts = {"train": [], "val": [], "test": []}
    y_parts = {"train": [], "val": [], "test": []}
    sample_rows: list[dict[str, object]] = []
    trial_rows: list[dict[str, object]] = []
    specs = []
    skipped: list[dict[str, object]] = []

    for sequence in trial_ids:
        split = split_for_trial[str(sequence)]
        try:
            spec = load_trial_spec(
                args.dataset_root,
                sequence=str(sequence),
                split=split,
                obstacle_radius_m=args.obstacle_radius,
                safety_distance_m=args.safety_distance,
            )
            specs.append(spec)
            started = perf_counter()
            expert = mppi_path(
                start=np.asarray(spec.start),
                goal=np.asarray(spec.goal),
                obstacles_xy=np.asarray(spec.obstacles),
                config=MPPIConfig(
                    num_rollouts=args.mppi_rollouts,
                    max_iterations=args.mppi_iterations,
                    horizon_steps=args.mppi_horizon_steps,
                    obstacle_radius_m=args.obstacle_radius,
                    safety_distance_m=args.safety_distance,
                    seed=args.seed + int(sequence),
                ),
                num_points=args.samples_per_trial,
            )
            expert_ms = (perf_counter() - started) * 1000.0
            rows, x, y = observations_actions_from_path(
                sequence=str(sequence),
                split=split,
                trajectory_xy=expert.trajectory_xy,
                obstacles_xy=np.asarray(spec.obstacles),
                duration_s=spec.duration_s,
                obstacle_radius_m=args.obstacle_radius,
                safety_distance_m=args.safety_distance,
            )
            x_parts[split].append(x)
            y_parts[split].append(y)
            sample_rows.extend(rows)
            trial_rows.append(
                {
                    **spec.to_row(),
                    "expert": args.expert,
                    "expert_compute_time_ms": round(expert_ms, 4),
                    "samples": len(rows),
                }
            )
        except Exception as exc:
            skipped.append({"sequence": sequence, "split": split, "reason": str(exc)})
            print(f"Skipped trial {sequence}: {exc}", file=sys.stderr)

    arrays: dict[str, np.ndarray] = {}
    for split in ["train", "val", "test"]:
        arrays[f"x_{split}"] = (
            np.concatenate(x_parts[split], axis=0).astype(np.float32)
            if x_parts[split]
            else np.zeros((0, len(FEATURE_COLUMNS)), dtype=np.float32)
        )
        arrays[f"y_{split}"] = (
            np.concatenate(y_parts[split], axis=0).astype(np.float32)
            if y_parts[split]
            else np.zeros((0, len(ACTION_COLUMNS)), dtype=np.float32)
        )

    arrays["feature_names"] = np.asarray(FEATURE_COLUMNS)
    arrays["action_names"] = np.asarray(ACTION_COLUMNS)
    dataset_path = data_dir / "oda_mppi_policy_dataset.npz"
    np.savez_compressed(dataset_path, **arrays)

    write_csv(tables / "oda_policy_dataset_samples.csv", sample_rows)
    write_csv(tables / "oda_policy_dataset_trials.csv", trial_rows)
    write_csv(tables / "oda_policy_dataset_skipped.csv", skipped)
    save_trial_specs(data_dir / "oda_policy_trial_specs.csv", specs)

    split_summary = []
    for split in ["train", "val", "test"]:
        split_summary.append(
            {
                "split": split,
                "trials": len(splits[split]),
                "samples": int(arrays[f"x_{split}"].shape[0]),
            }
        )
    write_csv(tables / "oda_policy_dataset_split_summary.csv", split_summary)
    print(f"Wrote {dataset_path}")
    print(f"Wrote {tables / 'oda_policy_dataset_split_summary.csv'}")


if __name__ == "__main__":
    main()
