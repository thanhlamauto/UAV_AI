#!/usr/bin/env python3
"""Create a balanced 20-trial ODA target manifest from trial_overview.csv."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.oda_io import dataset_root, read_trial_overview


CORRUPTED = {"1306", "1321", "1344"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--output", default="outputs/tables/target_20_trials_manifest.csv")
    parser.add_argument("--one-obstacle", type=int, default=10)
    parser.add_argument("--two-obstacle", type=int, default=10)
    parser.add_argument(
        "--seed-trials",
        nargs="*",
        default=["3", "10", "345"],
        help="Trial IDs to include first when they match the filters.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    trials = read_trial_overview(dataset_root(args.dataset_root))
    selected = []
    selected_ids = set()
    for obstacle_count, wanted in [(1, args.one_obstacle), (2, args.two_obstacle)]:
        candidates = [
            trial
            for trial in trials.values()
            if trial.obstacle_count == obstacle_count
            and trial.has_video
            and trial.sequence not in CORRUPTED
            and trial.lux == "100"
        ]
        candidates = sorted(candidates, key=lambda trial: int(trial.sequence))
        seeded = [
            trial
            for trial in candidates
            if trial.sequence in set(args.seed_trials) and trial.sequence not in selected_ids
        ]
        for trial in seeded:
            if len([item for item in selected if item.obstacle_count == obstacle_count]) < wanted:
                selected.append(trial)
                selected_ids.add(trial.sequence)
        for trial in candidates:
            current_count = len([item for item in selected if item.obstacle_count == obstacle_count])
            if current_count >= wanted:
                break
            if trial.sequence in selected_ids:
                continue
            selected.append(trial)
            selected_ids.add(trial.sequence)

    rows = [
        {
            "sequence": trial.sequence,
            "obstacles": trial.obstacle_count,
            "lux": trial.lux,
            "has_video": int(trial.has_video),
            "required_files": f"dataset/{trial.sequence}/optitrack.csv;dataset/{trial.sequence}/imu.csv;dataset/{trial.sequence}/radar.csv;dataset/{trial.sequence}/{trial.sequence}.avi",
        }
        for trial in selected
    ]

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {out}")
    print(f"Selected {len(rows)} trials: {', '.join(row['sequence'] for row in rows)}")


if __name__ == "__main__":
    main()
