#!/usr/bin/env python3
"""Create a balanced 20-trial ODA target manifest from trial_overview.csv."""

from __future__ import annotations

import argparse
import csv
import io
import os
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.oda_io import dataset_root, read_trial_overview, trial_infos_from_rows


CORRUPTED = {"1306", "1321", "1344"}
ODA_FULL_ZIP_NAME = "Dupeyroux_et_al_2021_ODA_DATASET_Full.zip"
ODA_INNER_ZIP_NAME = "Dupeyroux_et_al_2021_ODA_DATASET.zip"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument(
        "--zip-path",
        default=None,
        help=(
            "Optional full ODA ZIP path. Used to read dataset/trial_overview.csv "
            "when the dataset has not been extracted yet."
        ),
    )
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


def normalized_member_name(name: str) -> str:
    parts = Path(name).parts
    if "dataset" in parts:
        idx = parts.index("dataset")
        return "/".join(parts[idx:])
    return name


def candidate_zip_paths(explicit_path: str | None) -> list[Path]:
    candidates: list[Path] = []
    if explicit_path:
        candidates.append(Path(explicit_path))
    env_path = os.environ.get("ODA_FULL_ZIP")
    if env_path:
        candidates.append(Path(env_path))
    candidates.extend(
        [
            Path("/workspace/data") / ODA_FULL_ZIP_NAME,
            Path("/workspace/data") / ODA_INNER_ZIP_NAME,
            Path("data") / ODA_FULL_ZIP_NAME,
            Path("data") / ODA_INNER_ZIP_NAME,
            Path("data/raw") / ODA_FULL_ZIP_NAME,
            Path("data/raw") / ODA_INNER_ZIP_NAME,
        ]
    )

    deduped = []
    seen = set()
    for path in candidates:
        key = str(path)
        if key not in seen:
            deduped.append(path)
            seen.add(key)
    return deduped


def read_trial_overview_from_zip(zip_path: Path):
    with zipfile.ZipFile(zip_path) as zf:
        matches = [
            info
            for info in zf.infolist()
            if normalized_member_name(info.filename) == "dataset/trial_overview.csv"
        ]
        if not matches:
            raise FileNotFoundError(
                f"dataset/trial_overview.csv was not found inside {zip_path}"
            )
        with zf.open(matches[0]) as raw:
            text = io.TextIOWrapper(raw, newline="")
            return trial_infos_from_rows(csv.DictReader(text))


def load_trial_overview(dataset_dir: Path, zip_path: str | None):
    overview_path = dataset_dir / "trial_overview.csv"
    if overview_path.exists():
        return read_trial_overview(dataset_dir)

    for candidate in candidate_zip_paths(zip_path):
        if candidate.exists():
            print(f"Reading trial_overview.csv from {candidate}")
            return read_trial_overview_from_zip(candidate)

    checked = ", ".join(str(path) for path in candidate_zip_paths(zip_path))
    raise FileNotFoundError(
        "trial_overview.csv is not available locally and no full ODA ZIP was found. "
        f"Checked: {checked}"
    )


def main() -> None:
    args = parse_args()
    trials = load_trial_overview(dataset_root(args.dataset_root), args.zip_path)
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
