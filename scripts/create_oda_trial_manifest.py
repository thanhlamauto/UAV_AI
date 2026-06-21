#!/usr/bin/env python3
"""Create a balanced ODA trial manifest for larger planner benchmarks."""

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
    parser.add_argument("--zip-path", default=None)
    parser.add_argument("--output", default="outputs/tables/target_trials_manifest.csv")
    parser.add_argument("--total", type=int, default=100)
    parser.add_argument("--one-obstacle", type=int, default=None)
    parser.add_argument("--two-obstacle", type=int, default=None)
    parser.add_argument("--lux", default="100")
    parser.add_argument("--require-video", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--exclude-corrupted", action=argparse.BooleanOptionalAction, default=True)
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
            Path("/workspace/data") / ODA_INNER_ZIP_NAME,
            Path("/workspace/data") / ODA_FULL_ZIP_NAME,
            Path("data") / ODA_INNER_ZIP_NAME,
            Path("data") / ODA_FULL_ZIP_NAME,
            Path("data/raw") / ODA_INNER_ZIP_NAME,
            Path("data/raw") / ODA_FULL_ZIP_NAME,
        ]
    )

    deduped: list[Path] = []
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
        "trial_overview.csv is not available locally and no ODA ZIP was found. "
        f"Checked: {checked}"
    )


def balanced_targets(total: int, one_obstacle: int | None, two_obstacle: int | None) -> tuple[int, int]:
    if one_obstacle is None and two_obstacle is None:
        one_obstacle = total // 2
        two_obstacle = total - one_obstacle
    elif one_obstacle is None:
        one_obstacle = total - int(two_obstacle)
    elif two_obstacle is None:
        two_obstacle = total - int(one_obstacle)

    one = int(one_obstacle)
    two = int(two_obstacle)
    if one < 0 or two < 0 or one + two != total:
        raise ValueError(
            f"Invalid obstacle targets: one={one}, two={two}, total={total}"
        )
    return one, two


def select_trials(args: argparse.Namespace):
    trials = load_trial_overview(dataset_root(args.dataset_root), args.zip_path)
    one_wanted, two_wanted = balanced_targets(args.total, args.one_obstacle, args.two_obstacle)

    selected = []
    selected_ids = set()
    seed_set = set(args.seed_trials)
    targets = [(1, one_wanted), (2, two_wanted)]
    for obstacle_count, wanted in targets:
        candidates = [
            trial
            for trial in trials.values()
            if trial.obstacle_count == obstacle_count
            and (not args.require_video or trial.has_video)
            and (not args.exclude_corrupted or trial.sequence not in CORRUPTED)
            and (args.lux == "any" or trial.lux == args.lux)
        ]
        candidates = sorted(candidates, key=lambda trial: int(trial.sequence))
        seeded = [
            trial
            for trial in candidates
            if trial.sequence in seed_set and trial.sequence not in selected_ids
        ]
        current = 0
        for trial in seeded:
            if current >= wanted:
                break
            selected.append(trial)
            selected_ids.add(trial.sequence)
            current += 1
        for trial in candidates:
            if current >= wanted:
                break
            if trial.sequence in selected_ids:
                continue
            selected.append(trial)
            selected_ids.add(trial.sequence)
            current += 1
        if current < wanted:
            raise ValueError(
                f"Only found {current}/{wanted} trials for obstacle_count={obstacle_count}"
            )
    return selected


def main() -> None:
    args = parse_args()
    selected = select_trials(args)
    rows = [
        {
            "sequence": trial.sequence,
            "obstacles": trial.obstacle_count,
            "lux": trial.lux,
            "has_video": int(trial.has_video),
            "required_files": (
                f"dataset/{trial.sequence}/optitrack.csv;"
                f"dataset/{trial.sequence}/imu.csv;"
                f"dataset/{trial.sequence}/radar.csv;"
                f"dataset/{trial.sequence}/{trial.sequence}.avi"
            ),
        }
        for trial in selected
    ]

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    counts = {}
    for row in rows:
        counts[row["obstacles"]] = counts.get(row["obstacles"], 0) + 1
    print(f"Wrote {out}")
    print(f"Selected {len(rows)} trials, obstacle counts: {counts}")
    print("Trial IDs:", ", ".join(row["sequence"] for row in rows))


if __name__ == "__main__":
    main()
