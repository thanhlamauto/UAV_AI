#!/usr/bin/env python3
"""Extract selected ODA trial files from the full 4TU ZIP archive.

The 4TU record currently provides one ~98GB archive, not per-trial files.
Use this script after downloading the full ZIP to extract only selected
OptiTrack/RGB/radar/IMU files into data/raw/ODA_Dataset/dataset/.
"""

from __future__ import annotations

import argparse
import csv
import shutil
import zipfile
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("zip_path", help="Path to Dupeyroux_et_al_2021_ODA_DATASET_Full.zip")
    parser.add_argument("--manifest", default="outputs/tables/target_20_trials_manifest.csv")
    parser.add_argument("--output-root", default="data/raw/ODA_Dataset")
    parser.add_argument(
        "--include-bag",
        action="store_true",
        help="Also extract ROS .bag files. Disabled by default because they are large.",
    )
    return parser.parse_args()


def read_manifest(path: Path) -> list[str]:
    with path.open(newline="") as f:
        return [row["sequence"] for row in csv.DictReader(f)]


def wanted_suffixes(sequences: list[str], include_bag: bool) -> set[str]:
    suffixes = {"dataset/trial_overview.csv"}
    for seq in sequences:
        suffixes.update(
            {
                f"dataset/{seq}/optitrack.csv",
                f"dataset/{seq}/imu.csv",
                f"dataset/{seq}/radar.csv",
                f"dataset/{seq}/{seq}.avi",
            }
        )
        if include_bag:
            suffixes.add(f"dataset/{seq}/{seq}.bag")
    return suffixes


def normalized_member_name(name: str) -> str:
    parts = Path(name).parts
    if "dataset" in parts:
        idx = parts.index("dataset")
        return "/".join(parts[idx:])
    return name


def main() -> None:
    args = parse_args()
    zip_path = Path(args.zip_path)
    output_root = Path(args.output_root)
    sequences = read_manifest(Path(args.manifest))
    wanted = wanted_suffixes(sequences, args.include_bag)

    extracted = []
    with zipfile.ZipFile(zip_path) as zf:
        members = zf.infolist()
        by_normalized = {normalized_member_name(info.filename): info for info in members}
        for suffix in sorted(wanted):
            info = by_normalized.get(suffix)
            if info is None:
                print(f"Missing in ZIP: {suffix}")
                continue
            target = output_root / suffix
            target.parent.mkdir(parents=True, exist_ok=True)
            with zf.open(info) as src, target.open("wb") as dst:
                shutil.copyfileobj(src, dst, length=1024 * 1024)
            extracted.append(str(target))
            print(f"Extracted {target}")
    print(f"Extracted {len(extracted)} files for {len(sequences)} trials")


if __name__ == "__main__":
    main()
