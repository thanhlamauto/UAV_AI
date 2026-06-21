#!/usr/bin/env python3
"""Check whether target ODA trials have all required local files."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", default="outputs/tables/target_20_trials_manifest.csv")
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--output", default="outputs/tables/target_20_trials_readiness.csv")
    return parser.parse_args()


def required_paths(dataset_root: Path, sequence: str) -> dict[str, Path]:
    return {
        "optitrack": dataset_root / sequence / "optitrack.csv",
        "rgb_video": dataset_root / sequence / f"{sequence}.avi",
        "radar": dataset_root / sequence / "radar.csv",
        "imu": dataset_root / sequence / "imu.csv",
    }


def main() -> None:
    args = parse_args()
    manifest = Path(args.manifest)
    dataset_root = Path(args.dataset_root)
    rows = []
    with manifest.open(newline="") as f:
        for row in csv.DictReader(f):
            sequence = row["sequence"]
            paths = required_paths(dataset_root, sequence)
            statuses = {f"has_{name}": path.exists() for name, path in paths.items()}
            sizes = {
                f"{name}_bytes": path.stat().st_size if path.exists() else 0
                for name, path in paths.items()
            }
            ready = all(statuses.values())
            rows.append(
                {
                    "sequence": sequence,
                    "obstacles": row.get("obstacles", ""),
                    "ready": int(ready),
                    **{key: int(value) for key, value in statuses.items()},
                    **sizes,
                }
            )

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    ready_count = sum(row["ready"] for row in rows)
    print(f"Wrote {out}")
    print(f"Ready trials: {ready_count}/{len(rows)}")
    missing = [row["sequence"] for row in rows if not row["ready"]]
    if missing:
        print(f"Missing/incomplete trials: {', '.join(missing)}")


if __name__ == "__main__":
    main()
