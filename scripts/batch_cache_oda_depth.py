#!/usr/bin/env python3
"""Batch cache monocular depth for ready ODA trials."""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--readiness", default="outputs/tables/target_20_trials_readiness.csv")
    parser.add_argument("--trial-ids", nargs="*", default=None)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "mps", "cuda"])
    parser.add_argument("--model-id", default="Intel/dpt-hybrid-midas")
    parser.add_argument("--input-width", type=int, default=384)
    parser.add_argument("--max-duration", type=float, default=None)
    parser.add_argument("--output-root", default="data/processed/depth")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def ready_sequences(path: Path) -> list[str]:
    with path.open(newline="") as f:
        return [row["sequence"] for row in csv.DictReader(f) if str(row.get("ready", "0")) == "1"]


def depth_output_path(output_root: Path, sequence: str, fps: float) -> Path:
    return output_root / str(sequence) / f"depth_{fps:g}fps.npz"


def main() -> None:
    args = parse_args()
    sequences = [str(item) for item in args.trial_ids] if args.trial_ids else ready_sequences(Path(args.readiness))
    if not sequences:
        raise SystemExit("No ready sequences found for depth caching.")

    output_root = Path(args.output_root)
    failures: list[tuple[str, int]] = []
    for sequence in sequences:
        output = depth_output_path(output_root, sequence, args.fps)
        if output.exists() and not args.force:
            print(f"Skip {sequence}: cache already exists at {output}")
            continue

        cmd = [
            sys.executable,
            "experiments/cache_monocular_depth.py",
            "--dataset-root",
            args.dataset_root,
            "--trial-id",
            sequence,
            "--fps",
            f"{args.fps:g}",
            "--device",
            args.device,
            "--model-id",
            args.model_id,
            "--input-width",
            str(args.input_width),
            "--output",
            str(output),
        ]
        if args.max_duration is not None:
            cmd.extend(["--max-duration", str(args.max_duration)])

        print(f"Cache depth for trial {sequence} -> {output}")
        result = subprocess.run(cmd, check=False)
        if result.returncode != 0:
            failures.append((sequence, result.returncode))
            print(f"Warning: depth caching failed for trial {sequence} with code {result.returncode}", file=sys.stderr)

    if failures:
        print("Depth caching finished with failures:", file=sys.stderr)
        for sequence, code in failures:
            print(f"  {sequence}: exit {code}", file=sys.stderr)
        raise SystemExit(1)

    print(f"Depth caching complete for {len(sequences)} requested trial(s).")


if __name__ == "__main__":
    main()
