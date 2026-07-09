#!/usr/bin/env python3
"""Validate outputs of the indoor 3D ESDF MPPI demo."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metrics", type=Path, default=Path("outputs/tables/indoor_3d_esdf_mppi_metrics.csv"))
    parser.add_argument("--summary", type=Path, default=Path("outputs/indoor_3d_esdf_mppi_summary.md"))
    parser.add_argument("--figure", type=Path, default=Path("outputs/figures/indoor_3d_esdf_mppi_path.png"))
    parser.add_argument("--slice-figure", type=Path, default=Path("outputs/figures/indoor_3d_esdf_mppi_slice.png"))
    parser.add_argument("--min-safety-margin", type=float, default=0.0)
    parser.add_argument("--min-altitude-change", type=float, default=0.20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    missing = [path for path in [args.metrics, args.summary, args.figure, args.slice_figure] if not path.exists()]
    if missing:
        for path in missing:
            print(f"MISS {path}")
        return 1

    with args.metrics.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if len(rows) != 1:
        print(f"FAIL expected 1 metrics row, found {len(rows)}")
        return 1
    row = rows[0]
    checks = {
        "collision": int(float(row["collision"])) == 0,
        "safety_violation": int(float(row["safety_violation"])) == 0,
        "min_safety_margin_m": float(row["min_safety_margin_m"]) > args.min_safety_margin,
        "path_length_m": float(row["path_length_m"]) > 0.0,
        "altitude_change_m": float(row["altitude_change_m"]) >= args.min_altitude_change,
        "planner_compute_time_ms": float(row["planner_compute_time_ms"]) > 0.0,
        "cost_reduction_pct": float(row["cost_reduction_pct"]) > 0.0,
    }
    for name, ok in checks.items():
        print(f"{'PASS' if ok else 'FAIL'} {name}: {row.get(name, '')}")
    if not all(checks.values()):
        return 1
    print("3D ESDF MPPI demo check PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
