#!/usr/bin/env python3
"""Audit current evidence against the active ODA planner benchmark goal."""

from __future__ import annotations

import csv
from pathlib import Path


def count_rows(path: str | Path) -> int:
    path = Path(path)
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(newline="") as f:
        return sum(1 for _ in csv.DictReader(f))


def csv_values(path: str | Path, column: str) -> list[str]:
    with Path(path).open(newline="") as f:
        return [row[column] for row in csv.DictReader(f)]


def main() -> None:
    checks = []

    readiness = Path("outputs/tables/target_20_trials_readiness.csv")
    manifest = Path("outputs/tables/target_20_trials_manifest.csv")
    metrics = Path("outputs/tables/batch_planner_metrics.csv")
    summary = Path("outputs/tables/planner_comparison_summary.csv")

    manifest_rows = count_rows(manifest)
    ready_rows = 0
    if readiness.exists():
        with readiness.open(newline="") as f:
            ready_rows = sum(int(row["ready"]) for row in csv.DictReader(f))

    methods = set(csv_values(metrics, "method")) if metrics.exists() and metrics.stat().st_size else set()
    required_methods = {"human", "straight_line", "astar_grid"}
    has_geometric = any(method.startswith("geometric_bypass") for method in methods)

    checks.append(("20-trial manifest exists", manifest_rows >= 20, f"{manifest_rows} rows"))
    checks.append(("20 trials fully downloaded", ready_rows >= 20, f"{ready_rows}/20 ready"))
    checks.append(("batch benchmark output exists", count_rows(metrics) > 0, f"{count_rows(metrics)} metric rows"))
    checks.append(("risk labels included", "future_risk_count" in (Path(metrics).read_text().splitlines()[0] if metrics.exists() and metrics.stat().st_size else ""), "future_risk_count column"))
    checks.append(("straight-line baseline included", "straight_line" in methods, f"methods={sorted(methods)}"))
    checks.append(("geometric bypass baseline included", has_geometric, f"methods={sorted(methods)}"))
    checks.append(("human trajectory included", "human" in methods, f"methods={sorted(methods)}"))
    checks.append(("A* baseline included", "astar_grid" in methods, f"methods={sorted(methods)}"))
    checks.append(("RRT baseline included", "rrt" in methods, f"methods={sorted(methods)}"))
    checks.append(("RRT* baseline included", "rrt_star" in methods, f"methods={sorted(methods)}"))
    checks.append(("MPPI baseline included", "mppi" in methods, f"methods={sorted(methods)}"))
    checks.append(("planner summary exists", count_rows(summary) > 0, f"{count_rows(summary)} rows"))

    print("Goal completion audit")
    print("=====================")
    all_ok = True
    for label, ok, evidence in checks:
        status = "PASS" if ok else "MISSING"
        print(f"{status:7} {label}: {evidence}")
        all_ok = all_ok and ok
    print()
    print("COMPLETE" if all_ok else "INCOMPLETE")


if __name__ == "__main__":
    main()
