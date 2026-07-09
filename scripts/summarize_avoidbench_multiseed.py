#!/usr/bin/env python3
import ast
import csv
import math
import re
import sys
from pathlib import Path


PLANNERS = ["rrt", "rrt_star", "mppi"]
SPEEDS = ["1", "2", "3", "4", "5", "6"]


def parse_array(text, key):
    match = re.search(rf"{key}:\s*(\[[^\]]*\])", text)
    if not match:
        return []
    try:
        return [float(x) for x in ast.literal_eval(match.group(1))]
    except Exception:
        return []


def outcome_counts(metrics_text, status_text):
    if "rc=124" in status_text or "factors:" not in metrics_text:
        return 0, 0, 8
    progress = parse_array(metrics_text, "mission_progress")
    collisions = parse_array(metrics_text, "collision_number")
    n = max(len(progress), len(collisions), 1)
    safe = coll = timeout = 0
    for i in range(n):
        p = progress[i] if i < len(progress) else math.nan
        c = collisions[i] if i < len(collisions) else math.nan
        if not math.isfinite(p):
            timeout += 1
        elif c > 0:
            coll += 1
        elif p >= 0.999:
            safe += 1
        else:
            timeout += 1
    return safe, coll, timeout


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/root/avoidbench_results/matrix_8seed")
    rows = []
    for planner in PLANNERS:
        for speed in SPEEDS:
            tag = f"{planner}_{speed}"
            metrics = (root / f"{tag}_metrics.yaml").read_text(errors="ignore") if (root / f"{tag}_metrics.yaml").exists() else ""
            status = (root / f"{tag}_status.txt").read_text(errors="ignore").strip() if (root / f"{tag}_status.txt").exists() else ""
            safe, coll, timeout = outcome_counts(metrics, status)
            compute = parse_array(metrics, "processing_time")
            progress = parse_array(metrics, "mission_progress")
            rows.append(
                {
                    "planner": planner,
                    "speed_mps": float(speed),
                    "safe": safe,
                    "collision": coll,
                    "timeout": timeout,
                    "safe_pct": 100.0 * safe / 8.0,
                    "collision_pct": 100.0 * coll / 8.0,
                    "timeout_pct": 100.0 * timeout / 8.0,
                    "mean_compute_ms": sum(compute) / len(compute) if compute else math.nan,
                    "mean_progress": sum(progress) / len(progress) if progress else math.nan,
                    "status": status,
                }
            )

    with (root / "summary_8seed.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    first_fail = {}
    mean_compute = {}
    for planner in PLANNERS:
        p_rows = [r for r in rows if r["planner"] == planner]
        valid_compute = [r["mean_compute_ms"] for r in p_rows if math.isfinite(r["mean_compute_ms"])]
        mean_compute[planner] = sum(valid_compute) / len(valid_compute) if valid_compute else math.nan
        ff = ">6"
        for r in p_rows:
            if r["collision"] > 0 or r["timeout"] > 0:
                ff = f"{r['speed_mps']:.0f}"
                break
        first_fail[planner] = ff

    with (root / "summary_8seed_rates.md").open("w") as f:
        f.write("# AvoidBench 8-map Outcome Rates\n\n")
        f.write("| Planner | 1 m/s | 2 m/s | 3 m/s | 4 m/s | 5 m/s | 6 m/s |\n")
        f.write("|---|---:|---:|---:|---:|---:|---:|\n")
        for planner in PLANNERS:
            vals = []
            for speed in SPEEDS:
                r = next(x for x in rows if x["planner"] == planner and x["speed_mps"] == float(speed))
                vals.append(f"{r['safe_pct']:.1f}/{r['collision_pct']:.1f}/{r['timeout_pct']:.1f}")
            f.write(f"| {planner} | " + " | ".join(vals) + " |\n")
        f.write("\nOutcome cell format: Safe/Collision/Timeout %. Denominator: 8 generated AvoidBench outdoor maps.\n\n")
        f.write("| Planner | Mean compute ms | First fail speed |\n")
        f.write("|---|---:|---:|\n")
        for planner in PLANNERS:
            name = {"rrt": "RRT", "rrt_star": "RRT*", "mppi": "MPPI"}[planner]
            f.write(f"| {name} | {mean_compute[planner]:.1f} | {first_fail[planner]} m/s |\n")

    print((root / "summary_8seed.csv").read_text())
    print((root / "summary_8seed_rates.md").read_text())


if __name__ == "__main__":
    main()
