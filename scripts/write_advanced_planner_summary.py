#!/usr/bin/env python3
"""Write a concise advanced-planner benchmark summary."""

from __future__ import annotations

import csv
from pathlib import Path


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def markdown_table(rows: list[dict[str, str]]) -> str:
    if not rows:
        return "_No planner summary rows found._"
    headers = [
        "method",
        "trials",
        "collision_rate",
        "safety_violation_rate",
        "mean_min_clearance_m",
        "mean_path_length_m",
        "mean_planner_compute_time_ms",
    ]
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] + ["---:"] * (len(headers) - 1)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(row.get(header, "") for header in headers) + " |")
    return "\n".join(lines)


def main() -> None:
    summary_path = Path("outputs/tables/planner_comparison_summary.csv")
    failures_path = Path("outputs/tables/planner_failures.csv")
    output = Path("outputs/advanced_planner_summary.md")
    output.parent.mkdir(parents=True, exist_ok=True)

    rows = read_csv(summary_path)
    failures = read_csv(failures_path)
    methods = {row.get("method", "") for row in rows}
    advanced_present = sorted(method for method in ["rrt_star", "mppi"] if method in methods)

    text = f"""# Advanced Planner Summary

## Status

Advanced planner methods present in the latest benchmark: {", ".join(advanced_present) if advanced_present else "none yet"}.

The comparison uses the same ODA ground-plane obstacle model and the same metrics as the earlier human/straight-line/geometric/A*/RRT benchmark:

- collision rate;
- safety-distance violation rate;
- minimum obstacle-boundary clearance;
- path length;
- heading-change smoothness;
- planner compute time.

## Latest Planner Table

{markdown_table(rows)}

## Planner Failures

"""
    if failures:
        text += "| sequence | method | reason |\n| --- | --- | --- |\n"
        for failure in failures:
            text += f"| {failure.get('sequence', '')} | {failure.get('method', '')} | {failure.get('reason', '')} |\n"
    else:
        text += "No planner-level failures were recorded in `outputs/tables/planner_failures.csv`.\n"

    text += """
## Interpretation Checklist

- Prefer methods with zero collision and zero safety-distance violation before optimizing path length.
- Treat MPPI and RRT* compute time as prototype Python timings, not optimized controller timings.
- Compare against the straight-line baseline to show why obstacle-aware planning is necessary.
"""
    output.write_text(text)
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
