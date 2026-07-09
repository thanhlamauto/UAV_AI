#!/usr/bin/env python3
"""Validate a saved `/perception/costmap_mux_status` ROS2 echo sample."""

from __future__ import annotations

import argparse
import ast
import json
import re
from pathlib import Path


DEFAULT_REQUIRED_INPUTS = [
    "perception/bbox_occupancy_grid",
    "perception/depth_occupancy_grid",
]


def _norm_topic(topic: str) -> str:
    return topic.strip().lstrip("/")


def _extract_data(text: str) -> str:
    for line in text.splitlines():
        if line.startswith("data:"):
            value = line.split(":", 1)[1].strip()
            if not value:
                break
            try:
                parsed = ast.literal_eval(value)
            except Exception:  # noqa: BLE001 - fallback to raw value.
                parsed = value
            if isinstance(parsed, str):
                return parsed
            return str(parsed)

    match = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if match:
        return match.group(0)
    raise ValueError("Could not find a JSON payload in mux status sample")


def validate_status(
    path: Path,
    required_inputs: list[str],
    min_source_occupied: int,
    min_merged_occupied: int,
) -> dict[str, object]:
    if not path.exists():
        raise FileNotFoundError(path)
    text = path.read_text(errors="replace")
    payload = json.loads(_extract_data(text))

    required = {_norm_topic(topic) for topic in required_inputs}
    input_topics = {_norm_topic(str(topic)) for topic in payload.get("input_topics", [])}
    received_topics = {_norm_topic(str(topic)) for topic in payload.get("received_topics", [])}
    missing_topics = [_norm_topic(str(topic)) for topic in payload.get("missing_topics", [])]
    source_occupied_raw = payload.get("source_occupied_cells", {})
    if not isinstance(source_occupied_raw, dict):
        raise ValueError("source_occupied_cells must be a dictionary")
    source_occupied = {_norm_topic(str(topic)): int(value) for topic, value in source_occupied_raw.items()}

    failures: list[str] = []
    if payload.get("state") != "merged":
        failures.append(f"state is {payload.get('state')!r}, expected 'merged'")
    if payload.get("require_all_inputs") is not True:
        failures.append("require_all_inputs is not true")
    if missing_topics:
        failures.append(f"missing_topics is not empty: {missing_topics}")

    missing_input_tokens = sorted(required - input_topics)
    if missing_input_tokens:
        failures.append(f"input_topics missing {missing_input_tokens}")
    missing_received = sorted(required - received_topics)
    if missing_received:
        failures.append(f"received_topics missing {missing_received}")

    for topic in sorted(required):
        occupied = source_occupied.get(topic, 0)
        if occupied < min_source_occupied:
            failures.append(f"source_occupied_cells[{topic}]={occupied}, expected >= {min_source_occupied}")

    merged = int(payload.get("merged_occupied_cells", 0))
    if merged < min_merged_occupied:
        failures.append(f"merged_occupied_cells={merged}, expected >= {min_merged_occupied}")

    if failures:
        raise ValueError("; ".join(failures))
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sample", type=Path)
    parser.add_argument("--required-input", action="append", default=[], help="Required mux input topic.")
    parser.add_argument("--min-source-occupied", type=int, default=1)
    parser.add_argument("--min-merged-occupied", type=int, default=1)
    args = parser.parse_args()

    required_inputs = args.required_input or DEFAULT_REQUIRED_INPUTS
    try:
        payload = validate_status(args.sample, required_inputs, args.min_source_occupied, args.min_merged_occupied)
    except Exception as exc:  # noqa: BLE001 - CLI should report all validation failures.
        print(f"Costmap mux status validation FAILED: {exc}")
        return 1

    print(
        "Costmap mux status validation PASSED: "
        f"state={payload.get('state')} merged_occupied_cells={payload.get('merged_occupied_cells')} "
        f"received_topics={payload.get('received_topics')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
