#!/usr/bin/env python3
"""Self-test the costmap mux status validator with pass/fail samples."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

from validate_costmap_mux_status_sample import validate_status


def _write_sample(path: Path, payload: dict[str, object]) -> None:
    path.write_text(f"data: {json.dumps(json.dumps(payload))}\n---\n", encoding="utf-8")


def _base_payload() -> dict[str, object]:
    return {
        "state": "merged",
        "input_topics": ["perception/bbox_occupancy_grid", "perception/depth_occupancy_grid"],
        "received_topics": ["perception/bbox_occupancy_grid", "perception/depth_occupancy_grid"],
        "missing_topics": [],
        "require_all_inputs": True,
        "source_occupied_cells": {
            "perception/bbox_occupancy_grid": 12,
            "perception/depth_occupancy_grid": 8,
        },
        "merged_occupied_cells": 19,
    }


def _expect_pass(path: Path, payload: dict[str, object]) -> None:
    _write_sample(path, payload)
    validate_status(path, ["perception/bbox_occupancy_grid", "perception/depth_occupancy_grid"], 1, 1)


def _expect_fail(path: Path, payload: dict[str, object], label: str) -> None:
    _write_sample(path, payload)
    try:
        validate_status(path, ["perception/bbox_occupancy_grid", "perception/depth_occupancy_grid"], 1, 1)
    except Exception:
        return
    raise AssertionError(f"Expected validator failure for {label}")


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp_dir:
        sample = Path(tmp_dir) / "mux_status.txt"
        good = _base_payload()
        _expect_pass(sample, good)

        waiting = _base_payload()
        waiting["state"] = "waiting"
        _expect_fail(sample, waiting, "state waiting")

        missing = _base_payload()
        missing["missing_topics"] = ["perception/depth_occupancy_grid"]
        _expect_fail(sample, missing, "missing topic")

        no_depth = _base_payload()
        no_depth["received_topics"] = ["perception/bbox_occupancy_grid"]
        _expect_fail(sample, no_depth, "missing received depth topic")

        zero_source = _base_payload()
        zero_source["source_occupied_cells"] = {
            "perception/bbox_occupancy_grid": 12,
            "perception/depth_occupancy_grid": 0,
        }
        _expect_fail(sample, zero_source, "zero source occupancy")

        zero_merged = _base_payload()
        zero_merged["merged_occupied_cells"] = 0
        _expect_fail(sample, zero_merged, "zero merged occupancy")

    print("Costmap mux status validator self-test PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
