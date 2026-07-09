#!/usr/bin/env python3
"""Audit the focused LiDAR-bbox + depth-mux ROS2/Gazebo demo.

This is a narrower gate than `audit_ros2_demo_status.py`: it proves the most
important integration branch for this project, namely:

LiDAR bbox costmap + cached predicted-depth costmap -> costmap mux -> planner.
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


FOCUSED_MODE = "bbox_cached_depth_mux"
REQUIRED_TOPIC_FILES = [
    "perception_occupancy_grid.txt",
    "planned_path.txt",
    "uav_current_pose.txt",
    "odom.txt",
    "uav_marker.txt",
    "camera_depth_image.txt",
    "perception_bbox_occupancy_grid.txt",
    "perception_depth_occupancy_grid.txt",
    "perception_costmap_mux_status.txt",
    "perception_bbox_markers.txt",
    "perception_depth_obstacle_markers.txt",
]


@dataclass(frozen=True)
class Check:
    label: str
    ok: bool
    evidence: str


def _run(args: list[str], timeout_s: int = 30) -> tuple[bool, str]:
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=timeout_s, check=False)
    except Exception as exc:  # noqa: BLE001 - report command-launch failures.
        return False, str(exc)
    text = (result.stdout + result.stderr).strip().replace("\n", " | ")
    return result.returncode == 0, text[:500] or "ok"


def _read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _latest_focused_row(rows: list[dict[str, str]]) -> dict[str, str] | None:
    candidates = [row for row in rows if row.get("mode") == FOCUSED_MODE]
    return candidates[-1] if candidates else None


def _row_complete(row: dict[str, str] | None) -> tuple[bool, str]:
    if not row:
        return False, "no runtime row for bbox_cached_depth_mux"
    status_ok = row.get("status") == "passed"
    topics_ok = row.get("topics_present") == row.get("topics_expected")
    messages_ok = row.get("messages_received") == row.get("messages_expected")
    evidence = (
        f"status={row.get('status')} topics={row.get('topics_present')}/{row.get('topics_expected')} "
        f"messages={row.get('messages_received')}/{row.get('messages_expected')} run_dir={row.get('run_dir')}"
    )
    return status_ok and topics_ok and messages_ok, evidence


def _topic_files_complete(row: dict[str, str] | None) -> tuple[bool, str]:
    if not row or not row.get("run_dir"):
        return False, "no focused run_dir"
    run_dir = Path(row["run_dir"])
    missing = []
    empty = []
    for name in REQUIRED_TOPIC_FILES:
        path = run_dir / name
        if not path.exists():
            missing.append(name)
        elif path.stat().st_size == 0:
            empty.append(name)
    if missing or empty:
        pieces = []
        if missing:
            pieces.append("missing " + ", ".join(missing))
        if empty:
            pieces.append("empty " + ", ".join(empty))
        return False, "; ".join(pieces)
    return True, f"{len(REQUIRED_TOPIC_FILES)} topic sample file(s) present in {run_dir}"


def _mux_status_valid(row: dict[str, str] | None) -> tuple[bool, str]:
    if not row or not row.get("run_dir"):
        return False, "no focused run_dir"
    sample = Path(row["run_dir"]) / "perception_costmap_mux_status.txt"
    return _run([sys.executable, "scripts/validate_costmap_mux_status_sample.py", str(sample)])


def build_checks(summary_csv: Path) -> list[Check]:
    checks: list[Check] = []

    ok, evidence = _run([sys.executable, "scripts/check_perception_to_planner_contract.py"])
    checks.append(Check("Offline perception-to-planner contract", ok, evidence))

    ok, evidence = _run([sys.executable, "scripts/check_perception_planner_matrix.py"])
    checks.append(Check("Offline perception/planner matrix", ok, evidence))

    ok, evidence = _run([sys.executable, "scripts/check_ros2_launch_contract.py"])
    checks.append(Check("ROS2 launch/package contract", ok, evidence))

    rows = _read_rows(summary_csv)
    row = _latest_focused_row(rows)
    ok, evidence = _row_complete(row)
    checks.append(Check("Focused runtime row passed", ok, evidence))

    ok, evidence = _topic_files_complete(row)
    checks.append(Check("Focused topic samples saved", ok, evidence))

    ok, evidence = _mux_status_valid(row)
    checks.append(Check("Focused mux status proves merged inputs", ok, evidence))

    if row and row.get("video_file"):
        video = Path(row["video_file"])
        checks.append(
            Check(
                "Focused MP4 artifact saved",
                video.exists() and video.stat().st_size > 0,
                f"{video} ({video.stat().st_size if video.exists() else 0} bytes)",
            )
        )
    else:
        checks.append(Check("Focused MP4 artifact saved", False, "no video_file in focused runtime row"))

    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--summary-csv", type=Path, default=Path("outputs/tables/ros2_demo_runtime_summary.csv"))
    parser.add_argument("--fail-on-incomplete", action="store_true")
    args = parser.parse_args()

    checks = build_checks(args.summary_csv)
    all_ok = True
    print("ROS2/Gazebo fused perception demo audit")
    print("=======================================")
    for check in checks:
        status = "PASS" if check.ok else "MISSING"
        print(f"{status:7} {check.label}: {check.evidence}")
        all_ok = all_ok and check.ok
    print()
    print("FOCUSED_COMPLETE" if all_ok else "FOCUSED_INCOMPLETE")
    return 1 if args.fail_on_incomplete and not all_ok else 0


if __name__ == "__main__":
    raise SystemExit(main())
