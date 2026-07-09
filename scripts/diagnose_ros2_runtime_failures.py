#!/usr/bin/env python3
"""Diagnose ROS2/Gazebo runtime verifier evidence.

The verifier writes raw logs and one-message topic samples.  This script turns
those files into a compact failure table with likely next actions.
"""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path


REQUIRED_BY_MODE = {
    "bbox": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/perception/bbox_markers",
    ],
    "synthetic": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/lidar/points",
    ],
    "depth_image": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/camera/depth/image",
        "/perception/depth_obstacle_markers",
    ],
    "cached_depth": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/camera/depth/image",
        "/perception/depth_obstacle_markers",
    ],
    "bbox_cached_depth_mux": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/camera/depth/image",
        "/perception/bbox_occupancy_grid",
        "/perception/depth_occupancy_grid",
        "/perception/costmap_mux_status",
        "/perception/bbox_markers",
        "/perception/depth_obstacle_markers",
    ],
    "gazebo_depth": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/camera/depth/image",
        "/perception/depth_obstacle_markers",
    ],
    "gazebo_laserscan": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/uav_oda/lidar_scan",
    ],
    "gazebo_fused": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/lidar/points",
        "/camera/depth/image",
        "/uav_oda/lidar_scan",
        "/perception/pointcloud_occupancy_grid",
        "/perception/depth_occupancy_grid",
        "/perception/laserscan_occupancy_grid",
        "/perception/costmap_mux_status",
        "/perception/depth_obstacle_markers",
    ],
}

LOG_PATTERNS = [
    "Traceback",
    "Exception",
    "Error",
    "ERROR",
    "Failed",
    "failed",
    "No module named",
    "PackageNotFoundError",
    "Could not",
    "not found",
]


@dataclass(frozen=True)
class DiagnosticRow:
    run_dir: str
    mode: str
    planner: str
    status: str
    missing_topics: str
    missing_messages: str
    launch_findings: str
    likely_next_action: str


def _read(path: Path) -> str:
    return path.read_text(errors="replace") if path.exists() else ""


def _topic_to_file(topic: str) -> str:
    return re.sub(r"^_", "", re.sub(r"[/ ]", "_", topic)) + ".txt"


def _infer_mode_planner_timestamp(name: str) -> tuple[str, str, str]:
    match = re.match(r"(.+?)_([^_]+)_(\d{8}_\d{6})$", name)
    if not match:
        return "unknown", "unknown", ""
    return match.group(1), match.group(2), match.group(3)


def _status(verify_log: str) -> str:
    if "Runtime verification PASSED" in verify_log:
        return "passed"
    if "Runtime verification FAILED" in verify_log:
        return "failed"
    if verify_log:
        return "incomplete"
    return "missing-log"


def _sample_ok(text: str) -> bool:
    if not text:
        return False
    bad_tokens = [
        "Could not determine the type",
        "Traceback",
        "No module named",
        "ERROR",
        "Error",
        "timed out",
        "No message received",
    ]
    return not any(token in text for token in bad_tokens)


def _log_findings(text: str, max_items: int = 5) -> list[str]:
    findings: list[str] = []
    for line in text.splitlines():
        if any(pattern in line for pattern in LOG_PATTERNS):
            cleaned = " ".join(line.strip().split())
            if cleaned and cleaned not in findings:
                findings.append(cleaned[:180])
        if len(findings) >= max_items:
            break
    return findings


def _likely_action(mode: str, missing_topics: list[str], missing_messages: list[str], findings: list[str]) -> str:
    joined = " ".join(missing_topics + missing_messages + findings)
    if mode == "bbox" and "/perception/bbox_markers" in joined:
        return "Check bbox CSV path and bbox_costmap_publisher startup logs."
    if mode == "cached_depth" and "/camera/depth/image" in joined:
        return "Check cached depth NPZ path and cached_depth_image_publisher startup logs."
    if mode == "bbox_cached_depth_mux" and "/perception/occupancy_grid" in joined:
        return "Check costmap_mux startup logs and source topics /perception/bbox_occupancy_grid and /perception/depth_occupancy_grid."
    if mode == "gazebo_depth" and "/camera/depth/image" in joined:
        return "Check Gazebo depth camera world, gz sim startup, and ros_gz_bridge Image bridge."
    if mode == "gazebo_laserscan" and "/uav_oda/lidar_scan" in joined:
        return "Check Gazebo GPU LiDAR world, gz sim startup, and ros_gz_bridge LaserScan bridge."
    if mode == "gazebo_fused" and "/perception/costmap_mux_status" in joined:
        return "Check fused source grids: pointcloud, depth, and laserscan costmaps must all publish before costmap_mux emits the planner map."
    if "/perception/occupancy_grid" in joined:
        return "Inspect costmap converter node in launch.log; sensor topic may be missing or decoder failed."
    if "/planned_path" in joined:
        return "Inspect costmap_planner logs; verify occupancy grid, odom/current pose, and goal_pose topics."
    if "/uav/current_pose" in joined or "/odom" in joined:
        return "Inspect kinematic_path_follower/static_pose_publisher startup logs."
    if "PackageNotFoundError" in joined or "package" in joined.lower() and "not found" in joined.lower():
        return "Run setup script/preflight and rebuild ros2_ws with sourced ROS2 environment."
    if "No module named" in joined:
        return "Install missing Python/ROS package reported in launch.log."
    if missing_messages:
        return "Topics exist but no samples arrived; increase DURATION_S/BAG_DURATION_S or inspect publisher rate."
    return "Inspect verify.log and launch.log for this run."


def _diagnose_run(run_dir: Path) -> DiagnosticRow:
    mode, planner, _timestamp = _infer_mode_planner_timestamp(run_dir.name)
    required = REQUIRED_BY_MODE.get(mode, [])
    verify_log = _read(run_dir / "verify.log")
    launch_log = _read(run_dir / "launch.log")
    colcon_log = _read(run_dir / "colcon_build.log")
    static_log = _read(run_dir / "static_planner_check.log")
    mux_validation_log = _read(run_dir / "costmap_mux_status_validation.log")
    topic_list = set(_read(run_dir / "topic_list.txt").splitlines())

    missing_topics = [topic for topic in required if topic not in topic_list]
    missing_messages = []
    for topic in required:
        sample_text = _read(run_dir / _topic_to_file(topic))
        if not _sample_ok(sample_text):
            missing_messages.append(topic)

    findings = _log_findings("\n".join([verify_log, launch_log, colcon_log, static_log, mux_validation_log]))
    return DiagnosticRow(
        run_dir=str(run_dir),
        mode=mode,
        planner=planner,
        status=_status(verify_log),
        missing_topics=";".join(missing_topics),
        missing_messages=";".join(missing_messages),
        launch_findings=" | ".join(findings),
        likely_next_action=_likely_action(mode, missing_topics, missing_messages, findings),
    )


def _find_runs(runtime_root: Path) -> list[Path]:
    if not runtime_root.exists():
        return []
    return sorted(path for path in runtime_root.iterdir() if path.is_dir())


def _write_csv(rows: list[DiagnosticRow], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(DiagnosticRow.__dataclass_fields__))
        writer.writeheader()
        for row in rows:
            writer.writerow(row.__dict__)


def _write_md(rows: list[DiagnosticRow], output_md: Path, output_csv: Path) -> None:
    output_md.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# ROS2 Runtime Diagnostics",
        "",
        f"CSV: `{output_csv}`",
        "",
    ]
    if not rows:
        lines.extend(
            [
                "No runtime verifier folders were found yet.",
                "",
                "Run `scripts/verify_ros2_costmap_all_modes.sh astar` on the ROS2/Gazebo server first.",
                "",
            ]
        )
    else:
        lines.extend(
            [
                "| Run | Mode | Status | Missing topics | Missing messages | Likely next action |",
                "|---|---|---|---|---|---|",
            ]
        )
        for row in rows:
            missing_topics = row.missing_topics or "-"
            missing_messages = row.missing_messages or "-"
            lines.append(
                f"| `{Path(row.run_dir).name}` | {row.mode} | {row.status} | "
                f"{missing_topics} | {missing_messages} | {row.likely_next_action} |"
            )
        lines.extend(
            [
                "",
                "Use each run folder's `verify.log`, `launch.log`, `topic_list.txt`, and topic sample files for detailed debugging.",
                "",
            ]
        )
    output_md.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-root", type=Path, default=Path("outputs/ros2_demo_runtime"))
    parser.add_argument("--output-csv", type=Path, default=Path("outputs/tables/ros2_runtime_diagnostics.csv"))
    parser.add_argument("--output-md", type=Path, default=Path("outputs/ros2_runtime_diagnostics.md"))
    args = parser.parse_args()

    rows = [_diagnose_run(run_dir) for run_dir in _find_runs(args.runtime_root)]
    _write_csv(rows, args.output_csv)
    _write_md(rows, args.output_md, args.output_csv)
    print(f"Wrote {args.output_md} and {args.output_csv} for {len(rows)} runtime run(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
