#!/usr/bin/env python3
"""Summarize ROS2/Gazebo runtime verifier outputs.

The verifier stores one directory per run under `outputs/ros2_demo_runtime/`.
This script turns those raw logs into a small CSV and Markdown summary that can
be pulled back from the server and cited in the progress report.
"""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path

from validate_costmap_mux_status_sample import DEFAULT_REQUIRED_INPUTS, validate_status


REQUIRED_BY_MODE = {
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
    "bbox": [
        "/perception/occupancy_grid",
        "/planned_path",
        "/uav/current_pose",
        "/odom",
        "/uav/marker",
        "/perception/bbox_markers",
    ],
}


@dataclass(frozen=True)
class RuntimeRun:
    run_dir: Path
    mode: str
    planner: str
    timestamp: str
    status: str
    topics_present: int
    topics_expected: int
    messages_received: int
    messages_expected: int
    mux_status_valid: str
    mux_validation_log: str
    launch_log_lines: int
    bag_dir: str
    bag_files: int
    bag_bytes: int
    video_file: str
    video_bytes: int


def _topic_to_file(topic: str) -> str:
    return re.sub(r"^_", "", re.sub(r"[/ ]", "_", topic)) + ".txt"


def _infer_mode_planner_timestamp(name: str) -> tuple[str, str, str]:
    match = re.match(r"(.+?)_([^_]+)_(\d{8}_\d{6})$", name)
    if not match:
        return "unknown", "unknown", ""
    return match.group(1), match.group(2), match.group(3)


def _read(path: Path) -> str:
    return path.read_text(errors="replace") if path.exists() else ""


def _mux_status_valid(run_dir: Path, mode: str) -> tuple[str, str]:
    if mode not in {"bbox_cached_depth_mux", "gazebo_fused"}:
        return "not_applicable", ""
    sample = run_dir / "perception_costmap_mux_status.txt"
    validation_log = run_dir / "costmap_mux_status_validation.log"
    required_inputs = DEFAULT_REQUIRED_INPUTS
    if mode == "gazebo_fused":
        required_inputs = [
            "perception/pointcloud_occupancy_grid",
            "perception/depth_occupancy_grid",
            "perception/laserscan_occupancy_grid",
        ]
    try:
        validate_status(sample, required_inputs, min_source_occupied=1, min_merged_occupied=1)
    except Exception:
        return "failed", str(validation_log)
    return "passed", str(validation_log)


def _summarize_run(run_dir: Path) -> RuntimeRun:
    mode, planner, timestamp = _infer_mode_planner_timestamp(run_dir.name)
    verify_log = _read(run_dir / "verify.log")
    topic_list = set(_read(run_dir / "topic_list.txt").splitlines())
    required_topics = REQUIRED_BY_MODE.get(mode, [])

    if "Runtime verification PASSED" in verify_log:
        status = "passed"
    elif "Runtime verification FAILED" in verify_log:
        status = "failed"
    elif verify_log:
        status = "incomplete"
    else:
        status = "missing-log"

    topics_present = sum(1 for topic in required_topics if topic in topic_list)
    messages_received = 0
    for topic in required_topics:
        sample = run_dir / _topic_to_file(topic)
        text = _read(sample)
        if text and "Could not determine the type" not in text and "Traceback" not in text:
            messages_received += 1
    mux_status_valid, mux_validation_log = _mux_status_valid(run_dir, mode)

    launch_log = _read(run_dir / "launch.log")
    launch_log_lines = len(launch_log.splitlines()) if launch_log else 0
    bag_dirs = sorted(path for path in run_dir.glob("rosbag_*") if path.is_dir())
    bag_dir = str(bag_dirs[0]) if bag_dirs else ""
    bag_files = 0
    bag_bytes = 0
    if bag_dirs:
        for file_path in bag_dirs[0].rglob("*"):
            if file_path.is_file():
                bag_files += 1
                bag_bytes += file_path.stat().st_size
    videos = sorted(run_dir.glob("*.mp4"))
    video_file = str(videos[0]) if videos else ""
    video_bytes = videos[0].stat().st_size if videos else 0

    return RuntimeRun(
        run_dir=run_dir,
        mode=mode,
        planner=planner,
        timestamp=timestamp,
        status=status,
        topics_present=topics_present,
        topics_expected=len(required_topics),
        messages_received=messages_received,
        messages_expected=len(required_topics),
        mux_status_valid=mux_status_valid,
        mux_validation_log=mux_validation_log,
        launch_log_lines=launch_log_lines,
        bag_dir=bag_dir,
        bag_files=bag_files,
        bag_bytes=bag_bytes,
        video_file=video_file,
        video_bytes=video_bytes,
    )


def _find_runs(runtime_root: Path) -> list[RuntimeRun]:
    if not runtime_root.exists():
        return []
    runs = [_summarize_run(path) for path in sorted(runtime_root.iterdir()) if path.is_dir()]
    return sorted(runs, key=lambda run: (run.timestamp, run.mode, run.planner))


def _write_csv(runs: list[RuntimeRun], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "run_dir",
                "mode",
                "planner",
                "timestamp",
                "status",
                "topics_present",
                "topics_expected",
                "messages_received",
                "messages_expected",
                "mux_status_valid",
                "mux_validation_log",
                "launch_log_lines",
                "bag_dir",
                "bag_files",
                "bag_bytes",
                "video_file",
                "video_bytes",
            ],
        )
        writer.writeheader()
        for run in runs:
            writer.writerow(
                {
                    "run_dir": str(run.run_dir),
                    "mode": run.mode,
                    "planner": run.planner,
                    "timestamp": run.timestamp,
                    "status": run.status,
                    "topics_present": run.topics_present,
                    "topics_expected": run.topics_expected,
                    "messages_received": run.messages_received,
                    "messages_expected": run.messages_expected,
                    "mux_status_valid": run.mux_status_valid,
                    "mux_validation_log": run.mux_validation_log,
                    "launch_log_lines": run.launch_log_lines,
                    "bag_dir": run.bag_dir,
                    "bag_files": run.bag_files,
                    "bag_bytes": run.bag_bytes,
                    "video_file": run.video_file,
                    "video_bytes": run.video_bytes,
                }
            )


def _write_markdown(runs: list[RuntimeRun], output_md: Path, runtime_root: Path, output_csv: Path) -> None:
    output_md.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# ROS2 Runtime Evidence Summary",
        "",
        f"Runtime root: `{runtime_root}`",
        f"CSV table: `{output_csv}`",
        "",
    ]
    if not runs:
        lines.extend(
            [
                "No runtime verifier folders were found yet.",
                "",
                "Run on a ROS2/Gazebo server:",
                "",
                "```bash",
                "scripts/verify_ros2_costmap_runtime.sh bbox astar",
                "scripts/verify_ros2_costmap_runtime.sh synthetic astar",
                "scripts/verify_ros2_costmap_runtime.sh depth_image astar",
                "scripts/verify_ros2_costmap_runtime.sh cached_depth astar",
                "scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar",
                "scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar",
                "scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar",
                "scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar",
                "```",
                "",
            ]
        )
    else:
        lines.extend(
            [
                "| Run | Mode | Planner | Status | Topics | Messages | Mux | Bag MB | Video MB |",
                "|---|---|---|---|---:|---:|---|---:|---:|",
            ]
        )
        for run in runs:
            bag_mb = run.bag_bytes / (1024 * 1024)
            video_mb = run.video_bytes / (1024 * 1024)
            lines.append(
                "| "
                f"`{run.run_dir.name}` | {run.mode} | {run.planner} | {run.status} | "
                f"{run.topics_present}/{run.topics_expected} | "
                f"{run.messages_received}/{run.messages_expected} | "
                f"{run.mux_status_valid} | "
                f"{bag_mb:.2f} | "
                f"{video_mb:.2f} |"
            )
        lines.extend(
            [
                "",
                "A run is report-ready when `status=passed`, all expected topics are present, and one message sample exists for every required topic. For muxed modes such as `bbox_cached_depth_mux` and `gazebo_fused`, `Mux` must also be `passed`.",
                "",
            ]
        )
    lines.extend(
        [
            "Verifier commands:",
            "",
            "```bash",
            "scripts/verify_ros2_costmap_runtime.sh bbox astar",
            "scripts/verify_ros2_costmap_runtime.sh synthetic astar",
            "scripts/verify_ros2_costmap_runtime.sh depth_image astar",
            "scripts/verify_ros2_costmap_runtime.sh cached_depth astar",
            "scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar",
            "scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar",
            "scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar",
            "scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar",
            "```",
            "",
        ]
    )
    output_md.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-root", default="outputs/ros2_demo_runtime", type=Path)
    parser.add_argument("--output-md", default="outputs/ros2_demo_runtime_summary.md", type=Path)
    parser.add_argument("--output-csv", default="outputs/tables/ros2_demo_runtime_summary.csv", type=Path)
    args = parser.parse_args()

    runs = _find_runs(args.runtime_root)
    _write_csv(runs, args.output_csv)
    _write_markdown(runs, args.output_md, args.runtime_root, args.output_csv)
    print(f"Wrote {args.output_md} and {args.output_csv} for {len(runs)} runtime run(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
