#!/usr/bin/env python3
"""Audit ROS2/Gazebo costmap demo readiness.

This audit checks static repository artifacts locally and, when available,
runtime evidence generated on a ROS2/Gazebo server by
`scripts/verify_ros2_costmap_runtime.sh`.
"""

from __future__ import annotations

import argparse
import csv
import os
import py_compile
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


REQUIRED_FILES = [
    "ros2_ws/src/uav_oda_ros2_demo/package.xml",
    "ros2_ws/src/uav_oda_ros2_demo/setup.py",
    "ros2_ws/src/uav_oda_ros2_demo/config/demo_params.yaml",
    "ros2_ws/src/uav_oda_ros2_demo/config/rviz_demo.rviz",
    "ros2_ws/src/uav_oda_ros2_demo/worlds/indoor_obstacles.sdf",
    "ros2_ws/src/uav_oda_ros2_demo/launch/bbox_replay_planner.launch.py",
    "ros2_ws/src/uav_oda_ros2_demo/launch/px4_gazebo_costmap_demo.launch.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/grid_planners.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/costmap_converters.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/bbox_costmap_publisher.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/cached_depth_image_publisher.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/costmap_mux_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/depth_image_costmap_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/pointcloud_costmap_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/laserscan_costmap_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/costmap_planner_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/mppi_local_controller.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/kinematic_path_follower_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/px4_odometry_bridge_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/px4_mppi_offboard_controller_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/px4_waypoint_follower_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/synthetic_depth_image_publisher.py",
    "scripts/setup_ros2_gazebo_server.sh",
    "scripts/check_ros2_server_preflight.sh",
    "scripts/run_ros2_costmap_demo.sh",
    "scripts/run_ros2_gazebo_fused_px4.sh",
    "scripts/run_headless_ros2_runtime_video.sh",
    "scripts/verify_ros2_fused_perception_demo.sh",
    "scripts/verify_ros2_costmap_runtime.sh",
    "scripts/verify_ros2_costmap_all_modes.sh",
    "scripts/audit_ros2_fused_demo_status.py",
    "scripts/validate_costmap_mux_status_sample.py",
    "scripts/check_costmap_mux_status_validator.py",
    "scripts/summarize_ros2_runtime_evidence.py",
    "scripts/diagnose_ros2_runtime_failures.py",
    "scripts/write_ros2_demo_report_section.py",
    "scripts/render_ros2_costmap_demo_video.py",
    "scripts/render_perception_to_planner_contract_figure.py",
    "scripts/bundle_ros2_demo_artifacts.py",
    "scripts/check_ros2_costmap_demo_static.py",
    "scripts/check_ros2_launch_contract.py",
    "scripts/check_ros2_mode_consistency.py",
    "scripts/check_perception_to_planner_contract.py",
    "scripts/check_perception_planner_matrix.py",
    "docs/ros2_gazebo_costmap_demo.md",
    "outputs/ros2_gazebo_costmap_demo_summary.md",
    "outputs/perception_planner_matrix.md",
    "outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv",
    "outputs/tables/perception_planner_matrix.csv",
    "data/processed/depth_sample_3_5fps.npz",
    "outputs/figures/perception_to_planner_contract.svg",
    "outputs/videos/ros2_costmap_demo_astar.mp4",
]

EXECUTABLE_SCRIPTS = [
    "scripts/setup_ros2_gazebo_server.sh",
    "scripts/check_ros2_server_preflight.sh",
    "scripts/run_ros2_costmap_demo.sh",
    "scripts/run_ros2_gazebo_fused_px4.sh",
    "scripts/run_headless_ros2_runtime_video.sh",
    "scripts/verify_ros2_fused_perception_demo.sh",
    "scripts/verify_ros2_costmap_runtime.sh",
    "scripts/verify_ros2_costmap_all_modes.sh",
    "scripts/audit_ros2_fused_demo_status.py",
    "scripts/validate_costmap_mux_status_sample.py",
    "scripts/check_costmap_mux_status_validator.py",
    "scripts/summarize_ros2_runtime_evidence.py",
    "scripts/diagnose_ros2_runtime_failures.py",
    "scripts/write_ros2_demo_report_section.py",
    "scripts/render_ros2_costmap_demo_video.py",
    "scripts/render_perception_to_planner_contract_figure.py",
    "scripts/bundle_ros2_demo_artifacts.py",
    "scripts/check_ros2_costmap_demo_static.py",
    "scripts/check_ros2_launch_contract.py",
    "scripts/check_ros2_mode_consistency.py",
    "scripts/check_perception_to_planner_contract.py",
    "scripts/check_perception_planner_matrix.py",
]

PYTHON_FILES = [
    "scripts/summarize_ros2_runtime_evidence.py",
    "scripts/diagnose_ros2_runtime_failures.py",
    "scripts/write_ros2_demo_report_section.py",
    "scripts/render_ros2_costmap_demo_video.py",
    "scripts/render_perception_to_planner_contract_figure.py",
    "scripts/bundle_ros2_demo_artifacts.py",
    "scripts/audit_ros2_fused_demo_status.py",
    "scripts/validate_costmap_mux_status_sample.py",
    "scripts/check_costmap_mux_status_validator.py",
    "scripts/check_ros2_costmap_demo_static.py",
    "scripts/check_ros2_launch_contract.py",
    "scripts/check_ros2_mode_consistency.py",
    "ros2_ws/src/uav_oda_ros2_demo/launch/bbox_replay_planner.launch.py",
    "ros2_ws/src/uav_oda_ros2_demo/launch/px4_gazebo_costmap_demo.launch.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/grid_planners.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/costmap_converters.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/bbox_costmap_publisher.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/cached_depth_image_publisher.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/costmap_mux_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/depth_image_costmap_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/pointcloud_costmap_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/laserscan_costmap_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/costmap_planner_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/mppi_local_controller.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/kinematic_path_follower_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/px4_odometry_bridge_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/static_pose_publisher.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/synthetic_depth_image_publisher.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/synthetic_pointcloud_publisher.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/px4_mppi_offboard_controller_node.py",
    "ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/px4_waypoint_follower_node.py",
    "scripts/check_perception_to_planner_contract.py",
    "scripts/check_perception_planner_matrix.py",
]

RUNTIME_REQUIRED_MODES = [
    "bbox",
    "synthetic",
    "depth_image",
    "cached_depth",
    "bbox_cached_depth_mux",
    "gazebo_depth",
    "gazebo_laserscan",
    "gazebo_fused",
]


@dataclass(frozen=True)
class Check:
    label: str
    ok: bool
    evidence: str


def _exists(path: str) -> bool:
    return Path(path).exists()


def _is_executable(path: str) -> bool:
    return os.access(path, os.X_OK)


def _compile_python(paths: list[str]) -> tuple[bool, str]:
    failures = []
    for path in paths:
        try:
            py_compile.compile(path, doraise=True)
        except Exception as exc:  # noqa: BLE001 - report all compile failures.
            failures.append(f"{path}: {exc}")
    if failures:
        return False, "; ".join(failures[:3])
    return True, f"{len(paths)} files"


def _parse_xml(paths: list[str]) -> tuple[bool, str]:
    failures = []
    for path in paths:
        try:
            ET.parse(path)
        except Exception as exc:  # noqa: BLE001 - report all XML parse failures.
            failures.append(f"{path}: {exc}")
    if failures:
        return False, "; ".join(failures)
    return True, ", ".join(paths)


def _run_command(args: list[str], timeout_s: int = 30) -> tuple[bool, str]:
    try:
        result = subprocess.run(args, check=False, capture_output=True, text=True, timeout=timeout_s)
    except Exception as exc:  # noqa: BLE001 - report command launch failures.
        return False, str(exc)
    text = (result.stdout + result.stderr).strip().replace("\n", " | ")
    if result.returncode != 0:
        return False, text[:500] or f"exit code {result.returncode}"
    return True, text[:500] or "ok"


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _runtime_mode_passed(rows: list[dict[str, str]], mode: str) -> tuple[bool, str]:
    candidates = [row for row in rows if row.get("mode") == mode and row.get("status") == "passed"]
    for row in candidates:
        topics_ok = row.get("topics_present") == row.get("topics_expected")
        messages_ok = row.get("messages_received") == row.get("messages_expected")
        mux_ok = mode not in {"bbox_cached_depth_mux", "gazebo_fused"} or row.get("mux_status_valid") == "passed"
        if topics_ok and messages_ok:
            if mux_ok:
                return True, f"{row.get('run_dir', '')}"
            return False, f"{row.get('run_dir', '')}: mux_status_valid={row.get('mux_status_valid', 'missing')}"
    if candidates:
        return False, f"{len(candidates)} passed row(s), but topic/message counts incomplete"
    return False, "no passed runtime row"


def build_checks() -> list[Check]:
    checks: list[Check] = []

    missing_files = [path for path in REQUIRED_FILES if not _exists(path)]
    checks.append(
        Check(
            "ROS2 demo required files exist",
            not missing_files,
            "all present" if not missing_files else ", ".join(missing_files),
        )
    )

    missing_exec = [path for path in EXECUTABLE_SCRIPTS if not _is_executable(path)]
    checks.append(
        Check(
            "ROS2 helper scripts are executable",
            not missing_exec,
            "all executable" if not missing_exec else ", ".join(missing_exec),
        )
    )

    py_ok, py_evidence = _compile_python([path for path in PYTHON_FILES if _exists(path)])
    checks.append(Check("ROS2 Python files compile", py_ok, py_evidence))

    xml_ok, xml_evidence = _parse_xml(
        [
            "ros2_ws/src/uav_oda_ros2_demo/package.xml",
            "ros2_ws/src/uav_oda_ros2_demo/worlds/indoor_obstacles.sdf",
        ]
    )
    checks.append(Check("ROS2 XML/SDF files parse", xml_ok, xml_evidence))

    contract_ok, contract_evidence = _run_command([sys.executable, "scripts/check_perception_to_planner_contract.py"])
    checks.append(Check("Perception-to-planner contract passes", contract_ok, contract_evidence))

    matrix_ok, matrix_evidence = _run_command([sys.executable, "scripts/check_perception_planner_matrix.py"])
    checks.append(Check("Perception/planner matrix passes", matrix_ok, matrix_evidence))

    figure_ok, figure_evidence = _run_command([sys.executable, "scripts/render_perception_to_planner_contract_figure.py"])
    checks.append(Check("Perception-to-planner contract figure renders", figure_ok, figure_evidence))

    mode_ok, mode_evidence = _run_command([sys.executable, "scripts/check_ros2_mode_consistency.py"])
    checks.append(Check("ROS2 demo mode consistency", mode_ok, mode_evidence))

    launch_ok, launch_evidence = _run_command([sys.executable, "scripts/check_ros2_launch_contract.py"])
    checks.append(Check("ROS2 launch/package contract", launch_ok, launch_evidence))

    mux_validator_ok, mux_validator_evidence = _run_command([sys.executable, "scripts/check_costmap_mux_status_validator.py"])
    checks.append(Check("Costmap mux status validator self-test", mux_validator_ok, mux_validator_evidence))

    runbook = Path("docs/ros2_gazebo_costmap_demo.md")
    runbook_text = runbook.read_text(errors="replace") if runbook.exists() else ""
    checks.append(
        Check(
            "Runbook documents setup-to-verify flow",
            all(
                token in runbook_text
                for token in [
                    "scripts/setup_ros2_gazebo_server.sh",
                    "scripts/check_ros2_server_preflight.sh",
                    "scripts/run_headless_ros2_runtime_video.sh",
                    "scripts/verify_ros2_fused_perception_demo.sh astar",
                    "scripts/verify_ros2_costmap_all_modes.sh astar",
                    "scripts/verify_ros2_costmap_runtime.sh bbox astar",
                    "scripts/verify_ros2_costmap_runtime.sh synthetic astar",
                    "scripts/verify_ros2_costmap_runtime.sh depth_image astar",
                    "scripts/verify_ros2_costmap_runtime.sh cached_depth astar",
                    "scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar",
                    "scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar",
                    "scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar",
                    "scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar",
                ]
            ),
            "setup + preflight + focused fused verifier + all-mode verifier + bbox + synthetic + depth_image + cached_depth + bbox_cached_depth_mux + gazebo_depth + gazebo_laserscan + gazebo_fused commands",
        )
    )

    video_path = Path("outputs/videos/ros2_costmap_demo_astar.mp4")
    checks.append(
        Check(
            "Standalone MP4 demo exists",
            video_path.exists() and video_path.stat().st_size > 0,
            f"{video_path} ({video_path.stat().st_size} bytes)" if video_path.exists() else "missing; run render_ros2_costmap_demo_video.py",
        )
    )

    summary_csv = Path("outputs/tables/ros2_demo_runtime_summary.csv")
    rows = _read_csv(summary_csv)
    checks.append(
        Check(
            "Runtime evidence summary CSV exists",
            bool(rows),
            f"{len(rows)} row(s)" if rows else "missing or empty; run verifier on ROS2/Gazebo server",
        )
    )
    for mode in RUNTIME_REQUIRED_MODES:
        ok, evidence = _runtime_mode_passed(rows, mode)
        checks.append(Check(f"Runtime verifier passed for {mode}", ok, evidence))

    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fail-on-incomplete", action="store_true", help="Return exit code 1 when any check fails.")
    args = parser.parse_args()

    checks = build_checks()
    all_ok = True
    print("ROS2/Gazebo demo audit")
    print("======================")
    for check in checks:
        status = "PASS" if check.ok else "MISSING"
        print(f"{status:7} {check.label}: {check.evidence}")
        all_ok = all_ok and check.ok
    print()
    print("COMPLETE" if all_ok else "INCOMPLETE")
    return 1 if args.fail_on_incomplete and not all_ok else 0


if __name__ == "__main__":
    raise SystemExit(main())
