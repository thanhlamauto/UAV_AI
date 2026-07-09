#!/usr/bin/env python3
"""Check that ROS2/Gazebo demo modes are documented and verified consistently."""

from __future__ import annotations

from pathlib import Path


EXPECTED_MODES = [
    "bbox",
    "synthetic",
    "depth_image",
    "cached_depth",
    "bbox_cached_depth_mux",
    "gazebo_depth",
    "gazebo_laserscan",
    "gazebo_fused",
]

SCRIPT_FILES = [
    Path("scripts/run_ros2_costmap_demo.sh"),
    Path("scripts/verify_ros2_costmap_runtime.sh"),
    Path("scripts/verify_ros2_costmap_all_modes.sh"),
    Path("scripts/check_ros2_server_preflight.sh"),
    Path("scripts/summarize_ros2_runtime_evidence.py"),
    Path("scripts/diagnose_ros2_runtime_failures.py"),
    Path("scripts/write_ros2_demo_report_section.py"),
    Path("scripts/audit_ros2_demo_status.py"),
]


def _read(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(path)
    return path.read_text(errors="replace")


def _require_tokens(path: Path, tokens: list[str]) -> list[str]:
    text = _read(path)
    return [token for token in tokens if token not in text]


def main() -> int:
    failures: list[str] = []

    mode_list = " ".join(EXPECTED_MODES)
    for path in [Path("scripts/verify_ros2_costmap_all_modes.sh"), Path("scripts/check_ros2_server_preflight.sh")]:
        missing = _require_tokens(path, [mode_list])
        if missing:
            failures.append(f"{path}: default mode list missing `{mode_list}`")

    missing = _require_tokens(Path("scripts/verify_ros2_fused_perception_demo.sh"), ["bbox_cached_depth_mux"])
    if missing:
        failures.append("scripts/verify_ros2_fused_perception_demo.sh: missing focused mode token bbox_cached_depth_mux")
    missing = _require_tokens(
        Path("scripts/run_headless_ros2_runtime_video.sh"),
        ["bbox_cached_depth_mux", "mux_status_valid", "ros2_fused_perception_runtime"],
    )
    if missing:
        failures.append(f"scripts/run_headless_ros2_runtime_video.sh: missing headless video token(s): {', '.join(missing)}")
    missing = _require_tokens(
        Path("scripts/verify_ros2_costmap_runtime.sh"),
        ["scripts/validate_costmap_mux_status_sample.py", "costmap_mux_status_validation.log", "MUX_STATUS_TIMEOUT_S"],
    )
    if missing:
        failures.append(f"scripts/verify_ros2_costmap_runtime.sh: missing mux status validation token(s): {', '.join(missing)}")
    for path in [
        Path("scripts/summarize_ros2_runtime_evidence.py"),
        Path("scripts/audit_ros2_demo_status.py"),
        Path("scripts/write_ros2_demo_report_section.py"),
    ]:
        missing = _require_tokens(path, ["mux_status_valid", "bbox_cached_depth_mux"])
        if missing:
            failures.append(f"{path}: missing mux validation summary token(s): {', '.join(missing)}")
    for path in [
        Path("README.md"),
        Path("docs/ros2_gazebo_costmap_demo.md"),
        Path("outputs/ros2_gazebo_costmap_demo_summary.md"),
        Path("outputs/perception_to_planner_integration_status.md"),
    ]:
        missing = _require_tokens(path, ["check_perception_planner_matrix.py", "perception_planner_matrix.csv"])
        if missing:
            failures.append(f"{path}: missing planner matrix token(s): {', '.join(missing)}")
    for path in [
        Path("scripts/verify_ros2_fused_perception_demo.sh"),
        Path("scripts/verify_ros2_costmap_all_modes.sh"),
        Path("scripts/audit_ros2_demo_status.py"),
        Path("scripts/audit_ros2_fused_demo_status.py"),
        Path("scripts/bundle_ros2_demo_artifacts.py"),
    ]:
        missing = _require_tokens(path, ["check_perception_planner_matrix.py"])
        if missing:
            failures.append(f"{path}: missing planner matrix runner token(s): {', '.join(missing)}")

    for path in SCRIPT_FILES:
        missing = _require_tokens(path, EXPECTED_MODES)
        if missing:
            failures.append(f"{path}: missing mode token(s): {', '.join(missing)}")

    run_commands = [f"scripts/run_ros2_costmap_demo.sh {mode} astar" for mode in EXPECTED_MODES]
    verify_commands = [f"scripts/verify_ros2_costmap_runtime.sh {mode} astar" for mode in EXPECTED_MODES]
    text_requirements = {
        Path("README.md"): run_commands + verify_commands,
        Path("docs/ros2_gazebo_costmap_demo.md"): run_commands + verify_commands,
        Path("outputs/perception_to_planner_integration_status.md"): run_commands + verify_commands,
        Path("outputs/ros2_demo_runtime_summary.md"): verify_commands,
        Path("outputs/ros2_demo_report_section.md"): verify_commands,
        Path("outputs/ros2_gazebo_costmap_demo_summary.md"): verify_commands,
        Path("outputs/ros2_gazebo_runtime_handoff.md"): verify_commands,
    }
    for path, commands in text_requirements.items():
        missing = _require_tokens(path, commands)
        if missing:
            failures.append(f"{path}: missing command(s): {', '.join(missing[:4])}")

    mux_topics = [
        "/perception/bbox_occupancy_grid",
        "/perception/depth_occupancy_grid",
        "/perception/pointcloud_occupancy_grid",
        "/perception/laserscan_occupancy_grid",
        "/perception/costmap_mux_status",
        "/perception/occupancy_grid",
    ]
    for path in [
        Path("scripts/verify_ros2_costmap_runtime.sh"),
        Path("scripts/summarize_ros2_runtime_evidence.py"),
        Path("scripts/diagnose_ros2_runtime_failures.py"),
    ]:
        missing = _require_tokens(path, mux_topics)
        if missing:
            failures.append(f"{path}: missing fused mux topic(s): {', '.join(missing)}")

    contract_path = Path("outputs/tables/perception_to_planner_contract.csv")
    missing = _require_tokens(contract_path, ["lidar_bbox_plus_relative_depth_mux", "lidar_bbox_plus_cached_depth_mux"])
    if missing:
        failures.append(f"{contract_path}: fused mux contract row missing: {', '.join(missing)}")

    if failures:
        print("ROS2 mode consistency check FAILED")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("ROS2 mode consistency check PASSED")
    print("Modes:", ", ".join(EXPECTED_MODES))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
