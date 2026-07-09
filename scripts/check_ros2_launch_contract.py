#!/usr/bin/env python3
"""Static contract check for the ROS2/Gazebo perception-to-planner launch.

This intentionally avoids ROS imports.  It catches launch/package/config drift
that plain Python compilation cannot detect.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


LAUNCH = Path("ros2_ws/src/uav_oda_ros2_demo/launch/px4_gazebo_costmap_demo.launch.py")
SETUP = Path("ros2_ws/src/uav_oda_ros2_demo/setup.py")
PACKAGE = Path("ros2_ws/src/uav_oda_ros2_demo/package.xml")
CONFIG = Path("ros2_ws/src/uav_oda_ros2_demo/config/demo_params.yaml")
RUNNER = Path("scripts/run_ros2_costmap_demo.sh")
PX4_RUNNER = Path("scripts/run_ros2_gazebo_fused_px4.sh")
VERIFIER = Path("scripts/verify_ros2_costmap_runtime.sh")
ALL_MODES = Path("scripts/verify_ros2_costmap_all_modes.sh")

REQUIRED_EXECUTABLES = [
    "bbox_costmap_publisher",
    "cached_depth_image_publisher",
    "costmap_mux",
    "costmap_planner",
    "depth_image_costmap",
    "kinematic_path_follower",
    "laserscan_costmap",
    "pointcloud_costmap",
    "px4_odometry_bridge",
    "px4_mppi_offboard_controller",
    "static_pose_publisher",
    "synthetic_depth_image_publisher",
    "synthetic_pointcloud_publisher",
]

REQUIRED_LAUNCH_ARGUMENTS = [
    "bbox_costmap_topic",
    "costmap_mux_input_topics_csv",
    "depth_costmap_topic",
    "laserscan_costmap_topic",
    "pointcloud_costmap_topic",
    "use_bbox_costmap",
    "use_cached_depth",
    "use_depth_image",
    "use_costmap_mux",
    "use_gazebo_depth_image",
    "use_gazebo_laserscan",
    "use_pointcloud_costmap",
    "use_synthetic_cloud",
    "use_synthetic_depth",
    "enable_px4_odometry_bridge",
    "enable_px4_mppi_controller",
]

REQUIRED_PACKAGE_DEPS = [
    "geometry_msgs",
    "launch",
    "launch_ros",
    "nav_msgs",
    "python3-numpy",
    "rclpy",
    "ros_gz_bridge",
    "sensor_msgs",
    "std_msgs",
    "tf2_ros",
    "visualization_msgs",
]


def _read(path: Path) -> str:
    if not path.exists():
        raise FileNotFoundError(path)
    return path.read_text(errors="replace")


def _require(text: str, tokens: list[str], label: str, failures: list[str]) -> None:
    missing = [token for token in tokens if token not in text]
    if missing:
        failures.append(f"{label}: missing {', '.join(missing)}")


def _check_setup(failures: list[str]) -> None:
    text = _read(SETUP)
    for executable in REQUIRED_EXECUTABLES:
        _require(text, [f'"{executable} = uav_oda_ros2_demo.'], f"{SETUP} entry point {executable}", failures)


def _check_package_xml(failures: list[str]) -> None:
    root = ET.parse(PACKAGE).getroot()
    deps = {node.text.strip() for node in root.findall("exec_depend") if node.text}
    missing = [dep for dep in REQUIRED_PACKAGE_DEPS if dep not in deps]
    if missing:
        failures.append(f"{PACKAGE}: missing exec_depend {', '.join(missing)}")


def _check_launch(failures: list[str]) -> None:
    text = _read(LAUNCH)
    for argument in REQUIRED_LAUNCH_ARGUMENTS:
        _require(text, [f'"{argument}"', f'LaunchConfiguration("{argument}")'], f"{LAUNCH} launch argument {argument}", failures)
    for executable in REQUIRED_EXECUTABLES:
        _require(text, [f'executable="{executable}"'], f"{LAUNCH} node {executable}", failures)

    _require(
        text,
        [
            '"/uav_oda/lidar_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"',
            '"/camera/depth/image@sensor_msgs/msg/Image@gz.msgs.Image"',
            '"costmap_topic": bbox_costmap_topic',
            '{"costmap_topic": depth_costmap_topic}',
            "condition=IfCondition(use_costmap_mux)",
            'executable="costmap_mux"',
        ],
        f"{LAUNCH} perception bridge wiring",
        failures,
    )


def _check_config(failures: list[str]) -> None:
    text = _read(CONFIG)
    _require(
        text,
        [
            "costmap_mux:",
            "input_topics_csv",
            "perception/bbox_occupancy_grid",
            "perception/depth_occupancy_grid",
            "output_topic: perception/occupancy_grid",
            "status_topic: perception/costmap_mux_status",
            "require_all_inputs: true",
            "costmap_planner:",
            "costmap_topic: perception/occupancy_grid",
            "px4_mppi_offboard_controller:",
            "predicted_path_topic: mppi/predicted_path",
            "status_topic: mppi/status",
            "max_accel_mps2",
            "depth_image_costmap:",
            "marker_topic: perception/depth_obstacle_markers",
            "laserscan_costmap:",
            "scan_topic: /uav_oda/lidar_scan",
        ],
        f"{CONFIG} planner-facing topics",
        failures,
    )


def _check_mode_scripts(failures: list[str]) -> None:
    for path in [RUNNER, VERIFIER]:
        text = _read(path)
        _require(
            text,
            [
                "bbox_cached_depth_mux)",
                "gazebo_fused)",
                "bbox_costmap_topic:=perception/bbox_occupancy_grid",
                "depth_costmap_topic:=perception/depth_occupancy_grid",
                "pointcloud_costmap_topic:=perception/pointcloud_occupancy_grid",
                "laserscan_costmap_topic:=perception/laserscan_occupancy_grid",
                "use_bbox_costmap:=true",
                "use_cached_depth:=true",
                "use_depth_image:=true",
                "use_costmap_mux:=true",
            ],
            f"{path} fused bbox-depth mode",
            failures,
        )

    _require(
        _read(VERIFIER),
        [
            '"/perception/bbox_occupancy_grid"',
            '"/perception/depth_occupancy_grid"',
            '"/perception/pointcloud_occupancy_grid"',
            '"/perception/laserscan_occupancy_grid"',
            '"/perception/costmap_mux_status"',
            '"/perception/occupancy_grid"',
            '"/planned_path"',
        ],
        f"{VERIFIER} fused mode evidence topics",
        failures,
    )
    _require(
        _read(ALL_MODES),
        [
            'MODE_LIST="${ROS2_DEMO_MODES:-bbox synthetic depth_image cached_depth bbox_cached_depth_mux gazebo_depth gazebo_laserscan gazebo_fused}"',
            'if [[ "${mode}" == "bbox" || "${mode}" == "bbox_cached_depth_mux" ]]',
            '"${mode}" "${PLANNER}" "${BBOX_CSV}"',
        ],
        f"{ALL_MODES} all-mode bbox CSV forwarding",
        failures,
    )
    _require(
        _read(PX4_RUNNER),
        [
            'PX4_CONTROLLER="${PX4_CONTROLLER:-mppi}"',
            "ENABLE_PX4_MPPI=true",
            "enable_px4_mppi_controller:=\"${ENABLE_PX4_MPPI}\"",
            "enable_px4_bridge:=\"${ENABLE_PX4_WAYPOINT}\"",
        ],
        f"{PX4_RUNNER} MPPI Offboard controller mode",
        failures,
    )


def main() -> int:
    failures: list[str] = []
    _check_setup(failures)
    _check_package_xml(failures)
    _check_launch(failures)
    _check_config(failures)
    _check_mode_scripts(failures)

    if failures:
        print("ROS2 launch contract check FAILED")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("ROS2 launch contract check PASSED")
    print("Verified setup entry points, package deps, launch args, mux topics, and fused mode runners.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
