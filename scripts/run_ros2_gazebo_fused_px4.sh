#!/usr/bin/env bash
set -euo pipefail

PLANNER="${1:-astar}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
START_PX4="${START_PX4:-0}"
START_GAZEBO_WORLD="${START_GAZEBO_WORLD:-1}"
PX4_CONTROLLER="${PX4_CONTROLLER:-mppi}"
COLCON_SYMLINK_INSTALL="${COLCON_SYMLINK_INSTALL:-0}"

case "${PX4_CONTROLLER}" in
  mppi)
    ENABLE_PX4_WAYPOINT=false
    ENABLE_PX4_MPPI=true
    ;;
  waypoint)
    ENABLE_PX4_WAYPOINT=true
    ENABLE_PX4_MPPI=false
    ;;
  none)
    ENABLE_PX4_WAYPOINT=false
    ENABLE_PX4_MPPI=false
    ;;
  *)
    echo "Unknown PX4_CONTROLLER=${PX4_CONTROLLER}; use mppi, waypoint, or none." >&2
    exit 2
    ;;
esac

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"

cd "${REPO_ROOT}/ros2_ws"
build_args=(build --packages-select uav_oda_ros2_demo)
if [[ "${COLCON_SYMLINK_INSTALL}" == "1" ]]; then
  build_args+=(--symlink-install)
fi
colcon "${build_args[@]}"
source_setup install/setup.bash

cd "${REPO_ROOT}"
python3 scripts/check_ros2_costmap_demo_static.py

ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
  planner:="${PLANNER}" \
  pointcloud_costmap_topic:=perception/pointcloud_occupancy_grid \
  depth_costmap_topic:=perception/depth_occupancy_grid \
  laserscan_costmap_topic:=perception/laserscan_occupancy_grid \
  costmap_mux_input_topics_csv:=perception/pointcloud_occupancy_grid,perception/depth_occupancy_grid,perception/laserscan_occupancy_grid \
  use_bbox_costmap:=false \
  use_synthetic_cloud:=true \
  use_pointcloud_costmap:=true \
  use_synthetic_depth:=false \
  use_cached_depth:=false \
  use_depth_image:=true \
  use_gazebo_depth_image:=true \
  use_gazebo_laserscan:=true \
  use_costmap_mux:=true \
  start_gazebo_world:="${START_GAZEBO_WORLD}" \
  start_px4:="${START_PX4}" \
  enable_kinematic_follower:=false \
  publish_static_start:=false \
  enable_px4_odometry_bridge:=true \
  enable_px4_bridge:="${ENABLE_PX4_WAYPOINT}" \
  enable_px4_mppi_controller:="${ENABLE_PX4_MPPI}"
