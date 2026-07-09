#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-synthetic}"
PLANNER="${2:-astar}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
COLCON_SYMLINK_INSTALL="${COLCON_SYMLINK_INSTALL:-0}"

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

case "${MODE}" in
  synthetic)
    exec ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
      planner:="${PLANNER}" \
      use_synthetic_cloud:=true \
      use_pointcloud_costmap:=true \
      use_synthetic_depth:=false \
      use_cached_depth:=false \
      use_depth_image:=false \
      use_gazebo_depth_image:=false \
      use_gazebo_laserscan:=false \
      use_costmap_mux:=false \
      start_gazebo_world:=false \
      start_px4:=false \
      enable_px4_bridge:=false
    ;;
  depth_image)
    exec ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
      planner:="${PLANNER}" \
      use_synthetic_cloud:=false \
      use_pointcloud_costmap:=false \
      use_synthetic_depth:=true \
      use_cached_depth:=false \
      use_depth_image:=true \
      use_gazebo_depth_image:=false \
      use_gazebo_laserscan:=false \
      use_costmap_mux:=false \
      start_gazebo_world:=false \
      start_px4:=false \
      enable_px4_bridge:=false
    ;;
  cached_depth)
    exec ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
      planner:="${PLANNER}" \
      use_synthetic_cloud:=false \
      use_pointcloud_costmap:=false \
      use_synthetic_depth:=false \
      use_cached_depth:=true \
      use_depth_image:=true \
      use_gazebo_depth_image:=false \
      use_gazebo_laserscan:=false \
      use_costmap_mux:=false \
      start_gazebo_world:=false \
      start_px4:=false \
      enable_px4_bridge:=false
    ;;
  bbox_cached_depth_mux)
    BBOX_CSV="${3:-${REPO_ROOT}/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv}"
    exec ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
      planner:="${PLANNER}" \
      bbox_csv:="${BBOX_CSV}" \
      bbox_costmap_topic:=perception/bbox_occupancy_grid \
      depth_costmap_topic:=perception/depth_occupancy_grid \
      use_bbox_costmap:=true \
      use_synthetic_cloud:=false \
      use_pointcloud_costmap:=false \
      use_synthetic_depth:=false \
      use_cached_depth:=true \
      use_depth_image:=true \
      use_gazebo_depth_image:=false \
      use_gazebo_laserscan:=false \
      use_costmap_mux:=true \
      start_gazebo_world:=false \
      start_px4:=false \
      enable_px4_bridge:=false \
      start_x:=0.0 \
      start_y:=0.0 \
      goal_x:=24.0 \
      goal_y:=4.0
    ;;
  gazebo_depth)
    exec ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
      planner:="${PLANNER}" \
      use_synthetic_cloud:=false \
      use_pointcloud_costmap:=false \
      use_synthetic_depth:=false \
      use_cached_depth:=false \
      use_depth_image:=true \
      use_gazebo_depth_image:=true \
      use_gazebo_laserscan:=false \
      use_costmap_mux:=false \
      start_gazebo_world:=true \
      start_px4:=false \
      enable_px4_bridge:=false
    ;;
  gazebo_laserscan)
    exec ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
      planner:="${PLANNER}" \
      use_synthetic_cloud:=false \
      use_pointcloud_costmap:=false \
      use_synthetic_depth:=false \
      use_cached_depth:=false \
      use_depth_image:=false \
      use_gazebo_depth_image:=false \
      use_gazebo_laserscan:=true \
      use_costmap_mux:=false \
      start_gazebo_world:=true \
      start_px4:=false \
      enable_px4_bridge:=false
    ;;
  gazebo_fused)
    exec ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
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
      start_gazebo_world:=true \
      start_px4:=false \
      enable_px4_bridge:=false
    ;;
  bbox)
    BBOX_CSV="${3:-${REPO_ROOT}/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv}"
    exec ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
      planner:="${PLANNER}" \
      bbox_csv:="${BBOX_CSV}" \
      use_bbox_costmap:=true \
      use_synthetic_cloud:=false \
      use_pointcloud_costmap:=false \
      use_synthetic_depth:=false \
      use_cached_depth:=false \
      use_depth_image:=false \
      use_gazebo_depth_image:=false \
      use_gazebo_laserscan:=false \
      use_costmap_mux:=false \
      start_gazebo_world:=false \
      start_px4:=false \
      enable_px4_bridge:=false \
      start_x:=7.5 \
      start_y:=0.0 \
      goal_x:=24.0 \
      goal_y:=4.0
    ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    echo "Usage: $0 [synthetic|depth_image|cached_depth|bbox_cached_depth_mux|gazebo_depth|gazebo_laserscan|gazebo_fused|bbox] [astar|rrt|mppi] [bbox_csv]" >&2
    exit 2
    ;;
esac
