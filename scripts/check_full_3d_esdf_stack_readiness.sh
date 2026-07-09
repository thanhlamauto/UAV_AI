#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

echo "== Full 3D Voxel/ESDF Stack Readiness =="
echo "ROS_DISTRO=${ROS_DISTRO}"
echo

check_cmd() {
  local name="$1"
  local cmd="$2"
  if bash -lc "command -v ${cmd} >/dev/null 2>&1"; then
    echo "PASS command: ${name} (${cmd})"
  else
    echo "MISS command: ${name} (${cmd})"
  fi
}

check_apt() {
  local pkg="$1"
  if apt-cache policy "$pkg" >/tmp/uav_esdf_apt_policy.txt 2>/dev/null; then
    local candidate
    candidate="$(awk '/Candidate:/ {print $2}' /tmp/uav_esdf_apt_policy.txt | head -1)"
    if [[ -n "${candidate}" && "${candidate}" != "(none)" ]]; then
      echo "PASS apt candidate: ${pkg} -> ${candidate}"
    else
      echo "MISS apt candidate: ${pkg}"
    fi
  else
    echo "MISS apt query: ${pkg}"
  fi
}

check_ros_pkg() {
  local pkg="$1"
  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "PASS ros pkg: ${pkg}"
  else
    echo "MISS ros pkg: ${pkg}"
  fi
}

check_cmd "NVIDIA SMI" "nvidia-smi"
check_cmd "ROS2 CLI" "ros2"
check_cmd "Gazebo" "gz"
check_cmd "colcon" "colcon"

echo
echo "== GPU =="
if command -v nvidia-smi >/dev/null 2>&1; then
  nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader || true
else
  echo "nvidia-smi is unavailable"
fi

echo
echo "== ROS packages currently installed =="
check_ros_pkg "nvblox_ros"
check_ros_pkg "nvblox_msgs"
check_ros_pkg "isaac_ros_nvblox"
check_ros_pkg "ros_gz_bridge"
check_ros_pkg "rviz2"

echo
echo "== Apt package candidates =="
check_apt "ros-${ROS_DISTRO}-isaac-ros-nvblox"
check_apt "ros-${ROS_DISTRO}-nvblox-msgs"
check_apt "ros-${ROS_DISTRO}-ros-gz-bridge"
check_apt "ros-${ROS_DISTRO}-octomap"
check_apt "ros-${ROS_DISTRO}-octomap-msgs"

echo
echo "== Required next evidence =="
cat <<'EOF'
1. Launch NVBlox with depth image + camera_info + pose/tf.
2. Verify non-empty topics:
   - /nvblox_node/mesh or /nvblox_node/mesh_marker
   - /nvblox_node/tsdf_layer or /nvblox_node/occupancy_layer
   - /nvblox_node/static_esdf_pointcloud or /nvblox_node/static_map_slice
3. Record rosbag and video evidence.
4. Bridge DistanceMapSlice/ESDF into MPPI clearance cost.
EOF
