#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
MODE_LIST="${ROS2_DEMO_MODES:-bbox synthetic depth_image cached_depth bbox_cached_depth_mux gazebo_depth gazebo_laserscan gazebo_fused}"
BBOX_CSV="${BBOX_CSV:-${REPO_ROOT}/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv}"
CACHED_DEPTH_NPZ="${CACHED_DEPTH_NPZ:-${REPO_ROOT}/data/processed/depth_sample_3_5fps.npz}"
PX4_REQUIRED="${PX4_REQUIRED:-0}"
OUT_ROOT="${REPO_ROOT}/outputs/ros2_demo_runtime"
LOG_PATH="${OUT_ROOT}/preflight_$(date +%Y%m%d_%H%M%S).log"

mkdir -p "${OUT_ROOT}"

ok=1

log() {
  printf '[ros2-preflight] %s\n' "$*" | tee -a "${LOG_PATH}"
}

fail() {
  ok=0
  log "MISSING: $*"
}

pass() {
  log "PASS: $*"
}

warn() {
  log "WARN: $*"
}

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

need_cmd() {
  if command -v "$1" >/dev/null 2>&1; then
    pass "command $1 -> $(command -v "$1")"
  else
    fail "command $1"
  fi
}

need_file() {
  if [[ -e "$1" ]]; then
    pass "file $1"
  else
    fail "file $1"
  fi
}

need_ros_pkg() {
  if ros2 pkg prefix "$1" >/dev/null 2>&1; then
    pass "ROS package $1"
  else
    fail "ROS package $1"
  fi
}

mode_enabled() {
  local needle="$1"
  for mode in ${MODE_LIST}; do
    if [[ "${mode}" == "${needle}" ]]; then
      return 0
    fi
  done
  return 1
}

log "Repo root: ${REPO_ROOT}"
log "ROS distro: ${ROS_DISTRO}"
log "Modes: ${MODE_LIST}"
log "Log path: ${LOG_PATH}"

need_file "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

need_cmd python3
need_cmd colcon
need_cmd ros2
need_cmd timeout
need_cmd setsid

if command -v ffmpeg >/dev/null 2>&1; then
  pass "command ffmpeg -> $(command -v ffmpeg)"
else
  warn "ffmpeg missing; MP4 rendering will fail unless RENDER_VIDEO=0"
fi

need_file "${REPO_ROOT}/ros2_ws/src/uav_oda_ros2_demo/package.xml"
need_file "${REPO_ROOT}/ros2_ws/src/uav_oda_ros2_demo/launch/px4_gazebo_costmap_demo.launch.py"
need_file "${REPO_ROOT}/ros2_ws/src/uav_oda_ros2_demo/worlds/indoor_obstacles.sdf"
need_file "${BBOX_CSV}"
if mode_enabled cached_depth || mode_enabled bbox_cached_depth_mux; then
  need_file "${CACHED_DEPTH_NPZ}"
fi

if command -v python3 >/dev/null 2>&1; then
  if python3 - <<'PY' >/dev/null 2>&1
import numpy
PY
  then
    pass "python module numpy"
  else
    fail "python module numpy"
  fi
fi

if command -v ros2 >/dev/null 2>&1; then
  for pkg in rclpy nav_msgs sensor_msgs std_msgs geometry_msgs visualization_msgs tf2_ros launch launch_ros; do
    need_ros_pkg "${pkg}"
  done

  if mode_enabled gazebo_depth || mode_enabled gazebo_laserscan || mode_enabled gazebo_fused; then
    need_ros_pkg ros_gz_bridge
    need_cmd gz
  fi

  if [[ "${PX4_REQUIRED}" == "1" ]]; then
    need_ros_pkg px4_msgs
  else
    if ros2 pkg prefix px4_msgs >/dev/null 2>&1; then
      pass "optional ROS package px4_msgs"
    else
      warn "optional ROS package px4_msgs missing; PX4 bridge must remain disabled"
    fi
  fi
fi

if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source_setup "${REPO_ROOT}/ros2_ws/install/setup.bash"
  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix uav_oda_ros2_demo >/dev/null 2>&1; then
    pass "built ROS package uav_oda_ros2_demo"
  else
    warn "ros2_ws/install exists but uav_oda_ros2_demo is not visible; rebuild with colcon"
  fi
else
  warn "ros2_ws/install/setup.bash not present yet; all-mode verifier will build before runtime"
fi

if [[ "${ok}" -ne 1 ]]; then
  log "Preflight FAILED"
  exit 1
fi

log "Preflight PASSED"
