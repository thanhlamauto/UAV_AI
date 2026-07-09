#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DURATION_S="${DURATION_S:-24}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${REPO_ROOT}/outputs/nvblox_distance_slice_planner_runtime/$(date +%Y%m%d_%H%M%S)"

mkdir -p "${OUT_DIR}"

log() {
  printf '[nvblox-slice-verify] %s\n' "$*" | tee -a "${OUT_DIR}/verify.log"
}

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

cleanup() {
  if [[ -n "${LAUNCHER_PID:-}" ]]; then
    kill -INT -"${LAUNCHER_PID}" 2>/dev/null || true
    sleep 2
    kill -TERM -"${LAUNCHER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"

log "Building uav_oda_ros2_demo"
cd "${REPO_ROOT}/ros2_ws"
colcon build --packages-select uav_oda_ros2_demo 2>&1 | tee "${OUT_DIR}/colcon_build.log"
source_setup install/setup.bash

cd "${REPO_ROOT}"
log "Launching synthetic depth -> NVBlox DistanceMapSlice -> MPPI planner"
setsid ros2 launch uav_oda_ros2_demo nvblox_distance_slice_planner.launch.py \
  >"${OUT_DIR}/launch.log" 2>&1 &
LAUNCHER_PID=$!

sleep "${DURATION_S}"

ros2 topic list >"${OUT_DIR}/topics.txt"
ros2 topic info /nvblox_node/static_map_slice >"${OUT_DIR}/static_map_slice_info.txt" 2>&1 || true
ros2 topic info /planned_path_from_nvblox >"${OUT_DIR}/planned_path_info.txt" 2>&1 || true
ros2 topic info /nvblox_distance_slice_planner/status >"${OUT_DIR}/status_info.txt" 2>&1 || true

timeout 8 ros2 topic echo --once /nvblox_node/static_map_slice >"${OUT_DIR}/static_map_slice_echo.txt" 2>&1 || true
timeout 8 ros2 topic echo --once /planned_path_from_nvblox >"${OUT_DIR}/planned_path_echo.txt" 2>&1 || true
timeout 8 ros2 topic echo --once /nvblox_distance_slice_planner/status >"${OUT_DIR}/status_echo.txt" 2>&1 || true

cleanup
unset LAUNCHER_PID

cp "${OUT_DIR}/topics.txt" "${REPO_ROOT}/outputs/nvblox_distance_slice_topics.txt"
cp "${OUT_DIR}/static_map_slice_echo.txt" "${REPO_ROOT}/outputs/nvblox_distance_slice_static_map_slice_echo.txt"
cp "${OUT_DIR}/planned_path_echo.txt" "${REPO_ROOT}/outputs/nvblox_distance_slice_planned_path_echo.txt"
cp "${OUT_DIR}/status_echo.txt" "${REPO_ROOT}/outputs/nvblox_distance_slice_status_echo.txt"
cp "${OUT_DIR}/launch.log" "${REPO_ROOT}/outputs/nvblox_distance_slice_planner_ros2.log"

log "Collected evidence under ${OUT_DIR}"
wc -c \
  "${OUT_DIR}/static_map_slice_echo.txt" \
  "${OUT_DIR}/planned_path_echo.txt" \
  "${OUT_DIR}/status_echo.txt" | tee -a "${OUT_DIR}/verify.log"

grep -q "/nvblox_node/static_map_slice" "${OUT_DIR}/topics.txt"
grep -q "/planned_path_from_nvblox" "${OUT_DIR}/topics.txt"
grep -q "/nvblox_distance_slice_planner/status" "${OUT_DIR}/topics.txt"
grep -q '"state":"planned"' "${OUT_DIR}/status_echo.txt"
grep -q "poses:" "${OUT_DIR}/planned_path_echo.txt"

if [[ ! -s "${OUT_DIR}/static_map_slice_echo.txt" ]]; then
  log "DistanceMapSlice echo is empty"
  tail -80 "${OUT_DIR}/launch.log" >&2
  exit 1
fi

log "NVBlox DistanceMapSlice planner verification PASSED"
