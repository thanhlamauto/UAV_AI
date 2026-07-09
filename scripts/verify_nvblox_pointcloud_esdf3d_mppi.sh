#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
DURATION_S="${DURATION_S:-34}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${REPO_ROOT}/outputs/nvblox_pointcloud_esdf3d_mppi_runtime/$(date +%Y%m%d_%H%M%S)"

mkdir -p "${OUT_DIR}"

log() {
  printf '[nvblox-esdf3d-verify] %s\n' "$*" | tee -a "${OUT_DIR}/verify.log"
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
log "Launching PointCloud2 -> planner-side 3D ESDF/MPPI with NVBlox 3D TSDF runtime"
setsid ros2 launch uav_oda_ros2_demo nvblox_pointcloud_esdf3d_mppi.launch.py \
  >"${OUT_DIR}/launch.log" 2>&1 &
LAUNCHER_PID=$!

sleep "${DURATION_S}"

ros2 topic list >"${OUT_DIR}/topics.txt"
ros2 topic info /lidar/points >"${OUT_DIR}/lidar_points_info.txt" 2>&1 || true
ros2 topic info -v /nvblox_node/tsdf_layer >"${OUT_DIR}/tsdf_layer_info.txt" 2>&1 || true
ros2 topic info /planned_path_3d_from_nvblox >"${OUT_DIR}/planned_path_3d_info.txt" 2>&1 || true
ros2 topic info /nvblox_esdf3d_mppi/status >"${OUT_DIR}/status_info.txt" 2>&1 || true

timeout 8 ros2 topic echo --once /lidar/points >"${OUT_DIR}/lidar_points_echo.txt" 2>&1 || true
timeout 8 ros2 topic echo --once /planned_path_3d_from_nvblox >"${OUT_DIR}/planned_path_3d_echo.txt" 2>&1 || true
timeout 8 ros2 topic echo --once --full-length /nvblox_esdf3d_mppi/status >"${OUT_DIR}/status_echo.txt" 2>&1 || true

cleanup
unset LAUNCHER_PID

cp "${OUT_DIR}/topics.txt" "${REPO_ROOT}/outputs/nvblox_esdf3d_topics.txt"
cp "${OUT_DIR}/lidar_points_echo.txt" "${REPO_ROOT}/outputs/nvblox_esdf3d_lidar_points_echo.txt"
cp "${OUT_DIR}/tsdf_layer_info.txt" "${REPO_ROOT}/outputs/nvblox_esdf3d_tsdf_layer_info.txt"
cp "${OUT_DIR}/planned_path_3d_echo.txt" "${REPO_ROOT}/outputs/nvblox_esdf3d_planned_path_echo.txt"
cp "${OUT_DIR}/status_echo.txt" "${REPO_ROOT}/outputs/nvblox_esdf3d_status_echo.txt"
cp "${OUT_DIR}/launch.log" "${REPO_ROOT}/outputs/nvblox_esdf3d_mppi_ros2.log"

log "Collected evidence under ${OUT_DIR}"
wc -c \
  "${OUT_DIR}/lidar_points_echo.txt" \
  "${OUT_DIR}/planned_path_3d_echo.txt" \
  "${OUT_DIR}/status_echo.txt" | tee -a "${OUT_DIR}/verify.log"

grep -q "/lidar/points" "${OUT_DIR}/topics.txt"
grep -q "/nvblox_node/tsdf_layer" "${OUT_DIR}/topics.txt"
grep -q "/planned_path_3d_from_nvblox" "${OUT_DIR}/topics.txt"
grep -q "/nvblox_esdf3d_mppi/status" "${OUT_DIR}/topics.txt"
grep -q '"state":"planned"' "${OUT_DIR}/status_echo.txt"
grep -q '"safety_violation":false' "${OUT_DIR}/status_echo.txt"
grep -q "Topic type: nvblox_msgs/msg/VoxelBlockLayer" "${OUT_DIR}/tsdf_layer_info.txt"
grep -q "ros/update_esdf" "${OUT_DIR}/launch.log"
grep -q "poses:" "${OUT_DIR}/planned_path_3d_echo.txt"

python3 - "${OUT_DIR}/status_echo.txt" <<'PY'
import json
import re
import sys
from pathlib import Path

text = Path(sys.argv[1]).read_text()
match = re.search(r"data: '([^']+)'", text)
if not match:
    raise SystemExit("status JSON not found")
payload = json.loads(match.group(1))
shape = payload.get("grid_shape") or []
if len(shape) != 3 or int(shape[2]) < 2:
    raise SystemExit(f"expected 3D ESDF grid, got grid_shape={shape}")
if float(payload.get("esdf_z_span_m", 0.0)) < 0.1:
    raise SystemExit(f"expected ESDF z span >= 0.1 m, got {payload.get('esdf_z_span_m')}")
if payload.get("safety_violation") is not False:
    raise SystemExit(f"expected no safety violation, got {payload}")
if payload.get("source_mode") != "pointcloud_occupancy_esdf":
    raise SystemExit(f"expected PointCloud2-derived local ESDF source, got {payload.get('source_mode')}")
print(
    "ESDF3D status validation PASSED: "
    f"shape={shape}, z_span={payload.get('esdf_z_span_m')}, "
    f"min_esdf={payload.get('min_esdf_distance_m')}, compute={payload.get('compute_time_ms')}"
)
PY

for required in lidar_points_echo.txt tsdf_layer_info.txt planned_path_3d_echo.txt status_echo.txt; do
  if [[ ! -s "${OUT_DIR}/${required}" ]]; then
    log "${required} is empty"
    tail -160 "${OUT_DIR}/launch.log" >&2
    exit 1
  fi
done

log "PointCloud2 LiDAR -> local 3D ESDF/MPPI with NVBlox 3D TSDF runtime verification PASSED"
