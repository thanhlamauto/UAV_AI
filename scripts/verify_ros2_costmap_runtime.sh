#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-synthetic}"
PLANNER="${2:-astar}"
DURATION_S="${DURATION_S:-25}"
RECORD_BAG="${RECORD_BAG:-1}"
BAG_DURATION_S="${BAG_DURATION_S:-6}"
RENDER_VIDEO="${RENDER_VIDEO:-1}"
SKIP_BUILD="${SKIP_BUILD:-0}"
COLCON_SYMLINK_INSTALL="${COLCON_SYMLINK_INSTALL:-0}"
MUX_STATUS_TIMEOUT_S="${MUX_STATUS_TIMEOUT_S:-20}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
OUT_DIR="${REPO_ROOT}/outputs/ros2_demo_runtime/${MODE}_${PLANNER}_$(date +%Y%m%d_%H%M%S)"

mkdir -p "${OUT_DIR}"

log() {
  printf '[ros2-demo-verify] %s\n' "$*" | tee -a "${OUT_DIR}/verify.log"
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    log "Missing command: $1"
    exit 127
  fi
}

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"
require_cmd colcon
require_cmd ros2

cd "${REPO_ROOT}/ros2_ws"
if [[ "${SKIP_BUILD}" == "1" ]]; then
  if [[ ! -f install/setup.bash ]]; then
    log "SKIP_BUILD=1 but ros2_ws/install/setup.bash does not exist"
    exit 1
  fi
  log "Skipping colcon build because SKIP_BUILD=1"
  printf 'Skipped because SKIP_BUILD=1\n' >"${OUT_DIR}/colcon_build.log"
else
  log "Building uav_oda_ros2_demo"
  build_args=(build --packages-select uav_oda_ros2_demo)
  if [[ "${COLCON_SYMLINK_INSTALL}" == "1" ]]; then
    build_args+=(--symlink-install)
  fi
  colcon "${build_args[@]}" 2>&1 | tee "${OUT_DIR}/colcon_build.log"
fi
source_setup install/setup.bash

cd "${REPO_ROOT}"
log "Running non-ROS planner smoke test"
python3 scripts/check_ros2_costmap_demo_static.py 2>&1 | tee "${OUT_DIR}/static_planner_check.log"

launch_cmd=()
required_topics=(
  "/perception/occupancy_grid"
  "/planned_path"
  "/uav/current_pose"
  "/odom"
  "/uav/marker"
)

case "${MODE}" in
  synthetic)
    launch_cmd=(
      ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py
      planner:="${PLANNER}"
      use_synthetic_cloud:=true
      use_pointcloud_costmap:=true
      use_synthetic_depth:=false
      use_cached_depth:=false
      use_depth_image:=false
      use_gazebo_depth_image:=false
      use_gazebo_laserscan:=false
      use_costmap_mux:=false
      start_gazebo_world:=false
      start_px4:=false
      enable_px4_bridge:=false
    )
    required_topics+=("/lidar/points")
    ;;
  depth_image)
    launch_cmd=(
      ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py
      planner:="${PLANNER}"
      use_synthetic_cloud:=false
      use_pointcloud_costmap:=false
      use_synthetic_depth:=true
      use_cached_depth:=false
      use_depth_image:=true
      use_gazebo_depth_image:=false
      use_gazebo_laserscan:=false
      use_costmap_mux:=false
      start_gazebo_world:=false
      start_px4:=false
      enable_px4_bridge:=false
    )
    required_topics+=("/camera/depth/image" "/perception/depth_obstacle_markers")
    ;;
  cached_depth)
    launch_cmd=(
      ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py
      planner:="${PLANNER}"
      use_synthetic_cloud:=false
      use_pointcloud_costmap:=false
      use_synthetic_depth:=false
      use_cached_depth:=true
      use_depth_image:=true
      use_gazebo_depth_image:=false
      use_gazebo_laserscan:=false
      use_costmap_mux:=false
      start_gazebo_world:=false
      start_px4:=false
      enable_px4_bridge:=false
    )
    required_topics+=("/camera/depth/image" "/perception/depth_obstacle_markers")
    ;;
  bbox_cached_depth_mux)
    bbox_csv="${3:-${REPO_ROOT}/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv}"
    launch_cmd=(
      ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py
      planner:="${PLANNER}"
      bbox_csv:="${bbox_csv}"
      bbox_costmap_topic:=perception/bbox_occupancy_grid
      depth_costmap_topic:=perception/depth_occupancy_grid
      use_bbox_costmap:=true
      use_synthetic_cloud:=false
      use_pointcloud_costmap:=false
      use_synthetic_depth:=false
      use_cached_depth:=true
      use_depth_image:=true
      use_gazebo_depth_image:=false
      use_gazebo_laserscan:=false
      use_costmap_mux:=true
      start_gazebo_world:=false
      start_px4:=false
      enable_px4_bridge:=false
      start_x:=0.0
      start_y:=0.0
      goal_x:=24.0
      goal_y:=4.0
    )
    required_topics+=(
      "/camera/depth/image"
      "/perception/bbox_occupancy_grid"
      "/perception/depth_occupancy_grid"
      "/perception/costmap_mux_status"
      "/perception/bbox_markers"
      "/perception/depth_obstacle_markers"
    )
    ;;
  gazebo_depth)
    launch_cmd=(
      ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py
      planner:="${PLANNER}"
      use_synthetic_cloud:=false
      use_pointcloud_costmap:=false
      use_synthetic_depth:=false
      use_cached_depth:=false
      use_depth_image:=true
      use_gazebo_depth_image:=true
      use_gazebo_laserscan:=false
      use_costmap_mux:=false
      start_gazebo_world:=true
      start_px4:=false
      enable_px4_bridge:=false
    )
    required_topics+=("/camera/depth/image" "/perception/depth_obstacle_markers")
    ;;
  gazebo_laserscan)
    launch_cmd=(
      ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py
      planner:="${PLANNER}"
      use_synthetic_cloud:=false
      use_pointcloud_costmap:=false
      use_synthetic_depth:=false
      use_cached_depth:=false
      use_depth_image:=false
      use_gazebo_depth_image:=false
      use_gazebo_laserscan:=true
      use_costmap_mux:=false
      start_gazebo_world:=true
      start_px4:=false
      enable_px4_bridge:=false
    )
    required_topics+=("/uav_oda/lidar_scan")
    ;;
  gazebo_fused)
    launch_cmd=(
      ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py
      planner:="${PLANNER}"
      pointcloud_costmap_topic:=perception/pointcloud_occupancy_grid
      depth_costmap_topic:=perception/depth_occupancy_grid
      laserscan_costmap_topic:=perception/laserscan_occupancy_grid
      costmap_mux_input_topics_csv:=perception/pointcloud_occupancy_grid,perception/depth_occupancy_grid,perception/laserscan_occupancy_grid
      use_bbox_costmap:=false
      use_synthetic_cloud:=true
      use_pointcloud_costmap:=true
      use_synthetic_depth:=false
      use_cached_depth:=false
      use_depth_image:=true
      use_gazebo_depth_image:=true
      use_gazebo_laserscan:=true
      use_costmap_mux:=true
      start_gazebo_world:=true
      start_px4:=false
      enable_px4_bridge:=false
    )
    required_topics+=(
      "/lidar/points"
      "/camera/depth/image"
      "/uav_oda/lidar_scan"
      "/perception/pointcloud_occupancy_grid"
      "/perception/depth_occupancy_grid"
      "/perception/laserscan_occupancy_grid"
      "/perception/costmap_mux_status"
      "/perception/depth_obstacle_markers"
    )
    ;;
  bbox)
    bbox_csv="${3:-${REPO_ROOT}/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv}"
    launch_cmd=(
      ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py
      planner:="${PLANNER}"
      bbox_csv:="${bbox_csv}"
      use_bbox_costmap:=true
      use_synthetic_cloud:=false
      use_pointcloud_costmap:=false
      use_synthetic_depth:=false
      use_cached_depth:=false
      use_depth_image:=false
      use_gazebo_depth_image:=false
      use_gazebo_laserscan:=false
      use_costmap_mux:=false
      start_gazebo_world:=false
      start_px4:=false
      enable_px4_bridge:=false
      start_x:=7.5
      start_y:=0.0
      goal_x:=24.0
      goal_y:=4.0
    )
    required_topics+=("/perception/bbox_markers")
    ;;
  *)
    log "Unknown mode: ${MODE}"
    echo "Usage: $0 [synthetic|depth_image|cached_depth|bbox_cached_depth_mux|gazebo_depth|gazebo_laserscan|gazebo_fused|bbox] [astar|rrt|mppi] [bbox_csv]" >&2
    exit 2
    ;;
esac

log "Starting launch: ${launch_cmd[*]}"
setsid "${launch_cmd[@]}" >"${OUT_DIR}/launch.log" 2>&1 &
launch_pid="$!"

cleanup() {
  if kill -0 "${launch_pid}" >/dev/null 2>&1; then
    log "Stopping launch process group ${launch_pid}"
    kill -- "-${launch_pid}" >/dev/null 2>&1 || true
    sleep 2
    kill -9 -- "-${launch_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

log "Waiting ${DURATION_S}s for topics"
sleep "${DURATION_S}"

ros2 topic list | sort | tee "${OUT_DIR}/topic_list.txt"

missing=0
for topic in "${required_topics[@]}"; do
  if grep -Fxq "${topic}" "${OUT_DIR}/topic_list.txt"; then
    log "Topic present: ${topic}"
  else
    log "Missing required topic: ${topic}"
    missing=1
  fi
done

record_rosbag() {
  if [[ "${RECORD_BAG}" != "1" ]]; then
    log "Skipping rosbag recording because RECORD_BAG=${RECORD_BAG}"
    return 0
  fi
  if ! ros2 bag --help >/dev/null 2>&1; then
    log "Skipping rosbag recording because ros2 bag is unavailable"
    return 0
  fi
  local bag_dir="${OUT_DIR}/rosbag_${MODE}_${PLANNER}"
  log "Recording rosbag for ${BAG_DURATION_S}s: ${bag_dir}"
  set +e
  timeout --signal=INT --kill-after=3s "${BAG_DURATION_S}s" \
    ros2 bag record -o "${bag_dir}" "${required_topics[@]}" \
    >"${OUT_DIR}/rosbag_record.log" 2>&1
  local status="$?"
  set -e
  if [[ "${status}" -eq 0 || "${status}" -eq 124 || "${status}" -eq 130 ]]; then
    log "Rosbag recording finished with expected timeout/interrupt status ${status}"
    return 0
  fi
  log "Rosbag recording returned non-fatal status ${status}; see ${OUT_DIR}/rosbag_record.log"
  return 0
}

if [[ "${missing}" -eq 0 ]]; then
  record_rosbag
else
  log "Skipping rosbag recording because required topics are missing"
fi

check_topic_once() {
  local topic="$1"
  local safe_name
  safe_name="$(echo "${topic}" | sed 's#[/ ]#_#g' | sed 's#^_##')"
  log "Checking one message from ${topic}"
  if timeout 8s ros2 topic echo --once "${topic}" >"${OUT_DIR}/${safe_name}.txt" 2>&1; then
    log "Received message: ${topic}"
  else
    log "No message received within timeout: ${topic}"
    return 1
  fi
}

validate_mux_status_until_ready() {
  local topic="/perception/costmap_mux_status"
  local sample="${OUT_DIR}/perception_costmap_mux_status.txt"
  local validation_log="${OUT_DIR}/costmap_mux_status_validation.log"
  local deadline=$((SECONDS + MUX_STATUS_TIMEOUT_S))
  local attempt=0

  : >"${validation_log}"
  while (( SECONDS < deadline )); do
    attempt=$((attempt + 1))
    log "Validating merged costmap mux status, attempt ${attempt}"
    if timeout 8s ros2 topic echo --once --full-length "${topic}" >"${sample}" 2>&1; then
      {
        printf 'Attempt %s\n' "${attempt}"
        validation_args=()
        if [[ "${MODE}" == "gazebo_fused" ]]; then
          validation_args+=(
            --required-input perception/pointcloud_occupancy_grid
            --required-input perception/depth_occupancy_grid
            --required-input perception/laserscan_occupancy_grid
          )
        fi
        python3 scripts/validate_costmap_mux_status_sample.py "${sample}" "${validation_args[@]}"
      } >>"${validation_log}" 2>&1 && {
        log "Costmap mux status validation passed"
        return 0
      }
    else
      {
        printf 'Attempt %s\n' "${attempt}"
        printf 'No message received from %s\n' "${topic}"
      } >>"${validation_log}"
    fi
    sleep 1
  done

  log "Costmap mux status validation failed after ${MUX_STATUS_TIMEOUT_S}s; see ${validation_log}"
  return 1
}

for topic in "${required_topics[@]}"; do
  check_topic_once "${topic}" || missing=1
done

if [[ "${MODE}" == "bbox_cached_depth_mux" || "${MODE}" == "gazebo_fused" ]]; then
  validate_mux_status_until_ready || missing=1
fi

render_demo_video() {
  if [[ "${RENDER_VIDEO}" != "1" ]]; then
    log "Skipping MP4 render because RENDER_VIDEO=${RENDER_VIDEO}"
    return 0
  fi
  local video_path="${OUT_DIR}/ros2_costmap_demo_${MODE}_${PLANNER}.mp4"
  local mode_label
  local render_args=()
  mode_label="$(echo "${MODE}" | tr '[:lower:]_' '[:upper:] ')"
  if [[ "${MODE}" == "bbox_cached_depth_mux" ]]; then
    render_args+=(--scene cached_mux --bbox-csv "${bbox_csv}")
  fi
  log "Rendering MP4 demo video: ${video_path}"
  if python3 scripts/render_ros2_costmap_demo_video.py \
    --planner "${PLANNER}" \
    --output "${video_path}" \
    --title "ROS2 COSTMAP RUNTIME" \
    --status-text "MODE ${mode_label}" \
    "${render_args[@]}" \
    >"${OUT_DIR}/video_render.log" 2>&1; then
    log "Rendered MP4 demo video"
  else
    log "Video render failed non-fatally; see ${OUT_DIR}/video_render.log"
  fi
}

render_demo_video

if [[ "${missing}" -ne 0 ]]; then
  log "Runtime verification FAILED. Evidence saved to ${OUT_DIR}"
  python3 scripts/summarize_ros2_runtime_evidence.py || true
  python3 scripts/diagnose_ros2_runtime_failures.py || true
  python3 scripts/write_ros2_demo_report_section.py || true
  exit 1
fi

log "Runtime verification PASSED. Evidence saved to ${OUT_DIR}"
python3 scripts/summarize_ros2_runtime_evidence.py || true
python3 scripts/diagnose_ros2_runtime_failures.py || true
python3 scripts/write_ros2_demo_report_section.py || true
