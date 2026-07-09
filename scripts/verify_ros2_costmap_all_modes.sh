#!/usr/bin/env bash
set -euo pipefail

PLANNER="${1:-${PLANNER:-astar}}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
BBOX_CSV="${BBOX_CSV:-${REPO_ROOT}/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv}"
MODE_LIST="${ROS2_DEMO_MODES:-bbox synthetic depth_image cached_depth bbox_cached_depth_mux gazebo_depth gazebo_laserscan gazebo_fused}"
RUN_BOOTSTRAP="${RUN_BOOTSTRAP:-0}"
COLCON_SYMLINK_INSTALL="${COLCON_SYMLINK_INSTALL:-0}"
OUT_ROOT="${REPO_ROOT}/outputs/ros2_demo_runtime"
RUN_LOG="${OUT_ROOT}/verify_all_${PLANNER}_$(date +%Y%m%d_%H%M%S).log"

mkdir -p "${OUT_ROOT}"

log() {
  printf '[ros2-demo-verify-all] %s\n' "$*" | tee -a "${RUN_LOG}"
}

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

if [[ "${RUN_BOOTSTRAP}" == "1" ]]; then
  log "Running ROS2/Gazebo bootstrap"
  "${REPO_ROOT}/scripts/setup_ros2_gazebo_server.sh" 2>&1 | tee -a "${RUN_LOG}"
fi

log "Starting all-mode runtime verification with planner=${PLANNER}"
log "Modes: ${MODE_LIST}"

log "Running ROS2/Gazebo server preflight"
ROS2_DEMO_MODES="${MODE_LIST}" BBOX_CSV="${BBOX_CSV}" \
  "${REPO_ROOT}/scripts/check_ros2_server_preflight.sh" 2>&1 | tee -a "${RUN_LOG}"

source_setup "/opt/ros/${ROS_DISTRO}/setup.bash"

log "Running offline perception-to-planner contract check"
python3 "${REPO_ROOT}/scripts/check_perception_to_planner_contract.py" 2>&1 | tee -a "${RUN_LOG}"

log "Running offline perception/planner matrix check"
python3 "${REPO_ROOT}/scripts/check_perception_planner_matrix.py" 2>&1 | tee -a "${RUN_LOG}"

log "Building ROS2 package once before mode loop"
cd "${REPO_ROOT}/ros2_ws"
build_args=(build --packages-select uav_oda_ros2_demo)
if [[ "${COLCON_SYMLINK_INSTALL}" == "1" ]]; then
  build_args+=(--symlink-install)
fi
colcon "${build_args[@]}" 2>&1 | tee -a "${RUN_LOG}"
cd "${REPO_ROOT}"

failed_modes=()
for mode in ${MODE_LIST}; do
  log "Verifying mode=${mode}"
  set +e
  if [[ "${mode}" == "bbox" || "${mode}" == "bbox_cached_depth_mux" ]]; then
    SKIP_BUILD=1 "${REPO_ROOT}/scripts/verify_ros2_costmap_runtime.sh" "${mode}" "${PLANNER}" "${BBOX_CSV}" 2>&1 | tee -a "${RUN_LOG}"
  else
    SKIP_BUILD=1 "${REPO_ROOT}/scripts/verify_ros2_costmap_runtime.sh" "${mode}" "${PLANNER}" 2>&1 | tee -a "${RUN_LOG}"
  fi
  status="${PIPESTATUS[0]}"
  set -e
  if [[ "${status}" -ne 0 ]]; then
    log "Mode failed: ${mode} (exit ${status})"
    failed_modes+=("${mode}")
  else
    log "Mode passed: ${mode}"
  fi
done

log "Refreshing runtime summaries, report section, audit, and artifact bundle"
python3 scripts/summarize_ros2_runtime_evidence.py 2>&1 | tee -a "${RUN_LOG}" || true
python3 scripts/diagnose_ros2_runtime_failures.py 2>&1 | tee -a "${RUN_LOG}" || true
python3 scripts/write_ros2_demo_report_section.py 2>&1 | tee -a "${RUN_LOG}" || true
python3 scripts/bundle_ros2_demo_artifacts.py 2>&1 | tee -a "${RUN_LOG}" || true

set +e
python3 scripts/audit_ros2_demo_status.py --fail-on-incomplete 2>&1 | tee -a "${RUN_LOG}"
audit_status="${PIPESTATUS[0]}"
set -e

if [[ "${#failed_modes[@]}" -ne 0 ]]; then
  log "All-mode verification finished with failed modes: ${failed_modes[*]}"
  exit 1
fi

if [[ "${audit_status}" -ne 0 ]]; then
  log "All modes returned success, but final audit is still incomplete"
  exit "${audit_status}"
fi

log "All-mode runtime verification COMPLETE"
