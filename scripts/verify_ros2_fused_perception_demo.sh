#!/usr/bin/env bash
set -euo pipefail

PLANNER="${1:-${PLANNER:-astar}}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BBOX_CSV="${BBOX_CSV:-${REPO_ROOT}/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv}"
MODE="bbox_cached_depth_mux"
OUT_ROOT="${REPO_ROOT}/outputs/ros2_demo_runtime"
RUN_LOG="${OUT_ROOT}/verify_fused_${PLANNER}_$(date +%Y%m%d_%H%M%S).log"
RUN_MODE_CONSISTENCY="${RUN_MODE_CONSISTENCY:-0}"

mkdir -p "${OUT_ROOT}"

log() {
  printf '[ros2-demo-fused] %s\n' "$*" | tee -a "${RUN_LOG}"
}

log "Starting focused fused-perception verification with planner=${PLANNER}"
log "Mode: ${MODE}"
log "Bbox CSV: ${BBOX_CSV}"

cd "${REPO_ROOT}"

log "Running local/static contracts"
python3 scripts/check_perception_to_planner_contract.py 2>&1 | tee -a "${RUN_LOG}"
python3 scripts/check_perception_planner_matrix.py 2>&1 | tee -a "${RUN_LOG}"
python3 scripts/check_ros2_launch_contract.py 2>&1 | tee -a "${RUN_LOG}"
if [[ "${RUN_MODE_CONSISTENCY}" == "1" ]]; then
  python3 scripts/check_ros2_mode_consistency.py 2>&1 | tee -a "${RUN_LOG}"
else
  log "Skipping all-mode documentation consistency check for focused runtime; set RUN_MODE_CONSISTENCY=1 to enable it."
fi

log "Running focused ROS2/Gazebo server preflight"
ROS2_DEMO_MODES="${MODE}" BBOX_CSV="${BBOX_CSV}" \
  scripts/check_ros2_server_preflight.sh 2>&1 | tee -a "${RUN_LOG}"

log "Running focused runtime verifier"
scripts/verify_ros2_costmap_runtime.sh "${MODE}" "${PLANNER}" "${BBOX_CSV}" 2>&1 | tee -a "${RUN_LOG}"

log "Refreshing runtime summary and focused audit"
python3 scripts/summarize_ros2_runtime_evidence.py 2>&1 | tee -a "${RUN_LOG}"
python3 scripts/diagnose_ros2_runtime_failures.py 2>&1 | tee -a "${RUN_LOG}" || true
python3 scripts/write_ros2_demo_report_section.py 2>&1 | tee -a "${RUN_LOG}"
python3 scripts/audit_ros2_fused_demo_status.py --fail-on-incomplete 2>&1 | tee -a "${RUN_LOG}"
python3 scripts/bundle_ros2_demo_artifacts.py 2>&1 | tee -a "${RUN_LOG}"

log "Focused fused-perception verification COMPLETE"
