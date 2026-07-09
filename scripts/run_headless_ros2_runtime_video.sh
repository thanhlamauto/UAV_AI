#!/usr/bin/env bash
set -euo pipefail

PLANNER="${1:-${PLANNER:-astar}}"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN_ALL_MODES="${RUN_ALL_MODES:-0}"
OUTPUT_VIDEO="${OUTPUT_VIDEO:-${REPO_ROOT}/outputs/videos/ros2_fused_perception_runtime_${PLANNER}.mp4}"
SUMMARY_CSV="${REPO_ROOT}/outputs/tables/ros2_demo_runtime_summary.csv"

log() {
  printf '[ros2-headless-video] %s\n' "$*"
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    log "Missing command: $1"
    exit 127
  fi
}

cd "${REPO_ROOT}"
mkdir -p "$(dirname "${OUTPUT_VIDEO}")"

require_cmd python3
require_cmd ffmpeg

log "Running headless ROS2/Gazebo runtime evidence with planner=${PLANNER}"
log "GUI/RViz interaction is not required; verifier records topics and renders MP4 with ffmpeg."

if [[ "${RUN_ALL_MODES}" == "1" ]]; then
  log "RUN_ALL_MODES=1, running all perception-to-planner modes"
  RENDER_VIDEO=1 RECORD_BAG=1 scripts/verify_ros2_costmap_all_modes.sh "${PLANNER}"
else
  log "Running focused fused mode: LiDAR bbox + cached depth -> costmap mux -> planner"
  RENDER_VIDEO=1 RECORD_BAG=1 scripts/verify_ros2_fused_perception_demo.sh "${PLANNER}"
fi

log "Copying latest passed fused runtime video to ${OUTPUT_VIDEO}"
python3 - "${SUMMARY_CSV}" "${OUTPUT_VIDEO}" <<'PY'
import csv
import shutil
import sys
from pathlib import Path

summary = Path(sys.argv[1])
output = Path(sys.argv[2])
if not summary.exists() or summary.stat().st_size == 0:
    raise SystemExit(f"Missing runtime summary CSV: {summary}")

with summary.open(newline="") as f:
    rows = list(csv.DictReader(f))

def complete(row: dict[str, str]) -> bool:
    return (
        row.get("mode") == "bbox_cached_depth_mux"
        and row.get("status") == "passed"
        and row.get("topics_present") == row.get("topics_expected")
        and row.get("messages_received") == row.get("messages_expected")
        and row.get("mux_status_valid") == "passed"
        and int(row.get("video_bytes") or 0) > 0
    )

candidates = [row for row in rows if complete(row)]
if not candidates:
    raise SystemExit("No passed bbox_cached_depth_mux runtime row with mux_status_valid=passed and MP4 video")

row = candidates[-1]
video = Path(row["video_file"])
if not video.exists() or video.stat().st_size == 0:
    raise SystemExit(f"Runtime video missing or empty: {video}")

output.parent.mkdir(parents=True, exist_ok=True)
shutil.copy2(video, output)
print(f"copied_video={output}")
print(f"source_runtime_folder={row.get('run_dir')}")
print(f"source_video={video}")
print(f"video_bytes={output.stat().st_size}")
PY

if command -v ffprobe >/dev/null 2>&1; then
  ffprobe -v error -select_streams v:0 \
    -show_entries stream=width,height,nb_frames,duration \
    -of default=noprint_wrappers=1 "${OUTPUT_VIDEO}" || true
fi

python3 scripts/bundle_ros2_demo_artifacts.py
log "Headless runtime video ready: ${OUTPUT_VIDEO}"
