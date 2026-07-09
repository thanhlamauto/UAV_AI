#!/usr/bin/env bash
set -u
set +u
source /AvoidBench/devel/setup.bash
set -u
export AVOIDBENCH_PATH=/AvoidBench/src/avoidbench
export DISPLAY=:0
export XAUTHORITY=/root/.Xauthority

cd /AvoidBench
mkdir -p /root/avoidbench_results/matrix

task_yaml=/AvoidBench/src/avoidbench/avoid_manage/params/task_outdoor.yaml
task_backup=/root/avoidbench_results/task_outdoor.yaml.matrix.bak
cp "$task_yaml" "$task_backup"
python3 - <<'PY'
from pathlib import Path

p = Path("/AvoidBench/src/avoidbench/avoid_manage/params/task_outdoor.yaml")
s = p.read_text()
s = s.replace("flight_number: 30", "flight_number: 1")
s = s.replace("  trials: 2", "  trials: 1")
p.write_text(s)
PY

restore_task_yaml() {
  cp "$task_backup" "$task_yaml" 2>/dev/null || true
}
trap restore_task_yaml EXIT

stop_stack() {
  pkill -x roslaunch || true
  pkill -x rosmaster || true
  pkill -x gzserver || true
  pkill -x AvoidBench.x86_64 || true
  pkill -x python3 || true
}

run_one() {
  local planner="$1"
  local speed="$2"
  local tag="${planner}_${speed}"
  echo "=== RUN ${tag} ==="
  stop_stack
  sleep 2
  rm -f "/root/avoidbench_results/matrix/${tag}_"*

  roslaunch avoid_manage rotors_gazebo.launch \
    >"/root/avoidbench_results/matrix/${tag}_launch.log" 2>&1 &
  local launch_pid=$!
  sleep 8

  python3 /root/avoidbench_results/avoidbench_ros_planner.py \
    --planner "$planner" --speed "$speed" --seed 32 \
    >"/root/avoidbench_results/matrix/${tag}_planner.log" 2>&1 &
  local planner_pid=$!

  timeout 260s rostopic echo -n 1 /hummingbird/metrics \
    >"/root/avoidbench_results/matrix/${tag}_metrics.yaml" 2>&1
  local rc=$?

  timeout 5s rostopic echo -n 1 /hummingbird/task_state \
    >"/root/avoidbench_results/matrix/${tag}_state.yaml" 2>&1 || true
  timeout 5s rostopic echo -n 1 /hummingbird/ground_truth/odometry \
    >"/root/avoidbench_results/matrix/${tag}_odom.yaml" 2>&1 || true

  kill "$planner_pid" 2>/dev/null || true
  kill "$launch_pid" 2>/dev/null || true
  stop_stack
  sleep 2

  echo "${tag},rc=${rc}" | tee "/root/avoidbench_results/matrix/${tag}_status.txt"
  grep -E "mission_progress|collision_number|processing_time|average_goal_velocity|optimality_factor|collision_percent" \
    "/root/avoidbench_results/matrix/${tag}_metrics.yaml" || true
}

for planner in rrt rrt_star mppi; do
  for speed in 1 2 3 4 5 6; do
    run_one "$planner" "$speed"
  done
done

python3 - <<'PY'
from pathlib import Path
import csv
import math
import re

root = Path("/root/avoidbench_results/matrix")
rows = []
for planner in ["rrt", "rrt_star", "mppi"]:
    for speed in ["1", "2", "3", "4", "5", "6"]:
        tag = f"{planner}_{speed}"
        text = (root / f"{tag}_metrics.yaml").read_text(errors="ignore") if (root / f"{tag}_metrics.yaml").exists() else ""
        status = (root / f"{tag}_status.txt").read_text(errors="ignore").strip() if (root / f"{tag}_status.txt").exists() else ""

        def val(name):
            match = re.search(name + r": \[?([-+0-9.eE]+)", text)
            return float(match.group(1)) if match else math.nan

        rows.append(
            {
                "planner": planner,
                "speed_mps": float(speed),
                "status": status,
                "progress": val("mission_progress"),
                "collision_number": val("collision_number"),
                "processing_time_ms": val("processing_time"),
                "avg_goal_velocity": val("average_goal_velocity"),
                "optimality_factor": val("optimality_factor"),
                "collision_percent": val("collision_percent"),
            }
        )

with (root / "summary.csv").open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    writer.writeheader()
    writer.writerows(rows)

print((root / "summary.csv").read_text())
PY
