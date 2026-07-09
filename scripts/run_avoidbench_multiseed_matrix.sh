#!/usr/bin/env bash
set -u
set +u
source /AvoidBench/devel/setup.bash
set -u
export AVOIDBENCH_PATH=/AvoidBench/src/avoidbench
export DISPLAY=:0
export XAUTHORITY=/root/.Xauthority

cd /AvoidBench
out_dir=/root/avoidbench_results/matrix_8seed
mkdir -p "$out_dir"

task_yaml=/AvoidBench/src/avoidbench/avoid_manage/params/task_outdoor.yaml
task_backup=/root/avoidbench_results/task_outdoor.yaml.multiseed.bak
cp "$task_yaml" "$task_backup"
python3 - <<'PY'
from pathlib import Path

p = Path("/AvoidBench/src/avoidbench/avoid_manage/params/task_outdoor.yaml")
s = p.read_text()
s = s.replace("flight_number: 30", "flight_number: 8")
s = s.replace("flight_number: 1", "flight_number: 8")
s = s.replace("  trials: 2", "  trials: 1")
s = s.replace("  trials: 8", "  trials: 1")
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
  if [ -s "${out_dir}/${tag}_metrics.yaml" ] && grep -q "factors:" "${out_dir}/${tag}_metrics.yaml"; then
    echo "skip ${tag}: metrics already exist"
    return
  fi

  stop_stack
  sleep 2
  rm -f "${out_dir}/${tag}_"*

  roslaunch avoid_manage rotors_gazebo.launch >"${out_dir}/${tag}_launch.log" 2>&1 &
  local launch_pid=$!
  sleep 8

  python3 /root/avoidbench_results/avoidbench_ros_planner.py \
    --planner "$planner" --speed "$speed" --seed 32 >"${out_dir}/${tag}_planner.log" 2>&1 &
  local planner_pid=$!

  timeout 1800s rostopic echo -n 1 /hummingbird/metrics >"${out_dir}/${tag}_metrics.yaml" 2>&1
  local rc=$?

  timeout 5s rostopic echo -n 1 /hummingbird/task_state >"${out_dir}/${tag}_state.yaml" 2>&1 || true
  timeout 5s rostopic echo -n 1 /hummingbird/ground_truth/odometry >"${out_dir}/${tag}_odom.yaml" 2>&1 || true

  kill "$planner_pid" 2>/dev/null || true
  kill "$launch_pid" 2>/dev/null || true
  stop_stack
  sleep 2

  echo "${tag},rc=${rc}" | tee "${out_dir}/${tag}_status.txt"
  grep -E "mission_progress|collision_number|processing_time|average_goal_velocity|optimality_factor|collision_percent" \
    "${out_dir}/${tag}_metrics.yaml" || true
}

for planner in rrt rrt_star mppi; do
  for speed in 1 2 3 4 5 6; do
    run_one "$planner" "$speed"
  done
done

python3 /root/avoidbench_results/summarize_avoidbench_multiseed.py "$out_dir"
