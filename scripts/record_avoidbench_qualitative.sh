#!/usr/bin/env bash
set -u

planner="${1:-mppi}"
speed="${2:-5}"
seed="${3:-32}"
duration="${4:-180}"
tag="qual_${planner}_${speed}_seed${seed}"

cleanup() {
  pkill -x ffmpeg || true
  docker exec noetic_ab bash -lc "pkill -x roslaunch || true; pkill -x rosmaster || true; pkill -x gzserver || true; pkill -x AvoidBench.x86_64 || true; pkill -x python3 || true" >/dev/null 2>&1 || true
  docker exec noetic_ab bash -lc "cp /root/avoidbench_results/task_outdoor.yaml.qual.bak /AvoidBench/src/avoidbench/avoid_manage/params/task_outdoor.yaml 2>/dev/null || true" >/dev/null 2>&1 || true
}
trap cleanup EXIT

mkdir -p /root/avoidbench_results
rm -f "/root/avoidbench_results/${tag}"*

docker exec noetic_ab bash -lc "
set -e
cp /AvoidBench/src/avoidbench/avoid_manage/params/task_outdoor.yaml /root/avoidbench_results/task_outdoor.yaml.qual.bak
perl -0pi -e 's/flight_number: 30/flight_number: 1/; s/flight_number: 8/flight_number: 1/; s/  trials: 2/  trials: 1/; s/  trials: 8/  trials: 1/' /AvoidBench/src/avoidbench/avoid_manage/params/task_outdoor.yaml
pkill -x roslaunch || true
pkill -x rosmaster || true
pkill -x gzserver || true
pkill -x AvoidBench.x86_64 || true
pkill -x python3 || true
"

XAUTHORITY=/home/user/.Xauthority ffmpeg -y \
  -f x11grab -video_size 1024x768 -framerate 12 -i :0.0 \
  -t "$duration" -pix_fmt yuv420p "/root/avoidbench_results/${tag}.mp4" \
  >"/root/avoidbench_results/${tag}_ffmpeg.log" 2>&1 &
ffmpeg_pid=$!

docker exec noetic_ab bash -lc "
set +u
source /AvoidBench/devel/setup.bash
set -u
export AVOIDBENCH_PATH=/AvoidBench/src/avoidbench DISPLAY=:0 XAUTHORITY=/root/.Xauthority
cd /AvoidBench
roslaunch avoid_manage rotors_gazebo.launch > /root/avoidbench_results/${tag}_launch.log 2>&1 &
launch_pid=\$!
sleep 8
python3 /root/avoidbench_results/avoidbench_ros_planner.py --planner ${planner} --speed ${speed} --seed ${seed} > /root/avoidbench_results/${tag}_planner.log 2>&1 &
planner_pid=\$!
timeout 240s rostopic echo -n 1 /hummingbird/metrics > /root/avoidbench_results/${tag}_metrics.yaml 2>&1
echo rc=\$? > /root/avoidbench_results/${tag}_status.txt
kill \$planner_pid 2>/dev/null || true
kill \$launch_pid 2>/dev/null || true
pkill -x roslaunch || true
pkill -x rosmaster || true
pkill -x gzserver || true
pkill -x AvoidBench.x86_64 || true
pkill -x python3 || true
"

wait "$ffmpeg_pid" || true
docker cp "noetic_ab:/root/avoidbench_results/${tag}_metrics.yaml" "/root/avoidbench_results/${tag}_metrics.yaml" >/dev/null 2>&1 || true
docker cp "noetic_ab:/root/avoidbench_results/${tag}_status.txt" "/root/avoidbench_results/${tag}_status.txt" >/dev/null 2>&1 || true
docker cp "noetic_ab:/root/avoidbench_results/${tag}_launch.log" "/root/avoidbench_results/${tag}_launch.log" >/dev/null 2>&1 || true
docker cp "noetic_ab:/root/avoidbench_results/${tag}_planner.log" "/root/avoidbench_results/${tag}_planner.log" >/dev/null 2>&1 || true

ls -lh "/root/avoidbench_results/${tag}.mp4"
cat "/root/avoidbench_results/${tag}_status.txt" 2>/dev/null || true
grep -E "mission_progress|collision_number|processing_time|average_goal_velocity" "/root/avoidbench_results/${tag}_metrics.yaml" 2>/dev/null || true
