# AvoidBench Costmap Planner Benchmark Runbook

This repo now has a local benchmark harness for the planning contract needed by
AvoidBench:

```text
AvoidBench sensor stream -> sensor algorithm -> occupancy costmap -> planner/controller
```

The local harness does not launch AvoidBench, Unity, Flightmare, RotorS, or ROS
Noetic.  It verifies that the planner side is ready before the runtime work.

## Local Benchmark

Run from the project root:

```bash
.venv/bin/python scripts/benchmark_avoidbench_costmap_planners.py
```

Outputs:

```text
outputs/tables/avoidbench_sensor_costmap_planner_matrix.csv
outputs/avoidbench_sensor_costmap_benchmark.md
outputs/figures/avoidbench_sensor_costmap_planner_matrix.png
```

The current local matrix covers:

```text
sensor/costmap sources:
- sgm_depth_forest
- unity_depth_indoor
- monocular_relative_depth_proxy
- stereo_depth_plus_rgb_mask_mux
- pointcloud_bbox_export

planners/controllers:
- astar
- rrt
- mppi
- mpc
```

The MPC row is a reference-tracking MPC-style local planner: A* gives the
global reference, then a local receding-horizon controller rolls out velocity
candidates. If the local rollout gets stuck, it returns the safe global
reference. Do not describe it as a pure unconstrained MPC solver.

## Runtime AvoidBench Integration

Use this only on a machine that can run AvoidBench's ROS Noetic/Flightmare/Unity
stack.

Expected data contract:

```text
/depth or stereo/RGB-derived depth
    -> depth_image_to_grid(...)
    -> nav_msgs/OccupancyGrid-like local costmap

/hummingbird/ground_truth/odometry
    -> current UAV state

/hummingbird/goal_point
    -> local/global goal

costmap + current state + goal
    -> astar / rrt / mppi / mpc
    -> command publisher
```

Runtime benchmark rows should add these fields beyond the local CSV:

```text
avoidbench_task
environment
algorithm
planner
success
collision
travel_time_s
path_length_m
min_clearance_m
mean_iter_time_ms
max_iter_time_ms
command_topic
```

## Sensor Algorithms To Test

Use these as the first practical set:

```text
1. SGM/stereo depth -> metric depth costmap
2. Unity/ground-truth depth -> upper-bound metric costmap
3. RGB monocular depth -> relative-depth costmap
4. RGB mask/relative depth + metric depth -> fused costmap
5. pointcloud export -> 3D bbox or voxel footprint costmap
```

The first two are the most defensible for AvoidBench because the benchmark is
vision-centric. The pointcloud row is useful as an adapter/stress test but
should be presented separately unless the AvoidBench run actually exports a
point cloud.

## Claim Boundary

Safe current claim:

```text
We added an AvoidBench-style sensor-costmap planner matrix. It verifies that
depth/relative-depth/fused/pointcloud-derived maps can drive A*, RRT, MPPI, and
an MPC-style local planner through the same occupancy-grid contract.
```

Do not claim yet:

```text
We have completed the official AvoidBench runtime benchmark.
```

That claim requires running the AvoidBench simulator, publishing control
commands for full episodes, and recording success/collision/travel-time metrics.
