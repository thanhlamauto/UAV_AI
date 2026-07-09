# UAV Obstacle Avoidance on ODA Dataset

## Overview

This repository contains ODA-Bench, an offline benchmark and evaluation pipeline
for indoor/GNSS-denied UAV obstacle avoidance using the ODA Dataset. The project
converts raw ODA logs into reproducible safety metrics, planner baselines,
policy-learning artifacts, risk labels, latency replay checks, and PyBullet
validation outputs.

## Dataset

Primary dataset:

- ODA Dataset: https://github.com/JuSquare/ODA_Dataset

Why this dataset:

- It is directly related to obstacle detection and avoidance for MAVs.
- It contains indoor MAV trials with 1-2 obstacles.
- It includes RGB camera, event camera, radar, IMU, and OptiTrack ground truth.
- It is suitable for reconstructing MAV trajectory, obstacle position, safety distance, and collision-risk behavior.

Related resources:

- Multi-LiDAR Multi-UAV Dataset: https://tiers.github.io/multi_lidar_multi_uav_dataset/
- ARCO Dataset: https://robotics.upo.es/datasets/ArcoDataset/main.html
- MPPI controller: https://github.com/rapyuta-robotics/mppi_controller
- FAST-LIVO2: https://github.com/hku-mars/FAST-LIVO2
- HEPP paper: https://arxiv.org/abs/2505.17438

## Core Outputs

The repository is organized around reproducible benchmark outputs:

1. Dataset structure summary.
2. A visualization of one or more ODA trials.
3. MAV trajectory plotted with obstacle position.
4. Initial benchmark metrics:
   - minimum distance to obstacle;
   - collision or near-collision risk;
   - time/index of closest approach;
   - basic left/right/straight avoidance behavior if extractable.
5. Planner, learning, risk-prediction, latency-replay, and PyBullet validation artifacts.

## Recommended First Pipeline

Start simple. Use CSV/metadata and ground-truth trajectory before touching ROS, Gazebo, or heavy learning models.

```text
Download / inspect ODA Dataset
        ↓
Read trial metadata
        ↓
Load OptiTrack / obstacle information
        ↓
Plot MAV trajectory + obstacle boundary
        ↓
Compute safety-distance metrics
        ↓
Implement a simple geometric avoidance baseline
        ↓
Compare A* / RRT* / MPPI or MPC if time permits
```

## Suggested Project Layout

Create these folders as needed:

```text
data/
  raw/              # downloaded ODA files, do not commit large files
  processed/        # cleaned CSV/npz/parquet files
notebooks/          # quick exploration
src/
  oda_io.py         # dataset loading helpers
  metrics.py        # distance/collision/safety metrics
  visualize.py      # trajectory and obstacle plotting
  planners/
    astar.py
    rrt.py
    mppi.py
experiments/
  benchmark_oda.py
outputs/
  figures/
  tables/
```

## Minimal Python Setup

MacBook Air M2 should be enough for the first phase.

Use a light Python environment first:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pandas matplotlib scipy opencv-python tqdm
```

Avoid starting with ROS/Gazebo/FAST-LIVO2 unless there is already a working pipeline.

## Experiment Ideas

Baseline metrics using ground truth:

- minimum distance from MAV trajectory to obstacle center/boundary;
- safety-distance violation count;
- closest-approach timestamp;
- success/failure if a safety radius is defined;
- trajectory smoothness;
- approximate computation time.

Planner comparison:

- straight-line baseline;
- geometric left/right bypass baseline;
- A* on a 2D occupancy grid;
- RRT* or RRT;
- MPPI/MPC later, if the basic pipeline is already working.

Possible settings:

- 1 obstacle vs 2 obstacles;
- full light vs dim light;
- different safety radii;
- different assumed UAV speeds;
- clean ground-truth map vs noisy local map.

## Lightweight 3D Simulation

The repo includes a browser-based Three.js simulation for presenting the
planner pipeline before a full ROS2/Gazebo runtime is available:

```text
obstacle geometry -> occupancy/safety map -> A*/RRT/MPPI path -> 3D UAV motion
```

Run:

```bash
scripts/serve_3d_simulation.sh 8765
```

Open:

```text
http://localhost:8765/
```

Files:

```text
sim3d/index.html
sim3d/uav_sim_data.json
docs/3d_simulation.md
scripts/export_3d_simulation_data.py
scripts/verify_3d_simulation_render.js
scripts/record_3d_simulation_video.js
scripts/bundle_3d_simulation_artifacts.py
scripts/audit_3d_simulation_status.py
```

This is a lightweight visual simulation, not a physics-level quadrotor
simulator.  Gazebo/ROS2 remains the route for runtime sensor/costmap evidence.

Verified outputs:

```text
outputs/figures/uav_3d_sim_desktop.png
outputs/figures/uav_3d_sim_mobile.png
outputs/videos/uav_3d_simulation_astar.mp4
outputs/3d_simulation_artifacts.tar.gz
```

Submission audit:

```bash
python3 scripts/audit_progress_submission.py --fail-on-incomplete
```

## Advanced ROS2/Gazebo Extension

After the offline ODA benchmark is stable, the project now includes a small
ROS2 perception-to-planning demo:

```text
LiDAR bbox / PointCloud2 / synthetic depth / cached predicted depth / merged bbox+depth / Gazebo depth / Gazebo LiDAR
    -> OccupancyGrid costmap -> A*/RRT/MPPI path
    -> kinematic UAV marker or optional PX4 waypoint follower
```

Implementation:

```text
ros2_ws/src/uav_oda_ros2_demo
```

Runbook:

```text
docs/ros2_gazebo_costmap_demo.md
```

Integration status note:

```text
outputs/perception_to_planner_integration_status.md
```

This extension is not a replacement for the ODA benchmark.  It is the next
integration layer used to demonstrate that LiDAR/perception outputs can become
an obstacle map consumed by the UAV planners. The focused fused verifier only
passes when `/perception/costmap_mux_status` proves that both LiDAR bbox and
cached-depth costmaps were received and merged into the planner input.

Fast server runner:

```bash
python3 scripts/check_perception_to_planner_contract.py
python3 scripts/check_perception_planner_matrix.py
python3 scripts/check_ros2_launch_contract.py
python3 scripts/check_ros2_mode_consistency.py
scripts/run_ros2_costmap_demo.sh synthetic astar
scripts/run_ros2_costmap_demo.sh depth_image astar
scripts/run_ros2_costmap_demo.sh cached_depth astar
scripts/run_ros2_costmap_demo.sh bbox_cached_depth_mux astar
scripts/run_ros2_costmap_demo.sh gazebo_depth astar
scripts/run_ros2_costmap_demo.sh gazebo_laserscan astar
scripts/run_ros2_costmap_demo.sh gazebo_fused astar
scripts/run_ros2_costmap_demo.sh bbox astar
```

Runtime evidence runner:

```bash
scripts/setup_ros2_gazebo_server.sh
python3 scripts/audit_ros2_demo_status.py
python3 scripts/check_ros2_launch_contract.py
scripts/check_ros2_server_preflight.sh
scripts/run_headless_ros2_runtime_video.sh astar
scripts/verify_ros2_fused_perception_demo.sh astar
scripts/verify_ros2_costmap_all_modes.sh astar
python3 scripts/audit_ros2_demo_status.py --fail-on-incomplete
python3 scripts/bundle_ros2_demo_artifacts.py
```

On a rented server without GUI interaction, run only this for the focused video
evidence:

```bash
scripts/run_headless_ros2_runtime_video.sh astar
```

It writes the downloadable MP4 to:

```text
outputs/videos/ros2_fused_perception_runtime_astar.mp4
```

Single-mode fallback/debug commands:

```bash
scripts/verify_ros2_costmap_runtime.sh bbox astar
scripts/verify_ros2_costmap_runtime.sh synthetic astar
scripts/verify_ros2_costmap_runtime.sh depth_image astar
scripts/verify_ros2_costmap_runtime.sh cached_depth astar
scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar
scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar
scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar
scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar
python3 scripts/write_ros2_demo_report_section.py
python3 scripts/render_ros2_costmap_demo_video.py --planner astar --output outputs/videos/ros2_costmap_demo_astar.mp4
```

PX4 fused sensor-control runner, only after PX4 SITL, `px4_msgs`, and the PX4
ROS2 DDS bridge are available:

```bash
scripts/run_ros2_gazebo_fused_px4.sh astar
```

See `docs/ros2_gazebo_px4_sensor_fusion.md`.

Runtime summaries are written to:

```text
outputs/ros2_demo_runtime_summary.md
outputs/tables/ros2_demo_runtime_summary.csv
outputs/ros2_demo_report_section.md
outputs/videos/ros2_costmap_demo_astar.mp4
outputs/ros2_demo_artifacts.tar.gz
```

The local planner matrix writes `outputs/tables/perception_planner_matrix.csv`
and verifies that LiDAR bbox, depth-derived, and fused obstacle maps can feed
`astar`, `rrt`, and `mppi` without crossing inflated occupied cells.

## Metric Depth and Radar Range-Doppler

The perception-risk path now has an upgraded experiment path documented in
`docs/perception_metric_depth_radar_rd.md`:

- Radar Level-3: raw I/Q -> range FFT -> Doppler FFT -> compact range-Doppler features.
- Metric depth: RGB -> Depth Anything V2 Metric Indoor Small -> metric depth -> point cloud -> occupancy/ESDF.
- Local benchmark outputs include the retrained perception-risk tables and a depth-occupancy planner summary under `outputs/tables/`.

Radar Level-3 is not radar occupancy mapping, and MacBook M2/MPS latency is only
a development proxy rather than onboard UAV timing.
