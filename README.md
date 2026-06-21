# UAV Obstacle Avoidance on ODA Dataset

## Context

This folder is the starting point for a focused UAV obstacle avoidance project.

The mentor's latest guidance:

- Do not continue with a broad survey only.
- Choose one concrete problem based on an available dataset.
- Run open-source/data pipelines, understand how they work, and produce measurable results.
- The goal is to show concrete outputs: visualization, benchmark metrics, and method comparison.

Current chosen scope:

> Indoor/GNSS-denied MAV obstacle avoidance and collision-risk analysis using the ODA Dataset.

Avoid mentioning diffusion policy to the mentor for now. Keep the first deliverable aligned with dataset processing, local obstacle representation, collision/safety metrics, and classical planners.

## Dataset

Primary dataset:

- ODA Dataset: https://github.com/JuSquare/ODA_Dataset

Why this dataset:

- It is directly related to obstacle detection and avoidance for MAVs.
- It contains indoor MAV trials with 1-2 obstacles.
- It includes RGB camera, event camera, radar, IMU, and OptiTrack ground truth.
- It is suitable for reconstructing MAV trajectory, obstacle position, safety distance, and collision-risk behavior.

Other mentor-provided sources for later reference:

- Multi-LiDAR Multi-UAV Dataset: https://tiers.github.io/multi_lidar_multi_uav_dataset/
- ARCO Dataset: https://robotics.upo.es/datasets/ArcoDataset/main.html
- MPPI controller: https://github.com/rapyuta-robotics/mppi_controller
- FAST-LIVO2: https://github.com/hku-mars/FAST-LIVO2
- HEPP paper: https://arxiv.org/abs/2505.17438

## First Deliverable

Prepare something concrete to show the mentor:

1. Dataset structure summary.
2. A visualization of one or more ODA trials.
3. MAV trajectory plotted with obstacle position.
4. Initial benchmark metrics:
   - minimum distance to obstacle;
   - collision or near-collision risk;
   - time/index of closest approach;
   - basic left/right/straight avoidance behavior if extractable.
5. A short plan for comparing planners next.

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

## Message to Mentor

Useful concise update:

```text
Dạ em hiểu rồi ạ. Em sẽ chốt một bài toán cụ thể trước thay vì survey rộng nữa.

Em sẽ bắt đầu với bài toán indoor MAV obstacle avoidance/collision-risk analysis trên ODA Dataset, vì dataset này có MAV bay tránh 1-2 vật cản trong indoor, kèm RGB/event camera, radar, IMU và OptiTrack ground truth. Trước mắt em sẽ chạy visualization/sample của dataset, dựng lại trajectory + obstacle position, rồi benchmark các metric như minimum safety distance, collision risk và computation time. Sau đó em sẽ thử baseline tránh vật cản trước, rồi mới so sánh thêm A*/RRT*/MPPI hoặc MPC.

Ngày mai em sẽ show anh phần chạy được gồm dataset structure, sample visualization, benchmark ban đầu và hướng so sánh các phương pháp tiếp theo ạ.
```

## Handoff Prompt for a New Agent

Use this when starting a new Codex session inside this folder:

```text
I am working on an indoor/GNSS-denied MAV obstacle avoidance project using the ODA Dataset. The mentor asked me to stop doing broad survey and produce concrete runnable results. Please help me set up the project, inspect/download the ODA Dataset, run or recreate a sample visualization, plot MAV trajectory with obstacle position, and compute initial benchmark metrics such as minimum distance to obstacle, collision/safety-distance violation, closest-approach time, and computation time. Start lightweight with Python CSV/metadata processing. Do not start with ROS/Gazebo/heavy learning unless needed.
```
