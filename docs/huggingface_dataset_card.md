---
license: other
task_categories:
- robotics
- computer-vision
- tabular-classification
tags:
- uav
- obstacle-avoidance
- lidar
- radar
- imu
- trajectory
- point-cloud
pretty_name: UAV ODA Obstacle Avoidance Processed Results
---

# UAV ODA Obstacle Avoidance Processed Results

This private repository stores processed project data and experiment outputs for
an indoor/GNSS-denied UAV obstacle avoidance and collision-risk benchmark.

## Contents

- `data/`: extracted/processed local project data used by the experiments.
- `outputs/`: benchmark tables, figures, qualitative videos, logs, and summaries.
- `reports/`: LaTeX/PDF report artifacts.
- `src/`, `experiments/`, `scripts/`, `docs/`: code and documentation needed to
  reproduce the project pipeline.

## Dataset Sources

Primary benchmark:

- ODA Dataset: https://github.com/JuSquare/ODA_Dataset
- 4TU record: https://doi.org/10.4121/14214236.v1

External sensing stress tests:

- ARCO Dataset: https://robotics.upo.es/datasets/ArcoDataset/main.html
- Multi-LiDAR Multi-UAV Dataset: https://tiers.github.io/multi_lidar_multi_uav_dataset/

## Scope

ODA remains the main UAV obstacle-avoidance benchmark for trajectory risk,
clearance, planner comparison, and perception-risk features. ARCO is used as a
LiDAR/radar/IMU sensing stress test, including real PointCloud2 segmentation and
3D bounding boxes. Multi-LiDAR is tracked as a UAV LiDAR extension target; its
SharePoint downloads currently require authentication.

## Reproducibility

The main audit command is:

```bash
python3 scripts/audit_goal_status.py
python3 -m py_compile src/*.py src/planners/*.py experiments/*.py scripts/*.py
```

The repository intentionally does not include the duplicate full ODA ZIP archives
stored on the server under `/workspace/data`; those should be downloaded from the
original dataset source when needed.
