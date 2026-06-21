# Cross-Dataset Positioning

## Core Narrative

The project should remain an ODA-based multi-sensor risk-aware UAV obstacle avoidance benchmark. ODA is the primary dataset because it directly contains indoor MAV obstacle-avoidance trials with RGB, radar, IMU, OptiTrack ground truth, and obstacle metadata.

The other README sources are useful for positioning and future validation, but they should not replace ODA as the main benchmark unless the ODA results are already complete.

## Source Roles

| Source | Role in this project | Use now? |
| --- | --- | --- |
| ODA Dataset | Primary benchmark for trajectory risk, obstacle clearance, sensor panels, and planner comparison. | Yes |
| Multi-LiDAR Multi-UAV Dataset | Future cross-dataset validation for GNSS-denied UAV tracking/localization context; current SharePoint links require login. | Later/auth needed |
| ARCO Dataset | Radar/LiDAR/IMU reference dataset; useful sensing context but ground-robot, not UAV avoidance. | Stress probe done |
| MPPI controller repo | Conceptual reference for MPPI cost terms and collision critics. Avoid ROS/Nav2 dependency in this phase. | Reference |
| FAST-LIVO2 | Related work for visual-LiDAR-inertial SLAM. Out of scope for the current ODA benchmark. | Future work |
| HEPP paper | Motivation for high-speed UAV planning, low-latency planning, and future planner design. | Cite/read |

## Why ODA Remains Primary

- It matches the target vehicle class: MAV/UAV.
- It contains obstacle-avoidance trials rather than only localization or mapping.
- OptiTrack ground truth makes safety-distance metrics measurable.
- RGB, radar, and IMU allow a perception-risk extension without changing datasets.
- Existing scripts already process ODA metadata, OptiTrack, radar, IMU, depth cache, and planner outputs.

## What Multi-LiDAR Could Validate Later

Multi-LiDAR can help position the work in broader GNSS-denied UAV perception and tracking. The current link probe found 27/27 SharePoint links require Microsoft login, so it should remain a documented future validation target unless authenticated/direct downloads become available. Porting the full planner benchmark to rosbag-based LiDAR data is out of scope because ODA already carries the core UAV avoidance benchmark.

## Why ARCO Is Not Directly Comparable

ARCO is useful for radar/LiDAR/IMU sensing ideas, but it is a ground-robot dataset. The current probe inspected 3 ROS2 bag ZIP samples, 33 topic rows, and 175997 messages without ROS. It should not be mixed into the main UAV obstacle-avoidance metrics because vehicle dynamics, obstacle geometry, and task framing differ from ODA.

## How HEPP and MPPI Motivate Future Work

HEPP motivates low-latency planning for high-speed UAV obstacle avoidance. The MPPI controller repo motivates cost terms such as goal progress, smoothness, and collision critics. For this project, these should become lightweight Python planner baselines first, not ROS/Gazebo dependencies.

## Explicit Out-of-Scope Items For Current Benchmark

- ROS/Gazebo integration.
- FAST-LIVO2 deployment.
- Full rosbag ingestion for external datasets beyond lightweight sqlite topic/message probes.
- Claiming metric depth from monocular depth; current depth is relative unless calibrated.

## Recommended Thesis/Report Wording

This project builds an ODA-based multi-sensor risk-aware UAV obstacle avoidance benchmark. External datasets and controllers are used to motivate future generalization and planner design, while the implemented evaluation remains focused on reproducible ODA trajectory, clearance, risk-label, and planner-comparison metrics.
