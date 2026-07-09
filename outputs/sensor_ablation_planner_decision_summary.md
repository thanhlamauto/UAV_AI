# Sensor Ablation Planner Decision Demo

This artifact compares the planner decision with full sensor fusion against the same planner when one sensor input is removed.

- Video: `outputs/videos/sensor_ablation_planner_decision_demo.mp4`
- Preview: `outputs/figures/level3_video_preview/sensor_ablation_planner_decision_midframe.png`
- Metrics: `outputs/tables/sensor_ablation_planner_decision_metrics.csv`

| Removed sensor | Full collision | Ablated collision | Full clearance m | Ablated clearance m |
|---|---:|---:|---:|---:|
| lidar | 0 | 1 | 0.2850 | -0.1057 |
| depth | 0 | 1 | 0.2100 | -0.3234 |
| radar | 0 | 1 | 0.2000 | -0.3683 |

Scope note: the sensor panels are simulated sensor-derived obstacle maps. This is an explanatory ablation video, not a raw ROS/Gazebo screen recording.
