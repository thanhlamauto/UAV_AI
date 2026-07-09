# Perception-to-Planner Contract Check

This is an offline, non-ROS check. It proves that the same converter helpers used by ROS2 nodes can produce a non-empty `OccupancyGrid`-style array and that the planner can consume it.

CSV: `outputs/tables/perception_to_planner_contract.csv`
Figure: `outputs/figures/perception_to_planner_contract.svg`

| Source | Grid | Occupied cells | Path waypoints | Path length m |
|---|---:|---:|---:|---:|
| lidar_bbox_csv | 88x72 | 626 | 85 | 18.46 |
| metric_depth_image | 80x80 | 269 | 61 | 6.58 |
| relative_predicted_depth_proxy | 80x80 | 30 | 66 | 6.87 |
| lidar_bbox_plus_relative_depth_mux | 133x75 | 638 | 127 | 27.44 |
| lidar_bbox_plus_cached_depth_mux | 133x75 | 666 | 128 | 27.80 |

Passing this check does not replace ROS2/Gazebo runtime verification. It only verifies the local conversion/planning contract before server execution.
