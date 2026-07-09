# Perception Planner Matrix

This offline check verifies that every perception-derived obstacle map used by the ROS2 demo can feed every lightweight planner.

CSV: `outputs/tables/perception_planner_matrix.csv`

| Source | Planner | Grid | Occupied | Waypoints | Length m | Collision-free |
|---|---|---:|---:|---:|---:|---:|
| lidar_bbox_csv | astar | 88x72 | 626 | 85 | 18.46 | 1 |
| lidar_bbox_csv | rrt | 88x72 | 626 | 43 | 20.89 | 1 |
| lidar_bbox_csv | mppi | 88x72 | 626 | 60 | 18.22 | 1 |
| metric_depth_image | astar | 80x80 | 269 | 61 | 6.58 | 1 |
| metric_depth_image | rrt | 80x80 | 269 | 9 | 7.24 | 1 |
| metric_depth_image | mppi | 80x80 | 269 | 60 | 6.52 | 1 |
| relative_predicted_depth_proxy | astar | 80x80 | 30 | 66 | 6.87 | 1 |
| relative_predicted_depth_proxy | rrt | 80x80 | 30 | 5 | 7.17 | 1 |
| relative_predicted_depth_proxy | mppi | 80x80 | 30 | 60 | 6.82 | 1 |
| lidar_bbox_plus_relative_depth_mux | astar | 133x75 | 638 | 127 | 27.44 | 1 |
| lidar_bbox_plus_relative_depth_mux | rrt | 133x75 | 638 | 67 | 32.98 | 1 |
| lidar_bbox_plus_relative_depth_mux | mppi | 133x75 | 638 | 60 | 26.85 | 1 |
| lidar_bbox_plus_cached_depth_mux | astar | 133x75 | 666 | 128 | 27.80 | 1 |
| lidar_bbox_plus_cached_depth_mux | rrt | 133x75 | 666 | 78 | 32.68 | 1 |
| lidar_bbox_plus_cached_depth_mux | mppi | 133x75 | 666 | 60 | 27.19 | 1 |

`Collision-free` is checked after applying the same robot-radius plus safety-distance inflation used by the planner.
This does not replace ROS2/Gazebo runtime evidence; it is a local contract check before server execution.
