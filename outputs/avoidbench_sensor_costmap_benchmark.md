# AvoidBench-Style Sensor-Costmap Planner Benchmark

This is a local adapter benchmark, not a full AvoidBench Unity/ROS Noetic run. It checks the planner contract needed for AvoidBench: sensor-derived maps are converted to occupancy costmaps, then A*, RRT, MPPI, and MPC consume the same map.

CSV: `outputs/tables/avoidbench_sensor_costmap_planner_matrix.csv`
Figure: `outputs/figures/avoidbench_sensor_costmap_planner_matrix.png`

## Sensor Sources

| Source | Sensor model | Note |
|---|---|---|
| sgm_depth_forest | AvoidBench /depth SGM-like metric depth | Depth image projected to local 2D costmap; mirrors AvoidBench /depth mono16 contract. |
| unity_depth_indoor | AvoidBench Unity depth / ideal depth | Ideal metric depth is useful for an upper-bound planner test. |
| monocular_relative_depth_proxy | RGB monocular-depth proxy | Relative depth is not metric; high near-response pixels are treated as obstacle evidence. |
| stereo_depth_plus_rgb_mask_mux | SGM depth + RGB/relative mask fusion | Two sensor-derived maps are merged before planning. |
| pointcloud_bbox_export | pointcloud -> 3D bbox -> costmap adapter | Adapter for AvoidBench pointcloud-unity exports or external PointCloud2 maps. |

## Planner Matrix

| Source | Planner | Waypoints | Length m | Min clearance m | Collision-free | Reached goal | Compute ms | Status |
|---|---|---:|---:|---:|---:|---:|---:|---|
| sgm_depth_forest | astar | 69 | 7.88 | 0.58 | 1 | 1 | 22.34 | ok |
| sgm_depth_forest | rrt | 10 | 8.05 | 0.70 | 1 | 1 | 41.01 | ok |
| sgm_depth_forest | mppi | 60 | 7.72 | 0.60 | 1 | 1 | 31.24 | ok |
| sgm_depth_forest | mpc | 80 | 7.72 | 0.58 | 1 | 1 | 104.53 | ok |
| unity_depth_indoor | astar | 67 | 8.17 | 0.58 | 1 | 1 | 27.98 | ok |
| unity_depth_indoor | rrt | 14 | 8.81 | 0.60 | 1 | 1 | 26.83 | ok |
| unity_depth_indoor | mppi | 60 | 7.95 | 0.58 | 1 | 1 | 37.15 | ok |
| unity_depth_indoor | mpc | 80 | 7.96 | 0.58 | 1 | 1 | 148.68 | ok |
| monocular_relative_depth_proxy | astar | 75 | 7.94 | 0.54 | 1 | 1 | 4.62 | ok |
| monocular_relative_depth_proxy | rrt | 6 | 7.85 | 0.54 | 1 | 1 | 2.61 | ok |
| monocular_relative_depth_proxy | mppi | 60 | 7.81 | 0.54 | 1 | 1 | 13.93 | ok |
| monocular_relative_depth_proxy | mpc | 80 | 7.82 | 0.54 | 1 | 1 | 10.34 | ok |
| stereo_depth_plus_rgb_mask_mux | astar | 83 | 8.99 | 0.54 | 1 | 1 | 25.14 | ok |
| stereo_depth_plus_rgb_mask_mux | rrt | 20 | 9.85 | 0.54 | 1 | 1 | 24.37 | ok |
| stereo_depth_plus_rgb_mask_mux | mppi | 60 | 8.70 | 0.54 | 1 | 1 | 34.62 | ok |
| stereo_depth_plus_rgb_mask_mux | mpc | 80 | 8.70 | 0.54 | 1 | 1 | 54.69 | ok |
| pointcloud_bbox_export | astar | 85 | 18.62 | 0.77 | 1 | 1 | 14.15 | ok |
| pointcloud_bbox_export | rrt | 41 | 21.36 | 0.80 | 1 | 1 | 16.83 | ok |
| pointcloud_bbox_export | mppi | 60 | 18.41 | 0.77 | 1 | 1 | 23.70 | ok |
| pointcloud_bbox_export | mpc | 80 | 18.40 | 0.77 | 1 | 1 | 148.39 | ok |

## Claim Boundary

- Safe to claim: the costmap/planner adapter is ready for AvoidBench-style sensor maps and runs locally for four planner families.
- Not safe to claim yet: full AvoidBench flight benchmark numbers, because this run does not launch Unity/Flightmare/RotorS or publish commands to `/hummingbird/autopilot/*`.
- Next runtime step: run AvoidBench Docker/ROS Noetic, subscribe to `/depth`, `/hummingbird/ground_truth/odometry`, `/hummingbird/goal_point`, publish velocity/pose commands, and publish `/hummingbird/iter_time` for official timing.
