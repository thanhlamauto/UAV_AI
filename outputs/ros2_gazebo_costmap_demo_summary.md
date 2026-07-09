# ROS2/Gazebo Costmap Demo Summary

## Purpose

This extension connects the offline UAV obstacle-avoidance benchmark to a
simulation-style perception-to-planning pipeline:

```text
LiDAR bbox / PointCloud2 / synthetic depth / cached predicted-depth / merged bbox+depth / Gazebo depth / Gazebo LiDAR
    -> OccupancyGrid costmap -> A*/RRT/MPPI path
    -> kinematic UAV marker or optional PX4 Offboard setpoints
```

It addresses the gap between dataset-only evaluation and a UAV demo where
perception output becomes an obstacle map consumed by a planner.

## Implemented Artifacts

- ROS2 package: `ros2_ws/src/uav_oda_ros2_demo`.
- `bbox_costmap_publisher`: replays Multi-LiDAR 3D bbox CSV as `nav_msgs/OccupancyGrid` and RViz markers.
- `depth_image_costmap`: projects metric, relative, synthetic, or Gazebo depth images into a 2D occupancy grid.
- `pointcloud_costmap`: projects `sensor_msgs/PointCloud2` into a 2D occupancy grid.
- `laserscan_costmap`: converts Gazebo/ROS2 `sensor_msgs/LaserScan` into the same costmap interface.
- `costmap_planner`: consumes the costmap and publishes `nav_msgs/Path`.
- Pure Python planners: `astar`, `rrt`, and lightweight `mppi`.
- `synthetic_pointcloud_publisher`: creates repeatable obstacle point clouds for smoke testing.
- `synthetic_depth_image_publisher`: creates repeatable metric depth images for smoke testing.
- `cached_depth_image_publisher`: replays cached monocular predicted-depth frames as ROS2 `mono8` images.
- `costmap_mux`: waits for configured source grids and merges multiple perception-derived occupancy grids into one planner input.
- `static_pose_publisher`: publishes fixed start/goal poses.
- `kinematic_path_follower`: publishes simulated `/uav/current_pose`, `/odom`, and `/uav/marker` while following `/planned_path`.
- `px4_waypoint_follower`: optional SITL-only bridge from `nav_msgs/Path` to PX4 trajectory setpoints.
- Launch files for bbox replay, synthetic PointCloud2/depth costmaps, Gazebo depth/LiDAR costmaps, lightweight Gazebo world, and optional PX4 bridge.
- Server bootstrap: `scripts/setup_ros2_gazebo_server.sh`.
- Server preflight: `scripts/check_ros2_server_preflight.sh`.
- Server runner: `scripts/run_ros2_costmap_demo.sh`.
- Headless server video runner: `scripts/run_headless_ros2_runtime_video.sh`.
- Focused fused verifier: `scripts/verify_ros2_fused_perception_demo.sh`.
- Runtime verifier: `scripts/verify_ros2_costmap_runtime.sh`.
- All-mode runtime verifier: `scripts/verify_ros2_costmap_all_modes.sh`.
- Focused fused audit: `scripts/audit_ros2_fused_demo_status.py`.
- Runtime diagnostics: `scripts/diagnose_ros2_runtime_failures.py`.
- Readiness audit: `scripts/audit_ros2_demo_status.py`.
- Offline contract check: `scripts/check_perception_to_planner_contract.py`.
- Offline planner matrix check: `scripts/check_perception_planner_matrix.py`.
- Planner matrix table: `outputs/tables/perception_planner_matrix.csv`.
- Launch/package contract check: `scripts/check_ros2_launch_contract.py`.
- Mode consistency check: `scripts/check_ros2_mode_consistency.py`.
- Contract proof figure: `outputs/figures/perception_to_planner_contract.svg`.
- Artifact bundle: `scripts/bundle_ros2_demo_artifacts.py`.
- Report section writer: `scripts/write_ros2_demo_report_section.py`.
- MP4 renderer: `scripts/render_ros2_costmap_demo_video.py`.
- Runbook: `docs/ros2_gazebo_costmap_demo.md`.

## Verification Completed Locally

The local machine does not have ROS2/PX4 installed, so runtime ROS2 launch was
not executed here.  Static verification was completed:

```text
python -m py_compile ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/*.py
python -m py_compile ros2_ws/src/uav_oda_ros2_demo/launch/*.launch.py
python scripts/check_ros2_costmap_demo_static.py
python scripts/check_perception_to_planner_contract.py
python scripts/check_perception_planner_matrix.py
python scripts/check_ros2_launch_contract.py
python scripts/check_ros2_mode_consistency.py
```

Planner smoke-test output:

```text
astar: waypoints=61 length_m=6.17
rrt: waypoints=13 length_m=7.01
mppi: waypoints=60 length_m=6.12
```

Runtime verification on a ROS2/Gazebo server should be collected with:

```text
scripts/setup_ros2_gazebo_server.sh
python3 scripts/audit_ros2_demo_status.py
python3 scripts/check_ros2_launch_contract.py
scripts/run_headless_ros2_runtime_video.sh astar
scripts/verify_ros2_fused_perception_demo.sh astar
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
python3 scripts/audit_ros2_demo_status.py --fail-on-incomplete
python3 scripts/bundle_ros2_demo_artifacts.py
```

The verifier writes topic lists, launch logs, and one-message samples under
`outputs/ros2_demo_runtime/`.
For `bbox_cached_depth_mux`, it also retries `/perception/costmap_mux_status`
until the status sample proves `state=merged`, both bbox/depth source grids were
received, no configured input is missing, and the merged map contains occupied
cells. The runtime summary CSV records this as `mux_status_valid=passed`.

It also refreshes:

```text
outputs/ros2_demo_runtime_summary.md
outputs/tables/ros2_demo_runtime_summary.csv
outputs/ros2_demo_report_section.md
```

These files are intended as the report-facing runtime evidence table and
Vietnamese technical report section.

The artifact bundler writes `outputs/ros2_demo_artifacts.tar.gz`, which is the
small package to download from the server.

The renderer writes a standalone demo video such as
`outputs/videos/ros2_costmap_demo_astar.mp4`; runtime verifier runs also attempt
to write an MP4 into each evidence folder.
The headless server runner copies the latest passed fused runtime MP4 to
`outputs/videos/ros2_fused_perception_runtime_astar.mp4` for direct download.

Current local standalone video:

```text
outputs/videos/ros2_costmap_demo_astar.mp4
```

## Current Technical Scope

- Fixed-altitude 2D planning.
- Costmap inflation uses UAV radius plus safety margin.
- LiDAR bbox, PointCloud2, synthetic depth image, cached predicted-depth image, merged bbox+depth costmap, Gazebo depth image, and Gazebo LaserScan are now planner-input bridges, not only offline figures.
- Offline matrix evidence covers 5 perception-derived maps x 3 planners (`astar`, `rrt`, `mppi`) and rejects paths that intersect the inflated occupied map.
- A closed-loop RViz demo is available without PX4 through the kinematic follower.
- PX4 bridge is implemented but disabled by default for safety.
- The included Gazebo world publishes `/uav_oda/lidar_scan`; runtime verification requires ROS2/Gazebo on the server.

## Next Server Checks

1. Run `scripts/check_ros2_server_preflight.sh` to catch missing ROS2/Gazebo/ros_gz dependencies early.
2. Build the package with `colcon`.
3. Run `scripts/check_perception_to_planner_contract.py` to verify bbox/depth converters feed the planner offline.
4. Run bbox replay in RViz using Multi-LiDAR Tello03 Ouster bbox CSV.
5. Run synthetic `PointCloud2 -> costmap -> planner` launch.
6. Run synthetic `Depth Image -> costmap -> planner` launch.
7. Run cached `Predicted Depth -> costmap -> planner` launch with `scripts/run_ros2_costmap_demo.sh cached_depth astar`.
8. Run merged `LiDAR BBox + Cached Depth -> mux -> planner` launch with `scripts/run_ros2_costmap_demo.sh bbox_cached_depth_mux astar`.
   The verifier should save `perception_costmap_mux_status.txt` and `costmap_mux_status_validation.log` for this mode.
9. Run Gazebo `Depth Image -> costmap -> planner` launch with `scripts/run_ros2_costmap_demo.sh gazebo_depth astar`.
10. Run Gazebo `LaserScan -> costmap -> planner` launch with `scripts/run_ros2_costmap_demo.sh gazebo_laserscan astar`.
11. Run Gazebo fused `PointCloud2 + depth + LaserScan -> mux -> planner` launch with `scripts/run_ros2_costmap_demo.sh gazebo_fused astar`.
11. Run `scripts/verify_ros2_fused_perception_demo.sh astar` to collect focused evidence for the key fused branch.
12. Prefer `scripts/verify_ros2_costmap_all_modes.sh astar` to collect all runtime evidence and refresh the bundle in one pass.
13. If one mode fails, debug it with `scripts/verify_ros2_costmap_runtime.sh <mode> astar`.
13. Enable PX4 bridge only after `px4_msgs`, PX4 SITL, and the DDS bridge are confirmed working.
