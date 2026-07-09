# Perception-to-Planner Integration Status

This file answers whether perception outputs are connected to the obstacle
avoidance planner.

## Integrated Into ROS2 Planner Input

All integrated branches publish the same planner input:

```text
/perception/occupancy_grid -> costmap_planner -> /planned_path -> kinematic_path_follower
```

| Perception source | ROS2 node | Input topic/file | Planner input | Status |
|---|---|---|---|---|
| Multi-LiDAR 3D bbox CSV | `bbox_costmap_publisher` | `outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv` | `nav_msgs/OccupancyGrid` | Implemented; runtime mode `bbox` |
| PointCloud2 | `pointcloud_costmap` | `/lidar/points` | `nav_msgs/OccupancyGrid` | Implemented; runtime mode `synthetic`; can use real PointCloud2 with `use_pointcloud_costmap:=true` |
| Gazebo LiDAR LaserScan | `laserscan_costmap` | `/uav_oda/lidar_scan` | `nav_msgs/OccupancyGrid` | Implemented; runtime mode `gazebo_laserscan` |
| Synthetic or predicted-depth proxy | `depth_image_costmap` | `/camera/depth/image` | `nav_msgs/OccupancyGrid` | Implemented; runtime mode `depth_image` |
| Cached monocular predicted depth | `cached_depth_image_publisher` + `depth_image_costmap` | `data/processed/depth_sample_3_5fps.npz` -> `/camera/depth/image` | `nav_msgs/OccupancyGrid` | Implemented; runtime mode `cached_depth` |
| LiDAR bbox + cached depth fusion | `costmap_mux` | `/perception/bbox_occupancy_grid` + `/perception/depth_occupancy_grid` | `nav_msgs/OccupancyGrid` | Implemented; waits for both source grids and publishes `/perception/costmap_mux_status` in runtime mode `bbox_cached_depth_mux` |
| Gazebo depth camera | `depth_image_costmap` | `/camera/depth/image` via `ros_gz_bridge` | `nav_msgs/OccupancyGrid` | Implemented; runtime mode `gazebo_depth` |

## New Depth-Image Bridge

Files:

```text
ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/depth_image_costmap_node.py
ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/synthetic_depth_image_publisher.py
```

Supported depth encodings:

```text
32FC1   metric depth in meters
16UC1   metric depth in millimeters by default
mono8   relative depth
8UC1    relative depth
```

For relative monocular depth, high relative-depth pixels are treated as near
obstacles by default and mapped to a configurable pseudo-range.  This is useful
for predicted-depth demos, but it is not a calibrated metric-distance claim.

## Run Modes

```bash
scripts/run_ros2_costmap_demo.sh bbox astar
scripts/run_ros2_costmap_demo.sh synthetic astar
scripts/run_ros2_costmap_demo.sh depth_image astar
scripts/run_ros2_costmap_demo.sh cached_depth astar
scripts/run_ros2_costmap_demo.sh bbox_cached_depth_mux astar
scripts/run_ros2_costmap_demo.sh gazebo_depth astar
scripts/run_ros2_costmap_demo.sh gazebo_laserscan astar
scripts/run_ros2_costmap_demo.sh gazebo_fused astar
```

Runtime verification commands:

```bash
scripts/check_ros2_server_preflight.sh
scripts/verify_ros2_costmap_all_modes.sh astar
```

Single-mode fallback commands:

```bash
scripts/verify_ros2_costmap_runtime.sh bbox astar
scripts/verify_ros2_costmap_runtime.sh synthetic astar
scripts/verify_ros2_costmap_runtime.sh depth_image astar
scripts/verify_ros2_costmap_runtime.sh cached_depth astar
scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar
scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar
scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar
scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar
```

## Current Verification Status

Local static checks pass:

- Python compile for ROS2 nodes and launch files.
- Shell syntax for runner/verifier.
- ROS2 demo audit static checks.
- Artifact bundle includes the new depth bridge.
- Offline perception-to-planner contract check writes `outputs/tables/perception_to_planner_contract.csv` and proves:
  - Multi-LiDAR bbox CSV -> non-empty occupancy grid -> A* path.
  - Metric depth image -> non-empty occupancy grid -> A* path.
  - Relative predicted-depth proxy -> non-empty occupancy grid -> A* path.
  - LiDAR bbox + relative predicted-depth mux -> merged occupancy grid -> A* path.
  - LiDAR bbox + cached predicted-depth mux -> merged occupancy grid -> A* path.
- Offline planner matrix `scripts/check_perception_planner_matrix.py` writes `outputs/tables/perception_planner_matrix.csv` and proves 5 perception-derived obstacle maps x 3 planners (`astar`, `rrt`, `mppi`) all produce collision-free paths after planner inflation.
- `outputs/figures/perception_to_planner_contract.svg` visualizes the five conversion paths as obstacle maps with the resulting A* path.

Runtime ROS2/Gazebo evidence still requires a ROS2/Gazebo server.  The audit
will remain incomplete until the verifier commands above are run on that server
and `outputs/tables/ros2_demo_runtime_summary.csv` is generated.
