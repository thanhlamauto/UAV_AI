# ROS2/Gazebo/PX4 Sensor Fusion Runtime

This note describes the full sensor-to-planner-to-PX4 path added for the UAV obstacle-avoidance demo.

## Implemented Runtime Path

```text
PointCloud2 obstacle evidence
Gazebo depth image
Gazebo LaserScan
  -> pointcloud_costmap / depth_image_costmap / laserscan_costmap
  -> /perception/pointcloud_occupancy_grid
  -> /perception/depth_occupancy_grid
  -> /perception/laserscan_occupancy_grid
  -> costmap_mux
  -> /perception/occupancy_grid
  -> px4_mppi_offboard_controller
  -> MPPI receding-horizon velocity/acceleration setpoint
  -> /fmu/in/trajectory_setpoint
```

The older waypoint bridge is still available as a fallback:

```text
/perception/occupancy_grid
  -> costmap_planner
  -> /planned_path
  -> px4_waypoint_follower
  -> /fmu/in/trajectory_setpoint
```

PX4 state is bridged back into the planner by:

```text
/fmu/out/vehicle_odometry
  -> px4_odometry_bridge
  -> /odom
  -> /uav/current_pose
```

## Non-PX4 Verifier

Use this first because it verifies the fused sensor map without requiring PX4:

```bash
scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar
```

Success requires:

- `/lidar/points`
- `/camera/depth/image`
- `/uav_oda/lidar_scan`
- `/perception/pointcloud_occupancy_grid`
- `/perception/depth_occupancy_grid`
- `/perception/laserscan_occupancy_grid`
- `/perception/costmap_mux_status`
- `/perception/occupancy_grid`
- `/planned_path`
- `/uav/current_pose`
- `/odom`

For `gazebo_fused`, `/perception/costmap_mux_status` must report `state=merged` and all three source grids must have occupied cells.

## PX4 Runner

Use this only on a server/workspace where PX4 SITL, `px4_msgs`, and the PX4 ROS2 DDS bridge are available:

```bash
scripts/run_ros2_gazebo_fused_px4.sh astar
```

The runner now defaults to the MPPI local-controller path:

```text
PX4_CONTROLLER=mppi
```

Fallback modes:

```bash
PX4_CONTROLLER=waypoint scripts/run_ros2_gazebo_fused_px4.sh astar
PX4_CONTROLLER=none scripts/run_ros2_gazebo_fused_px4.sh astar
```

If PX4 is already running separately, keep:

```bash
START_PX4=0 scripts/run_ros2_gazebo_fused_px4.sh astar
```

If the workspace is configured to launch PX4 from this repo:

```bash
START_PX4=1 scripts/run_ros2_gazebo_fused_px4.sh astar
```

## Scope

This is a ROS2/Gazebo/PX4 offboard-control integration path, not a flight-ready real-world controller. The upgraded path now computes MPPI velocity/acceleration setpoints over a fused occupancy grid, while altitude is still held at a fixed setpoint. The next step for a stricter PX4/Gazebo demo is to mount the depth/LiDAR sensors on the PX4 vehicle model and record closed-loop SITL evidence.
