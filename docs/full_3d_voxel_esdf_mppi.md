# Full 3D Voxel/ESDF Roadmap for Indoor MAV Obstacle Avoidance

This document upgrades the current demo story from 2D occupancy-grid planning
to a realistic 3D mapping and local-planning stack:

```text
RGB-D / depth / LiDAR / point cloud
  -> 3D voxel map
  -> TSDF / occupancy layer
  -> ESDF distance field
  -> continuous-state MPPI local planner
  -> MAV trajectory in an indoor A-to-B task
```

## Decision

Use **NVBlox** as the primary implementation target.

Why:

- It is designed for robotics scene reconstruction from depth images and/or 3D
  LiDAR scans.
- It builds a voxel TSDF map and can construct an ESDF for planning.
- It is optimized for NVIDIA GPUs and Jetson/discrete GPU systems.
- It has ROS2/Isaac ROS integration and publishes planning-oriented map outputs.

Keep these as secondary references:

- **Voxblox:** strongest conceptual reference for MAV ESDF planning, but the
  public stack is older and mostly ROS1-oriented.
- **OctoMap:** useful 3D occupancy baseline, but it gives occupancy rather than
  direct signed/Euclidean distance gradients for MPPI.
- **FIESTA/EDT:** useful algorithmic reference for fast ESDF/EDT updates if we
  later implement a custom lightweight 3D planner backend.

## Important Clarification

Even the "continuous" robot stack still stores the world in discrete voxels.
The improvement over the current grid demo is:

```text
2D binary grid:
  occupied / free cell lookup

3D ESDF:
  query metric distance-to-obstacle at continuous x,y,z by interpolation
```

So the planner state and controls become continuous, while the map is a
finite-resolution metric field.

## Target Architecture

### Mapping

Input topics:

```text
/camera/depth/image
/camera/depth/camera_info
/lidar/points
/tf or /pose
```

NVBlox outputs to verify:

```text
/nvblox_node/mesh
/nvblox_node/mesh_marker
/nvblox_node/tsdf_layer
/nvblox_node/occupancy_layer
/nvblox_node/static_esdf_pointcloud
/nvblox_node/static_map_slice
```

The `static_map_slice` is a 2D ESDF slice for Nav2-style planning.  For the MAV
story, it is acceptable as an intermediate milestone, but the final target is
to query a 3D ESDF volume or a local 3D distance point cloud around the UAV.

### Planning

Replace the current occupancy-grid MPPI cost:

```text
cost += occupied_cell_penalty(x, y)
```

with ESDF-based continuous collision cost:

```text
d = esdf_distance(x, y, z)
cost += clearance_weight * max(0, safety_radius - d)^2
```

Recommended state and control for the first real MAV version:

```text
state   = [x, y, z, vx, vy, vz, yaw]
control = [ax, ay, az, yaw_rate]
```

Minimum MPPI cost terms:

```text
goal distance
ESDF clearance penalty
unknown-space penalty
velocity limit
acceleration/control effort
smoothness / jerk proxy
terminal goal cost
```

## Implementation Phases

### Phase 0 - Local ESDF/MPPI Proof Already Implemented

Goal: prove the project can optimize against a 3D distance field before
depending on NVBlox.

Implemented files:

```text
src/esdf3d.py
src/planners/mppi_3d_esdf.py
experiments/run_3d_esdf_mppi_demo.py
scripts/check_3d_esdf_mppi_demo.py
ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/esdf3d_mppi_replay_node.py
ros2_ws/src/uav_oda_ros2_demo/launch/esdf3d_mppi_replay.launch.py
```

Run:

```bash
scripts/run_full_3d_esdf_phase0.sh
```

Current evidence:

```text
outputs/tables/indoor_3d_esdf_mppi_metrics.csv
outputs/figures/indoor_3d_esdf_mppi_path.png
outputs/figures/indoor_3d_esdf_mppi_slice.png
outputs/indoor_3d_esdf_mppi_summary.md
data/processed/esdf3d/indoor_demo_esdf_mppi.npz
```

Current verified result:

```text
collision = 0
safety_violation = 0
min_esdf_distance_m = 0.6783
min_safety_margin_m = 0.2583
altitude_change_m = 0.4499
planner_compute_time_ms = 123.638 on the latest local run
```

ROS2 replay:

```bash
cd ros2_ws
colcon build --packages-select uav_oda_ros2_demo
source install/setup.bash
ros2 launch uav_oda_ros2_demo esdf3d_mppi_replay.launch.py \
  npz_path:=/absolute/path/to/data/processed/esdf3d/indoor_demo_esdf_mppi.npz
```

This phase is synthetic but important: it replaces binary occupied/free lookup
with continuous ESDF queries in `[x, y, z]`. Compute time varies by machine and
whether SciPy EDT is installed; the current gate checks correctness/safety, not
a fixed latency target.

Server evidence from the Vast RTX 2060 SUPER instance:

```text
outputs/tables/indoor_3d_esdf_mppi_metrics_server.csv
planner_compute_time_ms = 357.047
collision = 0
safety_violation = 0
altitude_change_m = 0.4499
```

ROS2 replay evidence:

```text
outputs/esdf3d_mppi_replay_topics.txt
  /planned_path_3d
  /esdf3d_mppi_markers

outputs/esdf3d_mppi_replay_ros2.log
  Loaded 72 3D MPPI waypoints; min_esdf=0.6783
```

NVBlox live depth-mapping smoke test:

```bash
cd ros2_ws
colcon build --packages-select uav_oda_ros2_demo
source install/setup.bash
ros2 launch uav_oda_ros2_demo nvblox_synthetic_depth.launch.py
```

This launch publishes synthetic metric depth plus CameraInfo and static TF, then
runs `nvblox_ros` as a composable node. Server evidence:

```text
outputs/nvblox_synthetic_depth_topics.txt
  /front_stereo_camera/depth/ground_truth
  /front_stereo_camera/left/camera_info
  /nvblox_node/tsdf_layer
  /nvblox_node/mesh
  /nvblox_node/static_esdf_pointcloud
  /nvblox_node/static_map_slice

outputs/nvblox_tsdf_layer_echo.txt = 2389 bytes
outputs/nvblox_mesh_echo.txt = 3704 bytes
outputs/nvblox_synthetic_depth_ros2.log
  ros/depth ~= 10.1 Hz
  ros/update_esdf ~= 4.9 Hz
  ros/esdf_integration mean delay ~= 0.078 s
```

This verifies that the server can run the NVIDIA mapping stack and publish a
TSDF layer from a depth stream. It is not yet the final closed loop because the
NVBlox ESDF output has not been converted into the MPPI clearance query.

NVBlox `DistanceMapSlice` bridge to planner:

```bash
scripts/verify_nvblox_distance_slice_planner.sh
```

This verifier runs:

```text
synthetic metric depth + CameraInfo + TF
  -> NVBlox static_map_slice / DistanceMapSlice
  -> nvblox_distance_slice_planner
  -> /planned_path_from_nvblox
```

Server evidence:

```text
outputs/nvblox_distance_slice_static_map_slice_echo.txt
  resolution = 0.05 m
  width x height = 105 x 160
  slice z = 1.2 m

outputs/nvblox_distance_slice_planned_path_echo.txt
  Path published on /planned_path_from_nvblox

outputs/nvblox_distance_slice_status_echo.txt
  state = planned
  planner = mppi
  waypoints = 56
  min_distance_m = 0.45
  safety_violation = false
  compute_time_ms ~= 215.6
```

This is the first verified perception-to-planner bridge using an NVBlox
distance output, but it is still a 2D ESDF slice bridge and still uses synthetic
depth.

Gazebo depth version:

```bash
scripts/verify_nvblox_gazebo_depth_slice_planner.sh
```

This verifier replaces the synthetic depth publisher with the package's
headless Gazebo indoor depth camera:

```text
Gazebo /camera/depth/image
  -> depth_image_republisher + CameraInfo
  -> NVBlox static_map_slice / DistanceMapSlice
  -> nvblox_distance_slice_planner
  -> /planned_path_from_nvblox
```

Server evidence:

```text
outputs/nvblox_gazebo_depth_image_echo.txt = 890 bytes
outputs/nvblox_gazebo_depth_static_map_slice_echo.txt = 1373 bytes
outputs/nvblox_gazebo_depth_planned_path_echo.txt = 13926 bytes
outputs/nvblox_gazebo_depth_status_echo.txt
  state = planned
  planner = mppi
  min_distance_m = 2.0
  safety_violation = false
  compute_time_ms ~= 272.5

outputs/nvblox_gazebo_depth_planner_ros2.log
  ros/depth ~= 9.4 Hz
  ros/update_esdf ~= 4.9 Hz
```

This moves the bridge beyond a purely synthetic ROS publisher: NVBlox is now
fed by a Gazebo depth sensor stream, then its distance slice drives the MPPI
planner. It is still a 2D slice bridge, not full 3D ESDF volume control.

PointCloud2 LiDAR version:

```bash
scripts/verify_nvblox_pointcloud_slice_planner.sh
```

This verifier feeds NVBlox with an organized synthetic LiDAR cloud shaped like a
small 3D range image:

```text
organized PointCloud2, 360 x 16 beams
  -> NVBlox lidar_image / TSDF / static_map_slice
  -> nvblox_distance_slice_planner
  -> /planned_path_from_nvblox
```

Server evidence:

```text
outputs/nvblox_pointcloud_lidar_points_echo.txt
  height = 16
  width = 360

outputs/nvblox_pointcloud_static_map_slice_echo.txt
  resolution = 0.05 m
  width x height = 169 x 288
  slice z = 1.2 m

outputs/nvblox_pointcloud_status_echo.txt
  state = planned
  planner = mppi
  waypoints = 56
  known_cells = 10505
  min_distance_m = 2.0
  safety_violation = false
  compute_time_ms ~= 239.5

outputs/nvblox_pointcloud_planner_ros2.log
  ros/lidar ~= 10.1 Hz
  ros/pointcloud_callback ~= 10.1 Hz
  ros/update_esdf ~= 4.9 Hz
  ros/pointcloud_integration delay ~= 0.010 s
```

This is the first verified LiDAR/PointCloud2-to-NVBlox-to-planner path in the
project. It still uses NVBlox's 2D `DistanceMapSlice` as the planner interface.

PointCloud2 local 3D ESDF version:

```bash
scripts/verify_nvblox_pointcloud_esdf3d_mppi.sh
```

This verifier keeps NVBlox running in 3D mode while the planner builds a local
3D ESDF from the same organized LiDAR PointCloud2 stream:

```text
organized PointCloud2, 360 x 24 beams
  -> NVBlox runtime with esdf_mode=3d and /nvblox_node/tsdf_layer
  -> planner-side local 3D occupancy ESDF
  -> MPPI in x,y,z
  -> /planned_path_3d_from_nvblox
```

Server evidence:

```text
outputs/nvblox_esdf3d_status_echo.txt
  state = planned
  planner = mppi_3d_esdf_pointcloud
  source_mode = pointcloud_occupancy_esdf
  grid_shape = [43, 33, 29]
  resolution_m = 0.12
  known_voxels = 41151
  esdf_z_span_m = 3.36
  min_esdf_distance_m = 0.5648
  safety_radius_m = 0.45
  safety_violation = false
  path_length_m = 2.9033
  altitude_change_m = 0.2499
  compute_time_ms = 18.433

outputs/nvblox_esdf3d_tsdf_layer_info.txt
  /nvblox_node/tsdf_layer type = nvblox_msgs/msg/VoxelBlockLayer

outputs/nvblox_esdf3d_mppi_ros2.log
  ros/lidar ~= 10.1 Hz
  ros/update_esdf ~= 4.9 Hz
```

This is the current Level-3 bridge. It is honest to describe it as
`PointCloud2 -> local 3D ESDF -> MPPI` running alongside NVBlox 3D TSDF/ESDF
runtime. It is not yet a direct query into NVBlox's internal 3D ESDF volume.

### Phase A - NVBlox Mapping Only

Goal: prove the server can build a 3D voxel map from sensor streams.

Current status:

```text
outputs/full_3d_esdf_stack_readiness_server.txt
GPU/ROS2/Gazebo/colcon = PASS
nvblox_ros/nvblox_msgs/isaac_ros_nvblox = PASS
ros_gz_bridge/rviz2 = PASS
```

Detected NVBlox runtime entry points:

```text
nvblox_ros nvblox_node
nvblox_ros fuser_node
nvblox_ros fuse_cusfm
```

Remaining tasks:

1. Replace the synthetic/Gazebo smoke-test stream with rosbag or simulator RGB-D
   and LiDAR topics from an indoor MAV scenario.
2. Replace the planner-side local ESDF bridge with direct NVBlox 3D ESDF
   volume queries if an API/export path is added.
3. Record evidence:

```text
ros2 topic list
ros2 topic hz /nvblox_node/mesh
ros2 topic hz /nvblox_node/static_esdf_pointcloud
ros2 topic hz /nvblox_node/static_map_slice
rosbag with depth, point cloud, pose, mesh, ESDF
RViz/Foxglove screen recording
```

Success criteria:

- `mesh` and `mesh_marker` update while the MAV moves.
- `tsdf_layer` or `occupancy_layer` is non-empty.
- `static_esdf_pointcloud` or `static_map_slice` is non-empty.
- Evidence video shows 3D reconstruction, not only a 2D costmap.

### Phase B - ESDF Slice to Current Planner

Goal: bridge NVBlox output into the current project before writing full 3D MPPI.

Implemented smoke-test:

```text
ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/nvblox_distance_slice_planner_node.py
ros2_ws/src/uav_oda_ros2_demo/launch/nvblox_distance_slice_planner.launch.py
scripts/verify_nvblox_distance_slice_planner.sh
```

Remaining tasks:

1. Compare NVBlox slice planning against the current binary occupancy-grid demo.
2. Move from `DistanceMapSlice` to a local 3D ESDF query for MPPI.
3. Add the same bridge to a moving indoor MAV simulation after the static
   sensor smoke tests are stable.

Success criteria:

- Same A-to-B indoor scenario runs using NVBlox distance output. The synthetic
  depth and Gazebo depth smoke-tests already satisfy this at interface level.
- Report shows smoother/less conservative behavior than binary occupancy.
- MPPI cost uses metric clearance instead of binary occupied/free cells.

### Phase C - Full 3D ESDF MPPI

Goal: remove the fixed-altitude simplification.

Tasks:

1. Maintain a local 3D ESDF query structure around the UAV.
2. Roll out MPPI trajectories in `[x, y, z]`.
3. Penalize unknown voxels and low-clearance voxels.
4. Publish a 3D path/trajectory for visualization.

Success criteria:

- UAV trajectory changes altitude when useful.
- Minimum 3D clearance is reported.
- Planner does not require a known global grid; it reacts to local sensor-built
  ESDF.

### Phase D - PX4 / Controller Integration

Goal: convert the MPPI trajectory into setpoints for a flight stack.

Tasks:

1. Add PX4 SITL only after Phase C is stable.
2. Send position/velocity/yaw setpoints.
3. Verify tracking error and safety distance.

Success criteria:

- The planned 3D trajectory is followed by a simulated UAV, not only a marker.
- Logs include tracking error, minimum ESDF clearance, compute time and failure
  cases.

## Server Strategy

Fastest credible route:

```text
ROS2 Jazzy + NVIDIA GPU + Isaac ROS NVBlox + Gazebo/rosbag sensor streams
```

Avoid starting with full Isaac Sim unless necessary. Isaac Sim is stronger for
photorealistic indoor sensors, but heavier to run on rented containers. The
first milestone should be NVBlox reconstruction from existing ROS2 depth and
point-cloud streams.

Server readiness:

```bash
scripts/check_full_3d_esdf_stack_readiness.sh
```

If `ros-jazzy-isaac-ros-nvblox` is missing, configure the NVIDIA Isaac ROS apt
repository and install NVBlox:

```bash
scripts/setup_isaac_ros_nvblox_apt.sh
scripts/check_full_3d_esdf_stack_readiness.sh
```

This follows NVIDIA's Isaac ROS release-4.4 apt setup for Ubuntu Noble/Jazzy.
Set `INSTALL_NAV2=1` only if the next server needs Nav2 bringup packages; the
default install keeps the dependency footprint smaller.

## Mentor-Facing Story

Short version:

> The project is moving from 2D costmap planning to a 3D voxel/ESDF stack. The
> next system will use NVBlox to reconstruct a TSDF/ESDF map from depth and
> LiDAR, then MPPI will optimize a continuous 3D MAV trajectory by querying
> distance-to-obstacle instead of checking occupied grid cells.

What this adds:

- Real 3D obstacle representation.
- Distance-to-obstacle field for smooth optimization.
- Better fit for indoor MAV navigation where the map is built online from
  sensors.
- Cleaner technical distinction from the colleague's simpler detection/planning
  demo.

## References

- NVIDIA Isaac ROS NVBlox documentation: https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/index.html
- NVBlox ROS topics/services: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/topics_and_services.html
- NVBlox ROS parameters: https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/api/parameters.html
- NVBlox RealSense tutorial: https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_realsense.html
- NVBlox Isaac Sim tutorial: https://nvidia-isaac-ros.github.io/v/release-3.1/concepts/scene_reconstruction/nvblox/tutorials/tutorial_isaac_sim.html
- OctoMap: https://octomap.github.io/
- Voxblox: https://github.com/ethz-asl/voxblox
