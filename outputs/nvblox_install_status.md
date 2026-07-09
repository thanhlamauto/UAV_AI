# NVBlox Server Install Status

Date: 2026-06-23

Server tested:

```text
ssh -p 62664 root@70.30.158.46
GPU: NVIDIA GeForce RTX 2060 SUPER, 8192 MiB
ROS_DISTRO: jazzy
```

## Verified

The Isaac ROS/NVBlox apt install completed successfully.

Evidence:

```text
outputs/setup_isaac_ros_nvblox_apt.log
outputs/full_3d_esdf_stack_readiness_server.txt
```

Installed ROS2 packages detected:

```text
nvblox_ros
nvblox_msgs
isaac_ros_nvblox
ros_gz_bridge
rviz2
```

Detected NVBlox executables:

```text
nvblox_ros nvblox_node
nvblox_ros fuser_node
nvblox_ros fuse_cusfm
```

The repo ROS2 package also builds on the server and publishes the synthetic
ESDF/MPPI 3D path replay:

```text
/planned_path_3d
/esdf3d_mppi_markers
```

The server also ran a lightweight NVBlox synthetic-depth mapping smoke test
through:

```text
ros2 launch uav_oda_ros2_demo nvblox_synthetic_depth.launch.py
```

Verified topics:

```text
/front_stereo_camera/depth/ground_truth
/front_stereo_camera/left/camera_info
/nvblox_node/tsdf_layer
/nvblox_node/mesh
/nvblox_node/static_esdf_pointcloud
/nvblox_node/static_map_slice
```

Evidence files:

```text
outputs/nvblox_synthetic_depth_topics.txt
outputs/nvblox_tsdf_layer_echo.txt
outputs/nvblox_mesh_echo.txt
outputs/nvblox_synthetic_depth_ros2.log
```

Measured from the NVBlox log:

```text
depth callback/integration ~= 10.1 Hz
ESDF update ~= 4.9 Hz
ESDF integration delay ~= 0.078 s
```

## DistanceMapSlice Planner Bridge

The server now verifies a minimal perception-to-planner bridge:

```text
synthetic metric depth + CameraInfo + TF
  -> NVBlox static_map_slice / DistanceMapSlice
  -> nvblox_distance_slice_planner
  -> /planned_path_from_nvblox
```

Command:

```bash
scripts/verify_nvblox_distance_slice_planner.sh
```

Evidence files:

```text
outputs/nvblox_distance_slice_topics.txt
outputs/nvblox_distance_slice_static_map_slice_echo.txt
outputs/nvblox_distance_slice_planned_path_echo.txt
outputs/nvblox_distance_slice_status_echo.txt
outputs/nvblox_distance_slice_planner_ros2.log
```

Verified result:

```text
DistanceMapSlice = 105 x 160 cells at 0.05 m
planner = mppi
waypoints = 56
min_distance_m = 0.45
safety_violation = false
compute_time_ms ~= 215.6
```

## Gazebo Depth Bridge

The bridge has also been verified using the package's Gazebo indoor depth camera:

```text
Gazebo indoor obstacle world
  -> /camera/depth/image
  -> depth_image_republisher + CameraInfo
  -> NVBlox static_map_slice / DistanceMapSlice
  -> nvblox_distance_slice_planner
  -> /planned_path_from_nvblox
```

Command:

```bash
scripts/verify_nvblox_gazebo_depth_slice_planner.sh
```

Evidence files:

```text
outputs/nvblox_gazebo_depth_slice_topics.txt
outputs/nvblox_gazebo_depth_image_echo.txt
outputs/nvblox_gazebo_depth_static_map_slice_echo.txt
outputs/nvblox_gazebo_depth_planned_path_echo.txt
outputs/nvblox_gazebo_depth_status_echo.txt
outputs/nvblox_gazebo_depth_planner_ros2.log
```

Verified result:

```text
Gazebo depth echo = 890 bytes
DistanceMapSlice echo = 1373 bytes
MPPI path echo = 13926 bytes
planner = mppi
min_distance_m = 2.0
safety_violation = false
compute_time_ms ~= 272.5
NVBlox depth rate ~= 9.4 Hz
NVBlox ESDF update ~= 4.9 Hz
```

## Not Yet Verified

Do not claim these as completed yet:

```text
LiDAR/PointCloud2 -> NVBlox TSDF/ESDF live map
3D NVBlox ESDF volume -> MPPI clearance query
PX4/Gazebo closed-loop UAV control with fused ESDF
```

## Next Command Target

The next server run should launch `nvblox_ros nvblox_node` or the
`nvblox_examples_bringup` perception launch with depth image, camera info,
pose/tf and later point cloud topics. The required proof is non-empty NVBlox
outputs such as mesh, occupancy/TSDF layer, ESDF point cloud or distance slice.
