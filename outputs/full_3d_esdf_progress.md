# Full 3D Voxel/ESDF Progress

The Level-3 direction is now partially implemented locally.

## Implemented

```text
3D voxel occupancy map
  -> signed distance field
  -> continuous xyz MPPI optimizer
  -> metrics/figures/NPZ artifact
  -> optional ROS2 Path replay node
```

## Evidence

| Artifact | Path |
|---|---|
| Metrics | `outputs/tables/indoor_3d_esdf_mppi_metrics.csv` |
| 3D path figure | `outputs/figures/indoor_3d_esdf_mppi_path.png` |
| ESDF slice figure | `outputs/figures/indoor_3d_esdf_mppi_slice.png` |
| Summary | `outputs/indoor_3d_esdf_mppi_summary.md` |
| ESDF/path data | `data/processed/esdf3d/indoor_demo_esdf_mppi.npz` |
| Server metrics | `outputs/tables/indoor_3d_esdf_mppi_metrics_server.csv` |
| Server stack readiness | `outputs/full_3d_esdf_stack_readiness_server.txt` |
| NVBlox install log | `outputs/setup_isaac_ros_nvblox_apt.log` |
| ROS2 replay topics | `outputs/esdf3d_mppi_replay_topics.txt` |
| ROS2 path echo | `outputs/esdf3d_mppi_path_echo.txt` |
| ROS2 marker echo | `outputs/esdf3d_mppi_markers_echo.txt` |
| NVBlox synthetic depth topics | `outputs/nvblox_synthetic_depth_topics.txt` |
| NVBlox TSDF echo | `outputs/nvblox_tsdf_layer_echo.txt` |
| NVBlox mesh echo | `outputs/nvblox_mesh_echo.txt` |
| NVBlox runtime log | `outputs/nvblox_synthetic_depth_ros2.log` |
| NVBlox slice bridge topics | `outputs/nvblox_distance_slice_topics.txt` |
| NVBlox DistanceMapSlice echo | `outputs/nvblox_distance_slice_static_map_slice_echo.txt` |
| NVBlox-planned path echo | `outputs/nvblox_distance_slice_planned_path_echo.txt` |
| NVBlox planner status | `outputs/nvblox_distance_slice_status_echo.txt` |
| Gazebo-depth NVBlox topics | `outputs/nvblox_gazebo_depth_slice_topics.txt` |
| Gazebo depth image echo | `outputs/nvblox_gazebo_depth_image_echo.txt` |
| Gazebo-depth NVBlox slice echo | `outputs/nvblox_gazebo_depth_static_map_slice_echo.txt` |
| Gazebo-depth planned path echo | `outputs/nvblox_gazebo_depth_planned_path_echo.txt` |
| Gazebo-depth planner status | `outputs/nvblox_gazebo_depth_status_echo.txt` |
| PointCloud2 local 3D ESDF summary | `outputs/nvblox_esdf3d_mppi_summary.md` |
| PointCloud2 local 3D ESDF status | `outputs/nvblox_esdf3d_status_echo.txt` |
| PointCloud2 local 3D ESDF path | `outputs/nvblox_esdf3d_planned_path_echo.txt` |
| NVBlox 3D TSDF topic info | `outputs/nvblox_esdf3d_tsdf_layer_info.txt` |

Latest metrics:

```text
collision = 0
safety_violation = 0
min_esdf_distance_m = 0.6783
min_safety_margin_m = 0.2583
altitude_change_m = 0.4499
path_length_m = 7.6455
planner_compute_time_ms = 123.638
cost_reduction_pct = 99.86
```

Server run on the Vast RTX 2060 SUPER instance produced the same safety result:

```text
outputs/tables/indoor_3d_esdf_mppi_metrics_server.csv
planner_compute_time_ms = 357.047
altitude_change_m = 0.4499
```

The ROS2 replay package was also built and tested on the same server. It loaded
the ESDF/MPPI NPZ artifact and published a 3D path plus visualization markers:

```text
/planned_path_3d
/esdf3d_mppi_markers
Loaded 72 3D MPPI waypoints; min_esdf=0.6783
```

The echoed messages were non-empty:

```text
outputs/esdf3d_mppi_path_echo.txt = 18725 bytes
outputs/esdf3d_mppi_markers_echo.txt = 7948 bytes
```

The same server also ran a lightweight NVBlox live-mapping smoke test from a
synthetic metric depth image plus CameraInfo and static TF:

```text
/front_stereo_camera/depth/ground_truth
/front_stereo_camera/left/camera_info
/nvblox_node/tsdf_layer
/nvblox_node/mesh
/nvblox_node/static_esdf_pointcloud
/nvblox_node/static_map_slice
```

Evidence:

```text
outputs/nvblox_tsdf_layer_echo.txt = 2389 bytes
outputs/nvblox_mesh_echo.txt = 3704 bytes
NVBlox depth callback rate ~= 10.1 Hz
NVBlox ESDF update rate ~= 4.9 Hz
```

The next bridge milestone is now verified as well:

```text
synthetic metric depth + CameraInfo + TF
  -> NVBlox static_map_slice / DistanceMapSlice
  -> nvblox_distance_slice_planner
  -> /planned_path_from_nvblox
```

Evidence from `scripts/verify_nvblox_distance_slice_planner.sh`:

```text
/nvblox_node/static_map_slice
/planned_path_from_nvblox
/nvblox_distance_slice_planner/status
DistanceMapSlice: 105 x 160 at 0.05 m resolution
MPPI path: 56 waypoints
status: planned
min_distance_m: 0.45
safety_violation: false
compute_time_ms: 215.572
```

The bridge now also runs with the Gazebo depth camera from the indoor obstacle
world rather than only the synthetic depth publisher:

```text
Gazebo indoor depth camera
  -> /camera/depth/image
  -> depth_image_republisher + CameraInfo
  -> NVBlox static_map_slice / DistanceMapSlice
  -> nvblox_distance_slice_planner
  -> /planned_path_from_nvblox
```

Evidence from `scripts/verify_nvblox_gazebo_depth_slice_planner.sh`:

```text
/camera/depth/image
/front_stereo_camera/depth/ground_truth
/nvblox_node/static_map_slice
/planned_path_from_nvblox
/nvblox_distance_slice_planner/status
Gazebo depth echo: 890 bytes
DistanceMapSlice echo: 1373 bytes
MPPI path echo: 13926 bytes
status: planned
min_distance_m: 2.0
safety_violation: false
compute_time_ms: 272.5
NVBlox depth rate ~= 9.4 Hz
NVBlox ESDF update rate ~= 4.9 Hz
```

Validation command:

```bash
.venv/bin/python scripts/check_3d_esdf_mppi_demo.py
```

## What This Means

The planner no longer needs a binary 2D costmap for this demo. It evaluates
continuous 3D points against an ESDF:

```text
d = esdf_distance(x, y, z)
cost += max(0, safety_radius - d)^2
```

This is the correct interface for replacing the synthetic ESDF with NVBlox
ESDF output on a GPU/ROS2 server.

## Server/NVBlox Status

The NVIDIA/ROS2 side is now installed on the server:

```text
GPU/ROS2/Gazebo/colcon = PASS
ros_gz_bridge/rviz2 = PASS
nvblox_ros/nvblox_msgs/isaac_ros_nvblox = PASS
```

NVBlox executables detected:

```text
nvblox_ros nvblox_node
nvblox_ros fuser_node
nvblox_ros fuse_cusfm
```

The project now also has a verified LiDAR/PointCloud2 path into NVBlox:

```text
organized PointCloud2, 360 x 16 beams
  -> NVBlox lidar_image / TSDF / DistanceMapSlice
  -> MPPI planner
```

Evidence from `scripts/verify_nvblox_pointcloud_slice_planner.sh`:

```text
/lidar/points
/nvblox_node/lidar_image
/nvblox_node/tsdf_layer
/nvblox_node/static_map_slice
/planned_path_from_nvblox
/nvblox_distance_slice_planner/status
DistanceMapSlice: 169 x 288 at 0.05 m
status: planned
waypoints: 56
known_cells: 10505
min_distance_m: 2.0
safety_violation: false
compute_time_ms: 239.518
NVBlox lidar rate ~= 10.1 Hz
NVBlox ESDF update rate ~= 4.9 Hz
pointcloud integration delay ~= 0.010 s
```

The Level-3 bridge now has a full 3D local ESDF verifier:

```text
organized PointCloud2, 360 x 24 beams
  -> NVBlox runtime with esdf_mode=3d and /nvblox_node/tsdf_layer
  -> planner-side local 3D occupancy ESDF
  -> MPPI in x,y,z
  -> /planned_path_3d_from_nvblox
```

Evidence from `scripts/verify_nvblox_pointcloud_esdf3d_mppi.sh`:

```text
state: planned
source_mode: pointcloud_occupancy_esdf
grid_shape: [43, 33, 29]
resolution_m: 0.12
known_voxels: 41151
esdf_z_span_m: 3.36
min_esdf_distance_m: 0.5648
safety_radius_m: 0.45
safety_violation: false
path_length_m: 2.9033
altitude_change_m: 0.2499
compute_time_ms: 18.433
NVBlox LiDAR rate ~= 10.1 Hz
NVBlox ESDF update rate ~= 4.9 Hz
```

This is still not a direct query into NVBlox's internal 3D ESDF volume. It is a
planner-side local 3D ESDF built from the same PointCloud2 stream while NVBlox
3D TSDF/ESDF runtime runs in parallel.

The remaining evidence must come from moving from 2D ESDF slice planning to a
fuller 3D ESDF query and closing the loop with a moving UAV/controller:

```text
depth / LiDAR / point cloud
  -> NVBlox TSDF/occupancy
  -> NVBlox ESDF volume
  -> MPPI 3D clearance query
```

Use:

```bash
scripts/check_full_3d_esdf_stack_readiness.sh
```

on the GPU server before attempting NVBlox launch.

Important: the project now has both a verified NVBlox `DistanceMapSlice` to MPPI
bridge and a verified PointCloud2-derived local 3D ESDF to MPPI bridge. Do not
yet claim full direct NVBlox 3D ESDF closed-loop UAV control until the planner
queries NVBlox's 3D ESDF volume directly and sends setpoints to a moving
UAV/controller.
