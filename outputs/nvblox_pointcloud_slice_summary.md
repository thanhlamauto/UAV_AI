# NVBlox PointCloud2 LiDAR Slice Planner Smoke Test

Command:

```bash
scripts/verify_nvblox_pointcloud_slice_planner.sh
```

Verified pipeline:

```text
organized synthetic PointCloud2 LiDAR, 360 x 16 beams
  -> nvblox_ros with use_lidar=true
  -> /nvblox_node/lidar_image, /nvblox_node/tsdf_layer, /nvblox_node/static_map_slice
  -> nvblox_distance_slice_planner
  -> /planned_path_from_nvblox
```

Server evidence copied to this folder:

```text
outputs/nvblox_pointcloud_slice_topics.txt
outputs/nvblox_pointcloud_lidar_points_echo.txt
outputs/nvblox_pointcloud_static_map_slice_echo.txt
outputs/nvblox_pointcloud_planned_path_echo.txt
outputs/nvblox_pointcloud_status_echo.txt
outputs/nvblox_pointcloud_planner_ros2.log
```

Result:

```text
DistanceMapSlice: 169 x 288 cells at 0.05 m
planner: MPPI
waypoints: 56
known_cells: 10505
min_distance_m: 2.0
safety_violation: false
compute_time_ms: 239.518
NVBlox lidar rate: ~10.1 Hz
NVBlox ESDF update rate: ~4.9 Hz
pointcloud integration delay: ~0.010 s
```

Scope note: this verifies LiDAR/PointCloud2 input into NVBlox and planner use of
the resulting DistanceMapSlice. It is still a 2D ESDF slice interface, not a
full 3D ESDF volume queried by MPPI.
