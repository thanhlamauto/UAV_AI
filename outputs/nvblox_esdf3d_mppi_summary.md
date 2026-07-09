# NVBlox 3D TSDF + PointCloud2 Local ESDF/MPPI Summary

## Verified Pipeline

```text
organized PointCloud2 LiDAR, 360 x 24 beams
  -> NVBlox runtime with esdf_mode=3d and TSDF layer publishing
  -> planner-side local 3D occupancy ESDF from the same PointCloud2
  -> MPPI trajectory in x,y,z
  -> /planned_path_3d_from_nvblox
```

This is a Level-3 bridge for the project: the planner no longer uses a 2D
fixed-altitude costmap in this verifier. It queries a 3D ESDF volume derived
from LiDAR points.

## Runtime Evidence

Verifier:

```bash
scripts/verify_nvblox_pointcloud_esdf3d_mppi.sh
```

Evidence files:

```text
outputs/nvblox_esdf3d_topics.txt
outputs/nvblox_esdf3d_lidar_points_echo.txt
outputs/nvblox_esdf3d_tsdf_layer_info.txt
outputs/nvblox_esdf3d_planned_path_echo.txt
outputs/nvblox_esdf3d_status_echo.txt
outputs/nvblox_esdf3d_mppi_ros2.log
```

Status:

```text
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
```

NVBlox runtime evidence:

```text
/nvblox_node/tsdf_layer
Topic type: nvblox_msgs/msg/VoxelBlockLayer
ros/lidar ~= 10.1 Hz
ros/update_esdf ~= 4.9 Hz
```

## Important Scope Note

The verifier proves:

- PointCloud2 LiDAR can feed NVBlox 3D runtime.
- The same PointCloud2 stream can be converted into a local 3D ESDF.
- MPPI can plan a safe continuous x,y,z path against that 3D ESDF.

It does not yet prove:

- Direct planner queries into NVBlox's internal 3D ESDF volume.
- Closed-loop PX4/offboard control.
- A moving MAV building a persistent 3D map while tracking setpoints.
