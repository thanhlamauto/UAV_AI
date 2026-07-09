# ROS2/Gazebo Runtime Verification Handoff

This note records the server-side evidence collected on the Vast instance
`root@70.30.158.46 -p 62664`.

## Verified Result

The fused Gazebo runtime was executed successfully with the project MPPI planner:

```text
Gazebo PointCloud2 + Gazebo depth image + Gazebo LaserScan
  -> source OccupancyGrid costmaps
  -> costmap_mux
  -> /perception/occupancy_grid
  -> MPPI path
  -> kinematic UAV marker + /odom
```

Main evidence folder:

```text
outputs/ros2_demo_runtime/gazebo_fused_mppi_20260623_112924/
```

Local convenience video:

```text
outputs/videos/ros2_gazebo_fused_sensor_runtime_mppi.mp4
```

Runtime status:

| Mode | Planner | Status | Topics | Messages | Mux |
|---|---|---|---:|---:|---|
| gazebo_fused | mppi | passed | 13/13 | 13/13 | passed |

The mux validation log confirms:

```text
state=merged
received_topics=[
  perception/depth_occupancy_grid,
  perception/laserscan_occupancy_grid,
  perception/pointcloud_occupancy_grid
]
merged_occupied_cells=1091
```

## 3D Visualization

The same A-to-B indoor scenario was rendered as a 3-column 3D MPPI visualization:

```text
outputs/videos/uav_3d_fused_sensor_multiview_mppi.mp4
```

Video metadata:

```text
3840 x 720, 12 s, 24 fps
```

Columns:

| Column | View |
|---|---|
| 1 | External indoor lab/corridor view with UAV, obstacles, point-cloud evidence and fused grid. |
| 2 | Drone POV inside the corridor with relative-depth and LiDAR-scan inset. |
| 3 | Top-down indoor layout with planner path and fused occupancy cells. |

Render check evidence:

```text
outputs/figures/uav_3d_sim_desktop.png
outputs/figures/uav_3d_sim_mobile.png
outputs/figures/uav_3d_sim_render_check.json
```

## What This Proves

- The planner input is not a hand-drawn static grid in the ROS2 runtime.
- PointCloud2, depth and LaserScan each produce source costmaps.
- `costmap_mux` merges the source maps into one planner costmap.
- MPPI consumes the fused costmap and publishes `nav_msgs/Path`.
- The simulated UAV marker follows the path, producing `/uav/current_pose` and `/odom`.

## Current Limits

- This is fixed-altitude 2D planning inside a 3D indoor scene, not full 6-DoF UAV dynamics.
- The planner is the repo's lightweight MPPI implementation, not Nav2 MPPI controller.
- PX4/Gazebo offboard flight was intentionally left disabled because `px4_msgs` and PX4 SITL were not installed on this instance.
- The 3D multiview video is a WebGL visualization aligned with the verified ROS2/Gazebo runtime; it is not an RViz/Gazebo GUI screen recording.

## Reproduce On Server

```bash
cd /workspace/UAV_AI
ROS_DISTRO=jazzy DURATION_S=40 BAG_DURATION_S=8 RENDER_VIDEO=1 \
  scripts/verify_ros2_costmap_runtime.sh gazebo_fused mppi
```

For the 3D MPPI visualization:

```bash
scripts/serve_3d_simulation.sh 8765
CHROME_PATH=/root/.cache/ms-playwright/chromium-1228/chrome-linux64/chrome \
  node scripts/record_3d_simulation_video.js \
  --url http://localhost:8765/ \
  --planner mppi \
  --camera pov \
  --duration-s 12 \
  --fps 24 \
  --width 1280 \
  --height 720 \
  --output outputs/videos/uav_3d_fused_sensor_pov_mppi.mp4
```
