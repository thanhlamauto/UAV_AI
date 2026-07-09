# UAV 3D Sensor Flight Video

External-view video: `outputs/videos/uav_3d_sensor_flight_astar.mp4`

Drone POV video: `outputs/videos/uav_3d_drone_pov_astar.mp4`

Fused drone POV with depth/LiDAR inset: `outputs/videos/uav_3d_fused_sensor_pov_astar.mp4`

Top-down video: `outputs/videos/uav_3d_topdown_astar.mp4`

Four-column multiview video: `outputs/videos/uav_3d_four_column_multiview_astar.mp4`

Fused 3D multiview video: `outputs/videos/uav_3d_fused_sensor_multiview_astar.mp4`

Fused 3D details: `outputs/uav_3d_fused_sensor_video.md`

This video is a 3D qualitative simulation view for the UAV obstacle-avoidance pipeline. It shows the UAV moving along the A* planned trajectory while sensor-derived obstacle evidence is rendered in the same 3D scene.

The drone POV version uses an onboard camera pose attached to the UAV path, so the frame shows what the UAV would see while approaching and passing the obstacle.

The four-column multiview version synchronizes:

| Column | View |
|---|---|
| 1 | External 3D sensor scene |
| 2 | Drone POV camera |
| 3 | Top-down planner and obstacle-map view |
| 4 | 2D sensor dashboard with point cloud, LiDAR, depth, depth-costmap and planner output |

Rendered layers:

| Layer | Meaning |
|---|---|
| UAV trajectory | Drone follows the A* path through the obstacle field. |
| Obstacles | 3D cylinder/box obstacles with safety footprints and 3D bounding boxes. |
| Point cloud | PointCloud2-style obstacle surface samples, with visible points emphasized from the UAV pose. |
| LiDAR | 180-degree horizontal LiDAR rays and obstacle hits. |
| Depth | Camera frustum plus depth-hit samples from the simulated depth projection. |
| Depth costmap | Depth-derived occupied floor cells used as planner input evidence. |
| Planner output | A* path over the inflated occupancy map. |

Verification:

- External-view resolution: 1920 x 1080.
- External-view duration: 12 seconds.
- External-view frames: 288 at 24 fps.
- POV resolution: 1920 x 1080.
- POV duration: 12 seconds.
- POV frames: 288 at 24 fps.
- Top-down resolution: 1920 x 1080.
- Top-down duration: 12 seconds.
- Top-down frames: 288 at 24 fps.
- Four-column multiview resolution: 3840 x 540.
- Four-column multiview duration: 12 seconds.
- Four-column multiview frames: 288 at 24 fps.
- Render check: `outputs/figures/uav_3d_sim_render_check.json`.

Scope note: this is a browser/WebGL 3D qualitative renderer aligned with the ROS2 costmap/planner geometry. It complements the ROS2/Gazebo runtime evidence videos rather than replacing the runtime verifier logs.
