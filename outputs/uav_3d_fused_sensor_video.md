# UAV 3D Fused Sensor Visualization

Primary multiview videos:

- A* baseline: `outputs/videos/uav_3d_fused_sensor_multiview_astar.mp4`
- MPPI obstacle-avoidance view: `outputs/videos/uav_3d_fused_sensor_multiview_mppi.mp4`

Single-view videos:

- External 3D fused view: `outputs/videos/uav_3d_fused_sensor_astar.mp4`
- Drone POV fused view: `outputs/videos/uav_3d_fused_sensor_pov_astar.mp4`
- Top-down fused view: `outputs/videos/uav_3d_fused_sensor_topdown_astar.mp4`
- External 3D fused view, MPPI: `outputs/videos/uav_3d_fused_sensor_mppi.mp4`
- Drone POV fused view, MPPI: `outputs/videos/uav_3d_fused_sensor_pov_mppi.mp4`
- Top-down fused view, MPPI: `outputs/videos/uav_3d_fused_sensor_topdown_mppi.mp4`

Preview frames:

- External preview: `outputs/figures/uav_3d_fused_sensor_preview.png`
- POV inset preview: `outputs/figures/uav_3d_fused_sensor_pov_inset_preview.png`
- POV inset contact sheet: `outputs/figures/uav_3d_fused_sensor_pov_inset_contact.png`
- Multiview preview: `outputs/figures/uav_3d_fused_sensor_multiview_preview.png`

Visual meaning:

| Color/layer | Meaning |
|---|---|
| Translucent walls, ceiling lights and lab benches | Indoor lab/corridor environment for the A-to-B UAV flight scenario. |
| Cyan points | PointCloud2-style obstacle surface evidence. |
| Cyan floor cells | Occupancy cells projected from visible point-cloud points. |
| Red/orange rays and cells | LiDAR scan returns and LiDAR-derived occupancy cells. |
| Purple frustum, hits and cells | Monocular/depth-camera projected obstacle evidence. |
| Green floor cells | Fused occupancy grid after union of point-cloud, LiDAR and depth evidence. |
| Blue line | Planner output over the obstacle map; the selected run can be A* or MPPI. |
| Red 3D objects | Obstacles with 3D bounding boxes. |
| Yellow footprints | Inflated safety regions around obstacles. |

The multiview video uses three synchronized columns:

| Column | View |
|---|---|
| 1 | External 3D indoor scene showing UAV, obstacles, source evidence and fused grid. |
| 2 | Drone POV inside the corridor, plus an inset with relative depth and LiDAR scan inputs. |
| 3 | Top-down indoor layout showing planner path, obstacle footprints and fused cells. |

Verification:

- External fused view: 1920 x 1080, 12 seconds, 288 frames.
- Drone POV fused view with sensor inset: 1920 x 1080, 12 seconds, 288 frames.
- Top-down fused view: 1920 x 1080, 12 seconds, 288 frames.
- Multiview fused video: 3840 x 720, 12 seconds, 288 frames.
- MPPI multiview fused video: 3840 x 720, 12 seconds, 288 frames.

Scope note: this is a WebGL 3D qualitative visualization aligned with the verified ROS2/Gazebo fused runtime pipeline. The environment is rendered as an indoor lab/corridor to make the A-to-B MAV obstacle-avoidance task easier to understand. The runtime verifier passed for `gazebo_fused + astar` and `gazebo_fused + mppi` with PointCloud2, Gazebo depth image and Gazebo LaserScan source grids merged by the costmap mux. This video makes that fused perception-to-planner concept visible in 3D; it is not a raw RViz/Gazebo screen recording.
