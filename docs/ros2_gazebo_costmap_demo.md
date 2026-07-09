# ROS2/Gazebo Costmap Demo Runbook

This runbook turns the project from an offline benchmark into a small
perception-to-planning simulation:

```text
LiDAR bbox / PointCloud2 / synthetic depth / cached predicted-depth / merged bbox+depth / Gazebo depth / Gazebo LiDAR
    -> OccupancyGrid costmap -> A*/RRT/MPPI path
    -> kinematic UAV marker or optional PX4 waypoint follower
```

The ROS2 package lives at:

```text
ros2_ws/src/uav_oda_ros2_demo
```

## What This Demo Proves

- Multi-LiDAR 3D bounding boxes can be replayed as a ROS2 `nav_msgs/OccupancyGrid`.
- A real or synthetic `sensor_msgs/PointCloud2` stream can be projected into a 2D costmap.
- A metric or relative `sensor_msgs/Image` depth stream can be projected into the same 2D costmap interface.
- Cached monocular predicted-depth `.npz` output can be replayed as a ROS2 `mono8` image and sent through the same planner input.
- `costmap_mux` can merge LiDAR bbox and depth-derived maps into one planner input.
- A Gazebo depth camera can be bridged into ROS2 and projected into the same planner input.
- A Gazebo GPU LiDAR `LaserScan` can be bridged with `ros_gz_bridge` and converted into the same 2D costmap interface.
- The costmap is consumed by classical planners: `astar`, `rrt`, and `mppi`.
- The planned path is published as `nav_msgs/Path` for RViz.
- A lightweight kinematic follower publishes `/uav/current_pose`, `/odom`, and `/uav/marker` so the path can be shown as a moving UAV demo without PX4.
- The same path can be sent to an optional PX4 Offboard bridge after SITL is ready.

This is intentionally lighter than a full Nav2 stack.  It is designed to
produce a reliable mentor-facing demo before adding heavier controller and
Gazebo sensor integration.

## Server Environment

Recommended:

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Garden/Fortress or PX4-supported Gazebo
Python 3.10+
```

Install ROS2 dependencies:

```bash
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-numpy
```

Optional for Gazebo LiDAR bridge and PX4 Offboard:

```bash
sudo apt install -y ros-humble-ros-gz ros-humble-px4-msgs
```

If `ros-humble-px4-msgs` is unavailable, build `px4_msgs` in the same ROS2
workspace before enabling the PX4 bridge.

For a fresh Ubuntu 22.04 server, use the bootstrap script from the repo root:

```bash
cd /workspace/uav-oda-obstacle-avoidance
scripts/setup_ros2_gazebo_server.sh
```

The bootstrap installs ROS2 Humble, colcon, numpy, and `ros_gz` support. It does
not install PX4/PX4 messages because those are only needed for the optional
Offboard bridge.

## Build

From the repository root:

```bash
cd /workspace/uav-oda-obstacle-avoidance
source /opt/ros/humble/setup.bash

cd ros2_ws
colcon build --symlink-install --packages-select uav_oda_ros2_demo
source install/setup.bash
```

Quick non-ROS planner smoke test:

```bash
cd /workspace/uav-oda-obstacle-avoidance
python3 scripts/check_ros2_costmap_demo_static.py
python3 scripts/check_perception_to_planner_contract.py
python3 scripts/check_perception_planner_matrix.py
python3 scripts/check_ros2_launch_contract.py
python3 scripts/check_ros2_mode_consistency.py
python3 scripts/check_costmap_mux_status_validator.py
```

Expected output contains one path summary for each planner:

```text
astar: waypoints=...
rrt: waypoints=...
mppi: waypoints=...
lidar_bbox_csv: grid=... occupied=... path_waypoints=...
metric_depth_image: grid=... occupied=... path_waypoints=...
relative_predicted_depth_proxy: grid=... occupied=... path_waypoints=...
lidar_bbox_plus_relative_depth_mux: grid=... occupied=... path_waypoints=...
lidar_bbox_plus_cached_depth_mux: grid=... occupied=... path_waypoints=...
lidar_bbox_plus_cached_depth_mux/mppi: grid=... occupied=... path_waypoints=... collision_free=1
```

The contract and planner-matrix checks write:

```text
outputs/tables/perception_to_planner_contract.csv
outputs/tables/perception_planner_matrix.csv
outputs/perception_to_planner_contract.md
outputs/perception_planner_matrix.md
outputs/figures/perception_to_planner_contract.svg
```

The matrix check verifies every perception-derived map against `astar`, `rrt`,
and `mppi`, and rejects paths that intersect the inflated occupied map.

The SVG figure is generated with:

```bash
python3 scripts/render_perception_to_planner_contract_figure.py
```

One-command server runner:

```bash
cd /workspace/uav-oda-obstacle-avoidance
scripts/run_ros2_costmap_demo.sh bbox astar
scripts/run_ros2_costmap_demo.sh synthetic astar
scripts/run_ros2_costmap_demo.sh depth_image astar
scripts/run_ros2_costmap_demo.sh cached_depth astar
scripts/run_ros2_costmap_demo.sh bbox_cached_depth_mux astar
scripts/run_ros2_costmap_demo.sh gazebo_depth astar
scripts/run_ros2_costmap_demo.sh gazebo_laserscan astar
scripts/run_ros2_costmap_demo.sh gazebo_fused astar
```

Runtime verifier that saves evidence:

```bash
cd /workspace/uav-oda-obstacle-avoidance
scripts/verify_ros2_costmap_all_modes.sh astar
```

Single-mode fallback/debug commands:

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

Recommended first server sequence:

```bash
scripts/setup_ros2_gazebo_server.sh
python3 scripts/audit_ros2_demo_status.py
python3 scripts/check_ros2_launch_contract.py
scripts/check_ros2_server_preflight.sh
scripts/run_headless_ros2_runtime_video.sh astar
scripts/verify_ros2_fused_perception_demo.sh astar
scripts/verify_ros2_costmap_all_modes.sh astar
python3 scripts/audit_ros2_demo_status.py --fail-on-incomplete
python3 scripts/bundle_ros2_demo_artifacts.py
```

For a rented server without GUI/RViz interaction, the shortest mentor-facing
command is:

```bash
scripts/run_headless_ros2_runtime_video.sh astar
```

It runs the focused fused branch
`LiDAR bbox + cached depth -> costmap_mux -> /perception/occupancy_grid -> planner`,
records ROS2 topic evidence, validates `mux_status_valid=passed`, renders an
MP4 with `ffmpeg`, and copies the final video to:

```text
outputs/videos/ros2_fused_perception_runtime_astar.mp4
```

Set `RUN_ALL_MODES=1` to run all ROS2/Gazebo modes before copying the fused
runtime video.

The focused fused verifier first proves the key integration branch:
`LiDAR bbox + cached depth -> costmap_mux -> /perception/occupancy_grid -> planner`.
The all-mode verifier then runs the server preflight, offline contract check,
builds the ROS2 package once, verifies all seven perception-to-planner modes,
refreshes summaries, runs the final audit, and writes
`outputs/ros2_demo_artifacts.tar.gz`.

If a single mode fails, use these debug commands:

```bash
scripts/verify_ros2_costmap_runtime.sh bbox astar
scripts/verify_ros2_costmap_runtime.sh synthetic astar
scripts/verify_ros2_costmap_runtime.sh depth_image astar
scripts/verify_ros2_costmap_runtime.sh cached_depth astar
scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar
scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar
scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar
scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar
python3 scripts/summarize_ros2_runtime_evidence.py
python3 scripts/write_ros2_demo_report_section.py
python3 scripts/render_ros2_costmap_demo_video.py --planner astar --output outputs/videos/ros2_costmap_demo_astar.mp4
```

The first audit should pass static checks and report missing runtime evidence.
The final audit should report `COMPLETE` after all verifier modes pass.

Verifier outputs are saved under:

```text
outputs/ros2_demo_runtime/<mode>_<planner>_<timestamp>/
```

Each run stores `colcon_build.log`, `static_planner_check.log`, `launch.log`,
`topic_list.txt`, and one-message samples from required topics such as
`/perception/occupancy_grid`, `/planned_path`, `/uav/marker`, and the active
sensor topic. By default it also attempts to render a small MP4 demo inside the
runtime evidence folder.

For the fused `bbox_cached_depth_mux` mode, the verifier also waits up to
`MUX_STATUS_TIMEOUT_S` seconds for `/perception/costmap_mux_status` to report a
real merged state. This status must prove that both source costmaps
`/perception/bbox_occupancy_grid` and `/perception/depth_occupancy_grid` were
received, no required input is missing, and the merged grid has occupied cells.
The validation log is saved as `costmap_mux_status_validation.log`.

The verifier also refreshes:

```text
outputs/ros2_demo_runtime_summary.md
outputs/tables/ros2_demo_runtime_summary.csv
outputs/ros2_runtime_diagnostics.md
outputs/tables/ros2_runtime_diagnostics.csv
outputs/ros2_demo_report_section.md
```

These files are the compact evidence table, failure diagnostics, and
report-ready Vietnamese section to use in progress reports. In
`outputs/tables/ros2_demo_runtime_summary.csv`, the fused
`bbox_cached_depth_mux` row must show `mux_status_valid=passed`; this is the
compact proof that LiDAR bbox and cached-depth costmaps were both merged before
the planner consumed `/perception/occupancy_grid`.

To pull back only the ROS2/Gazebo demo evidence and source snapshot:

```bash
python3 scripts/bundle_ros2_demo_artifacts.py
```

This writes:

```text
outputs/ros2_demo_artifacts.tar.gz
```

To create a standalone MP4 without ROS2/RViz, run:

```bash
python3 scripts/render_ros2_costmap_demo_video.py --planner astar --output outputs/videos/ros2_costmap_demo_astar.mp4
```

Current expected standalone video path:

```text
outputs/videos/ros2_costmap_demo_astar.mp4
```

Modes:

```text
synthetic         Synthetic PointCloud2 -> costmap -> planner -> UAV marker
depth_image       Synthetic depth Image -> costmap -> planner -> UAV marker
cached_depth      Cached monocular predicted-depth NPZ -> Image -> depth costmap -> planner -> UAV marker
bbox_cached_depth_mux LiDAR bbox costmap + cached depth costmap -> costmap mux -> planner -> UAV marker
gazebo_depth      Gazebo depth camera -> ros_gz_bridge -> depth costmap -> planner -> UAV marker
gazebo_laserscan Gazebo GPU LiDAR -> ros_gz_bridge -> LaserScan costmap -> planner -> UAV marker
gazebo_fused     PointCloud2 + Gazebo depth + Gazebo LaserScan -> costmap mux -> planner -> UAV marker
bbox              Multi-LiDAR bbox CSV -> costmap -> planner -> UAV marker
```

## Demo 1: Multi-LiDAR BBox Replay to Planner

This uses the project output table:

```text
outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv
```

Run:

```bash
cd /workspace/uav-oda-obstacle-avoidance
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch uav_oda_ros2_demo bbox_replay_planner.launch.py \
  planner:=astar \
  bbox_csv:=/workspace/uav-oda-obstacle-avoidance/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv
```

Equivalent first-class runner through the shared perception-to-planner launch:

```bash
scripts/run_ros2_costmap_demo.sh bbox astar \
  /workspace/uav-oda-obstacle-avoidance/outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv
```

Try other planners:

```bash
ros2 launch uav_oda_ros2_demo bbox_replay_planner.launch.py planner:=rrt
ros2 launch uav_oda_ros2_demo bbox_replay_planner.launch.py planner:=mppi
```

View in RViz:

```bash
rviz2 -d /workspace/uav-oda-obstacle-avoidance/ros2_ws/src/uav_oda_ros2_demo/config/rviz_demo.rviz
```

Expected topics:

```text
/perception/occupancy_grid
/perception/bbox_markers
/uav/current_pose
/goal_pose
/planned_path
/odom
/uav/marker
```

The launch file enables the kinematic follower by default. RViz should show a
blue UAV marker moving along `/planned_path`.

## Demo 2: PointCloud2 to Costmap to Planner

This smoke test does not need Gazebo.  It publishes synthetic obstacle
point-cloud cylinders and converts them into a costmap.

```bash
ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
  use_synthetic_cloud:=true \
  start_gazebo_world:=false \
  start_px4:=false \
  enable_px4_bridge:=false \
  planner:=astar
```

Expected flow:

```text
/lidar/points -> /perception/occupancy_grid -> /planned_path
```

This is the fastest proof that point cloud output can feed A*/RRT/MPPI through
an obstacle map. It also shows the kinematic UAV marker following the planned
path, which makes this the recommended first mentor-facing RViz demo.

## Demo 3: Depth Image to Costmap to Planner

This smoke test publishes a synthetic metric depth image, converts near-depth
pixels into an obstacle footprint, then feeds the same `OccupancyGrid` planner
interface.

```bash
scripts/run_ros2_costmap_demo.sh depth_image astar
```

Runtime verifier:

```bash
scripts/verify_ros2_costmap_runtime.sh depth_image astar
```

Expected flow:

```text
/camera/depth/image -> /perception/occupancy_grid -> /planned_path
```

The conversion node supports `32FC1`, `16UC1`, `mono8`, and `8UC1`. For
relative monocular depth, high relative-depth pixels are treated as near
obstacles by default and mapped to a configurable pseudo-range. This keeps the
bridge usable for predicted-depth outputs while keeping the report honest:
monocular depth should not be treated as metric distance until calibrated with
OptiTrack/radar.

## Demo 4: Cached Predicted Depth to Costmap to Planner

This replays an existing monocular depth cache from:

```text
data/processed/depth_sample_3_5fps.npz
```

The cached `depth_u8` frames are published as `sensor_msgs/Image` with `mono8`
encoding, then consumed by the same `depth_image_costmap` node.

```bash
scripts/run_ros2_costmap_demo.sh cached_depth astar
```

For evidence collection, prefer:

```bash
scripts/verify_ros2_costmap_runtime.sh cached_depth astar
```

Expected flow:

```text
depth_sample_3_5fps.npz
  -> cached_depth_image_publisher
  -> ROS2 /camera/depth/image
  -> depth_image_costmap
  -> /perception/occupancy_grid
  -> /planned_path
```

This is the closest lightweight bridge from the earlier monocular
predicted-depth experiment to planner input.

## Demo 5: LiDAR BBox + Cached Depth Mux to Planner

This mode runs two perception branches at the same time:

```text
LiDAR bbox CSV -> bbox_costmap_publisher -> /perception/bbox_occupancy_grid
Cached predicted depth -> depth_image_costmap -> /perception/depth_occupancy_grid
```

`costmap_mux` waits for both source grids, then merges them into the planner-facing:

```text
/perception/occupancy_grid -> costmap_planner -> /planned_path
```

It also publishes a JSON status sample on:

```text
/perception/costmap_mux_status
```

The runtime verifier parses this sample with
`scripts/validate_costmap_mux_status_sample.py` and requires `state=merged`,
both source topics, empty `missing_topics`, and nonzero occupied cells.

Run:

```bash
scripts/run_ros2_costmap_demo.sh bbox_cached_depth_mux astar
```

For evidence collection:

```bash
scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar
```

Focused one-command evidence collection:

```bash
scripts/verify_ros2_fused_perception_demo.sh astar
```

Expected source topics:

```text
/perception/bbox_occupancy_grid
/perception/depth_occupancy_grid
/perception/costmap_mux_status
/perception/occupancy_grid
/planned_path
```

This mode demonstrates that planner input can be a fused obstacle map rather
than a single perception source.

## Demo 6: Gazebo Depth Camera to Costmap to Planner

This starts the included Gazebo world, bridges the Gazebo depth camera image
into ROS2, converts near-depth pixels into `OccupancyGrid`, then plans and
follows the path.

```bash
scripts/run_ros2_costmap_demo.sh gazebo_depth astar
```

For evidence collection, prefer:

```bash
scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar
```

Equivalent explicit launch:

```bash
ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
  start_gazebo_world:=true \
  use_synthetic_cloud:=false \
  use_synthetic_depth:=false \
  use_depth_image:=true \
  use_gazebo_depth_image:=true \
  start_px4:=false \
  enable_px4_bridge:=false \
  planner:=astar
```

Expected sensor flow:

```text
Gazebo /camera/depth/image
  -> ros_gz_bridge
  -> ROS2 /camera/depth/image
  -> depth_image_costmap
  -> /perception/occupancy_grid
  -> costmap_planner
  -> /planned_path
  -> kinematic_path_follower
  -> /uav/marker
```

This is the runtime test that separates a synthetic depth-image smoke test from
a true Gazebo depth-camera source.

## Demo 7: Gazebo LiDAR Scan to Costmap to Planner

This starts the included Gazebo world, bridges the GPU LiDAR scan into ROS2,
converts the scan into `OccupancyGrid`, then plans and follows the path.

```bash
scripts/run_ros2_costmap_demo.sh gazebo_laserscan astar
```

For evidence collection, prefer:

```bash
scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar
```

## Demo 8: Gazebo Fused Sensors to Planner

This starts the included Gazebo world, bridges Gazebo depth and LaserScan, adds
PointCloud2 obstacle evidence, converts all three streams into separate
occupancy grids, and merges them with `costmap_mux` before planning.

```bash
scripts/run_ros2_costmap_demo.sh gazebo_fused astar
```

For evidence collection, prefer:

```bash
scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar
```

Equivalent explicit launch:

```bash
ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
  start_gazebo_world:=true \
  use_synthetic_cloud:=true \
  use_pointcloud_costmap:=true \
  pointcloud_costmap_topic:=perception/pointcloud_occupancy_grid \
  use_depth_image:=true \
  use_gazebo_depth_image:=true \
  depth_costmap_topic:=perception/depth_occupancy_grid \
  use_gazebo_laserscan:=true \
  laserscan_costmap_topic:=perception/laserscan_occupancy_grid \
  use_costmap_mux:=true \
  costmap_mux_input_topics_csv:=perception/pointcloud_occupancy_grid,perception/depth_occupancy_grid,perception/laserscan_occupancy_grid \
  start_px4:=false \
  enable_px4_bridge:=false \
  planner:=astar
```

Expected sensor flow:

```text
PointCloud2 / Gazebo /camera/depth/image / Gazebo /uav_oda/lidar_scan
  -> pointcloud_costmap / depth_image_costmap / laserscan_costmap
  -> source-specific occupancy grids
  -> costmap_mux
  -> /perception/occupancy_grid
  -> costmap_planner
  -> /planned_path
  -> kinematic_path_follower
  -> /uav/marker
```

The Gazebo/ROS bridge syntax follows the Gazebo ROS2 integration convention:

```text
/TOPIC@ROS_MSG@GZ_MSG
```

The bridge command used by the launch file is:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /uav_oda/lidar_scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

## Demo 9: Lightweight Gazebo World with Synthetic PointCloud2

Start the included indoor obstacle world next to the synthetic perception demo:

```bash
ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
  start_gazebo_world:=true \
  use_synthetic_cloud:=true \
  use_gazebo_laserscan:=false \
  start_px4:=false \
  enable_px4_bridge:=false \
  planner:=astar
```

The world file is:

```text
ros2_ws/src/uav_oda_ros2_demo/worlds/indoor_obstacles.sdf
```

This mode keeps the robust synthetic `PointCloud2` publisher as the perception
source while showing the same obstacle layout in Gazebo. Use Demo 7 when you
want the Gazebo LiDAR sensor itself to drive the costmap.

## Demo 10: PX4 SITL MPPI Offboard Controller

Only enable this after PX4 SITL and `px4_msgs` are working.

Start PX4 separately:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Then run the MPPI local-controller bridge:

```bash
cd /workspace/uav-oda-obstacle-avoidance
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch uav_oda_ros2_demo px4_gazebo_costmap_demo.launch.py \
  start_px4:=false \
  use_synthetic_cloud:=true \
  use_gazebo_laserscan:=false \
  enable_kinematic_follower:=false \
  enable_px4_bridge:=false \
  enable_px4_mppi_controller:=true \
  enable_px4_odometry_bridge:=true \
  planner:=astar
```

For the fused sensor pipeline with PX4 offboard setpoints, use:

```bash
scripts/run_ros2_gazebo_fused_px4.sh astar
```

The fused PX4 runner defaults to:

```text
PX4_CONTROLLER=mppi
```

This path is:

```text
fused OccupancyGrid -> MPPI local controller -> velocity/acceleration TrajectorySetpoint -> PX4 Offboard
```

Use the old waypoint bridge only as a fallback:

```bash
PX4_CONTROLLER=waypoint scripts/run_ros2_gazebo_fused_px4.sh astar
```

By default, the bridge publishes setpoints but does not arm or switch modes:

```text
auto_arm: false
auto_offboard: false
```

For SITL-only testing, set these to true in
`ros2_ws/src/uav_oda_ros2_demo/config/demo_params.yaml` or override them in a
launch file after confirming the simulated vehicle is safe to command.

When using PX4 odometry as the vehicle state, keep the demo-only follower off:

```text
enable_kinematic_follower:=false
```

## Relation to the Main Project

The offline ODA benchmark remains the main quantitative evaluation.  This ROS2
demo answers the missing integration question:

```text
Can perception output become an obstacle map that a UAV planner can consume?
```

Current scope:

- Fixed-altitude 2D planning.
- Costmap inflation approximates UAV radius plus safety distance.
- LiDAR bboxes, PointCloud2, synthetic depth image, Gazebo depth image, and Gazebo LaserScan are integrated into planner input.
- A kinematic follower provides a lightweight RViz demo before PX4 integration.
- PX4 Offboard now has an MPPI local-controller bridge that publishes velocity/acceleration setpoints; it still needs SITL tuning and runtime verification before any real-flight claim.

Runtime completion evidence should include a passing verifier folder from at
least:

```text
bbox astar
synthetic astar
depth_image astar
gazebo_depth astar
gazebo_laserscan astar
```

Next extension:

- Replace the synthetic depth publisher with a predicted-depth image stream from the monocular model.
- Calibrate monocular relative depth against OptiTrack/radar before claiming metric obstacle distance.
- Add a Gazebo PointCloudPacked bridge for full PointCloud2 from simulation.
- Compare planner using ground-truth ODA obstacles vs LiDAR-detected obstacles.
- Replace the lightweight costmap with Nav2 costmap layers if full ROS navigation is required.

## References For The Bridge Path

- Gazebo ROS2 integration documents the `ros_gz_bridge` topic syntax:
  `https://gazebosim.org/docs/latest/ros2_integration/`
- The Gazebo `ros_gz_point_cloud` example shows a GPU LiDAR SDF sensor setup:
  `https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_point_cloud/examples/gpu_lidar.sdf`
