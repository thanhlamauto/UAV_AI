# Progress Submission Index

Use this as the short handoff list for the current progress submission.

## Main Report

- `reports/uav_oda_progress_technical_report.pdf`

Current title-page fields:

- Student: Nguyen Thanh Lam / Nguyễn Thanh Lâm
- Mentor: Hoàng Công Phát
- Mentor unit: Viettel High Tech

## 3D Simulation Demo

- Live demo URL when the local server is running: `http://localhost:8765/`
- MP4 video: `outputs/videos/uav_3d_simulation_astar.mp4`
- Desktop screenshot: `outputs/figures/uav_3d_sim_desktop.png`
- Mobile screenshot: `outputs/figures/uav_3d_sim_mobile.png`
- Bundle: `outputs/3d_simulation_artifacts.tar.gz`

Run live demo:

```bash
scripts/serve_3d_simulation.sh 8765
```

Verify 3D simulation artifacts:

```bash
python3 scripts/audit_3d_simulation_status.py --fail-on-incomplete
```

## Level 3 Full 3D ESDF/MPPI Video

- MP4 video: `outputs/videos/level3_full_3d_voxel_esdf_mppi.mp4`
- Dynamic indoor technical video: `outputs/videos/level3_dynamic_indoor_events_esdf_mppi.mp4`
- Dynamic indoor drone POV video: `outputs/videos/level3_dynamic_indoor_pov_esdf_mppi.mp4`
- Realistic indoor chase video: `outputs/videos/level3_realistic_indoor_chase_fused_esdf_mppi.mp4`
- Realistic indoor drone POV video: `outputs/videos/level3_realistic_indoor_pov_esdf_mppi.mp4`
- Isaac Sim indoor RGB-D/LiDAR video: `outputs/videos/isaacsim_indoor_rgbd_lidar_dynamic_demo.mp4`
- Isaac Sim third-person/chase video: `outputs/videos/isaacsim_indoor_third_person_rgbd_lidar_dynamic_demo.mp4`
- Constrained doorway-turn video: `outputs/videos/level3_indoor_doorway_turn_esdf_mppi.mp4`
- Sensor-ablation planner decision video: `outputs/videos/sensor_ablation_planner_decision_demo.mp4`
- MPPI Offboard controller setpoint video: `outputs/videos/mppi_offboard_controller_setpoint_demo.mp4`
- Preview frame: `outputs/figures/level3_video_preview/level3_midframe.png`
- Dynamic POV preview: `outputs/figures/level3_video_preview/level3_dynamic_pov_midframe.png`
- Realistic chase preview: `outputs/figures/level3_video_preview/level3_realistic_chase_midframe.png`
- Realistic POV preview: `outputs/figures/level3_video_preview/level3_realistic_midframe.png`
- Isaac Sim preview: `outputs/figures/isaacsim_demo/isaacsim_indoor_midframe.png`
- Isaac Sim third-person preview: `outputs/figures/isaacsim_demo/isaacsim_indoor_third_person_midframe.png`
- Doorway-turn preview: `outputs/figures/level3_video_preview/level3_doorway_turn_midframe.png`
- Sensor-ablation preview: `outputs/figures/level3_video_preview/sensor_ablation_planner_decision_midframe.png`
- Sensor-ablation contact sheet: `outputs/figures/level3_video_preview/sensor_ablation_planner_decision_contact_sheet.png`
- MPPI Offboard controller preview: `outputs/figures/level3_video_preview/mppi_offboard_controller_midframe.png`
- Isaac Sim metrics: `outputs/tables/isaacsim_indoor_sensor_demo_metrics.csv`
- Isaac Sim third-person metrics: `outputs/tables/isaacsim_indoor_third_person_sensor_demo_metrics.csv`
- Doorway-turn metrics: `outputs/tables/level3_indoor_doorway_turn_mppi.csv`
- Sensor-ablation metrics: `outputs/tables/sensor_ablation_planner_decision_metrics.csv`
- MPPI Offboard controller metrics: `outputs/tables/mppi_offboard_controller_setpoint_metrics.csv`
- Source data: `data/processed/esdf3d/indoor_demo_esdf_mppi.npz`
- ROS2/NVBlox evidence: `outputs/nvblox_esdf3d_status_echo.txt`
- Online latency evidence: `outputs/tables/online_latency_feasibility_10hz.csv`
- Online latency figure: `outputs/figures/online_latency_feasibility_10hz.png`
- Sensor frontend latency evidence: `outputs/tables/sensor_frontend_latency_feasibility.csv`
- Sensor frontend latency figure: `outputs/figures/sensor_frontend_latency_feasibility.png`
- Sensor frontend feasibility rates: `outputs/tables/sensor_frontend_feasibility_rates.csv`
- Sensor frontend feasibility-rate figure: `outputs/figures/sensor_frontend_feasibility_rates.png`

This video is the Muc 3 result: 3D voxel occupancy, ESDF slice, MPPI trajectory
in `[x,y,z]`, altitude profile and runtime evidence from the PointCloud2/NVBlox
verifier.

The dynamic indoor videos add surprise events: a person crossing, a door panel
narrowing the corridor and a cart/box appearing near the old route. The POV
video shows the drone camera view plus an object-level indoor map and MPPI
replanning path.

The realistic indoor videos are WebGL/Three.js visual demonstrations rendered on
the RTX 4090 server. Use the chase video as the clearest mentor-facing demo; use
the POV video as onboard-camera evidence. Both show indoor objects, LiDAR point
cloud, 3D bounding boxes, relative depth inset, LiDAR inset, voxel/ESDF map and
MPPI path under the same dynamic-event sequence.

The Isaac Sim video is a headless Isaac Sim 6.0.1 render on the RTX 4090 server:
onboard RGB and distance-to-camera depth come from Isaac Replicator annotators,
while the LiDAR/point-cloud panel is a geometry-raycast cloud over the active
indoor objects. It is a higher-fidelity sensor visualization artifact, not a
PX4/offboard closed-loop claim.

The third-person Isaac Sim video uses the same scene, sensors and metrics, but
places the RGB-D camera in a chase view behind the UAV so the drone body, nearby
obstacles and the fused perception panels are visible together.

The constrained doorway-turn video addresses the fly-over/blind-spot critique:
it uses a low ceiling, partition wall and single door opening so the UAV must
turn through the doorway. The top-down map shows the closed wall, while the
third-person panel uses a cutaway view so the drone/path remain visible.

The sensor-ablation video explains planner decisions by comparing full fusion
against one missing sensor at a time. It shows LiDAR removal causing wall
collision, depth removal causing glass/low-obstacle collision, and radar removal
causing a moving-obstacle prediction failure.

The MPPI Offboard controller setpoint video was rendered on the GPU server after
syncing the upgraded controller code. It runs the project's MPPI local-controller
module and a kinematic UAV dynamics loop to show fused costmap input producing
velocity and acceleration setpoints. This is controller evidence, not a
PX4/Gazebo SITL screen recording.

Verify the full progress-submission package:

```bash
python3 scripts/audit_progress_submission.py --fail-on-incomplete
```

Run the narrow 10 Hz LiDAR latency/feasibility check:

```bash
python3 experiments/benchmark_online_latency_feasibility.py --prefer-scipy
```

Run the sensor-to-occupancy frontend latency/feasibility check:

```bash
python3 experiments/benchmark_sensor_frontend_latency_feasibility.py --prefer-scipy
```

Run the multi-case safe/violation/collision rate check:

```bash
python3 experiments/benchmark_sensor_frontend_feasibility_rates.py --prefer-scipy --cases-per-speed 8
```

Run the online 10 Hz planner comparison over 8 cases per speed:

```bash
python experiments/benchmark_online_10hz_planner_feasibility_rates.py --prefer-scipy --cases-per-speed 8
```

Use a Python environment with `numpy`, `scipy`, and `matplotlib`; on this machine
that is the Miniconda `python` command. Homebrew `python3` does not have those
packages installed.

Run the ODA replay online feasibility check using OptiTrack-delayed poses:

```bash
python experiments/benchmark_oda_replay_online_feasibility.py --cases-per-trial 8
```

Current audit status:

- `.venv/bin/python scripts/audit_progress_submission.py --fail-on-incomplete`: COMPLETE on 2026-07-01.
- `node scripts/verify_3d_simulation_render.js http://localhost:8765/ outputs/figures`: PASS on desktop and mobile.
- `.venv/bin/python scripts/check_perception_to_planner_contract.py`: COMPLETE on 2026-07-01.
- `.venv/bin/python scripts/check_perception_planner_matrix.py`: COMPLETE on 2026-07-01 for 5 perception sources x 3 planners.
- `.venv/bin/python -m compileall -q src experiments scripts ros2_ws/src/uav_oda_ros2_demo`: PASS on 2026-07-01.
- `scripts/audit_3d_simulation_status.py --fail-on-incomplete`: COMPLETE.
- `scripts/audit_goal_status.py`: COMPLETE for the ODA benchmark/perception/LiDAR deliverables.
- `scripts/audit_ros2_demo_status.py`: runtime evidence exists for fused costmap demos.

ROS2/Gazebo server handoff:

- `outputs/ros2_gazebo_runtime_handoff.md`
- `outputs/perception_to_planner_integration_status.md`
- `outputs/px4_mppi_offboard_controller_status.md`
- `outputs/mppi_offboard_controller_setpoint_summary.md`

## Key Supporting Outputs

- ODA planner metrics: `outputs/tables/planner_comparison_summary_300.csv`
- Full planner rows: `outputs/tables/batch_planner_metrics_300.csv`
- Perception-risk features: `outputs/tables/perception_risk_features_depth_anything_v2_small_50.csv`
- Perception-to-planner contract: `outputs/tables/perception_to_planner_contract.csv`
- Perception/planner matrix: `outputs/tables/perception_planner_matrix.csv`
- Defense key metrics: `outputs/tables/defense_key_metrics.csv`
- Online latency feasibility: `outputs/tables/online_latency_feasibility_10hz.csv`
- Online 10 Hz planner rates: `outputs/tables/online_10hz_planner_feasibility_rates.csv`
- ODA replay online feasibility: `outputs/tables/oda_replay_online_feasibility_by_planner.csv`
- Sensor frontend latency feasibility: `outputs/tables/sensor_frontend_latency_feasibility.csv`
- Sensor frontend feasibility rates: `outputs/tables/sensor_frontend_feasibility_rates.csv`
- LiDAR bbox evidence: `outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv`
- LiDAR figure: `outputs/figures/multilidar_tello03_ouster_pointcloud_3d_bboxes.png`
- Defense narrative: `outputs/defense_narrative.md`

## Scope Note

The 3D simulation is a visual simulation for:

```text
obstacle geometry -> occupancy/safety map -> A*/RRT/MPPI path -> UAV motion
```

The newest MPPI Offboard setpoint artifact executes the local controller and
closed-loop kinematic UAV motion from fused perception. It is still not a full
Gazebo/PX4 physics simulation. ROS2/Gazebo runtime evidence remains available
separately; full PX4/Gazebo SITL recording is the next verifier.
