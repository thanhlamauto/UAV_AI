# External Dataset Extension Plan

## Why Add External Data

The 300-trial ODA benchmark is useful for statistical confidence, but ODA remains relatively controlled: indoor MAV flights with one or two obstacles and OptiTrack ground truth. External datasets should therefore be used as stress/positioning evidence, not as a replacement benchmark.

## Probe Result

- Multi-LiDAR Multi-UAV: 27 SharePoint links were probed; all currently redirect to Microsoft login, so no direct download-ready sample is available from this environment. Best future targets remain `Tello03`, `Tello04`, `Tello05`, `TelloOut01`, or `TelloOut02` because they are hard UAV tracking cases with LiDAR/MOCAP context.
- ARCO: 3 direct ROS2 bag ZIP samples were downloaded/probed without ROS. The probe produced 33 topic rows and 175997 messages across `Trajectory1`, `Trajectory2`, and `TrafficMonitoring`.

## Recommended Next Experiments

1. Keep ODA as the planner benchmark at 300 trials and scale perception-risk only after the 50-trial depth table is stable.
2. Use the ARCO probe as evidence that radar/LiDAR/IMU ingestion can be inspected through ROS2 sqlite bags without adding ROS to the ODA benchmark.
3. Use Multi-LiDAR as a later UAV perception/tracking stress dataset only if authenticated/direct links become available.
4. In the report, present this as generalization pressure: ODA answers avoidance metrics; ARCO tests radar/LiDAR sensing context; Multi-LiDAR tests UAV tracking/perception complexity and access difficulty.

## Sources

- Multi-LiDAR Multi-UAV Dataset: https://tiers.github.io/multi_lidar_multi_uav_dataset/
- ARCO Dataset: https://robotics.upo.es/datasets/ArcoDataset/main.html
- HEPP paper for high-speed UAV motivation: https://arxiv.org/abs/2505.17438
