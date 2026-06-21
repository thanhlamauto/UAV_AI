# External Dataset Extension Plan

## Why Add External Data

The 300-trial ODA benchmark is useful for statistical confidence, but ODA remains relatively controlled: indoor MAV flights with one or two obstacles and OptiTrack ground truth. External datasets should therefore be used as stress/positioning evidence, not as a replacement benchmark.

## Probe Result

- Multi-LiDAR Multi-UAV: 26 advertised sequences, including 8 hard sequences. Best next probe: `Tello03`, `Tello04`, `Tello05`, `TelloOut01`, or `TelloOut02` because they are hard UAV tracking cases with LiDAR/MOCAP context.
- ARCO: 3 direct ROS2 bag ZIP entries probed. Smallest detected candidate: `TrafficMonitoring` at about `0.714` GiB.

## Recommended Next Experiments

1. Keep ODA as the planner benchmark and scale it to 300 trials.
2. Download one ARCO ZIP first only to validate radar/LiDAR/IMU ingestion from ROS2 sqlite bags. Do not compare ARCO path-planning metrics against ODA.
3. Use Multi-LiDAR as a later UAV perception/tracking stress dataset. Its rosbags are much larger, so start with one hard short sequence only after the ODA 300-trial table is stable.
4. In the report, present this as generalization pressure: ODA answers avoidance metrics; ARCO tests radar/LiDAR sensing context; Multi-LiDAR tests UAV tracking/perception complexity.

## Sources

- Multi-LiDAR Multi-UAV Dataset: https://tiers.github.io/multi_lidar_multi_uav_dataset/
- ARCO Dataset: https://robotics.upo.es/datasets/ArcoDataset/main.html
- HEPP paper for high-speed UAV motivation: https://arxiv.org/abs/2505.17438
