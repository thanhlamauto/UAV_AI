# ARCO ROS2 Bag Stress Probe

Purpose: validate that the project can inspect radar/LiDAR/IMU ROS2 bag samples without adding ROS to the ODA benchmark pipeline.

ARCO is a ground-robot dataset, so these counts support sensing generalization only. They are not directly comparable to ODA UAV obstacle-avoidance metrics.

## Samples

| Sample | DB files | Topics | Messages | Duration est. (s) | DB size (GiB) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Trajectory1 | 1 | 11 | 64104 | 193.7469 | 2.9765 |
| Trajectory2 | 1 | 11 | 71556 | 216.0559 | 3.3464 |
| TrafficMonitoring | 1 | 11 | 40337 | 121.7695 | 1.8113 |

## Sensor Topics

- `Trajectory1` `/tf_static` (tf2_msgs/msg/TFMessage): 6 messages
- `Trajectory1` `/tf` (tf2_msgs/msg/TFMessage): 6442 messages
- `Trajectory1` `/ouster/imu` (sensor_msgs/msg/Imu): 19373 messages
- `Trajectory1` `/PointCloudObject` (sensor_msgs/msg/PointCloud2): 3875 messages
- `Trajectory1` `/PointCloudDetection` (sensor_msgs/msg/PointCloud2): 3875 messages
- `Trajectory1` `/ObjectList` (ars548_messages/msg/ObjectList): 3875 messages
- `Trajectory1` `/arco/idmind_imu/imu` (sensor_msgs/msg/Imu): 13179 messages
- `Trajectory1` `/DetectionList` (ars548_messages/msg/DetectionList): 3875 messages
- `Trajectory2` `/tf_static` (tf2_msgs/msg/TFMessage): 6 messages
- `Trajectory2` `/tf` (tf2_msgs/msg/TFMessage): 7229 messages
- `Trajectory2` `/ouster/imu` (sensor_msgs/msg/Imu): 21605 messages
- `Trajectory2` `/PointCloudObject` (sensor_msgs/msg/PointCloud2): 4321 messages
- `Trajectory2` `/PointCloudDetection` (sensor_msgs/msg/PointCloud2): 4321 messages
- `Trajectory2` `/ObjectList` (ars548_messages/msg/ObjectList): 4321 messages
- `Trajectory2` `/arco/idmind_imu/imu` (sensor_msgs/msg/Imu): 14721 messages
- `Trajectory2` `/DetectionList` (ars548_messages/msg/DetectionList): 4321 messages
- `TrafficMonitoring` `/tf_static` (tf2_msgs/msg/TFMessage): 6 messages
- `TrafficMonitoring` `/tf` (tf2_msgs/msg/TFMessage): 4073 messages
- `TrafficMonitoring` `/ouster/imu` (sensor_msgs/msg/Imu): 12177 messages
- `TrafficMonitoring` `/PointCloudObject` (sensor_msgs/msg/PointCloud2): 2436 messages
- `TrafficMonitoring` `/PointCloudDetection` (sensor_msgs/msg/PointCloud2): 2436 messages
- `TrafficMonitoring` `/ObjectList` (ars548_messages/msg/ObjectList): 2436 messages
- `TrafficMonitoring` `/arco/idmind_imu/imu` (sensor_msgs/msg/Imu): 8349 messages
- `TrafficMonitoring` `/DetectionList` (ars548_messages/msg/DetectionList): 2436 messages

## Interpretation

- Use these samples to stress-test radar/LiDAR/IMU parsing and feature extraction assumptions.
- Keep ODA as the primary UAV avoidance benchmark because it provides MAV trajectory and obstacle ground truth.
- Do not use ARCO planner/path metrics as head-to-head results against ODA.
