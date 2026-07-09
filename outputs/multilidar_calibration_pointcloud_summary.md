# Multi-LiDAR Calibration Point-Cloud Segmentation Summary

Source bag: `data/raw/multi_lidar/Calibration.bag` (~1.3GB).
Bag duration: 9.28s.

## ROS Bag Topic Inventory

- `/avia/livox/lidar`: `livox_ros_driver/msg/CustomMsg`, 929 messages.
- `/camera/depth/color/points`: `sensor_msgs/msg/PointCloud2`, 278 messages.
- `/mid360/livox/lidar`: `livox_ros_driver2/msg/CustomMsg`, 928 messages.
- `/ouster/points`: `sensor_msgs/msg/PointCloud2`, 186 messages.

## Segmentation Runs

| Source | Frames | Mean raw points | Mean foreground points | Total 3D boxes | Mean clusters/frame |
|---|---:|---:|---:|---:|---:|
| Ouster LiDAR `/ouster/points` | 8 | 57525 | 21403 | 28 | 3.50 |
| Depth camera point cloud `/camera/depth/color/points` | 8 | 117636 | 10422 | 8 | 1.00 |

## Outputs

- `outputs/tables/multilidar_rosbag_topic_inventory.csv`
- `outputs/tables/multilidar_ouster_pointcloud_segmentation_summary.csv`
- `outputs/tables/multilidar_ouster_pointcloud_3d_bboxes.csv`
- `outputs/figures/multilidar_ouster_pointcloud_3d_bboxes.png`
- `outputs/tables/multilidar_depth_camera_pointcloud_segmentation_summary.csv`
- `outputs/tables/multilidar_depth_camera_pointcloud_3d_bboxes.csv`
- `outputs/figures/multilidar_depth_camera_pointcloud_3d_bboxes.png`

## Interpretation

This is a real PointCloud2 segmentation/clustering/3D bounding-box run on Multi-LiDAR calibration data. It should be presented as a sensing stress-test and cross-dataset extension; ODA remains the main UAV obstacle-avoidance benchmark.
