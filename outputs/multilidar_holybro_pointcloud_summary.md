# Multi-LiDAR HolybroStdn01 Point-Cloud Segmentation Summary

- Source bag uploaded to Hugging Face as `data/raw/multi_lidar/HolybroStdn01.bag.zst`.
- Server download was verified by SHA-256 and decompressed to `data/raw/multi_lidar/HolybroStdn01.from_hf.bag`.
- Bag duration: 31.614 s. Sensors used: Ouster PointCloud2, Livox Mid360 CustomMsg, Livox Avia CustomMsg.
- Method: ROI filtering, ground-percentile removal, voxel connected-component clustering, and 3D axis-aligned bounding boxes. No ROS runtime was used.

| Sensor/topic | Frames | Raw points/frame | Foreground points/frame | Clusters/frame | Total 3D boxes | Figure |
|---|---:|---:|---:|---:|---:|---|
| Ouster OS point cloud `/ouster/points` | 8 | 57817 | 17594 | 8.62 | 69 | `outputs/figures/multilidar_holybro_ouster_pointcloud_3d_bboxes.png` |
| Livox Mid360 `/mid360/livox/lidar` | 8 | 2004 | 1212 | 5.00 | 40 | `outputs/figures/multilidar_holybro_mid360_livox_3d_bboxes.png` |
| Livox Avia `/avia/livox/lidar` | 8 | 2400 | 809 | 6.38 | 51 | `outputs/figures/multilidar_holybro_avia_livox_3d_bboxes.png` |

## Output Files

- Ouster OS point cloud: `outputs/tables/multilidar_holybro_ouster_pointcloud_segmentation_summary.csv`, `outputs/tables/multilidar_holybro_ouster_pointcloud_3d_bboxes.csv`, `outputs/figures/multilidar_holybro_ouster_pointcloud_3d_bboxes.png`
- Livox Mid360: `outputs/tables/multilidar_holybro_mid360_livox_segmentation_summary.csv`, `outputs/tables/multilidar_holybro_mid360_livox_3d_bboxes.csv`, `outputs/figures/multilidar_holybro_mid360_livox_3d_bboxes.png`
- Livox Avia: `outputs/tables/multilidar_holybro_avia_livox_segmentation_summary.csv`, `outputs/tables/multilidar_holybro_avia_livox_3d_bboxes.csv`, `outputs/figures/multilidar_holybro_avia_livox_3d_bboxes.png`
- Topic inventory: `outputs/tables/multilidar_rosbag_topic_inventory.csv`

## Interpretation

This run is real point-cloud processing on a Multi-LiDAR Multi-UAV rosbag: segmentation/clustering is performed on decoded XYZ points, and each cluster is exported as a metric 3D bounding box. The result should be used as cross-dataset sensing evidence, while ODA remains the main UAV obstacle-avoidance benchmark.
