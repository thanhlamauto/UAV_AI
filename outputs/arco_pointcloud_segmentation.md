# ARCO Point-Cloud Segmentation and 3D Bounding Boxes

README source selected: **ARCO Dataset**.

Reason: ODA is still the main UAV obstacle-avoidance benchmark, but it does not contain LiDAR point clouds. Multi-LiDAR is more UAV-relevant but all 27 SharePoint links currently require login. ARCO has direct ROS2 bag samples already available on the server and includes real `sensor_msgs/msg/PointCloud2` topics such as `/ouster/points`.

Implemented experiment:

- Input: ARCO `TrafficMonitoring` ROS2 SQLite bag, topic `/ouster/points`.
- Parser: direct CDR decoding of `sensor_msgs/msg/PointCloud2` without ROS runtime.
- Processing: ROI crop, percentile ground removal, voxelization, connected-component clustering.
- Output: 3D axis-aligned bounding boxes with min/max/center/size/volume.

Results:

- Frames processed: 6.
- Raw valid points per frame: about 21k.
- Foreground points per frame after ROI and ground removal: 1132-1784.
- Cluster count per frame: 3-5.
- Total 3D bbox rows: 25.

Artifacts:

- `outputs/tables/arco_pointcloud_segmentation_summary.csv`
- `outputs/tables/arco_pointcloud_3d_bboxes.csv`
- `outputs/figures/arco_pointcloud_3d_bboxes.png`

Scope note: this is real point-cloud segmentation/clustering/3D bbox processing, but ARCO is ground-robot data. It supports the LiDAR perception requirement and sensing-generalization story; ODA remains the UAV trajectory/planner benchmark.
