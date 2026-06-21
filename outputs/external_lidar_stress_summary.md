# External LiDAR / Point-Cloud Stress Summary

ODA remains the main UAV obstacle-avoidance benchmark. These probes only document how external LiDAR or point-cloud sources can stress the sensing side later.

| Source | Evidence | Point-cloud messages | Scope |
|---|---:|---:|---|
| ARCO Dataset | downloaded_rosbag_sqlite_topics, 3 sample(s), 33 rows | 26324 | topic_and_message_stress_probe_not_segmentation |
| Multi-LiDAR Multi-UAV Dataset | link_probe_login_required, 27 sample(s), 27 rows | 0 | 27/27_links_require_login_no_local_pointcloud_processing |

Current limitation: no point-cloud segmentation, clustering, 3D bounding boxes, or voxel grid has been integrated into the ODA benchmark yet.

Next experiment: download one accessible point-cloud sequence, run ground removal plus DBSCAN/Euclidean clustering, and export cluster centroids/bounding boxes as a perception stress test. Keep ODA as the planner benchmark.
