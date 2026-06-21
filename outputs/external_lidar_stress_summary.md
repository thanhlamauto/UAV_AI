# External LiDAR / Point-Cloud Stress Summary

ODA remains the main UAV obstacle-avoidance benchmark. External LiDAR sources are used as sensing stress tests, not as a replacement for ODA trajectory/planner metrics.

| Source | Evidence | Point-cloud messages | Scope |
|---|---:|---:|---|
| ARCO Dataset | topic probe plus 6-frame `/ouster/points` segmentation, 3 sample(s), 33 topic rows | 26324 | real_pointcloud_segmentation_voxel_clusters_3d_aabb |
| Multi-LiDAR Multi-UAV Dataset | link_probe_login_required, 27 sample(s), 27 rows | 0 | 27/27_links_require_login_no_local_pointcloud_processing |

ARCO segmentation output: `outputs/tables/arco_pointcloud_segmentation_summary.csv`, `outputs/tables/arco_pointcloud_3d_bboxes.csv`, and `outputs/figures/arco_pointcloud_3d_bboxes.png`.

Remaining limitation: this is real point-cloud processing, but ARCO is a ground-robot dataset, so it should be reported as LiDAR perception stress evidence. Keep ODA as the UAV obstacle-avoidance planner benchmark.
