# Multi-LiDAR Two-Bag Point-Cloud Segmentation Summary

This table is a lightweight cross-dataset stress test for true LiDAR point-cloud processing: ground removal, voxel connected-component clustering, and 3D axis-aligned bounding boxes.

| Bag | Sensor | Frames | Raw pts/frame | Foreground pts/frame | Clusters/frame | Total 3D boxes |
|---|---|---:|---:|---:|---:|---:|
| TelloOut02 | Ouster | 8 | 43209 | 6385 | 3.25 | 26 |
| TelloOut02 | Mid360 | 8 | 1992 | 503 | 1.12 | 9 |
| TelloOut02 | Avia | 8 | 2400 | 1748 | 1.62 | 13 |
| Tello03 | Ouster | 8 | 58044 | 17530 | 7.62 | 61 |
| Tello03 | Mid360 | 8 | 2004 | 1173 | 5.38 | 43 |
| Tello03 | Avia | 8 | 2400 | 1042 | 5.62 | 45 |

Interpretation: ODA remains the UAV obstacle-avoidance benchmark; these Multi-LiDAR bags demonstrate that the project now includes real point-cloud segmentation/clustering/3D bounding-box evidence rather than only a metadata probe.
