# Sensor Frontend Feasibility Rates

Rates are computed over multiple MPPI seeds per frontend and speed.
`violation` means safety-radius violation without body collision; `collision` is counted separately.

## Overall Thresholds

| Frontend | Sensor ms | Frontend ms | Max speed with safe rate >= 50% | First unsafe speed | First collision speed |
|---|---:|---:|---:|---:|---:|
| bbox_cached_depth_mux_to_grid | 200.0 | 3.1 | 3.0 | 3.0 | 4.0 |
| external_lidar_bbox_to_grid | 100.0 | 0.0 | 4.0 | 5.0 | 6.0 |
| ideal_occupancy_esdf | 100.0 | 6.1 | 4.0 | 3.0 | 4.0 |
| oda_cached_depth_to_grid | 200.0 | 2.7 | 3.0 | 4.0 | 4.0 |
| oda_depth_anything_to_grid | 200.0 | 66.6 | 3.0 | 3.0 | 4.0 |

## Per-Speed Rates

| Frontend | Speed | Cases | Safe % | Violation % | Collision % | Mean delay ms | Mean clearance m |
|---|---:|---:|---:|---:|---:|---:|---:|
| bbox_cached_depth_mux_to_grid | 1.0 | 8 | 100.0 | 0.0 | 0.0 | 247.2 | 0.349 |
| bbox_cached_depth_mux_to_grid | 2.0 | 8 | 100.0 | 0.0 | 0.0 | 308.4 | 0.496 |
| bbox_cached_depth_mux_to_grid | 3.0 | 8 | 87.5 | 12.5 | 0.0 | 415.5 | 0.346 |
| bbox_cached_depth_mux_to_grid | 4.0 | 8 | 0.0 | 87.5 | 12.5 | 414.3 | 0.051 |
| bbox_cached_depth_mux_to_grid | 5.0 | 8 | 0.0 | 0.0 | 100.0 | 415.1 | -0.286 |
| bbox_cached_depth_mux_to_grid | 6.0 | 8 | 0.0 | 0.0 | 100.0 | 422.6 | -0.288 |
| external_lidar_bbox_to_grid | 1.0 | 8 | 100.0 | 0.0 | 0.0 | 150.1 | 0.388 |
| external_lidar_bbox_to_grid | 2.0 | 8 | 100.0 | 0.0 | 0.0 | 206.4 | 0.269 |
| external_lidar_bbox_to_grid | 3.0 | 8 | 100.0 | 0.0 | 0.0 | 322.2 | 0.513 |
| external_lidar_bbox_to_grid | 4.0 | 8 | 100.0 | 0.0 | 0.0 | 303.5 | 0.391 |
| external_lidar_bbox_to_grid | 5.0 | 8 | 0.0 | 100.0 | 0.0 | 301.0 | 0.183 |
| external_lidar_bbox_to_grid | 6.0 | 8 | 0.0 | 0.0 | 100.0 | 312.3 | -0.107 |
| ideal_occupancy_esdf | 1.0 | 8 | 100.0 | 0.0 | 0.0 | 159.2 | 0.380 |
| ideal_occupancy_esdf | 2.0 | 8 | 100.0 | 0.0 | 0.0 | 223.5 | 0.287 |
| ideal_occupancy_esdf | 3.0 | 8 | 87.5 | 12.5 | 0.0 | 374.7 | 0.437 |
| ideal_occupancy_esdf | 4.0 | 8 | 62.5 | 12.5 | 25.0 | 369.2 | 0.187 |
| ideal_occupancy_esdf | 5.0 | 8 | 0.0 | 100.0 | 0.0 | 316.0 | 0.136 |
| ideal_occupancy_esdf | 6.0 | 8 | 0.0 | 0.0 | 100.0 | 313.5 | -0.076 |
| oda_cached_depth_to_grid | 1.0 | 8 | 100.0 | 0.0 | 0.0 | 249.2 | 0.347 |
| oda_cached_depth_to_grid | 2.0 | 8 | 100.0 | 0.0 | 0.0 | 307.1 | 0.495 |
| oda_cached_depth_to_grid | 3.0 | 8 | 100.0 | 0.0 | 0.0 | 405.9 | 0.391 |
| oda_cached_depth_to_grid | 4.0 | 8 | 0.0 | 87.5 | 12.5 | 413.3 | 0.081 |
| oda_cached_depth_to_grid | 5.0 | 8 | 0.0 | 0.0 | 100.0 | 445.1 | -0.290 |
| oda_cached_depth_to_grid | 6.0 | 8 | 0.0 | 0.0 | 100.0 | 450.2 | -0.291 |
| oda_depth_anything_to_grid | 1.0 | 8 | 100.0 | 0.0 | 0.0 | 312.4 | 0.315 |
| oda_depth_anything_to_grid | 2.0 | 8 | 100.0 | 0.0 | 0.0 | 368.9 | 0.506 |
| oda_depth_anything_to_grid | 3.0 | 8 | 62.5 | 37.5 | 0.0 | 464.6 | 0.262 |
| oda_depth_anything_to_grid | 4.0 | 8 | 0.0 | 0.0 | 100.0 | 466.7 | -0.060 |
| oda_depth_anything_to_grid | 5.0 | 8 | 0.0 | 0.0 | 100.0 | 487.7 | -0.288 |
| oda_depth_anything_to_grid | 6.0 | 8 | 0.0 | 0.0 | 100.0 | 485.0 | -0.288 |
