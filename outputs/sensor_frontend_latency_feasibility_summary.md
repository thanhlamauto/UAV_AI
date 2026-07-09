# Sensor Frontend Latency Feasibility

This table plugs measured/reused sensor-to-occupancy compute time into the same MPPI kinematic-delay check.
Radar/IMU are excluded because the current ODA pipeline turns them into risk features, not occupancy grids.

| Frontend | Source | Sensor ms | Frontend ms | Delay at 2 m/s ms | Max no-violation speed | First violation | First collision |
|---|---|---:|---:|---:|---:|---:|---:|
| ideal_occupancy_esdf | synthetic_gt | 100.0 | 5.9 | 200.7 | 4.0 | 5.0 | 6.0 |
| oda_cached_depth_to_grid | ODA RGB cached depth | 200.0 | 2.3 | 300.4 | 3.0 | 4.0 | 5.0 |
| oda_depth_anything_to_grid | ODA RGB | 200.0 | 66.2 | 364.2 | 2.0 | 3.0 | 4.0 |
| external_lidar_bbox_to_grid | Multi-LiDAR bbox CSV | 100.0 | 0.0 | 201.2 | 4.0 | 5.0 |  |
| bbox_cached_depth_mux_to_grid | Multi-LiDAR bbox + ODA cached depth | 200.0 | 2.7 | 299.9 | 3.0 | 4.0 | 5.0 |

Full per-speed rows are in `outputs/tables/sensor_frontend_latency_feasibility.csv`.
