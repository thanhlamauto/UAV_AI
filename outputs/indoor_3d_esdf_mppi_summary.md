# Indoor 3D ESDF MPPI Demo

This experiment is a local Level-3 prototype: a 3D voxel occupancy map is
converted into an ESDF, then MPPI optimizes a continuous `[x, y, z]`
trajectory by querying metric obstacle distance.

## Outputs

- Metrics CSV: `outputs/tables/indoor_3d_esdf_mppi_metrics.csv`
- 3D/path figure: `outputs/figures/indoor_3d_esdf_mppi_path.png`
- ESDF slice figure: `outputs/figures/indoor_3d_esdf_mppi_slice.png`

## Key Metrics

| Metric | Value |
|---|---:|
| `path_length_m` | 7.6455 |
| `min_esdf_distance_m` | 0.6783 |
| `min_safety_margin_m` | 0.2583 |
| `smoothness` | 0.000444 |
| `altitude_change_m` | 0.4499 |
| `planner_compute_time_ms` | 123.638 |
| `collision` | 0 |
| `safety_violation` | 0 |

## Interpretation

This proves the project can evaluate clearance from a 3D distance field,
not only from a 2D binary occupancy grid.  The next server step is to
replace this synthetic ESDF with NVBlox ESDF topics generated from real
depth/LiDAR streams.
