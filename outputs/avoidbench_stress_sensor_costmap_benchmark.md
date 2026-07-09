# AvoidBench-Style Stress Benchmark

This stress test plans on corrupted sensor-estimated costmaps but evaluates safety on a separate ground-truth costmap.

Safe cases: `15/28`
CSV: `outputs/tables/avoidbench_stress_sensor_costmap_planner_matrix.csv`
Figure: `outputs/figures/avoidbench_stress_sensor_costmap_planner_matrix.png`

| Stress case | Planner | Status | Length m | GT clearance m | GT collision-free | Reached goal | Compute ms |
|---|---|---|---:|---:|---:|---:|---:|
| clean_depth_gt | astar | safe | 10.85 | 0.48 | 1 | 1 | 29.69 |
| clean_depth_gt | rrt | safe | 12.40 | 0.50 | 1 | 1 | 35.20 |
| clean_depth_gt | mppi | safe | 10.73 | 0.48 | 1 | 1 | 39.00 |
| clean_depth_gt | mpc | safe | 10.73 | 0.48 | 1 | 1 | 196.84 |
| depth_dropout_speckle | astar | unsafe_gt | 11.92 | 0.34 | 0 | 1 | 21.11 |
| depth_dropout_speckle | rrt | unsafe_gt | 13.73 | 0.44 | 0 | 1 | 26.56 |
| depth_dropout_speckle | mppi | unsafe_gt | 11.49 | 0.38 | 0 | 1 | 31.45 |
| depth_dropout_speckle | mpc | unsafe_gt | 11.50 | 0.38 | 0 | 1 | 48.78 |
| blind_central_obstacle | astar | safe | 10.85 | 0.48 | 1 | 1 | 24.14 |
| blind_central_obstacle | rrt | unsafe_gt | 13.14 | 0.44 | 0 | 1 | 37.17 |
| blind_central_obstacle | mppi | safe | 10.73 | 0.48 | 1 | 1 | 34.69 |
| blind_central_obstacle | mpc | safe | 10.73 | 0.48 | 1 | 1 | 177.67 |
| limited_range_fov | astar | unsafe_gt | 9.58 | 0.00 | 0 | 1 | 19.43 |
| limited_range_fov | rrt | safe | 12.40 | 0.50 | 1 | 1 | 28.57 |
| limited_range_fov | mppi | unsafe_gt | 9.36 | 0.00 | 0 | 1 | 33.45 |
| limited_range_fov | mpc | unsafe_gt | 9.36 | 0.00 | 0 | 1 | 160.82 |
| pose_shift_40cm | astar | unsafe_gt | 10.40 | 0.10 | 0 | 1 | 27.93 |
| pose_shift_40cm | rrt | unsafe_gt | 12.76 | 0.44 | 0 | 1 | 33.92 |
| pose_shift_40cm | mppi | unsafe_gt | 10.30 | 0.10 | 0 | 1 | 38.83 |
| pose_shift_40cm | mpc | unsafe_gt | 10.29 | 0.10 | 0 | 1 | 236.19 |
| false_positive_clutter | astar | safe | 12.85 | 0.50 | 1 | 1 | 50.66 |
| false_positive_clutter | rrt | planner_failed | 0.00 | nan | 0 | 0 | 113.94 |
| false_positive_clutter | mppi | safe | 12.55 | 0.50 | 1 | 1 | 51.69 |
| false_positive_clutter | mpc | safe | 12.56 | 0.50 | 1 | 1 | 238.92 |
| narrow_gate_clean | astar | safe | 9.35 | 0.48 | 1 | 1 | 84.28 |
| narrow_gate_clean | rrt | safe | 10.27 | 0.44 | 1 | 1 | 62.07 |
| narrow_gate_clean | mppi | safe | 9.25 | 0.48 | 1 | 1 | 41.80 |
| narrow_gate_clean | mpc | safe | 9.24 | 0.48 | 1 | 1 | 136.84 |

Interpretation:

- `safe` means the path planned on the sensor map also clears the ground-truth inflated map.
- `unsafe_gt` means the planner found a path, but the path is unsafe when checked against the true map.
- Sensor miss, limited range, and pose shift are expected to expose failures; those failures are useful evidence that clean costmap results were optimistic.
