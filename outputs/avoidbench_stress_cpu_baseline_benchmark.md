# AvoidBench-Style Stress Benchmark

This stress test plans on corrupted sensor-estimated costmaps but evaluates safety on a separate ground-truth costmap.

Safe cases: `20/49`
CSV: `outputs/tables/avoidbench_stress_cpu_baseline_matrix.csv`
Figure: `outputs/figures/avoidbench_stress_cpu_baseline_matrix.png`

| Stress case | Planner | Status | Length m | GT clearance m | GT collision-free | Reached goal | Compute ms |
|---|---|---|---:|---:|---:|---:|---:|
| clean_depth_gt | straight_line | unsafe_gt | 8.00 | 0.00 | 0 | 1 | 0.19 |
| clean_depth_gt | greedy_reactive | invalid_goal | 81.20 | 0.44 | 1 | 0 | 52.50 |
| clean_depth_gt | astar | safe | 10.85 | 0.48 | 1 | 1 | 28.89 |
| clean_depth_gt | rrt | safe | 12.40 | 0.50 | 1 | 1 | 33.68 |
| clean_depth_gt | rrt_star | safe | 10.68 | 0.52 | 1 | 1 | 513.67 |
| clean_depth_gt | mppi | safe | 10.73 | 0.48 | 1 | 1 | 114.74 |
| clean_depth_gt | mpc | safe | 10.73 | 0.48 | 1 | 1 | 185.14 |
| depth_dropout_speckle | straight_line | unsafe_gt | 8.00 | 0.00 | 0 | 1 | 0.16 |
| depth_dropout_speckle | greedy_reactive | unsafe_gt | 30.37 | 0.34 | 0 | 1 | 20.05 |
| depth_dropout_speckle | astar | unsafe_gt | 11.92 | 0.34 | 0 | 1 | 20.74 |
| depth_dropout_speckle | rrt | unsafe_gt | 13.73 | 0.44 | 0 | 1 | 26.25 |
| depth_dropout_speckle | rrt_star | unsafe_gt | 11.60 | 0.38 | 0 | 1 | 332.30 |
| depth_dropout_speckle | mppi | unsafe_gt | 11.49 | 0.38 | 0 | 1 | 31.22 |
| depth_dropout_speckle | mpc | unsafe_gt | 11.50 | 0.38 | 0 | 1 | 48.27 |
| blind_central_obstacle | straight_line | unsafe_gt | 8.00 | 0.00 | 0 | 1 | 0.15 |
| blind_central_obstacle | greedy_reactive | invalid_goal | 20.16 | 0.00 | 0 | 0 | 20.67 |
| blind_central_obstacle | astar | safe | 10.85 | 0.48 | 1 | 1 | 23.49 |
| blind_central_obstacle | rrt | unsafe_gt | 13.14 | 0.44 | 0 | 1 | 39.47 |
| blind_central_obstacle | rrt_star | safe | 10.73 | 0.50 | 1 | 1 | 554.96 |
| blind_central_obstacle | mppi | safe | 10.73 | 0.48 | 1 | 1 | 37.33 |
| blind_central_obstacle | mpc | safe | 10.73 | 0.48 | 1 | 1 | 167.68 |
| limited_range_fov | straight_line | unsafe_gt | 8.00 | 0.00 | 0 | 1 | 0.16 |
| limited_range_fov | greedy_reactive | unsafe_gt | 9.98 | 0.00 | 0 | 1 | 16.40 |
| limited_range_fov | astar | unsafe_gt | 9.58 | 0.00 | 0 | 1 | 17.52 |
| limited_range_fov | rrt | safe | 12.40 | 0.50 | 1 | 1 | 25.85 |
| limited_range_fov | rrt_star | unsafe_gt | 9.46 | 0.48 | 0 | 1 | 547.43 |
| limited_range_fov | mppi | unsafe_gt | 9.36 | 0.00 | 0 | 1 | 28.19 |
| limited_range_fov | mpc | unsafe_gt | 9.36 | 0.00 | 0 | 1 | 126.06 |
| pose_shift_40cm | straight_line | unsafe_gt | 8.00 | 0.00 | 0 | 1 | 0.16 |
| pose_shift_40cm | greedy_reactive | invalid_goal | 117.60 | 0.10 | 0 | 0 | 71.63 |
| pose_shift_40cm | astar | unsafe_gt | 10.40 | 0.10 | 0 | 1 | 24.25 |
| pose_shift_40cm | rrt | unsafe_gt | 12.76 | 0.44 | 0 | 1 | 29.04 |
| pose_shift_40cm | rrt_star | unsafe_gt | 10.19 | 0.10 | 0 | 1 | 540.49 |
| pose_shift_40cm | mppi | unsafe_gt | 10.30 | 0.10 | 0 | 1 | 34.68 |
| pose_shift_40cm | mpc | unsafe_gt | 10.29 | 0.10 | 0 | 1 | 179.19 |
| false_positive_clutter | straight_line | unsafe_gt | 8.00 | 0.00 | 0 | 1 | 0.15 |
| false_positive_clutter | greedy_reactive | invalid_goal | 4.48 | 0.48 | 1 | 0 | 28.48 |
| false_positive_clutter | astar | safe | 12.85 | 0.50 | 1 | 1 | 40.01 |
| false_positive_clutter | rrt | planner_failed | 0.00 | nan | 0 | 0 | 109.09 |
| false_positive_clutter | rrt_star | safe | 12.35 | 0.91 | 1 | 1 | 236.28 |
| false_positive_clutter | mppi | safe | 12.55 | 0.50 | 1 | 1 | 50.56 |
| false_positive_clutter | mpc | safe | 12.56 | 0.50 | 1 | 1 | 180.33 |
| narrow_gate_clean | straight_line | unsafe_gt | 8.00 | 0.00 | 0 | 1 | 0.15 |
| narrow_gate_clean | greedy_reactive | safe | 12.34 | 0.42 | 1 | 1 | 30.47 |
| narrow_gate_clean | astar | safe | 9.35 | 0.48 | 1 | 1 | 28.95 |
| narrow_gate_clean | rrt | safe | 10.27 | 0.44 | 1 | 1 | 49.64 |
| narrow_gate_clean | rrt_star | safe | 13.02 | 0.50 | 1 | 1 | 160.16 |
| narrow_gate_clean | mppi | safe | 9.25 | 0.48 | 1 | 1 | 39.07 |
| narrow_gate_clean | mpc | safe | 9.24 | 0.48 | 1 | 1 | 134.64 |

Interpretation:

- `safe` means the path planned on the sensor map also clears the ground-truth inflated map.
- `unsafe_gt` means the planner found a path, but the path is unsafe when checked against the true map.
- Sensor miss, limited range, and pose shift are expected to expose failures; those failures are useful evidence that clean costmap results were optimistic.
