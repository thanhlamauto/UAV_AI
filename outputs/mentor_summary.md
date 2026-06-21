# ODA UAV Obstacle Avoidance: First Results

## Dataset Structure

| item | value |
| --- | --- |
| Unique trial IDs in metadata | 1369 |
| Rows in trial_overview.csv | 1997 |
| Local CSV sample trials | 3, 10, 345 |
| Trials with 0 obstacle coordinates | 37 |
| Trials with 1 obstacle coordinate | 704 |
| Trials with 2 obstacle coordinates | 628 |
| Full-light trials in metadata | 1286 |
| Dim-light trials in metadata | 83 |
| Trials with RGB video flag | 1228 |
| Frame convention used here | ground plane = OptiTrack x vs z; height = OptiTrack y |

## Initial Benchmark Metrics

Obstacle radius is set to 0.20 m, matching the upstream sample scripts. Safety violation means boundary clearance is below 0.50 m.

| sequence | obstacles | min_center_distance_m | min_boundary_clearance_m | closest_time_s | collision | safety_violation | avoidance_label | computation_time_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 3 | 1 | 0.6348 | 0.4348 | 6.425 | 0 | 1 | left | 0.0956 |
| 10 | 1 | 1.03 | 0.83 | 9.325 | 0 | 0 | left | 0.0625 |
| 345 | 2 | 1.1689 | 0.9689 | 8.6875 | 0 | 0 | left | 0.1067 |

## Generated Visualizations

- `outputs/figures/trajectory_sample_3.png`
- `outputs/figures/trajectory_sample_10.png`
- `outputs/figures/trajectory_sample_345.png`
- `outputs/figures/planner_comparison_sample_3.png`
- `outputs/figures/planner_comparison_sample_10.png`
- `outputs/figures/planner_comparison_sample_345.png`

## Planner Benchmark Snapshot

The current local benchmark compares the human-flown OptiTrack trajectory with
six planner baselines: straight-line, geometric bypass, grid A*, RRT, RRT*, and
lightweight Python MPPI. It runs on the complete local ODA samples `3`, `10`,
and `345`.

| method | trials | collision_rate | safety_violation_rate | mean_min_clearance_m | mean_path_length_m | mean_compute_time_ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| human | 3 | 0.0000 | 0.3333 | 0.7446 | 7.5900 | 0.0000 |
| straight_line | 3 | 0.3333 | 0.6667 | 0.4055 | 6.6072 | 1.6204 |
| geometric_bypass / not_needed | 3 | 0.0000 | 0.0000 | 0.7386 | 6.6825 | 2.7720 |
| astar_grid | 3 | 0.0000 | 0.0000 | 0.6092 | 6.8479 | 5.6578 |
| rrt | 3 | 0.0000 | 0.0000 | 0.6343 | 6.7540 | 8.6344 |
| rrt_star | 3 | 0.0000 | 0.0000 | 0.6360 | 6.7399 | 75.5094 |
| mppi | 3 | 0.0000 | 0.0000 | 0.7386 | 6.6814 | 4.4417 |

Key takeaway: straight-line is shorter but unsafe on the local examples, while
geometric bypass, A*, RRT, RRT*, and MPPI remove collision/safety-distance
violations with small planning time.

## Notes

- The full 4TU archive is about 98 GB, so this first pass uses the GitHub-bundled CSV samples and full metadata file.
- Samples 593-629 have missing obstacle coordinates in metadata and should be skipped or repaired before obstacle-distance benchmarking.
- Avoidance labels are heuristic side labels computed from closest approach relative to the start-to-obstacle line.
- A 20-trial target manifest has been prepared, but only 3/20 selected trials
  are locally complete without downloading the full 4TU archive.

## Source Links

- ODA Dataset GitHub: https://github.com/JuSquare/ODA_Dataset
- Full 4TU dataset record: https://data.4tu.nl/articles/dataset/The_Obstacle_Detection_and_Avoidance_Dataset_for_Drones/14214236/1

## Next Planner Comparison Plan

1. Download or extract the remaining target trials from the 4TU full archive.
2. Re-run the existing batch benchmark on all 20 target trials.
3. Run RRT*/MPPI with default server settings on all 20 target trials.
4. Use radar/depth agreement as a lightweight perception-risk feature before
   attempting ROS, Gazebo, or heavy learning.
