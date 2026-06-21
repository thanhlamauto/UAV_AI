# Advanced Planner Summary

## Status

Advanced planner methods present in the latest benchmark: mppi, rrt_star.

The comparison uses the same ODA ground-plane obstacle model and the same metrics as the earlier human/straight-line/geometric/A*/RRT benchmark:

- collision rate;
- safety-distance violation rate;
- minimum obstacle-boundary clearance;
- path length;
- heading-change smoothness;
- planner compute time.

## Latest Planner Table

| method | trials | collision_rate | safety_violation_rate | mean_min_clearance_m | mean_path_length_m | mean_planner_compute_time_ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| astar_grid | 3 | 0.0 | 0.0 | 0.6092 | 6.8479 | 5.6578 |
| geometric_bypass | 2 | 0.0 | 0.0 | 0.7208 | 6.977 | 3.1941 |
| geometric_bypass_not_needed | 1 | 0.0 | 0.0 | 0.7742 | 6.0936 | 1.9279 |
| human | 3 | 0.0 | 0.3333 | 0.7446 | 7.59 | 0.0 |
| mppi | 3 | 0.0 | 0.0 | 0.7386 | 6.6814 | 4.4417 |
| rrt | 3 | 0.0 | 0.0 | 0.6343 | 6.754 | 8.6344 |
| rrt_star | 3 | 0.0 | 0.0 | 0.636 | 6.7399 | 75.5094 |
| straight_line | 3 | 0.3333 | 0.6667 | 0.4055 | 6.6072 | 1.6204 |

## Planner Failures

No planner-level failures were recorded in `outputs/tables/planner_failures.csv`.

## Interpretation Checklist

- Prefer methods with zero collision and zero safety-distance violation before optimizing path length.
- Treat MPPI and RRT* compute time as prototype Python timings, not optimized controller timings.
- Compare against the straight-line baseline to show why obstacle-aware planning is necessary.
