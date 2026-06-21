# Advanced Planner Summary

## Status

Advanced planner methods in the latest 300-trial benchmark: `rrt_star` and `mppi`.

All planners use the same ODA ground-plane obstacle model, obstacle radius, safety clearance, and metric definitions:

- collision rate;
- safety-distance violation rate;
- minimum obstacle-boundary clearance;
- path length;
- heading-change smoothness;
- planner compute time.

## Latest Planner Table

| method | trials | collision_rate | safety_violation_rate | mean_min_clearance_m | mean_planner_compute_time_ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| astar_grid | 300 | 0.0000 | 0.0000 | 0.6426 | 8.6793 |
| geometric_bypass | 196 | 0.0000 | 0.0051 | 0.7005 | 6.3860 |
| geometric_bypass_not_needed | 104 | 0.0000 | 0.0000 | 0.8647 | 2.4630 |
| human | 300 | 0.0733 | 0.4467 | 0.6126 | 0.0000 |
| mppi | 300 | 0.0000 | 0.0033 | 0.7574 | 39.9327 |
| rrt | 300 | 0.0000 | 0.0000 | 0.6702 | 3.6168 |
| rrt_star | 300 | 0.0000 | 0.0000 | 0.6360 | 1196.3979 |
| straight_line | 300 | 0.1400 | 0.6533 | 0.4203 | 2.8712 |

## Planner Failures

No planner-level failures were recorded in the 300-trial batch.

## Interpretation Checklist

- Prefer methods with zero collision and zero safety-distance violation before optimizing path length.
- Treat MPPI and RRT* compute time as prototype Python timings, not optimized onboard-controller timings.
- Compare against `straight_line` and `human` to show why obstacle-aware planning and perception-risk are both needed.
