# Planner Benchmark Summary

## Data Status

- Full ODA archive has been processed on the server.
- `outputs/tables/target_300_trials_readiness.csv` shows 300/300 ready locally after artifacts were pulled back.
- The 300-trial benchmark uses the same obstacle model and metric definitions as the earlier 20/100-trial runs.

## Implemented Baselines

- `human`: original OptiTrack trajectory.
- `straight_line`: direct start-to-goal baseline.
- `geometric_bypass`: waypoint-based left/right bypass around inflated obstacle safety circles.
- `astar_grid`: A* over a 2D occupancy grid with inflated obstacle radius.
- `rrt`: deterministic Rapidly-exploring Random Tree baseline.
- `rrt_star`: deterministic RRT* variant with neighborhood rewiring.
- `mppi`: lightweight Python MPPI-style waypoint optimizer.

## Latest 300-Trial Table

| method | trials | collision rate | safety violation rate | mean min clearance m | mean planner time ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| astar_grid | 300 | 0.0000 | 0.0000 | 0.6426 | 8.6793 |
| geometric_bypass | 196 | 0.0000 | 0.0051 | 0.7005 | 6.3860 |
| geometric_bypass_not_needed | 104 | 0.0000 | 0.0000 | 0.8647 | 2.4630 |
| human | 300 | 0.0733 | 0.4467 | 0.6126 | 0.0000 |
| mppi | 300 | 0.0000 | 0.0033 | 0.7574 | 39.9327 |
| rrt | 300 | 0.0000 | 0.0000 | 0.6702 | 3.6168 |
| rrt_star | 300 | 0.0000 | 0.0000 | 0.6360 | 1196.3979 |
| straight_line | 300 | 0.1400 | 0.6533 | 0.4203 | 2.8712 |

## Interpretation

- Straight-line is the unsafe lower-effort baseline.
- A*, RRT, and RRT* remove collisions and safety-distance violations on the selected 300 ODA trials.
- MPPI has the highest mean clearance among the planner baselines but is slower than A*/RRT in the current Python prototype.
- The next research value should come from perception-risk, depth calibration, and external sensing stress tests rather than only adding more geometric planners.

## Reproduce

```bash
python3 scripts/audit_goal_status.py
python3 -m py_compile src/*.py src/planners/*.py experiments/*.py scripts/*.py
```
