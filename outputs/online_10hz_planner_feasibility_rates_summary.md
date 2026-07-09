# Online 10 Hz Planner Feasibility Rates

Rates are computed over 8 seeds per planner and speed with a 10 Hz sensor period.
RRT, RRT*, and MPPI use seeded sampling.
RRT and RRT* are fixed-altitude 2D baselines lifted into the 3D ESDF for clearance evaluation.
`violation` means safety-radius violation without body collision; `collision` is counted separately.

Map/ESDF update samples ms: min=5.851, median=5.881, max=6.068.

## Overall Thresholds

| Planner | Max speed with safe rate >= 50% | First unsafe speed | First collision speed | Mean compute ms | Mean total delay ms |
|---|---:|---:|---:|---:|---:|
| mppi_3d_esdf | 4.0 | 5.0 | 5.0 | 161.5 | 277.4 |
| rrt | 6.0 | 1.0 |  | 2.8 | 118.7 |
| rrt_star | 5.0 | 1.0 | 5.0 | 98.8 | 214.7 |

## Per-Speed Rates

| Planner | Speed | Cases | Safe % | Violation % | Collision % | Mean compute ms | Mean delay ms | Mean clearance m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| mppi_3d_esdf | 1.0 | 8 | 100.0 | 0.0 | 0.0 | 40.3 | 156.2 | 0.382 |
| mppi_3d_esdf | 2.0 | 8 | 100.0 | 0.0 | 0.0 | 101.1 | 217.0 | 0.345 |
| mppi_3d_esdf | 3.0 | 8 | 100.0 | 0.0 | 0.0 | 214.4 | 330.3 | 0.447 |
| mppi_3d_esdf | 4.0 | 8 | 100.0 | 0.0 | 0.0 | 181.3 | 297.2 | 0.416 |
| mppi_3d_esdf | 5.0 | 8 | 0.0 | 37.5 | 62.5 | 251.9 | 367.8 | -0.093 |
| mppi_3d_esdf | 6.0 | 8 | 0.0 | 75.0 | 25.0 | 180.0 | 295.9 | 0.002 |
| rrt | 1.0 | 8 | 62.5 | 37.5 | 0.0 | 3.5 | 119.3 | 0.296 |
| rrt | 2.0 | 8 | 62.5 | 37.5 | 0.0 | 2.5 | 118.3 | 0.321 |
| rrt | 3.0 | 8 | 75.0 | 25.0 | 0.0 | 3.0 | 118.8 | 0.321 |
| rrt | 4.0 | 8 | 50.0 | 50.0 | 0.0 | 2.9 | 118.7 | 0.284 |
| rrt | 5.0 | 8 | 75.0 | 25.0 | 0.0 | 2.2 | 118.1 | 0.324 |
| rrt | 6.0 | 8 | 87.5 | 12.5 | 0.0 | 2.8 | 118.7 | 0.374 |
| rrt_star | 1.0 | 8 | 87.5 | 12.5 | 0.0 | 106.6 | 222.5 | 0.312 |
| rrt_star | 2.0 | 8 | 75.0 | 25.0 | 0.0 | 108.4 | 224.3 | 0.259 |
| rrt_star | 3.0 | 8 | 37.5 | 62.5 | 0.0 | 102.0 | 217.9 | 0.254 |
| rrt_star | 4.0 | 8 | 50.0 | 50.0 | 0.0 | 146.8 | 262.7 | 0.244 |
| rrt_star | 5.0 | 8 | 62.5 | 25.0 | 12.5 | 103.6 | 219.4 | 0.210 |
| rrt_star | 6.0 | 8 | 0.0 | 12.5 | 87.5 | 25.6 | 141.5 | -0.215 |
