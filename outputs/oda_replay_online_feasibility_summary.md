# ODA Replay Online Feasibility

UAV timing profile: `jetson_orin_nano` - Emulated Jetson Orin Nano-class companion computer.

This is a timing emulation/profile applied to local Python measurements, not a real onboard hardware benchmark. It is intended to stress the replay with representative companion-computer latency assumptions.

Each case is an ODA OptiTrack timestamp near an obstacle. The delayed pose is obtained by interpolating the recorded OptiTrack trajectory at `t + total_delay`, then the planner replans from that delayed pose.

Safety is scored in 3D using OptiTrack `(x, z, y_height)` points and finite cylinder obstacles from ODA metadata. RRT, RRT*, and MPPI plan in the ground-plane footprint and their paths are lifted between delayed and goal heights for 3D clearance scoring.

## Planner Summary

| Planner | Cases | Safe % | Violation % | Collision % | Mean speed m/s | Mean delay ms | Mean delay m | Mean compute ms | Mean clearance m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| mppi | 128 | 88.3 | 11.7 | 0.0 | 0.987 | 169.7 | 0.195 | 31.0 | 1.157 |
| rrt | 128 | 89.8 | 0.8 | 9.4 | 0.987 | 149.9 | 0.143 | 11.2 | 1.147 |
| rrt_star | 128 | 88.3 | 0.8 | 10.9 | 0.987 | 181.6 | 0.172 | 42.9 | 1.144 |

## Planner x Speed Bin

| Planner | Speed bin | Cases | Safe % | Violation % | Collision % | Mean delay ms | Mean clearance m |
|---|---:|---:|---:|---:|---:|---:|---:|
| mppi | 0.5-1.0 | 44 | 79.5 | 20.4 | 0.0 | 169.0 | 0.906 |
| mppi | 1.0-1.5 | 39 | 87.2 | 12.8 | 0.0 | 170.6 | 1.087 |
| mppi | <0.5 | 32 | 100.0 | 0.0 | 0.0 | 169.1 | 1.647 |
| mppi | >=1.5 | 13 | 92.3 | 7.7 | 0.0 | 170.6 | 1.015 |
| rrt | 0.5-1.0 | 44 | 84.1 | 0.0 | 15.9 | 155.1 | 0.896 |
| rrt | 1.0-1.5 | 39 | 87.2 | 2.6 | 10.3 | 148.0 | 1.078 |
| rrt | <0.5 | 32 | 100.0 | 0.0 | 0.0 | 144.4 | 1.633 |
| rrt | >=1.5 | 13 | 92.3 | 0.0 | 7.7 | 151.8 | 1.012 |
| rrt_star | 0.5-1.0 | 44 | 84.1 | 0.0 | 15.9 | 211.9 | 0.889 |
| rrt_star | 1.0-1.5 | 39 | 84.6 | 2.6 | 12.8 | 166.8 | 1.076 |
| rrt_star | <0.5 | 32 | 100.0 | 0.0 | 0.0 | 166.3 | 1.632 |
| rrt_star | >=1.5 | 13 | 84.6 | 0.0 | 15.4 | 161.3 | 1.003 |

Total detailed cases: 384.
