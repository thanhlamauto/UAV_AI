# Planner Input/Output Example

This example uses one simple 2D obstacle-avoidance scene so the planner inputs and outputs can be explained in a defense slide.

## Common Input

- Start: `[0.0, 0.0]`
- Goal: `[6.4, 0.0]`
- Obstacles: `[[2.0, 1.0], [4.2, -0.8]]`
- Obstacle radius: `0.35 m`
- Safety distance: `0.50 m`
- Inflated obstacle radius used for collision checking: `0.85 m`
- A* occupancy grid for this example: `85 x 39` cells, `450` occupied cells before safety inflation in the A* implementation.

## Planner Outputs

| Planner | Process | Waypoints | Path length m | Min clearance m | Safety violation | Output preview |
|---|---|---:|---:|---:|---:|---|
| A* | Build occupancy grid, inflate obstacles by obstacle radius + safety distance, search 8-neighbor cells with g+h cost, reconstruct cell path. | 65 | 6.4773 | 0.5083 | 0 | `(0.00,0.00) -> (0.10,0.00) -> (0.20,0.00) -> (0.30,0.00) -> (6.10,0.10) -> (6.20,0.10) -> (6.30,0.00) -> (6.40,0.00)` |
| RRT | Sample continuous points, steer from nearest tree node, reject nodes/edges that intersect inflated obstacles, shortcut the first found path. | 3 | 7.0202 | 0.5044 | 0 | `(0.00,0.00) -> (-0.27,0.22) -> (6.40,0.00)` |
| RRT* | Sample continuous points like RRT, choose lower-cost parent from neighbors, rewire nearby nodes, keep the best connection to the goal. | 10 | 6.4047 | 0.5061 | 0 | `(0.00,0.00) -> (0.64,-0.02) -> (1.37,-0.05) -> (1.68,-0.02) -> (3.39,0.03) -> (3.72,0.04) -> (4.39,0.06) -> (6.40,0.00)` |
| MPPI | Start from a geometric bypass path, sample noisy trajectory rollouts, score length/smoothness/clearance/collision, update the mean path by cost-weighted noise. | 60 | 6.4304 | 0.5050 | 0 | `(0.00,0.00) -> (0.11,0.01) -> (0.22,0.02) -> (0.33,0.02) -> (6.08,0.04) -> (6.18,0.03) -> (6.29,0.01) -> (6.40,0.00)` |

## How To Explain It

- A* output is a grid-cell path, so it is useful as a geometric baseline but can look angular.
- RRT/RRT* output is a sampled continuous waypoint path; it still needs smoothing/control before real UAV flight.
- MPPI output is an optimized trajectory-style path; in a full controller it would optimize control rollouts rather than only waypoints.
- In all cases, the planner output is evaluated by the same safety metrics: path length, minimum clearance, collision and safety-distance violation.

CSV: `outputs/tables/planner_io_example.csv`
