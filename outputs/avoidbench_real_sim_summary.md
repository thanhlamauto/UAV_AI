# AvoidBench Real Simulator Sweep

Runtime: Vast.ai Ubuntu 22.04 host, Docker `hangyutud/noetic_avoidbench:latest`
with ROS Noetic/Ubuntu 20.04 inside the container.

Simulator smoke test:

- AvoidBench Unity connected successfully to ROS/Flightmare.
- Published `/depth`, `/rgb/left`, `/rgb/right`, `/hummingbird/ground_truth/odometry`,
  `/hummingbird/goal_point`, `/hummingbird/task_state`, and `/hummingbird/metrics`.
- Observed SGM `/depth` publish rate in the smoke run: about 1.4 Hz, roughly 705 ms period.

Sweep:

- Same outdoor AvoidBench configuration seed, reduced to `flight_number: 1`, `trials: 1`
  for each planner-speed cell.
- Planner loop ran at 10 Hz and published `/hummingbird/autopilot/velocity_command`.
- Planner timing was published on `/hummingbird/iter_time`.
- Outcomes are from AvoidBench metrics: `Safe` means progress reached 1.0 with zero
  collision; `Coll` means progress reached 1.0 with one collision; `TO` means no metrics
  arrived before the 260 s wall-time timeout.

CSV: `outputs/tables/avoidbench_real_sim/summary.csv`

