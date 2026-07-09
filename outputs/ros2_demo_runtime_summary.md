# ROS2 Runtime Evidence Summary

Runtime root: `outputs/ros2_demo_runtime`
CSV table: `outputs/tables/ros2_demo_runtime_summary.csv`

| Run | Mode | Planner | Status | Topics | Messages | Mux | Bag MB | Video MB |
|---|---|---|---|---:|---:|---|---:|---:|
| `bbox_cached_depth_mux_astar_20260623_043046` | bbox_cached_depth_mux | astar | passed | 11/11 | 11/11 | passed | 0.00 | 0.05 |
| `gazebo_laserscan_astar_20260623_052303` | gazebo_laserscan | astar | passed | 6/6 | 6/6 | not_applicable | 0.00 | 0.03 |
| `gazebo_depth_astar_20260623_052445` | gazebo_depth | astar | passed | 7/7 | 7/7 | not_applicable | 0.00 | 0.03 |
| `gazebo_fused_astar_20260623_071634` | gazebo_fused | astar | passed | 13/13 | 13/13 | passed | 0.00 | 0.03 |
| `gazebo_fused_mppi_20260623_112924` | gazebo_fused | mppi | passed | 13/13 | 13/13 | passed | 12.71 | 0.03 |

A run is report-ready when `status=passed`, all expected topics are present, and one message sample exists for every required topic. For muxed modes such as `bbox_cached_depth_mux` and `gazebo_fused`, `Mux` must also be `passed`.

Verifier commands:

```bash
scripts/verify_ros2_costmap_runtime.sh bbox astar
scripts/verify_ros2_costmap_runtime.sh synthetic astar
scripts/verify_ros2_costmap_runtime.sh depth_image astar
scripts/verify_ros2_costmap_runtime.sh cached_depth astar
scripts/verify_ros2_costmap_runtime.sh bbox_cached_depth_mux astar
scripts/verify_ros2_costmap_runtime.sh gazebo_depth astar
scripts/verify_ros2_costmap_runtime.sh gazebo_laserscan astar
scripts/verify_ros2_costmap_runtime.sh gazebo_fused astar
```
