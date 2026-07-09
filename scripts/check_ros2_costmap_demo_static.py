#!/usr/bin/env python3
"""Static smoke test for the ROS2 costmap demo planners.

This script intentionally avoids ROS imports.  It validates the pure Python
OccupancyGrid planner helpers before the package is built on a ROS2 server.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    ros_pkg = repo_root / "ros2_ws" / "src" / "uav_oda_ros2_demo"
    sys.path.insert(0, str(ros_pkg))

    from uav_oda_ros2_demo.grid_planners import GridSpec, PlannerConfig, plan_path
    from uav_oda_ros2_demo.mppi_local_controller import MPPIControllerConfig, mppi_velocity_command

    spec = GridSpec(width=90, height=80, resolution=0.10, origin_x=-1.0, origin_y=-4.0)
    grid = np.zeros((spec.height, spec.width), dtype=np.int8)

    # Two compact obstacles similar to the synthetic PointCloud2 demo.
    def mark_box(cx: float, cy: float, half: float) -> None:
        c0 = int(np.floor((cx - half - spec.origin_x) / spec.resolution))
        c1 = int(np.ceil((cx + half - spec.origin_x) / spec.resolution))
        r0 = int(np.floor((cy - half - spec.origin_y) / spec.resolution))
        r1 = int(np.ceil((cy + half - spec.origin_y) / spec.resolution))
        grid[max(0, r0) : min(spec.height, r1 + 1), max(0, c0) : min(spec.width, c1 + 1)] = 100

    mark_box(2.0, 1.0, 0.35)
    mark_box(4.2, -0.8, 0.45)

    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([6.0, 0.0], dtype=float)
    config = PlannerConfig(
        robot_radius_m=0.15,
        safety_distance_m=0.20,
        rrt_max_iterations=1500,
        rrt_step_size_m=0.35,
        mppi_rollouts=96,
        mppi_iterations=3,
        seed=11,
    )

    for planner in ("astar", "rrt", "mppi"):
        path = plan_path(planner, grid, spec, start, goal, config)
        if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
            raise RuntimeError(f"{planner} returned invalid path shape {path.shape}")
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        print(f"{planner}: waypoints={len(path)} length_m={length:.2f}")

    controller = mppi_velocity_command(
        grid,
        spec,
        start,
        np.zeros(2, dtype=float),
        goal,
        MPPIControllerConfig(
            robot_radius_m=0.15,
            safety_distance_m=0.20,
            horizon_steps=18,
            num_rollouts=96,
            max_speed_mps=1.8,
            max_accel_mps2=2.2,
            seed=23,
        ),
    )
    if controller.velocity_sp_mps.shape != (2,) or controller.acceleration_sp_mps2.shape != (2,):
        raise RuntimeError("MPPI controller returned invalid setpoint shape")
    if not np.all(np.isfinite(controller.velocity_sp_mps)) or not np.all(np.isfinite(controller.acceleration_sp_mps2)):
        raise RuntimeError("MPPI controller returned non-finite setpoints")
    speed = float(np.linalg.norm(controller.velocity_sp_mps))
    accel = float(np.linalg.norm(controller.acceleration_sp_mps2))
    if speed > 1.8 + 1e-6 or accel > 2.2 + 1e-6:
        raise RuntimeError(f"MPPI controller exceeded limits: speed={speed:.2f}, accel={accel:.2f}")
    print(
        "mppi_controller: "
        f"velocity=({controller.velocity_sp_mps[0]:.2f},{controller.velocity_sp_mps[1]:.2f}) "
        f"accel=({controller.acceleration_sp_mps2[0]:.2f},{controller.acceleration_sp_mps2[1]:.2f}) "
        f"clearance={controller.min_clearance_m:.2f}"
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
