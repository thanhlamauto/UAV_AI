# PX4 MPPI Offboard Controller Status

## What Changed

The ROS2/PX4 path now has an MPPI local-controller option:

```text
LiDAR/depth/radar costmaps
  -> costmap_mux
  -> /perception/occupancy_grid
  -> px4_mppi_offboard_controller
  -> /fmu/in/offboard_control_mode
  -> /fmu/in/trajectory_setpoint
```

The controller is receding-horizon:

- State: `[x, y, vx, vy]`
- Control: `[ax, ay]`
- Output to PX4: horizontal velocity and acceleration setpoints
- Altitude: fixed altitude hold through the PX4 trajectory setpoint
- Cost terms: goal progress, terminal goal distance, clearance penalty,
  collision penalty, speed, acceleration and acceleration smoothness

## New Files

- `ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/mppi_local_controller.py`
- `ros2_ws/src/uav_oda_ros2_demo/uav_oda_ros2_demo/px4_mppi_offboard_controller_node.py`

## How To Run On PX4/Gazebo Server

Default MPPI controller:

```bash
PX4_CONTROLLER=mppi START_PX4=1 scripts/run_ros2_gazebo_fused_px4.sh astar
```

If PX4 is already running:

```bash
PX4_CONTROLLER=mppi START_PX4=0 scripts/run_ros2_gazebo_fused_px4.sh astar
```

Fallback waypoint bridge:

```bash
PX4_CONTROLLER=waypoint scripts/run_ros2_gazebo_fused_px4.sh astar
```

## Local Verification

Static checks pass locally:

```text
python3 -m py_compile ... OK
python3 scripts/check_ros2_launch_contract.py OK
.venv/bin/python scripts/check_ros2_costmap_demo_static.py OK
```

The local smoke test confirms the controller produces bounded velocity and
acceleration setpoints from an occupancy grid.

## Scope

This is a controller-code upgrade and static verification.  The remaining
runtime deliverable is a PX4/Gazebo SITL recording showing closed-loop vehicle
dynamics under the fused sensor costmap.
