# Level 3 Indoor Doorway-Turn ESDF/MPPI Demo

This demo addresses the failure mode where the UAV solves obstacle
avoidance by climbing above obstacles.  The simulated indoor environment
has a low ceiling, a full partition wall, and only one valid door
opening.  The path therefore has to turn through the doorway.

- Video: `outputs/videos/level3_indoor_doorway_turn_esdf_mppi.mp4`
- Preview: `outputs/figures/level3_video_preview/level3_doorway_turn_midframe.png`
- Metrics: `outputs/tables/level3_indoor_doorway_turn_mppi.csv`
- Door opening: `x=3.65 m`, `y in [-0.62, 0.62] m`, usable height below the lintel.
- Minimum ESDF clearance: `0.426 m`
- Door crossing: `y=+0.025 m`, `z=1.059 m`
- Maximum altitude: `1.090 m`
- Path length: `8.875 m`
- Smoothness: `0.0564`

Scope note: this is a lightweight procedural ESDF/MPPI visualization,
not a PX4/Gazebo closed-loop flight.  It is meant to demonstrate the
constrained indoor planner story before moving the same scenario into
Isaac Sim or Gazebo.
