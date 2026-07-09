# Level 3 Realistic Indoor Video Summary

Generated on the RTX 4090 Vast server at `159.48.242.14:41997`.

## Main Video

- `outputs/videos/level3_realistic_indoor_chase_fused_esdf_mppi.mp4`
- 1920x1080, 24 fps, 15 s, 360 frames.
- Recommended for mentor demo because it shows the MAV body, indoor route, obstacle events, LiDAR point cloud, 3D bounding boxes, relative-depth inset, LiDAR inset, voxel/ESDF map and MPPI replanning path.

## Onboard POV Video

- `outputs/videos/level3_realistic_indoor_pov_esdf_mppi.mp4`
- 1920x1080, 24 fps, 15 s, 360 frames.
- Use as onboard-camera style evidence. It is intentionally closer to a drone camera view, so it is less visually open than the chase version.

## Verification

- Render validator: `outputs/figures/level3_video_preview/level3_realistic_render_check.json`.
- Desktop and mobile WebGL canvas checks passed.
- Preview frames:
  - `outputs/figures/level3_video_preview/level3_realistic_chase_midframe.png`
  - `outputs/figures/level3_video_preview/level3_realistic_chase_lateframe.png`
  - `outputs/figures/level3_video_preview/level3_realistic_midframe.png`
  - `outputs/figures/level3_video_preview/level3_realistic_lateframe.png`

## Scope

This is a realistic WebGL/Three.js visualization of the full 3D perception-to-planner story. It is not Isaac Sim, Gazebo physics, PX4 SITL or a closed-loop UAV controller. The correct claim is:

```text
indoor geometry + dynamic obstacles + fused visual sensor panels + 3D bbox/point-cloud visualization + voxel/ESDF map + MPPI-style replanning demo
```

The heavier future version should move this same concept to Isaac Sim or Gazebo/PX4 with real depth/LiDAR topics and controller feedback.
