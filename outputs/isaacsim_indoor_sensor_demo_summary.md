# Isaac Sim Indoor RGB-D/LiDAR Demo

This artifact is a headless Isaac Sim render for the indoor UAV obstacle-avoidance story.

It shows an onboard RGB stream, distance-to-camera depth, a LiDAR/point-cloud top-down panel, dynamic indoor obstacles, and the fused obstacle-field/planner timeline.

```json
{
  "output": "outputs/videos/isaacsim_indoor_rgbd_lidar_dynamic_demo.mp4",
  "preview": "outputs/figures/isaacsim_demo/isaacsim_indoor_midframe.png",
  "metrics": "outputs/tables/isaacsim_indoor_sensor_demo_metrics.csv",
  "frames": 180,
  "fps": 18,
  "render_size": [
    960,
    540
  ],
  "elapsed_s": 70.808,
  "isaac_renderer": "RayTracedLighting",
  "sensor_status": {
    "rgb_depth": "isaac_replicator",
    "lidar": "geometry_raycast_panel"
  },
  "min_clearance_m": 0.3225,
  "risk_counts": {
    "SAFE": 64,
    "WARNING": 66,
    "DANGER": 50,
    "COLLISION": 0
  }
}
```
