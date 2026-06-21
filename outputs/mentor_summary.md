# ODA UAV Obstacle Avoidance: Current Results

## Dataset And Benchmark Status

- Full ODA archive has been used on the GPU server.
- 300/300 target ODA trials are ready.
- Planner benchmark output contains 2100 metric rows.
- Risk labels and future-risk counts are included in the planner metrics.
- ODA remains the primary UAV obstacle-avoidance benchmark.

## Main Planner Result

| method | trials | collision rate | safety violation rate | mean clearance m | mean compute ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| human | 300 | 0.0733 | 0.4467 | 0.6126 | 0.0000 |
| straight_line | 300 | 0.1400 | 0.6533 | 0.4203 | 2.8712 |
| geometric_bypass | 196 | 0.0000 | 0.0051 | 0.7005 | 6.3860 |
| astar_grid | 300 | 0.0000 | 0.0000 | 0.6426 | 8.6793 |
| rrt | 300 | 0.0000 | 0.0000 | 0.6702 | 3.6168 |
| rrt_star | 300 | 0.0000 | 0.0000 | 0.6360 | 1196.3979 |
| mppi | 300 | 0.0000 | 0.0033 | 0.7574 | 39.9327 |

Key takeaway: the geometric/planner benchmark is stable at 300 ODA trials. The stronger research angle is now perception-risk and sensing generalization, not just path geometry.

## Perception-Risk Result

- Depth Anything V2 Small was cached on 50 ODA trials: 2584 frames.
- Batch PyTorch CUDA timing: `0.0639 s/frame` wall time and `0.0174 s/frame` model inference time.
- Depth/radar/IMU feature table: `outputs/tables/perception_risk_features_depth_anything_v2_small_50.csv`.
- Recall-tuned depth+radar+IMU reaches future-risk recall `0.6667` with balanced accuracy `0.6631`.
- Depth remains relative monocular depth; it is not metric depth until calibrated against ODA ground truth/radar.

## TensorRT And External Data

- TensorRT engine timing is not claimed on the current Vast container: Docker/Podman, `trtexec`, Python `tensorrt`, and `libnvinfer.so.10` are missing.
- ARCO stress probe downloaded/probed 3 ROS2 bag ZIP samples without ROS: 33 topic rows, 175997 messages.
- Multi-LiDAR link probe found 27/27 SharePoint links require login, so it is documented as future/auth-needed stress data.

## Files To Show

- `reports/uav_oda_report.pdf`
- `outputs/server_experiment_summary.md`
- `outputs/arco_rosbag_stress_probe.md`
- `outputs/multilidar_download_probe.md`
- `docs/depth_inference_optimization.md`
- `docs/tensorrt_container_runbook.md`
