# Server Experiment Summary

## ODA 300-Trial Planner Benchmark

- Readiness: 300/300 ready.
- Metrics rows: 2100.
- Skipped trials: 0; planner failures: 0.

| Method | Trials | Collision | Violation | Mean clearance | Compute ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| astar_grid | 300 | 0.0 | 0.0 | 0.6426 | 8.6793 |
| geometric_bypass | 196 | 0.0 | 0.0051 | 0.7005 | 6.386 |
| geometric_bypass_not_needed | 104 | 0.0 | 0.0 | 0.8647 | 2.463 |
| human | 300 | 0.0733 | 0.4467 | 0.6126 | 0.0 |
| mppi | 300 | 0.0 | 0.0033 | 0.7574 | 39.9327 |
| rrt | 300 | 0.0 | 0.0 | 0.6702 | 3.6168 |
| rrt_star | 300 | 0.0 | 0.0 | 0.636 | 1196.3979 |
| straight_line | 300 | 0.14 | 0.6533 | 0.4203 | 2.8712 |

## Macro-F1 threshold tuning

| Feature group | Accuracy | Macro-F1 | Balanced acc. | Future-risk recall | Threshold |
| --- | ---: | ---: | ---: | ---: | ---: |
| radar | 0.8385 | 0.4561 | 0.5 | 0.0 | 0.8 |
| camera_depth | 0.7919 | 0.5269 | 0.5266 | 0.1346 | 0.4 |
| radar_imu | 0.8385 | 0.4561 | 0.5 | 0.0 | 0.8 |
| depth_radar_imu | 0.854 | 0.5615 | 0.5558 | 0.1154 | 0.45 |

## Future-risk recall threshold tuning

| Feature group | Accuracy | Macro-F1 | Balanced acc. | Future-risk recall | Threshold |
| --- | ---: | ---: | ---: | ---: | ---: |
| radar | 0.7236 | 0.6533 | 0.7731 | 0.8462 | 0.05 |
| camera_depth | 0.6832 | 0.5828 | 0.6481 | 0.5962 | 0.05 |
| radar_imu | 0.6708 | 0.6043 | 0.7261 | 0.8077 | 0.05 |
| depth_radar_imu | 0.6366 | 0.5835 | 0.729 | 0.8654 | 0.05 |

## ONNX Depth Probe

- ONNX Runtime CUDA probe: 3 trials, 177 frames, wall/frame 0.082662s, inference/frame 0.040498s.
- TensorRT CLI `trtexec` was not installed. ONNX Runtime TensorRT EP failed because `libnvinfer.so.10` was missing, so TensorRT speedup is not claimed.
