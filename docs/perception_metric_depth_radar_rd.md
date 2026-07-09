# Perception Upgrade: Metric Depth and Radar Range-Doppler

## Radar

The radar path now keeps the old Level-1 features and adds Level-3 range-Doppler features.

- Level 1: first chirp only -> 1D FFT magnitude -> `radar_peak`, `radar_peak_bin`, `radar_energy`.
- Level 3: all chirps in the sweep -> range FFT over fast time -> Doppler FFT over slow time -> compact range-Doppler features.

New classifier features:

- `radar_rd_peak`
- `radar_rd_range_bin`
- `radar_rd_doppler_bin`
- `radar_rd_energy`
- `radar_rd_near_energy`
- `radar_rd_doppler_spread`
- `radar_rd_range_spread`

This is not CFAR, angle-of-arrival estimation, target tracking, or radar occupancy mapping.

## Depth

The metric-depth path uses:

`RGB frame -> Depth Anything V2 Metric Indoor Small -> metric depth in meters -> point cloud -> local occupancy -> ESDF/planner`

Default model:

`depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf`

Local run config used for the updated slide table:

- Machine: MacBook Air with Apple M2, 16 GB RAM.
- Device: PyTorch MPS.
- Dataset scope: 15 local ODA trials with RGB videos available.
- Depth FPS: 5.
- Cached frames: 801.
- Mean inference time: 88.2 ms/frame.
- Mean wall time: 190.5 ms/frame.

## Outputs

- Metric depth timing: `outputs/tables/metric_depth_timing_depth_anything_v2_metric_indoor_small.csv`
- Perception-risk features: `outputs/tables/perception_risk_features_metric_depth_rd_15.csv`
- Macro-F1 table: `outputs/tables/perception_risk_ablation_balanced_metrics_metric_depth_rd_15.csv`
- Recall-tuned table: `outputs/tables/perception_risk_ablation_recall_tuned_metrics_metric_depth_rd_15.csv`
- Depth occupancy planner detail: `outputs/tables/depth_occupancy_planner_detail.csv`
- Depth occupancy planner summary: `outputs/tables/depth_occupancy_planner_summary.csv`

## Limitations

- Radar Level-3 still does not estimate angle-of-arrival or build radar occupancy maps.
- Monocular metric depth can have scale and domain errors; ODA depth-derived occupancy should be validated against metadata/OptiTrack before safety claims.
- The current depth occupancy vs GT occupancy comparison uses an approximate local projection from OptiTrack heading, not a fully calibrated camera extrinsic transform.
- MacBook M2/MPS latency is a development proxy, not onboard UAV latency.

