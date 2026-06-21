# Server Roadmap for ODA 300-Trial UAV Obstacle Avoidance

## Summary

Use the rented GPU server to turn the local ODA prototype into a stronger research-style project:

1. Scale the ODA benchmark from 100 to **300 balanced trials**.
2. Use GPU inference for predicted depth and perception-risk features, then test **ONNX Runtime / TensorRT** speedups.
3. Handle **future-risk class imbalance** using class weighting, train-only oversampling, threshold tuning, and balanced metrics.
4. Probe Multi-LiDAR / ARCO as external stress datasets because ODA is relatively controlled, but keep ODA as the core planner benchmark.

Default server assumptions:

- Linux server with CUDA GPU and at least 300 GB free disk.
- Repo path: `/workspace/uav-oda-obstacle-avoidance`.
- Full ODA ZIP: `/workspace/data/Dupeyroux_et_al_2021_ODA_DATASET_Full.zip`.
- ODA remains the primary dataset for safety/planner metrics.

Current server status after the follow-up run:

- ODA planner benchmark is complete at 300 trials.
- Depth Anything V2 Small has been cached on 50 ODA trials: 2584 frames, `0.0639 s/frame` wall time, `0.0174 s/frame` model inference time.
- Imbalance-aware risk ablations have been rerun on the 50-trial depth/radar/IMU table.
- TensorRT is blocked on the current Vast container because Docker/Podman, `trtexec`, Python `tensorrt`, and `libnvinfer.so.10` are missing. Use a TensorRT-enabled image for real engine timing.
- ARCO stress-test downloaded/probed 3 ROS2 bag ZIP samples; Multi-LiDAR link probe found 27/27 SharePoint links require login.

## Phase 0 - Server Setup

```bash
cd /workspace
git clone <YOUR_REPO_URL> uav-oda-obstacle-avoidance
cd /workspace/uav-oda-obstacle-avoidance

python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install numpy pandas matplotlib scipy opencv-python tqdm imageio imageio-ffmpeg
pip install torch torchvision transformers accelerate scikit-learn

# Optional for depth inference optimization:
pip install "optimum[exporters]" onnx onnxruntime-gpu
```

Sanity checks:

```bash
python3 - <<'PY'
import torch
print("CUDA available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("GPU:", torch.cuda.get_device_name(0))
PY

python3 scripts/audit_goal_status.py
```

Before downloading full ODA, the audit is expected to show `3/20 ready` locally while benchmark/risk/planner code passes.

## Phase 1 - Large ODA Benchmark

Run the existing human-vs-planner benchmark on 20, 100, then 300 trials. Keep metric definitions fixed across all scales:

```bash
mkdir -p /workspace/data

scripts/download_oda_full_zip.sh \
  /workspace/data/Dupeyroux_et_al_2021_ODA_DATASET_Full.zip

ODA_DATASET_ZIP="$(python3 scripts/unwrap_oda_full_zip.py \
  /workspace/data/Dupeyroux_et_al_2021_ODA_DATASET_Full.zip \
  --output /workspace/data/Dupeyroux_et_al_2021_ODA_DATASET.zip | tail -n 1)"

python3 scripts/create_oda_20_trial_manifest.py \
  --zip-path "${ODA_DATASET_ZIP}" \
  --output outputs/tables/target_20_trials_manifest.csv

python3 scripts/extract_oda_trials_from_full_zip.py \
  "${ODA_DATASET_ZIP}" \
  --manifest outputs/tables/target_20_trials_manifest.csv \
  --output-root data/raw/ODA_Dataset

scripts/run_oda_target20_benchmark.sh
python3 scripts/audit_goal_status.py
```

Required outputs:

- `outputs/tables/target_20_trials_readiness.csv`
- `outputs/tables/batch_planner_metrics.csv`
- `outputs/tables/planner_comparison_summary.csv`
- `outputs/tables/batch_skipped_trials.csv`
- `outputs/figures/planner_comparison_sample_<trial>.png`

Success criteria:

- `target_20_trials_readiness.csv` shows `20/20 ready`.
- `audit_goal_status.py` reports complete for the 20-trial goal.
- Planner summary includes `human`, `straight_line`, `geometric_bypass`, `astar_grid`, and `rrt`.
- The main table reports collision rate, safety violation rate, mean clearance, path length, smoothness, and planner compute time.

Scale to 100 or 300 trials:

```bash
python3 scripts/create_oda_trial_manifest.py \
  --zip-path "${ODA_DATASET_ZIP}" \
  --total 300 \
  --output outputs_300/tables/target_300_trials_manifest.csv

python3 scripts/extract_oda_trials_from_full_zip.py \
  "${ODA_DATASET_ZIP}" \
  --manifest outputs_300/tables/target_300_trials_manifest.csv \
  --output-root data/raw/ODA_Dataset

scripts/run_oda_manifest_benchmark.sh \
  outputs_300/tables/target_300_trials_manifest.csv \
  outputs_300
```

Required 300-trial outputs:

- `outputs_300/tables/target_300_trials_manifest.csv`
- `outputs_300/tables/target_300_trials_manifest_readiness.csv`
- `outputs_300/tables/batch_planner_metrics.csv`
- `outputs_300/tables/planner_comparison_summary.csv`
- `outputs_300/tables/batch_skipped_trials.csv`
- `outputs_300/tables/planner_failures.csv`

Success criteria:

- `target_300_trials_manifest_readiness.csv` shows `300/300 ready`.
- Planner summary includes `human`, `straight_line`, `geometric_bypass`, `astar_grid`, `rrt`, `rrt_star`, and `mppi`.
- Failures are logged, not allowed to kill the batch.
- Report compares safety and compute time at 20/100/300 scale.

## Phase 2 - GPU Perception-Risk

Cache predicted depth on all ready target trials:

```bash
source .venv/bin/activate

python3 scripts/batch_cache_oda_depth.py \
  --readiness outputs/tables/target_20_trials_readiness.csv \
  --fps 5 \
  --device cuda
```

Build perception-risk features:

```bash
python3 experiments/build_perception_risk_features.py \
  --readiness outputs/tables/target_20_trials_readiness.csv \
  --output outputs/tables/perception_risk_features.csv
```

Train a small risk classifier:

```bash
python3 experiments/train_perception_risk_classifier.py \
  --features outputs/tables/perception_risk_features.csv \
  --metrics-output outputs/tables/perception_risk_metrics.csv \
  --figure-output outputs/figures/perception_risk_confusion_matrix.png
```

Run sensor ablations:

```bash
python3 experiments/train_perception_risk_ablation.py \
  --features outputs/tables/perception_risk_features.csv \
  --output outputs/tables/perception_risk_ablation_metrics.csv \
  --figure-output outputs/figures/perception_risk_ablation.png
```

Run imbalance-aware ablations:

```bash
python3 experiments/train_perception_risk_ablation.py \
  --features outputs/tables/perception_risk_features.csv \
  --output outputs/tables/perception_risk_ablation_balanced_metrics.csv \
  --figure-output outputs/figures/perception_risk_ablation_balanced.png \
  --class-weight balanced \
  --resample-train oversample \
  --optimize-threshold \
  --threshold-metric macro_f1
```

Optimized batch depth inference loads the model once and processes multiple
frames per forward pass:

```bash
python3 experiments/cache_monocular_depth_batch.py \
  --readiness outputs/tables/target_20_trials_readiness.csv \
  --fps 5 \
  --device cuda \
  --batch-size 8 \
  --output-root data/processed/depth_batch \
  --timing-output outputs/tables/depth_batch_timing.csv
```

ONNX Runtime and TensorRT optimization path:

```bash
scripts/export_depth_to_onnx.sh \
  Intel/dpt-hybrid-midas \
  models/depth_onnx/dpt_hybrid_midas

python3 experiments/cache_monocular_depth_onnx.py \
  --readiness outputs/tables/target_20_trials_readiness.csv \
  --onnx-model models/depth_onnx/dpt_hybrid_midas/model.onnx \
  --fps 5 \
  --provider cuda \
  --batch-size 8 \
  --output-root data/processed/depth_onnx_dpt_20 \
  --timing-output outputs/tables/depth_onnx_timing_dpt_20.csv

scripts/build_tensorrt_depth_engine.sh \
  models/depth_onnx/dpt_hybrid_midas/model.onnx \
  models/depth_tensorrt/dpt_hybrid_midas_fp16.engine
```

Notes:

- ONNX Runtime is the practical inference fallback if TensorRT is not installed.
- TensorRT engine build requires `trtexec` and TensorRT runtime libraries; use an NVIDIA TensorRT container/image if the server image lacks them.
- Always compare timing against `outputs/tables/depth_batch_timing_dpt_20.csv`.

Depth stability and weak calibration against clearance/radar:

```bash
python3 experiments/analyze_depth_stability_calibration.py \
  --features \
    dpt=outputs/tables/perception_risk_features.csv \
    depth_anything=outputs/tables/perception_risk_features_depth_anything_v2_small.csv \
  --output outputs/tables/depth_stability_calibration.csv \
  --figure-output outputs/figures/depth_stability_calibration.png
```

Required outputs:

- `outputs/tables/perception_risk_features.csv`
- `outputs/tables/perception_risk_metrics.csv`
- `outputs/tables/perception_risk_ablation_metrics.csv`
- `outputs/tables/perception_risk_ablation_balanced_metrics.csv`
- `outputs/tables/depth_batch_timing.csv`
- `outputs/tables/depth_onnx_timing_dpt_20.csv`
- `outputs/tables/depth_stability_calibration.csv`
- `outputs/tables/depth_batch_timing_depth_anything_v2_small_50.csv`
- `outputs/tables/perception_risk_features_depth_anything_v2_small_50.csv`
- `outputs/tables/perception_risk_ablation_balanced_metrics_depth_anything_v2_small_50.csv`
- `outputs/tables/perception_risk_ablation_recall_tuned_metrics_depth_anything_v2_small_50.csv`
- `outputs/tables/depth_stability_calibration_depth_anything_v2_small_50.csv`
- `outputs/figures/perception_risk_confusion_matrix.png`
- `outputs/figures/perception_risk_ablation.png`
- `outputs/figures/depth_stability_calibration.png`
- qualitative video with RGB + depth + radar + IMU + risk timeline.

Success criteria:

- Depth cache exists for all 20 ready trials.
- Extended Depth Anything V2 Small cache/timing exists for at least 50 ODA trials.
- The classifier report includes majority accuracy, balanced accuracy, macro-F1, future-risk recall, and train/test positive rates.
- The imbalance-aware model is judged by recall/macro-F1, not accuracy alone.
- The report clearly states that depth is relative monocular depth, not metric depth.

## Phase 3 - Advanced Planner Comparison

RRT* and MPPI are integrated into `experiments/batch_benchmark_planners.py` and can be enabled by default or skipped:

```bash
python3 experiments/batch_benchmark_planners.py \
  --manifest outputs/tables/target_20_trials_manifest.csv \
  --readiness outputs/tables/target_20_trials_readiness.csv \
  --ready-only
```

Useful flags:

- `--skip-rrt-star`
- `--skip-mppi`
- `--mppi-rollouts`
- `--planner-seed`

Create the advanced summary:

```bash
python3 scripts/write_advanced_planner_summary.py
```

Required outputs:

- updated `outputs/tables/batch_planner_metrics.csv`
- updated `outputs/tables/planner_comparison_summary.csv`
- planner comparison plots
- `outputs/advanced_planner_summary.md`

Success criteria:

- RRT* and MPPI run on all ready trials without crashing.
- Failed planners are logged as warnings and do not kill the whole batch.
- The comparison reports safety and compute time, not just path length.

## Phase 4 - Cross-Dataset Extension

Use other README sources to test whether ODA is too easy and to strengthen positioning without diluting the implementation:

- Multi-LiDAR Multi-UAV Dataset: UAV tracking/perception stress reference with LiDAR, RGB-D/RGB, IMU, and MOCAP. Start with one hard short sequence only after ODA 300 is stable.
- ARCO Dataset: radar/LiDAR/IMU context with direct ROS2 bag ZIPs. It is ground-robot data, so use it for sensor parsing/context, not UAV planner metrics.
- MPPI controller repo: conceptual reference for cost terms, not a ROS dependency.
- FAST-LIVO2: related work/future work unless a separate SLAM task starts.
- HEPP paper: motivation for high-speed UAV planning and low-latency planning.

Create the external dataset probe:

```bash
python3 scripts/probe_external_datasets.py \
  --output outputs/tables/external_dataset_probe.csv \
  --summary-output outputs/external_dataset_extension_plan.md

python3 scripts/probe_arco_rosbag_sqlite.py /workspace/data/arco/*.zip \
  --output outputs/tables/arco_rosbag_topic_probe.csv \
  --summary-output outputs/arco_rosbag_stress_probe.md

python3 scripts/probe_multilidar_download_links.py \
  --output outputs/tables/multilidar_download_link_probe.csv \
  --summary-output outputs/multilidar_download_probe.md

python3 scripts/write_cross_dataset_positioning.py
```

Required outputs:

- `outputs/tables/external_dataset_probe.csv`
- `outputs/tables/arco_rosbag_topic_probe.csv`
- `outputs/tables/multilidar_download_link_probe.csv`
- `outputs/external_dataset_extension_plan.md`
- `outputs/arco_rosbag_stress_probe.md`
- `outputs/multilidar_download_probe.md`
- `outputs/cross_dataset_positioning.md`

Success criteria:

- Project narrative is: ODA-based multi-sensor risk-aware UAV obstacle avoidance benchmark.
- The report states why ODA is controlled/easier and why 300-trial scaling plus external stress probes are needed.
- ARCO probe is used for radar/LiDAR/IMU sensing stress only; Multi-LiDAR inaccessible links are documented instead of blocking ODA work.
- Other datasets support stress testing and positioning, not implementation scope creep.

## Final Deliverables To Pull Back From Server

Download only:

```text
outputs/
reports/
src/
experiments/
scripts/
docs/
plan.md
```

Do not download the full 98 GB ODA ZIP unless needed.

Final verification:

```bash
python3 scripts/audit_goal_status.py
python3 -m py_compile src/*.py src/planners/*.py experiments/*.py scripts/*.py
```

Expected after Phase 1:

- 300/300 ODA target trials ready.
- Batch benchmark complete.
- Risk labels present.
- Human-vs-planner comparison complete.
- A*, RRT, RRT*, and MPPI included.
- Report/summary updated with 300-trial results.

Expected after all phases:

- 300-trial ODA benchmark.
- Depth/radar/IMU perception-risk table.
- Imbalance-aware risk classifier baseline.
- ONNX Runtime depth timing and TensorRT engine-build attempt/result.
- RRT*, MPPI comparison.
- Cross-dataset probe table and positioning note.
