# 4-Phase Server Roadmap for ODA UAV Obstacle Avoidance

## Summary

Use the rented GPU server to turn the local ODA prototype into a stronger research-style project:

1. Scale the ODA benchmark from 3 local samples to 20-100 trials.
2. Use GPU inference for predicted depth and perception-risk features.
3. Add advanced planner baselines: RRT* and lightweight Python MPPI.
4. Use Multi-LiDAR / ARCO only for positioning or future validation, not as the core dataset.

Default server assumptions:

- Linux server with CUDA GPU and at least 300 GB free disk.
- Repo path: `/workspace/uav-oda-obstacle-avoidance`.
- Full ODA ZIP: `/workspace/data/Dupeyroux_et_al_2021_ODA_DATASET_Full.zip`.
- ODA remains the primary dataset.

## Phase 0 - Server Setup

```bash
cd /workspace
git clone <YOUR_REPO_URL> uav-oda-obstacle-avoidance
cd /workspace/uav-oda-obstacle-avoidance

python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install numpy pandas matplotlib scipy opencv-python tqdm torch torchvision transformers accelerate scikit-learn
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

Run the existing human-vs-planner benchmark on at least 20 trials:

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

Stretch after 20 trials:

- Add `outputs/tables/target_50_trials_manifest.csv`.
- Select balanced full-light trials with valid obstacle coordinates.
- Reuse the same extraction and benchmark scripts.
- Do not change metric definitions between 20-trial and 50-trial runs.

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

Required outputs:

- `outputs/tables/perception_risk_features.csv`
- `outputs/tables/perception_risk_metrics.csv`
- `outputs/figures/perception_risk_confusion_matrix.png`
- qualitative video with RGB + depth + radar + IMU + risk timeline.

Success criteria:

- Depth cache exists for all 20 ready trials.
- The classifier beats the majority-class baseline.
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

Use other README sources to strengthen positioning without diluting the implementation:

- Multi-LiDAR Multi-UAV Dataset: future-work or small validation only.
- ARCO Dataset: radar/LiDAR/IMU context, but ground-robot data rather than UAV avoidance.
- MPPI controller repo: conceptual reference for cost terms, not a ROS dependency.
- FAST-LIVO2: related work/future work unless a separate SLAM task starts.
- HEPP paper: motivation for high-speed UAV planning and low-latency planning.

Create the positioning note:

```bash
python3 scripts/write_cross_dataset_positioning.py
```

Required output:

- `outputs/cross_dataset_positioning.md`

Success criteria:

- Project narrative is: ODA-based multi-sensor risk-aware UAV obstacle avoidance benchmark.
- Other datasets support positioning, not implementation scope creep.

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

- 20/20 ODA target trials ready.
- Batch benchmark complete.
- Risk labels present.
- Human-vs-planner comparison complete.
- A* and RRT included.
- Report/summary updated with 20-trial results.

Expected after all phases:

- 20-100 trial ODA benchmark.
- Depth/radar/IMU perception-risk table.
- Risk classifier baseline.
- RRT*, MPPI comparison.
- Cross-dataset positioning note.
