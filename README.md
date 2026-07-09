# ODA-Bench: UAV Obstacle Avoidance Evaluation

Code for the report `reports/uav_oda_report.pdf`. The project turns ODA raw logs
into an offline UAV obstacle-avoidance benchmark with planner baselines,
latency replay, behavior cloning, risk prediction, and PyBullet validation.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Optional, only for PyBullet validation:

```bash
pip install gym-pybullet-drones pybullet
```

## Data

Download or place the ODA Dataset under:

```text
data/raw/ODA_Dataset/dataset/
```

Expected files include:

```text
data/raw/ODA_Dataset/dataset/trial_overview.csv
data/raw/ODA_Dataset/dataset/<trial_id>/optitrack.csv
data/raw/ODA_Dataset/dataset/<trial_id>/radar.csv
```

Raw data is not committed to git.

## Run Experiments

Planner benchmark:

```bash
python experiments/batch_benchmark_planners.py \
  --dataset-root data/raw/ODA_Dataset/dataset \
  --manifest outputs/tables/target_300_trials_manifest.csv \
  --limit 300 \
  --outputs-dir outputs
```

Reviewer/statistical tables:

```bash
python experiments/analyze_reviewer_requested_metrics.py --outputs-dir outputs
python experiments/benchmark_planner_budget_sweep.py \
  --dataset-root data/raw/ODA_Dataset/dataset \
  --manifest outputs/tables/target_300_trials_manifest.csv \
  --limit 45 \
  --outputs-dir outputs
```

3D trajectory proxy:

```bash
python experiments/analyze_trajectory_feasibility_3d.py \
  --dataset-root data/raw/ODA_Dataset/dataset
```

Latency and ODA replay:

```bash
python experiments/benchmark_sensor_frontend_latency_feasibility.py --prefer-scipy
python experiments/benchmark_sensor_frontend_feasibility_rates.py --prefer-scipy --cases-per-speed 8
python experiments/benchmark_oda_replay_online_feasibility.py \
  --dataset-root data/raw/ODA_Dataset/dataset \
  --cases-per-trial 8 \
  --output-dir outputs
```

Behavior cloning and risk prediction:

```bash
python experiments/build_oda_policy_dataset.py \
  --dataset-root data/raw/ODA_Dataset/dataset \
  --readiness outputs/tables/target_300_trials_readiness.csv \
  --limit-trials 300 \
  --outputs-dir outputs

python experiments/train_bc_mppi.py \
  --dataset outputs/datasets/oda_mppi_policy_dataset.npz \
  --trial-specs outputs/datasets/oda_policy_trial_specs.csv \
  --outputs-dir outputs

python experiments/train_risk_predictor.py \
  --samples outputs/tables/oda_policy_dataset_samples.csv \
  --outputs-dir outputs
```

PyBullet validation:

```bash
python experiments/run_pybullet_oda_course.py \
  --outputs-dir outputs \
  --seeds 20 \
  --speeds 1.0 \
  --backend gym-pybullet-drones

python experiments/summarize_offline_vs_sim.py
```

## Build Report

```bash
tectonic reports/uav_oda_report.tex --outdir reports
```

The generated report is:

```text
reports/uav_oda_report.pdf
```
