# ODA-Bench: UAV Obstacle Avoidance Evaluation

This repository contains the code and compact artifacts used in the report
`reports/uav_oda_report.pdf`. The scope is intentionally limited to the
experiments reported there: offline ODA-Bench construction, planner baselines,
trajectory-quality checks, latency replay, behavior cloning, risk prediction,
and PyBullet validation.

## Dataset

Primary dataset:

- ODA Dataset: https://github.com/JuSquare/ODA_Dataset

Expected local layout:

```text
data/raw/ODA_Dataset/dataset/
  trial_overview.csv
  1/optitrack.csv
  1/radar.csv
  ...
```

The raw ODA data is not committed. Place it under `data/raw/ODA_Dataset/` before
rerunning the experiments.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Optional for simulator validation:

```bash
pip install gym-pybullet-drones pybullet
```

## Report Artifacts

Main report:

```text
reports/uav_oda_report.tex
reports/uav_oda_report.pdf
```

Key tables and figures used by the report:

```text
outputs/tables/batch_planner_metrics_300.csv
outputs/tables/planner_comparison_summary_300.csv
outputs/tables/reviewer_radius_margin_sensitivity.csv
outputs/tables/reviewer_rate_confidence_intervals.csv
outputs/tables/reviewer_planner_budget_sweep_summary.csv
outputs/tables/trajectory_feasibility_3d_summary.csv
outputs/tables/sensor_frontend_latency_feasibility.csv
outputs/tables/sensor_frontend_feasibility_rates.csv
outputs/tables/oda_replay_online_feasibility_by_planner.csv
outputs/tables/oda_policy_dataset_samples.csv
outputs/tables/oda_bc_fit_metrics.csv
outputs/tables/oda_bc_rollout_detail.csv
outputs/tables/oda_risk_labeled_samples.csv
outputs/tables/reviewer_risk_operating_points.csv
outputs/tables/pybullet_validation_results.csv
outputs/tables/pybullet_validation_detail.csv
outputs/figures/trajectory_feasibility_3d_trial_345.png
```

## Experiments in the Report

### 1. ODA-Bench Planner Baselines

Purpose: evaluate human trajectory, straight-line, A*, geometric bypass, RRT,
RRT*, and MPPI on 300 ODA trials with a shared collision/violation/clearance
protocol.

Relevant code:

```text
experiments/benchmark_oda.py
experiments/batch_benchmark_planners.py
src/metrics.py
src/oda_io.py
src/planners/
```

Main artifacts:

```text
outputs/tables/batch_planner_metrics_300.csv
outputs/tables/planner_comparison_summary_300.csv
outputs/tables/planner_failures.csv
outputs/figures/oda_planner_safety_300.png
outputs/figures/oda_planner_compute_rrtstar_mppi.png
```

Typical rerun:

```bash
python experiments/batch_benchmark_planners.py \
  --dataset-root data/raw/ODA_Dataset/dataset \
  --manifest outputs/tables/target_300_trials_manifest.csv \
  --limit 300 \
  --outputs-dir outputs
```

### 2. Reviewer-Requested Statistics

Purpose: recompute sensitivity and uncertainty from saved artifacts, including
safety-margin sensitivity, Wilson confidence intervals, and risk operating
points.

Relevant code:

```text
experiments/analyze_reviewer_requested_metrics.py
```

Main artifacts:

```text
outputs/tables/reviewer_radius_margin_sensitivity.csv
outputs/tables/reviewer_rate_confidence_intervals.csv
outputs/tables/reviewer_risk_operating_points.csv
```

Typical rerun:

```bash
python experiments/analyze_reviewer_requested_metrics.py --outputs-dir outputs
```

### 3. Planner Budget Sweep

Purpose: compare RRT/RRT*/MPPI under smaller compute budgets on a 45-trial
subset.

Relevant code:

```text
experiments/benchmark_planner_budget_sweep.py
```

Main artifacts:

```text
outputs/tables/reviewer_planner_budget_sweep_summary.csv
outputs/tables/reviewer_planner_budget_sweep_detail.csv
outputs/tables/reviewer_planner_budget_sweep_failures.csv
```

Typical rerun:

```bash
python experiments/benchmark_planner_budget_sweep.py \
  --dataset-root data/raw/ODA_Dataset/dataset \
  --manifest outputs/tables/target_300_trials_manifest.csv \
  --limit 45 \
  --outputs-dir outputs
```

### 4. 3D Trajectory-Quality Proxy

Purpose: lift footprint paths to simple 3D paths and report length, smoothness,
and max-turn proxies on 16 ODA trials.

Relevant code:

```text
experiments/analyze_trajectory_feasibility_3d.py
```

Main artifacts:

```text
outputs/tables/trajectory_feasibility_3d_summary.csv
outputs/tables/trajectory_feasibility_3d_detail.csv
outputs/figures/trajectory_feasibility_3d_trial_345.png
```

Typical rerun:

```bash
python experiments/analyze_trajectory_feasibility_3d.py \
  --dataset-root data/raw/ODA_Dataset/dataset
```

### 5. Sensor Front-End Latency and Feasibility Rates

Purpose: estimate sensor/front-end/planner delay envelopes for ODA RGB/cached
depth and related occupancy-grid pipelines.

Relevant code:

```text
experiments/benchmark_sensor_frontend_latency_feasibility.py
experiments/benchmark_sensor_frontend_feasibility_rates.py
```

Main artifacts:

```text
outputs/tables/sensor_frontend_latency_feasibility.csv
outputs/tables/sensor_frontend_feasibility_rates.csv
outputs/tables/sensor_frontend_feasibility_rates_overall.csv
outputs/figures/sensor_frontend_latency_feasibility.png
outputs/figures/sensor_frontend_feasibility_rates.png
```

Typical rerun:

```bash
python experiments/benchmark_sensor_frontend_latency_feasibility.py --prefer-scipy
python experiments/benchmark_sensor_frontend_feasibility_rates.py --prefer-scipy --cases-per-speed 8
```

### 6. ODA Replay Online Feasibility

Purpose: replay OptiTrack poses with an explicit sensor-to-command delay model
and replan from delayed poses.

Relevant code:

```text
experiments/benchmark_oda_replay_online_feasibility.py
```

Main artifacts:

```text
outputs/tables/oda_replay_online_feasibility_by_planner.csv
outputs/tables/oda_replay_online_feasibility_by_speed_bin.csv
outputs/tables/oda_replay_online_feasibility_detail.csv
outputs/figures/oda_replay_online_feasibility.png
```

Typical rerun:

```bash
python experiments/benchmark_oda_replay_online_feasibility.py \
  --dataset-root data/raw/ODA_Dataset/dataset \
  --cases-per-trial 8 \
  --output-dir outputs
```

### 7. Behavior Cloning

Purpose: build a timestep state-action policy dataset and compare Plain BC-MPPI
against Filtered BC-MPPI.

A policy-dataset sample is one resampled timestep state-action pair. The state
contains footprint pose, relative goal vector, nearest-obstacle vector,
velocity, clearance, and obstacle count. The action is the local displacement
to the next waypoint.

Relevant code:

```text
experiments/build_oda_policy_dataset.py
experiments/train_bc_mppi.py
src/oda_bench_downstream.py
```

Main artifacts:

```text
outputs/tables/oda_policy_dataset_samples.csv
outputs/tables/oda_policy_dataset_trials.csv
outputs/tables/oda_bc_fit_metrics.csv
outputs/tables/oda_bc_rollout_detail.csv
outputs/figures/bc_plain_vs_filtered_safety.png
outputs/figures/bc_filtered_vs_plain_300.png
```

Typical rerun:

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
```

### 8. Risk Prediction

Purpose: compare TTC/distance heuristics with a small MLP future-risk predictor
on the same split.

Relevant code:

```text
experiments/train_risk_predictor.py
experiments/analyze_reviewer_requested_metrics.py
```

Main artifacts:

```text
outputs/tables/oda_risk_labeled_samples.csv
outputs/tables/oda_risk_results.csv
outputs/tables/reviewer_risk_operating_points.csv
outputs/figures/risk_predictor_pr_curve.png
```

Typical rerun:

```bash
python experiments/train_risk_predictor.py \
  --samples outputs/tables/oda_policy_dataset_samples.csv \
  --outputs-dir outputs

python experiments/analyze_reviewer_requested_metrics.py --outputs-dir outputs
```

### 9. PyBullet Validation

Purpose: validate selected candidates in a closed-loop PyBullet gate with
20 cases/method at speed 1 m/s, as reported in the paper.

Relevant code:

```text
experiments/run_pybullet_oda_course.py
experiments/summarize_offline_vs_sim.py
```

Main artifacts:

```text
outputs/tables/pybullet_validation_results.csv
outputs/tables/pybullet_validation_detail.csv
outputs/tables/offline_vs_sim_rank_correlation.csv
outputs/figures/offline_vs_sim_rank_correlation.png
outputs/figures/validation_course_summary.png
```

Typical rerun:

```bash
python experiments/run_pybullet_oda_course.py \
  --outputs-dir outputs \
  --seeds 20 \
  --speeds 1.0 \
  --backend gym-pybullet-drones

python experiments/summarize_offline_vs_sim.py
```

## Build the Report

```bash
tectonic reports/uav_oda_report.tex --outdir reports
```

The generated PDF is:

```text
reports/uav_oda_report.pdf
```

## Notes

- Raw ODA logs are intentionally excluded from git.
- Large videos and runtime scratch folders are ignored.
- Some PyBullet dependencies are optional and only required for rerunning the
  simulator validation.
- The README does not document exploratory files that are outside the submitted
  report scope.
