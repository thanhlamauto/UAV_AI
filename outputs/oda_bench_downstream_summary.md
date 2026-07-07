# ODA-Bench Downstream Experiment Summary

Contribution phrase:

> We propose ODA-Bench, a lightweight real-log benchmark for UAV obstacle avoidance. It converts ODA trajectories and sensor logs into clearance/collision/latency labels, evaluates planners and lightweight learned policies offline, and validates whether offline ranking transfers to a lightweight quadrotor simulator before expensive field testing.

## What was implemented

- `experiments/build_oda_policy_dataset.py`: exports low-dimensional MPPI expert samples with trial-level train/val/test split.
- `experiments/train_bc_mppi.py`: trains Plain BC-MPPI and Filtered BC-MPPI with a small NumPy MLP.
- `experiments/train_risk_predictor.py`: evaluates TTC/distance and a small NumPy MLP future-risk predictor.
- `experiments/run_pybullet_oda_course.py`: runs RRT*, MPPI, Plain BC-MPPI, and Filtered BC-MPPI on ODA-like obstacle courses. It records `gym-pybullet-drones` availability, and falls back to a deterministic kinematic simulator when PyBullet is unavailable.
- `experiments/summarize_offline_vs_sim.py`: compares offline ODA ranking with lightweight simulator ranking.
- `src/oda_bench_downstream.py`: shared evaluator, dataset, rollout, metrics, and plotting helpers.

## Current local run

The local workspace now contains the full 300-trial ODA CSV subset downloaded from Hugging Face:

- `data/raw/ODA_Dataset/dataset/*/optitrack.csv`: 300 files
- `data/raw/ODA_Dataset/dataset/*/imu.csv`: 300 files
- `data/raw/ODA_Dataset/dataset/*/radar.csv`: 300 files
- local ODA CSV footprint: about 5.6 GB

The downstream policy dataset uses a trial-level split:

- train: 210 trials, 10,290 samples
- val: 45 trials, 2,205 samples
- test: 45 trials, 2,205 samples

Main generated outputs:

- `outputs/tables/oda_bc_results.csv`
- `outputs/tables/oda_risk_results.csv`
- `outputs/tables/pybullet_validation_results.csv`
- `outputs/tables/offline_vs_sim_rank_correlation.csv`
- `outputs/figures/bc_plain_vs_filtered_safety.png`
- `outputs/figures/risk_predictor_pr_curve.png`
- `outputs/figures/offline_vs_sim_rank_correlation.png`

## Result interpretation

- Filtered BC-MPPI improves safety versus Plain BC-MPPI on the 45-trial test split: violation drops from 0.3111 to 0.0667 and mean min clearance rises from 1.3365 m to 1.3809 m. Collision rate is tied at 0.0444.
- The small MLP risk predictor now beats TTC/distance on the 300-trial split: PR-AUC 0.885 versus 0.784, risk recall 0.992, and false negative rate 0.008.
- `gym-pybullet-drones` backend code is implemented in `experiments/run_pybullet_oda_course.py` using `CtrlAviary + DSLPIDControl` and PyBullet cylinder obstacles. On this Apple Silicon environment, installing `pybullet` failed because no binary wheel is available and source build fails with clang, so the executed validation used `kinematic_fallback`.
- Current fallback rank correlation is modest (`Spearman rho = 0.4`). The honest claim is that ODA-Bench can filter and expose candidate behavior before heavier simulation; do not claim field safety or strong PyBullet transfer until PyBullet runs on a compatible environment.

## Reproduce commands

```bash
.venv/bin/python experiments/build_oda_policy_dataset.py --limit-trials 300 --samples-per-trial 50 --mppi-rollouts 96 --mppi-iterations 3 --mppi-horizon-steps 40
.venv/bin/python experiments/train_bc_mppi.py --epochs 160 --batch-size 256 --lr 0.0008
.venv/bin/python experiments/train_risk_predictor.py --epochs 220 --batch-size 256 --lr 0.001 --danger-clearance 0.8
.venv/bin/python experiments/run_pybullet_oda_course.py --backend auto --seeds 20 --speeds 1 2 3 4
.venv/bin/python experiments/summarize_offline_vs_sim.py
```
