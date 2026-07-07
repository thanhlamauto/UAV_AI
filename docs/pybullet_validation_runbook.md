# PyBullet validation runbook

Use this on a Linux/Windows machine where `pybullet` has a binary wheel or can be installed cleanly.

## Setup

```bash
python3.10 -m venv .venv-pybullet
source .venv-pybullet/bin/activate

python -m pip install -U pip setuptools wheel
python -m pip install "gym-pybullet-drones @ git+https://github.com/learnsyslab/gym-pybullet-drones.git"
python -m pip install numpy matplotlib
```

Smoke test:

```bash
python - <<'PY'
import pybullet
import gym_pybullet_drones
print("pybullet ok")
print("gym_pybullet_drones ok")
PY
```

## Run

From the repository root:

```bash
source .venv-pybullet/bin/activate

python experiments/run_pybullet_oda_course.py \
  --backend gym-pybullet-drones \
  --seeds 20 \
  --speeds 1 2 3 4 \
  --pybullet-control-freq-hz 48 \
  --pybullet-sim-freq-hz 240 \
  --pybullet-altitude-m 1.0
```

Optional GUI smoke test:

```bash
python experiments/run_pybullet_oda_course.py \
  --backend gym-pybullet-drones \
  --seeds 2 \
  --speeds 1 2 \
  --pybullet-gui
```

Then regenerate rank summary:

```bash
python experiments/summarize_offline_vs_sim.py
```

Expected outputs:

- `outputs/tables/pybullet_validation_results.csv`
- `outputs/tables/pybullet_validation_detail.csv`
- `outputs/tables/offline_vs_sim_rank_correlation.csv`
- `outputs/figures/offline_vs_sim_rank_correlation.png`

The committed BC model files under `outputs/models/` are enough to run the Plain BC and Filtered BC baselines without rebuilding the ODA policy dataset.
