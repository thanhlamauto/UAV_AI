# Planner Benchmark Progress

## Data acquisition status

The target 20-trial manifest is available at:

- `outputs/tables/target_20_trials_manifest.csv`

It selects 10 one-obstacle and 10 two-obstacle full-light ODA trials with RGB
video flags and non-missing obstacle coordinates. It intentionally includes
the local sample trials `3`, `10`, and `345`.

The 4TU data record currently exposes one full ZIP archive of about 98 GB
(`Dupeyroux_et_al_2021_ODA_DATASET_Full.zip`, MD5
`189639db8176ccdbd728b88d99c27309`). A remote HTTP range-request test was not
honored by the server: `Range: bytes=0-15` returned `HTTP/1.1 200 OK` and the
full content length, not `206 Partial Content`. Therefore this run cannot
download only 20 selected trials directly from 4TU without downloading the full
archive. The GitHub repository
contains rendered/sample data for trials `3`, `10`, and `345`, which are the
only complete local trials at the moment. The prepared extraction workflow is
documented in:

- `docs/oda_data_acquisition.md`
- `scripts/extract_oda_trials_from_full_zip.py`

Local disk feasibility was checked on 2026-06-21. No full/partial ODA ZIP was
found locally, and the current disk has about 11 GiB free, which is below the
about 92.4 GiB needed for the full archive plus 1 GiB slack. The download script
now exits early with this check instead of starting a download that cannot
finish.

## Implemented benchmark

Implemented baselines:

- `human`: original OptiTrack trajectory.
- `straight_line`: direct start-to-goal baseline.
- `geometric_bypass`: waypoint-based left/right bypass around inflated obstacle
  safety circles.
- `astar_grid`: A* over a 2D occupancy grid with inflated obstacle radius.
- `rrt`: deterministic Rapidly-exploring Random Tree baseline over the same
  inflated obstacle representation.
- `rrt_star`: deterministic RRT* variant with neighborhood rewiring.
- `mppi`: lightweight Python MPPI-style waypoint optimizer warm-started from
  the geometric bypass path.

Implemented labels and metrics:

- minimum center distance;
- minimum obstacle-boundary clearance;
- collision and safety-distance violation;
- closest-approach time;
- path length and mean speed;
- simple heading-change smoothness;
- per-frame risk labels: safe, warning, danger, collision;
- future-risk labels over a 1 second horizon;
- planner computation time.

## Verified manifest/local run

The batch benchmark was run against the 20-trial manifest. At the current local
data state, trials `3`, `10`, and `345` have all required OptiTrack/RGB/radar/IMU
files and 17 manifest trials are recorded in `batch_skipped_trials.csv` as
missing local files. After extracting the full ZIP, the same command will
benchmark all 20 target trials without changing the metrics code.

Current completion audit:

- PASS: 20-trial manifest exists.
- MISSING: 20 trials fully downloaded (`3/20` currently ready).
- PASS: batch benchmark output exists.
- PASS: clearance-based risk labels are included.
- PASS: human, straight-line, geometric bypass, and A* baselines are included.
- PASS: RRT, RRT*, and MPPI baselines are included.
- PASS: planner comparison summary exists.

Outputs:

- `outputs/tables/batch_planner_metrics.csv`
- `outputs/tables/planner_comparison_summary.csv`
- `outputs/tables/batch_skipped_trials.csv`
- `outputs/tables/target_20_trials_readiness.csv`
- `outputs/figures/planner_comparison_sample_3.png`
- `outputs/figures/planner_comparison_sample_10.png`
- `outputs/figures/planner_comparison_sample_345.png`

Summary from the local run:

| method | trials | collision rate | safety violation rate | mean min clearance m | mean path length m | mean planner time ms |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| human | 3 | 0.0000 | 0.3333 | 0.7446 | 7.5900 | 0.0000 |
| straight_line | 3 | 0.3333 | 0.6667 | 0.4055 | 6.6072 | 1.6204 |
| geometric_bypass / not_needed | 3 | 0.0000 | 0.0000 | 0.7386 | 6.6825 | 2.7720 |
| astar_grid | 3 | 0.0000 | 0.0000 | 0.6092 | 6.8479 | 5.6578 |
| rrt | 3 | 0.0000 | 0.0000 | 0.6343 | 6.7540 | 8.6344 |
| rrt_star | 3 | 0.0000 | 0.0000 | 0.6360 | 6.7399 | 75.5094 |
| mppi | 3 | 0.0000 | 0.0000 | 0.7386 | 6.6814 | 4.4417 |

Interpretation:

- The straight-line baseline is shorter but unsafe on the local samples,
  including a collision on sample `345`.
- The geometric bypass, A*, RRT, RRT*, and MPPI baselines avoid collision and
  safety-distance violation on all local samples.
- The human OptiTrack trajectory avoids collision but enters the safety region
  on sample `3`, matching the earlier clearance analysis.

## Reproduce

```bash
python3 scripts/create_oda_20_trial_manifest.py
python3 scripts/check_oda_trial_readiness.py
scripts/run_oda_target20_benchmark.sh
python3 scripts/audit_goal_status.py
```

For a clean local demo that benchmarks only complete trials:

```bash
scripts/run_oda_ready_benchmark.sh
```

After extracting the 20 target trials from the full ZIP:

```bash
scripts/run_oda_target20_benchmark.sh
```
