#!/usr/bin/env bash
set -euo pipefail

MANIFEST="${1:-outputs/tables/target_20_trials_manifest.csv}"
READINESS="${2:-outputs/tables/target_20_trials_readiness.csv}"

if [[ ! -f "${MANIFEST}" ]]; then
  python3 scripts/create_oda_20_trial_manifest.py --output "${MANIFEST}"
fi

python3 scripts/check_oda_trial_readiness.py --manifest "${MANIFEST}" --output "${READINESS}"
python3 experiments/batch_benchmark_planners.py \
  --manifest "${MANIFEST}" \
  --readiness "${READINESS}" \
  --ready-only
