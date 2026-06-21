#!/usr/bin/env bash
set -euo pipefail

MANIFEST="${1:?Usage: scripts/run_oda_manifest_benchmark.sh MANIFEST [OUTPUTS_DIR]}"
OUTPUTS_DIR="${2:-outputs}"
READINESS="${OUTPUTS_DIR}/tables/$(basename "${MANIFEST}" .csv)_readiness.csv"

mkdir -p "${OUTPUTS_DIR}/tables" "${OUTPUTS_DIR}/figures"

python3 scripts/check_oda_trial_readiness.py \
  --manifest "${MANIFEST}" \
  --output "${READINESS}"

python3 experiments/batch_benchmark_planners.py \
  --manifest "${MANIFEST}" \
  --readiness "${READINESS}" \
  --ready-only \
  --outputs-dir "${OUTPUTS_DIR}"
