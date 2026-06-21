#!/usr/bin/env bash
set -euo pipefail

SAMPLE_ID="${1:-345}"
DEPTH_FPS="${2:-5}"
OUTPUT_PATH="${3:-data/processed/depth_sample_${SAMPLE_ID}_${DEPTH_FPS}fps.npz}"

python3 experiments/cache_monocular_depth.py \
  --trial-id "${SAMPLE_ID}" \
  --fps "${DEPTH_FPS}" \
  --output "${OUTPUT_PATH}"
