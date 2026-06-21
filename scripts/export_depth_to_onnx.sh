#!/usr/bin/env bash
set -euo pipefail

MODEL_ID="${1:-Intel/dpt-hybrid-midas}"
OUTPUT_DIR="${2:-models/depth_onnx/dpt_hybrid_midas}"
TASK="${TASK:-depth-estimation}"

mkdir -p "${OUTPUT_DIR}"

if ! command -v optimum-cli >/dev/null 2>&1; then
  echo "Missing optimum-cli. Install with:" >&2
  echo "  pip install optimum-onnx onnx 'onnxruntime-gpu==1.23.2'" >&2
  echo "Use onnxruntime instead of onnxruntime-gpu on CPU-only machines." >&2
  exit 2
fi

if ! optimum-cli export onnx --help >/dev/null 2>&1; then
  echo "This optimum-cli does not include the ONNX exporter. Install with:" >&2
  echo "  pip install optimum-onnx onnx 'onnxruntime-gpu==1.23.2'" >&2
  exit 2
fi

optimum-cli export onnx \
  --model "${MODEL_ID}" \
  --task "${TASK}" \
  "${OUTPUT_DIR}"

echo "ONNX export written to ${OUTPUT_DIR}"
find "${OUTPUT_DIR}" -maxdepth 2 -type f | sort
