#!/usr/bin/env bash
set -euo pipefail

ONNX_MODEL="${1:-models/depth_onnx/dpt_hybrid_midas/model.onnx}"
ENGINE_PATH="${2:-models/depth_tensorrt/dpt_hybrid_midas_fp16.engine}"
WORKSPACE_MB="${WORKSPACE_MB:-4096}"

if [ ! -f "${ONNX_MODEL}" ]; then
  echo "Missing ONNX model: ${ONNX_MODEL}" >&2
  exit 2
fi

if ! command -v trtexec >/dev/null 2>&1; then
  echo "Missing trtexec. Install/use an NVIDIA TensorRT container or package first." >&2
  echo "ONNX Runtime GPU remains the fallback path for this project." >&2
  exit 2
fi

mkdir -p "$(dirname "${ENGINE_PATH}")"

trtexec \
  --onnx="${ONNX_MODEL}" \
  --saveEngine="${ENGINE_PATH}" \
  --fp16 \
  --memPoolSize="workspace:${WORKSPACE_MB}" \
  --verbose

echo "TensorRT engine written to ${ENGINE_PATH}"
