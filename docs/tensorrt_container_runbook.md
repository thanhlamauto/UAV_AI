# TensorRT Engine Runbook

This project has verified CUDA PyTorch and ONNX Runtime CUDA on the rented RTX 3060 server. It has not verified a TensorRT engine on that instance because the current Vast image is an unprivileged container without Docker/Podman, `trtexec`, Python `tensorrt`, or `libnvinfer.so.10`.

## Current Blocker

Observed on the server:

```text
docker: command not found
podman: command not found
trtexec: command not found
tensorrt Python package: missing
ONNX Runtime TensorRT EP: missing libnvinfer.so.10
```

The ONNX Runtime probe correctly refused to fall back to CPU when `--provider tensorrt` was requested. Therefore, current results should be described as PyTorch CUDA / ONNX CUDA timing, not TensorRT timing.

## Recommended Server Choice

Rent or launch a fresh GPU instance with one of these:

- NVIDIA TensorRT / NGC container image.
- Vast image that already includes TensorRT runtime libraries and `trtexec`.
- VM-style instance where Docker can run an NGC TensorRT container.

Do not spend the main ODA benchmark budget trying to run Docker inside the current unprivileged container.

## Engine Build

After exporting the ONNX model:

```bash
scripts/export_depth_to_onnx.sh \
  Intel/dpt-hybrid-midas \
  models/depth_onnx/dpt_hybrid_midas
```

Build an FP16 TensorRT engine:

```bash
trtexec \
  --onnx=models/depth_onnx/dpt_hybrid_midas/model.onnx \
  --saveEngine=models/depth_tensorrt/dpt_hybrid_midas_fp16.engine \
  --fp16 \
  --workspace=4096
```

Record the `trtexec` latency summary and engine build log in:

```text
outputs/server_logs/tensorrt_engine_build.log
outputs/tables/depth_tensorrt_engine_timing.csv
```

## Fair Comparison

Compare three numbers on the same trial set and FPS:

| Runtime | Required output |
| --- | --- |
| PyTorch CUDA batch | `depth_batch_timing_*.csv` |
| ONNX Runtime CUDA | `depth_onnx_timing_*.csv` |
| TensorRT FP16 engine | `depth_tensorrt_engine_timing.csv` |

Report both wall time per frame and model inference time per frame. If TensorRT only improves inference time but decode/preprocess stays dominant, state that clearly.

## Wording For Report

Use this wording until an engine is actually built:

```text
TensorRT has been scoped and probed, but a real TensorRT engine was not measured on the current server because the rented container does not include TensorRT runtime libraries or container runtime access. The verified acceleration paths are batched PyTorch CUDA and ONNX Runtime CUDA.
```
