# Depth Inference Optimization

This project treats monocular depth as relative depth for qualitative/risk features, not metric distance. The optimization target is therefore throughput and latency for the same relative-depth feature pipeline.

## Baseline

The current fastest verified baseline is batched PyTorch inference:

```bash
python3 experiments/cache_monocular_depth_batch.py \
  --readiness outputs/tables/target_20_trials_readiness.csv \
  --fps 5 \
  --device cuda \
  --batch-size 8 \
  --output-root data/processed/depth_batch_dpt_20 \
  --timing-output outputs/tables/depth_batch_timing_dpt_20.csv
```

The 20-trial DPT run processed 1096 frames at about `0.0886 s/frame` wall time and `0.0423 s/frame` model inference time.

## Depth Anything V2 Small Batch Run

The stronger 50-trial perception run uses Depth Anything V2 Small through the same batched PyTorch cache path:

```bash
python3 experiments/cache_monocular_depth_batch.py \
  --readiness outputs/tables/target_50_trials_readiness.csv \
  --fps 5 \
  --device cuda \
  --batch-size 8 \
  --model-id depth-anything/Depth-Anything-V2-Small-hf \
  --output-root data/processed/depth_anything_v2_small_50 \
  --timing-output outputs/tables/depth_batch_timing_depth_anything_v2_small_50.csv
```

On the RTX 3060 server, this processed 50 ODA trials / 2584 frames at about `0.0639 s/frame` wall time and `0.0174 s/frame` model inference time. The gap between wall time and inference time is mostly video decode, image preprocessing, CPU/GPU transfer, and compressed NPZ output. This is the current best practical depth pipeline for the project, but the output remains relative monocular depth and must be calibrated against ODA ground truth/radar before being used as metric distance.

## ONNX Runtime

Export the Hugging Face depth model:

```bash
pip install optimum-onnx onnx "onnxruntime-gpu==1.23.2"

scripts/export_depth_to_onnx.sh \
  Intel/dpt-hybrid-midas \
  models/depth_onnx/dpt_hybrid_midas
```

Run depth cache through ONNX Runtime:

```bash
python3 experiments/cache_monocular_depth_onnx.py \
  --readiness outputs/tables/target_20_trials_readiness.csv \
  --onnx-model models/depth_onnx/dpt_hybrid_midas/model.onnx \
  --fps 5 \
  --provider cuda \
  --batch-size 8 \
  --output-root data/processed/depth_onnx_dpt_20 \
  --timing-output outputs/tables/depth_onnx_timing_dpt_20.csv
```

On the RTX 3060 server, the verified CUDA ONNX Runtime probe on trials 3, 10, and 345 processed 177 frames at about `0.0827 s/frame` wall time and `0.0405 s/frame` inference time. This is only a small speedup over PyTorch batch, but it proves the ONNX path works.

Compare against PyTorch batch using:

```bash
python3 - <<'PY'
import csv
from pathlib import Path
for path in [
    Path("outputs/tables/depth_batch_timing_dpt_20.csv"),
    Path("outputs/tables/depth_onnx_timing_dpt_20.csv"),
]:
    rows = list(csv.DictReader(path.open()))
    frames = sum(int(row["frames"]) for row in rows)
    wall = sum(float(row["wall_seconds"]) for row in rows)
    infer = sum(float(row["inference_seconds"]) for row in rows)
    print(path, "frames", frames, "wall/frame", wall / frames, "infer/frame", infer / frames)
PY
```

## TensorRT

Build a TensorRT engine only if `trtexec` is available:

```bash
scripts/build_tensorrt_depth_engine.sh \
  models/depth_onnx/dpt_hybrid_midas/model.onnx \
  models/depth_tensorrt/dpt_hybrid_midas_fp16.engine
```

On the current Vast server, Docker/Podman are unavailable because the instance is an unprivileged container, `trtexec` is not installed, Python `tensorrt` is not installed, and ONNX Runtime TensorRT EP fails because `libnvinfer.so.10` is missing. Use an NVIDIA TensorRT container/image on a fresh instance, or install TensorRT runtime libraries into this container if the provider allows it. Keep ONNX Runtime CUDA or batched PyTorch as the practical fallback. Do not claim TensorRT speedups until an engine build and timing run are recorded.

If ONNX Runtime exposes `TensorrtExecutionProvider`, a lightweight probe can be run without `trtexec`:

```bash
python3 experiments/cache_monocular_depth_onnx.py \
  --trial-ids 3 10 345 \
  --onnx-model models/depth_onnx/dpt_hybrid_midas/model.onnx \
  --fps 5 \
  --provider tensorrt \
  --batch-size 8 \
  --output-root data/processed/depth_onnx_trt_dpt_probe \
  --timing-output outputs/tables/depth_onnx_trt_timing_dpt_probe.csv \
  --force
```
