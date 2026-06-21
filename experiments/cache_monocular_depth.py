#!/usr/bin/env python3
"""Cache monocular relative-depth predictions for ODA RGB video frames."""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import imageio
import numpy as np
from PIL import Image

from src.oda_io import dataset_root, load_optitrack


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--trial-id", default="345")
    parser.add_argument(
        "--model-id",
        default="Intel/dpt-hybrid-midas",
        help="Hugging Face monocular depth model.",
    )
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--max-duration", type=float, default=None)
    parser.add_argument("--input-width", type=int, default=384)
    parser.add_argument(
        "--device",
        default="auto",
        choices=["auto", "cpu", "mps", "cuda"],
        help="Inference device.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Depth cache path. Defaults to data/processed/depth_sample_<id>_<fps>fps.npz.",
    )
    return parser.parse_args()


def choose_device(requested: str) -> str:
    import torch

    if requested != "auto":
        return requested
    if torch.cuda.is_available():
        return "cuda"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def resize_for_depth(image: Image.Image, input_width: int) -> Image.Image:
    if image.width <= input_width:
        return image
    height = max(1, round(image.height * input_width / image.width))
    return image.resize((input_width, height), Image.Resampling.BICUBIC)


def normalize_depth_stack(depth: np.ndarray) -> tuple[np.ndarray, float, float]:
    low = float(np.percentile(depth, 2.0))
    high = float(np.percentile(depth, 98.0))
    if high <= low:
        high = low + 1.0
    norm = np.clip((depth - low) / (high - low), 0.0, 1.0)
    return (norm * 255.0).astype(np.uint8), low, high


def main() -> None:
    args = parse_args()

    import torch
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation

    dataset_dir = dataset_root(args.dataset_root)
    sequence = str(args.trial_id)
    video_path = dataset_dir / sequence / f"{sequence}.avi"
    if not video_path.exists():
        raise FileNotFoundError(
            f"Missing RGB video {video_path}. Fetch it with: "
            f"scripts/fetch_oda_video_sample.sh data/raw/ODA_Dataset {sequence}"
        )

    output = (
        Path(args.output)
        if args.output
        else Path("data/processed") / f"depth_sample_{sequence}_{args.fps:g}fps.npz"
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    optitrack = load_optitrack(dataset_dir, sequence)
    reader = imageio.get_reader(video_path)
    meta = reader.get_meta_data()
    source_fps = float(meta.get("fps", 29.97))
    source_duration = float(meta.get("duration", optitrack["time_s"][-1]))
    duration = min(source_duration, float(optitrack["time_s"][-1]))
    if args.max_duration is not None:
        duration = min(duration, args.max_duration)

    device = choose_device(args.device)
    print(f"Loading depth model {args.model_id} on {device}...")
    processor = AutoImageProcessor.from_pretrained(args.model_id)
    model = AutoModelForDepthEstimation.from_pretrained(args.model_id)
    model.to(device)
    model.eval()

    frame_count = int(math.floor(duration * args.fps))
    times = np.arange(frame_count, dtype=float) / args.fps
    predictions: list[np.ndarray] = []

    for idx, t in enumerate(times):
        source_i = min(int(round(t * source_fps)), int(source_duration * source_fps) - 1)
        frame = reader.get_data(source_i)
        image = resize_for_depth(Image.fromarray(frame).convert("RGB"), args.input_width)
        inputs = processor(images=image, return_tensors="pt")
        inputs = {key: value.to(device) for key, value in inputs.items()}
        with torch.no_grad():
            output_depth = model(**inputs).predicted_depth
        depth = output_depth.detach().float().cpu().numpy()[0]
        predictions.append(depth.astype(np.float32))
        print(f"[{idx + 1:03d}/{frame_count:03d}] t={t:5.2f}s depth={depth.shape}")

    reader.close()
    depth_stack = np.stack(predictions, axis=0)
    depth_u8, depth_low, depth_high = normalize_depth_stack(depth_stack)

    np.savez_compressed(
        output,
        times=times.astype(np.float32),
        depth_u8=depth_u8,
        source_fps=np.float32(source_fps),
        depth_fps=np.float32(args.fps),
        depth_low=np.float32(depth_low),
        depth_high=np.float32(depth_high),
        model_id=np.asarray(args.model_id),
        note=np.asarray(
            "Relative monocular depth; brighter values correspond to larger model output, not calibrated metric distance."
        ),
    )
    print(f"Wrote {output}")
    print(f"Depth frames: {len(times)}, duration: {duration:.2f}s, fps: {args.fps:g}")
    sys.stdout.flush()
    sys.stderr.flush()
    if device == "mps":
        # Torch/MPS can leave a shutdown thread waiting in this Python 3.9
        # environment after all outputs are written. Exit the one-shot cache
        # process directly so automation does not hang after successful export.
        os._exit(0)


if __name__ == "__main__":
    main()
