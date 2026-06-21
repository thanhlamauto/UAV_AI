#!/usr/bin/env python3
"""Cache monocular depth for many ODA trials while loading the model once."""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
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
    parser.add_argument("--readiness", default="outputs/tables/target_20_trials_readiness.csv")
    parser.add_argument("--trial-ids", nargs="*", default=None)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--max-duration", type=float, default=None)
    parser.add_argument("--input-width", type=int, default=384)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "mps", "cuda"])
    parser.add_argument("--model-id", default="Intel/dpt-hybrid-midas")
    parser.add_argument("--output-root", default="data/processed/depth_batch")
    parser.add_argument("--timing-output", default="outputs/tables/depth_batch_timing.csv")
    parser.add_argument("--force", action="store_true")
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


def read_ready_sequences(path: Path) -> list[str]:
    with path.open(newline="") as f:
        return [row["sequence"] for row in csv.DictReader(f) if str(row.get("ready", "0")) == "1"]


def depth_output_path(output_root: Path, sequence: str, fps: float) -> Path:
    return output_root / str(sequence) / f"depth_{fps:g}fps.npz"


def batched(items: list[Image.Image], batch_size: int):
    for start in range(0, len(items), batch_size):
        yield items[start : start + batch_size]


def cache_trial(
    dataset_dir: Path,
    sequence: str,
    fps: float,
    max_duration: float | None,
    input_width: int,
    batch_size: int,
    output: Path,
    model_id: str,
    processor,
    model,
    device: str,
) -> dict[str, object]:
    import torch

    video_path = dataset_dir / sequence / f"{sequence}.avi"
    if not video_path.exists():
        raise FileNotFoundError(f"Missing RGB video {video_path}")

    started = time.perf_counter()
    output.parent.mkdir(parents=True, exist_ok=True)
    optitrack = load_optitrack(dataset_dir, sequence)
    reader = imageio.get_reader(video_path)
    meta = reader.get_meta_data()
    source_fps = float(meta.get("fps", 29.97))
    source_duration = float(meta.get("duration", optitrack["time_s"][-1]))
    duration = min(source_duration, float(optitrack["time_s"][-1]))
    if max_duration is not None:
        duration = min(duration, max_duration)

    frame_count = int(math.floor(duration * fps))
    times = np.arange(frame_count, dtype=float) / fps
    images: list[Image.Image] = []
    for t in times:
        source_i = min(int(round(t * source_fps)), int(source_duration * source_fps) - 1)
        frame = reader.get_data(source_i)
        image = resize_for_depth(Image.fromarray(frame).convert("RGB"), input_width)
        images.append(image)
    reader.close()

    predictions: list[np.ndarray] = []
    inference_started = time.perf_counter()
    for batch_images in batched(images, max(1, batch_size)):
        inputs = processor(images=batch_images, return_tensors="pt")
        inputs = {key: value.to(device) for key, value in inputs.items()}
        with torch.no_grad():
            output_depth = model(**inputs).predicted_depth
        batch_depth = output_depth.detach().float().cpu().numpy()
        predictions.extend(depth.astype(np.float32) for depth in batch_depth)
    inference_seconds = time.perf_counter() - inference_started

    if not predictions:
        raise ValueError(f"No frames selected for trial {sequence}")
    depth_stack = np.stack(predictions, axis=0)
    depth_u8, depth_low, depth_high = normalize_depth_stack(depth_stack)
    np.savez_compressed(
        output,
        times=times.astype(np.float32),
        depth_u8=depth_u8,
        source_fps=np.float32(source_fps),
        depth_fps=np.float32(fps),
        depth_low=np.float32(depth_low),
        depth_high=np.float32(depth_high),
        model_id=np.asarray(model_id),
        note=np.asarray(
            "Relative monocular depth from batch cache; not calibrated metric distance."
        ),
    )

    wall_seconds = time.perf_counter() - started
    return {
        "sequence": sequence,
        "model_id": model_id,
        "frames": len(times),
        "duration_s": round(float(duration), 4),
        "batch_size": batch_size,
        "wall_seconds": round(wall_seconds, 4),
        "inference_seconds": round(inference_seconds, 4),
        "seconds_per_frame": round(wall_seconds / max(1, len(times)), 6),
        "inference_seconds_per_frame": round(inference_seconds / max(1, len(times)), 6),
        "output": str(output),
    }


def write_timing(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    sequences = [str(item) for item in args.trial_ids] if args.trial_ids else read_ready_sequences(Path(args.readiness))
    if not sequences:
        raise SystemExit("No ready sequences found for depth caching.")

    import torch
    from transformers import AutoImageProcessor, AutoModelForDepthEstimation

    device = choose_device(args.device)
    print(f"Loading depth model {args.model_id} once on {device}...")
    processor = AutoImageProcessor.from_pretrained(args.model_id)
    model = AutoModelForDepthEstimation.from_pretrained(args.model_id)
    model.to(device)
    model.eval()

    dataset_dir = dataset_root(args.dataset_root)
    output_root = Path(args.output_root)
    timing_rows: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []
    for sequence in sequences:
        output = depth_output_path(output_root, sequence, args.fps)
        if output.exists() and not args.force:
            print(f"Skip {sequence}: cache already exists at {output}")
            continue
        try:
            print(f"Cache batch depth for trial {sequence} -> {output}")
            row = cache_trial(
                dataset_dir=dataset_dir,
                sequence=sequence,
                fps=args.fps,
                max_duration=args.max_duration,
                input_width=args.input_width,
                batch_size=args.batch_size,
                output=output,
                model_id=args.model_id,
                processor=processor,
                model=model,
                device=device,
            )
            timing_rows.append(row)
            print(row)
        except Exception as exc:
            failures.append({"sequence": sequence, "reason": str(exc)})
            print(f"Warning: depth caching failed for trial {sequence}: {exc}", file=sys.stderr)

    write_timing(Path(args.timing_output), timing_rows)
    if failures:
        failure_path = Path(args.timing_output).with_name(Path(args.timing_output).stem + "_failures.csv")
        write_timing(failure_path, failures)
        print(f"Wrote failures to {failure_path}", file=sys.stderr)
        raise SystemExit(1)
    print(f"Wrote {args.timing_output}")
    print(f"Batch depth caching complete for {len(timing_rows)} new trial(s).")


if __name__ == "__main__":
    main()
