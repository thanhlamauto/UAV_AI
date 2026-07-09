#!/usr/bin/env python3
"""Cache metric indoor depth for ODA RGB videos.

This uses Depth Anything V2 Metric Indoor Small by default.  Unlike the older
relative-depth cache, the output stores ``depth_m`` in meters.
"""

from __future__ import annotations

import argparse
import csv
import math
import platform
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import imageio
import numpy as np
from PIL import Image

from src.depth_metric import METRIC_INDOOR_SMALL_MODEL_ID, MetricDepthProvider
from src.oda_io import dataset_root, load_optitrack


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--readiness", default="outputs/tables/target_20_trials_readiness.csv")
    parser.add_argument("--trial-ids", nargs="*", default=None)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--max-duration", type=float, default=None)
    parser.add_argument("--input-width", type=int, default=384)
    parser.add_argument("--depth-model", default="depth-anything-v2-metric-indoor-small")
    parser.add_argument("--model-id", default=METRIC_INDOOR_SMALL_MODEL_ID)
    parser.add_argument("--device", default="auto", choices=["auto", "cpu", "mps", "cuda"])
    parser.add_argument("--cache-depth", default="true", choices=["true", "false"])
    parser.add_argument("--depth-cache-dir", default="data/processed/metric_depth")
    parser.add_argument("--depth-min-m", type=float, default=0.20)
    parser.add_argument("--depth-max-m", type=float, default=8.0)
    parser.add_argument("--timing-output", default="outputs/tables/metric_depth_timing_depth_anything_v2_metric_indoor_small.csv")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def read_ready_sequences(path: Path) -> list[str]:
    with path.open(newline="") as f:
        return [row["sequence"] for row in csv.DictReader(f) if str(row.get("ready", "0")) == "1"]


def resize_for_depth(image: Image.Image, input_width: int) -> Image.Image:
    if image.width <= input_width:
        return image
    height = max(1, round(image.height * input_width / image.width))
    return image.resize((input_width, height), Image.Resampling.BICUBIC)


def output_path(cache_dir: Path, sequence: str, fps: float) -> Path:
    return cache_dir / str(sequence) / f"metric_depth_{fps:g}fps.npz"


def cache_trial(
    dataset_dir: Path,
    sequence: str,
    provider: MetricDepthProvider,
    fps: float,
    max_duration: float | None,
    input_width: int,
    depth_min_m: float,
    depth_max_m: float,
    output: Path,
) -> dict[str, object]:
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
    predictions: list[np.ndarray] = []
    inference_ms = 0.0
    for t in times:
        source_i = min(int(round(t * source_fps)), int(source_duration * source_fps) - 1)
        image = resize_for_depth(Image.fromarray(reader.get_data(source_i)).convert("RGB"), input_width)
        result = provider.predict(image)
        depth = np.clip(result.depth_m, depth_min_m, depth_max_m).astype(np.float32)
        predictions.append(depth)
        inference_ms += float(result.inference_ms)
    reader.close()

    if not predictions:
        raise ValueError(f"No frames selected for trial {sequence}")
    depth_stack = np.stack(predictions, axis=0)
    np.savez_compressed(
        output,
        times=times.astype(np.float32),
        depth_m=depth_stack.astype(np.float32),
        source_fps=np.float32(source_fps),
        depth_fps=np.float32(fps),
        model_id=np.asarray(provider.model_id),
        device=np.asarray(provider.device),
        depth_min_m=np.float32(depth_min_m),
        depth_max_m=np.float32(depth_max_m),
        note=np.asarray("Metric indoor depth cache in meters from Depth Anything V2 Metric Indoor Small."),
    )
    wall_seconds = time.perf_counter() - started
    return {
        "sequence": sequence,
        "frames": len(times),
        "model_id": provider.model_id,
        "device": provider.device,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "processor": platform.processor(),
        "duration_s": round(float(duration), 4),
        "wall_seconds": round(wall_seconds, 4),
        "inference_seconds": round(inference_ms / 1000.0, 4),
        "seconds_per_frame": round(wall_seconds / max(1, len(times)), 6),
        "inference_seconds_per_frame": round((inference_ms / 1000.0) / max(1, len(times)), 6),
        "output": str(output),
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    if args.depth_model != "depth-anything-v2-metric-indoor-small":
        raise SystemExit(f"Unsupported --depth-model {args.depth_model}")
    if args.cache_depth == "false":
        raise SystemExit("This script is a cache writer; use --cache-depth true.")

    sequences = [str(item) for item in args.trial_ids] if args.trial_ids else read_ready_sequences(Path(args.readiness))
    dataset_dir = dataset_root(args.dataset_root)
    cache_dir = Path(args.depth_cache_dir)
    provider = MetricDepthProvider(model_id=args.model_id, device=args.device)
    rows: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []
    for sequence in sequences:
        out = output_path(cache_dir, sequence, args.fps)
        if out.exists() and not args.force:
            print(f"Skip {sequence}: cache already exists at {out}")
            continue
        try:
            print(f"Cache metric depth for trial {sequence} -> {out}")
            row = cache_trial(
                dataset_dir=dataset_dir,
                sequence=sequence,
                provider=provider,
                fps=args.fps,
                max_duration=args.max_duration,
                input_width=args.input_width,
                depth_min_m=args.depth_min_m,
                depth_max_m=args.depth_max_m,
                output=out,
            )
            print(row)
            rows.append(row)
        except Exception as exc:  # keep long batch jobs useful when a local trial is incomplete.
            failure = {"sequence": sequence, "error": str(exc)}
            failures.append(failure)
            print(f"Skip {sequence}: {exc}", file=sys.stderr)
    write_csv(Path(args.timing_output), rows)
    print(f"Wrote {args.timing_output}")
    if failures:
        failure_path = Path(args.timing_output).with_name(Path(args.timing_output).stem + "_failures.csv")
        write_csv(failure_path, failures)
        print(f"Wrote {failure_path}")


if __name__ == "__main__":
    main()
