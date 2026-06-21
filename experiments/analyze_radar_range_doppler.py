#!/usr/bin/env python3
"""Compute lightweight range-Doppler summaries from ODA radar CSV files."""

from __future__ import annotations

import argparse
import csv
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from src.oda_io import available_trial_ids, dataset_root


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--trial-ids", nargs="*", default=None)
    parser.add_argument("--output", default="outputs/tables/radar_range_doppler_summary.csv")
    parser.add_argument("--figure-output", default="outputs/figures/radar_range_doppler_summary.png")
    parser.add_argument("--max-sweeps", type=int, default=0, help="0 means all sweeps.")
    return parser.parse_args()


def read_radar_sweeps(path: Path, max_sweeps: int = 0) -> tuple[np.ndarray, np.ndarray]:
    times: list[float] = []
    sweeps: list[np.ndarray] = []
    with path.open(newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            block_len = int((len(row) - 1) / 4)
            chirp_count = 16
            chirp_len = int(block_len / chirp_count)
            rx1_re = np.asarray(row[1 : 1 + block_len], dtype=float).reshape(chirp_count, chirp_len)
            rx1_im = np.asarray(row[1 + block_len : 1 + 2 * block_len], dtype=float).reshape(chirp_count, chirp_len)
            rx2_re = np.asarray(row[1 + 2 * block_len : 1 + 3 * block_len], dtype=float).reshape(chirp_count, chirp_len)
            rx2_im = np.asarray(row[1 + 3 * block_len : 1 + 4 * block_len], dtype=float).reshape(chirp_count, chirp_len)
            times.append(float(row[0]) * 1e-9)
            sweeps.append(0.5 * ((rx1_re + 1j * rx1_im) + (rx2_re + 1j * rx2_im)))
            if max_sweeps > 0 and len(sweeps) >= max_sweeps:
                break
    if not sweeps:
        raise ValueError(f"No radar sweeps found in {path}")
    times_arr = np.asarray(times, dtype=float)
    return times_arr - times_arr[0], np.stack(sweeps, axis=0)


def range_doppler_mag(sweep: np.ndarray) -> np.ndarray:
    chirp_count, chirp_len = sweep.shape
    range_fft_len = chirp_len * 2
    range_fft = np.fft.fft(sweep, n=range_fft_len, axis=1)[:, :chirp_len]
    doppler_fft = np.fft.fftshift(np.fft.fft(range_fft, axis=0), axes=0)
    return np.abs(doppler_fft)


def entropy(values: np.ndarray) -> float:
    flat = values.astype(float).ravel()
    total = float(flat.sum())
    if total <= 0.0:
        return 0.0
    prob = flat / total
    prob = prob[prob > 0.0]
    return float(-(prob * np.log2(prob)).sum())


def summarize_trial(dataset_dir: Path, sequence: str, max_sweeps: int) -> tuple[dict[str, object], dict[str, np.ndarray]]:
    times, sweeps = read_radar_sweeps(dataset_dir / sequence / "radar.csv", max_sweeps=max_sweeps)
    rd_maps = np.stack([range_doppler_mag(sweep) for sweep in sweeps], axis=0)
    doppler_center = rd_maps.shape[1] // 2
    peak_flat = np.argmax(rd_maps.reshape(len(rd_maps), -1), axis=1)
    peak_doppler, peak_range = np.unravel_index(peak_flat, rd_maps.shape[1:])
    peak_doppler_rel = peak_doppler - doppler_center
    peak_values = rd_maps.reshape(len(rd_maps), -1)[np.arange(len(rd_maps)), peak_flat]
    energy = np.mean(rd_maps**2, axis=(1, 2))
    ent = np.asarray([entropy(item) for item in rd_maps], dtype=float)
    mode_range = Counter(peak_range.tolist()).most_common(1)[0][0]
    mode_doppler = Counter(peak_doppler_rel.tolist()).most_common(1)[0][0]
    zero_fraction = float(np.mean(peak_doppler_rel == 0))
    summary = {
        "sequence": sequence,
        "sweeps": len(rd_maps),
        "doppler_bins": rd_maps.shape[1],
        "range_bins": rd_maps.shape[2],
        "mean_peak_range_bin": round(float(np.mean(peak_range)), 4),
        "mode_peak_range_bin": int(mode_range),
        "mean_peak_doppler_bin": round(float(np.mean(peak_doppler_rel)), 4),
        "mode_peak_doppler_bin": int(mode_doppler),
        "zero_doppler_peak_fraction": round(zero_fraction, 4),
        "mean_rd_peak": round(float(np.mean(peak_values)), 4),
        "mean_rd_energy": round(float(np.mean(energy)), 4),
        "mean_rd_entropy": round(float(np.mean(ent)), 4),
    }
    detail = {
        "times": times,
        "mean_rd": np.mean(rd_maps, axis=0),
        "peak_range": peak_range,
        "peak_doppler_rel": peak_doppler_rel,
    }
    return summary, detail


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_summary(path: Path, details: dict[str, dict[str, np.ndarray]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    sequences = list(details)[:3]
    fig, axes = plt.subplots(len(sequences), 3, figsize=(10.5, 3.0 * len(sequences)), constrained_layout=True)
    if len(sequences) == 1:
        axes = np.asarray([axes])
    for row_idx, sequence in enumerate(sequences):
        detail = details[sequence]
        ax0, ax1, ax2 = axes[row_idx]
        im = ax0.imshow(
            np.log1p(detail["mean_rd"]),
            aspect="auto",
            origin="lower",
            cmap="magma",
        )
        ax0.set_title(f"Trial {sequence}: mean range-Doppler")
        ax0.set_xlabel("Range bin")
        ax0.set_ylabel("Doppler bin")
        fig.colorbar(im, ax=ax0, fraction=0.046)
        ax1.plot(detail["times"], detail["peak_range"], color="#4c78a8", linewidth=1.0)
        ax1.set_title("Peak range bin over time")
        ax1.set_xlabel("Time (s)")
        ax1.set_ylabel("Range bin")
        ax1.grid(alpha=0.25)
        ax2.plot(detail["times"], detail["peak_doppler_rel"], color="#f58518", linewidth=1.0)
        ax2.axhline(0, color="black", linewidth=0.8, alpha=0.5)
        ax2.set_title("Peak Doppler bin over time")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Relative Doppler bin")
        ax2.grid(alpha=0.25)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    sequences = [str(item) for item in args.trial_ids] if args.trial_ids else available_trial_ids(dataset_dir)
    rows = []
    details = {}
    for sequence in sequences:
        try:
            row, detail = summarize_trial(dataset_dir, sequence, args.max_sweeps)
        except Exception as exc:
            print(f"Warning: skip trial {sequence}: {exc}", file=sys.stderr)
            continue
        rows.append(row)
        details[sequence] = detail
        print(row)
    if not rows:
        raise SystemExit("No radar range-Doppler rows were produced.")
    write_csv(Path(args.output), rows)
    plot_summary(Path(args.figure_output), details)
    print(f"Wrote {args.output}")
    print(f"Wrote {args.figure_output}")


if __name__ == "__main__":
    main()
