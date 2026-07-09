#!/usr/bin/env python3
"""Build frame-level perception-risk features from ODA depth/radar/IMU data."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from src.oda_io import dataset_root, load_imu, load_optitrack, load_radar_spectra, obstacle_array, read_trial_overview
from src.risk import clearance_series, future_risk_labels, risk_labels_from_clearance


FIELDNAMES = [
    "sequence",
    "time_s",
    "depth_min",
    "depth_p10",
    "depth_median",
    "radar_peak",
    "radar_peak_bin",
    "radar_energy",
    "radar_rd_peak",
    "radar_rd_range_bin",
    "radar_rd_doppler_bin",
    "radar_rd_energy",
    "radar_rd_near_energy",
    "radar_rd_doppler_spread",
    "radar_rd_range_spread",
    "imu_acc_norm",
    "imu_gyro_norm",
    "clearance_m",
    "risk_label",
    "future_risk_label",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--readiness", default="outputs/tables/target_20_trials_readiness.csv")
    parser.add_argument("--trial-ids", nargs="*", default=None)
    parser.add_argument("--depth-root", default="data/processed/depth")
    parser.add_argument("--depth-fps", type=float, default=5.0)
    parser.add_argument("--output", default="outputs/tables/perception_risk_features.csv")
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--warning-clearance", type=float, default=0.80)
    parser.add_argument("--danger-clearance", type=float, default=0.50)
    parser.add_argument("--future-risk-horizon", type=float, default=1.0)
    parser.add_argument("--center-crop-fraction", type=float, default=0.50)
    return parser.parse_args()


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


def read_ready_sequences(path: Path) -> list[str]:
    with path.open(newline="") as f:
        return [row["sequence"] for row in csv.DictReader(f) if str(row.get("ready", "0")) == "1"]


def find_depth_cache(depth_root: Path, sequence: str, fps: float) -> Path | None:
    candidates = [
        depth_root / sequence / f"depth_{fps:g}fps.npz",
        depth_root / sequence / f"metric_depth_{fps:g}fps.npz",
        Path("data/processed") / f"depth_sample_{sequence}_{fps:g}fps.npz",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def nearest_index(times: np.ndarray, query_t: float) -> int:
    idx = int(np.searchsorted(times, query_t))
    if idx <= 0:
        return 0
    if idx >= len(times):
        return len(times) - 1
    before = idx - 1
    return before if abs(times[before] - query_t) <= abs(times[idx] - query_t) else idx


def center_crop(depth: np.ndarray, fraction: float) -> np.ndarray:
    fraction = float(np.clip(fraction, 0.10, 1.0))
    h, w = depth.shape
    crop_h = max(1, int(round(h * fraction)))
    crop_w = max(1, int(round(w * fraction)))
    y0 = max(0, (h - crop_h) // 2)
    x0 = max(0, (w - crop_w) // 2)
    return depth[y0 : y0 + crop_h, x0 : x0 + crop_w]


def build_trial_rows(
    dataset_dir: Path,
    sequence: str,
    depth_path: Path,
    obstacle_radius_m: float,
    warning_clearance_m: float,
    danger_clearance_m: float,
    future_risk_horizon_s: float,
    center_crop_fraction: float,
) -> list[dict[str, object]]:
    trials = read_trial_overview(dataset_dir)
    trial = trials[sequence]
    obstacles_xy = obstacle_array(trial.obstacles)
    optitrack = load_optitrack(dataset_dir, sequence)
    imu = load_imu(dataset_dir, sequence)
    radar = load_radar_spectra(dataset_dir, sequence)

    with np.load(depth_path, allow_pickle=False) as depth_cache:
        depth_times = np.asarray(depth_cache["times"], dtype=float)
        if "depth_m" in depth_cache:
            depth_images = np.asarray(depth_cache["depth_m"], dtype=np.float32)
        else:
            depth_images = np.asarray(depth_cache["depth_u8"], dtype=np.float32)

    valid = depth_times <= float(optitrack["time_s"][-1])
    depth_times = depth_times[valid]
    depth_images = depth_images[valid]
    if len(depth_times) == 0:
        return []

    traj_xy = np.column_stack(
        [
            np.interp(depth_times, optitrack["time_s"], optitrack["ground_x_m"]),
            np.interp(depth_times, optitrack["time_s"], optitrack["ground_y_m"]),
        ]
    )
    clearance = clearance_series(traj_xy, obstacles_xy, obstacle_radius_m)
    labels = risk_labels_from_clearance(
        clearance,
        warning_clearance_m=warning_clearance_m,
        danger_clearance_m=danger_clearance_m,
    )
    future = future_risk_labels(
        depth_times,
        clearance,
        horizon_s=future_risk_horizon_s,
        danger_clearance_m=danger_clearance_m,
    )

    # Backward-compatible Level-1 radar features: first-chirp 1D range profile.
    radar_mag = np.asarray(radar["mag_rx1"], dtype=float) + np.asarray(radar["mag_rx2"], dtype=float)
    imu_acc_norm = np.linalg.norm(np.asarray(imu["accel_filt_mps2"], dtype=float), axis=1)
    imu_gyro_norm = np.linalg.norm(np.asarray(imu["gyro_filt_radps"], dtype=float), axis=1)

    rows: list[dict[str, object]] = []
    for i, t in enumerate(depth_times):
        crop = center_crop(depth_images[i], center_crop_fraction).astype(float)
        radar_i = nearest_index(radar["time_s"], float(t))
        imu_i = nearest_index(imu["time_s"], float(t))
        radar_row = radar_mag[radar_i]
        rows.append(
            {
                "sequence": sequence,
                "time_s": round(float(t), 4),
                "depth_min": round(float(np.min(crop)), 4),
                "depth_p10": round(float(np.percentile(crop, 10.0)), 4),
                "depth_median": round(float(np.median(crop)), 4),
                "radar_peak": round(float(np.max(radar_row)), 4),
                "radar_peak_bin": int(np.argmax(radar_row)),
                "radar_energy": round(float(np.mean(radar_row**2)), 4),
                "radar_rd_peak": round(float(radar["radar_rd_peak"][radar_i]), 4),
                "radar_rd_range_bin": int(radar["radar_rd_range_bin"][radar_i]),
                "radar_rd_doppler_bin": int(radar["radar_rd_doppler_bin"][radar_i]),
                "radar_rd_energy": round(float(radar["radar_rd_energy"][radar_i]), 4),
                "radar_rd_near_energy": round(float(radar["radar_rd_near_energy"][radar_i]), 4),
                "radar_rd_doppler_spread": round(float(radar["radar_rd_doppler_spread"][radar_i]), 4),
                "radar_rd_range_spread": round(float(radar["radar_rd_range_spread"][radar_i]), 4),
                "imu_acc_norm": round(float(imu_acc_norm[imu_i]), 4),
                "imu_gyro_norm": round(float(imu_gyro_norm[imu_i]), 4),
                "clearance_m": round(float(clearance[i]), 4),
                "risk_label": str(labels[i]),
                "future_risk_label": "future_risk" if bool(future[i]) else "no_future_risk",
            }
        )
    return rows


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    sequences = [str(item) for item in args.trial_ids] if args.trial_ids else read_ready_sequences(Path(args.readiness))
    rows: list[dict[str, object]] = []
    skipped: list[tuple[str, str]] = []

    for sequence in sequences:
        depth_path = find_depth_cache(Path(args.depth_root), sequence, args.depth_fps)
        if depth_path is None:
            skipped.append((sequence, "missing depth cache"))
            continue
        try:
            trial_rows = build_trial_rows(
                dataset_dir=dataset_dir,
                sequence=sequence,
                depth_path=depth_path,
                obstacle_radius_m=args.obstacle_radius,
                warning_clearance_m=args.warning_clearance,
                danger_clearance_m=args.danger_clearance,
                future_risk_horizon_s=args.future_risk_horizon,
                center_crop_fraction=args.center_crop_fraction,
            )
            rows.extend(trial_rows)
            print(f"Added {len(trial_rows)} perception-risk rows for trial {sequence}")
        except Exception as exc:
            skipped.append((sequence, str(exc)))

    write_csv(Path(args.output), rows)
    print(f"Wrote {args.output} with {len(rows)} rows")
    if skipped:
        print("Skipped trials:", file=sys.stderr)
        for sequence, reason in skipped:
            print(f"  {sequence}: {reason}", file=sys.stderr)


if __name__ == "__main__":
    main()
