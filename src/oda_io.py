"""CSV/metadata loading helpers for the ODA Dataset.

The original ODA visualization scripts plot the ground plane as OptiTrack
``x`` versus OptiTrack ``z`` and use OptiTrack ``y`` as height.  These helpers
keep that convention explicit so metrics and plots match the upstream examples.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class Obstacle:
    """One cylindrical obstacle from ``trial_overview.csv``."""

    x: float
    ground_y: float
    height_y: float


@dataclass(frozen=True)
class TrialInfo:
    """Metadata grouped by ODA trial sequence."""

    sequence: str
    lux: str
    has_video: bool
    optitrack_y_rotation_offset: float | None
    obstacles: tuple[Obstacle, ...]

    @property
    def obstacle_count(self) -> int:
        return len(self.obstacles)


def dataset_root(path: str | Path) -> Path:
    root = Path(path)
    if (root / "dataset").is_dir():
        return root / "dataset"
    return root


def _to_float(value: str) -> float | None:
    value = value.strip()
    if not value:
        return None
    return float(value)


def read_trial_overview(dataset_dir: str | Path) -> dict[str, TrialInfo]:
    """Read and group ``trial_overview.csv`` by sequence ID."""

    root = dataset_root(dataset_dir)
    overview_path = root / "trial_overview.csv"
    grouped: dict[str, list[dict[str, str]]] = {}
    with overview_path.open(newline="") as f:
        for row in csv.DictReader(f):
            grouped.setdefault(row["Sequence"], []).append(row)

    trials: dict[str, TrialInfo] = {}
    for sequence, rows in grouped.items():
        first = rows[0]
        obstacles: list[Obstacle] = []
        for row in rows:
            obs_x = _to_float(row["Obstacle x"])
            obs_height = _to_float(row["Obstacle y"])
            obs_ground_y = _to_float(row["Obstacle z"])
            if obs_x is None or obs_height is None or obs_ground_y is None:
                continue
            obstacles.append(
                Obstacle(x=obs_x, ground_y=obs_ground_y, height_y=obs_height)
            )

        offset = _to_float(first["OptiTrack initial y rotation offset"])
        trials[sequence] = TrialInfo(
            sequence=sequence,
            lux=first["Lux"],
            has_video=first["Incl. Video"] == "1",
            optitrack_y_rotation_offset=offset,
            obstacles=tuple(obstacles),
        )
    return trials


def available_trial_ids(dataset_dir: str | Path) -> list[str]:
    """Return trial IDs with local sample folders."""

    root = dataset_root(dataset_dir)
    ids = [p.name for p in root.iterdir() if p.is_dir() and p.name.isdigit()]
    return sorted(ids, key=lambda item: int(item))


def load_optitrack(dataset_dir: str | Path, sequence: str | int) -> dict[str, np.ndarray]:
    """Load one OptiTrack CSV as numpy arrays in the ODA plotting frame."""

    root = dataset_root(dataset_dir)
    path = root / str(sequence) / "optitrack.csv"
    rows: list[list[float]] = []
    with path.open(newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            rows.append([float(value) for value in row])

    if not rows:
        raise ValueError(f"No OptiTrack rows found in {path}")

    arr = np.asarray(rows, dtype=float)
    time_s = (arr[:, 0] - arr[0, 0]) * 1e-9
    raw_x = arr[:, 1]
    raw_y = arr[:, 2]
    raw_z = arr[:, 3]

    return {
        "time_s": time_s,
        "raw_x_m": raw_x,
        "raw_y_m": raw_y,
        "raw_z_m": raw_z,
        "ground_x_m": raw_x,
        "ground_y_m": raw_z,
        "height_m": raw_y,
        "quat": arr[:, 4:8],
    }


def _moving_average(values: np.ndarray, window: int = 15) -> np.ndarray:
    if window <= 1 or len(values) < window:
        return values.copy()
    kernel = np.ones(window, dtype=float) / float(window)
    padded = np.pad(values, (window // 2, window - 1 - window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def load_imu(dataset_dir: str | Path, sequence: str | int) -> dict[str, np.ndarray]:
    """Load 6-axis IMU CSV arrays.

    Columns are linear acceleration ``ax, ay, az`` in m/s^2 and angular velocity
    ``p, q, r`` in rad/s.
    """

    root = dataset_root(dataset_dir)
    path = root / str(sequence) / "imu.csv"
    rows: list[list[float]] = []
    with path.open(newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if row:
                rows.append([float(value) for value in row])

    if not rows:
        raise ValueError(f"No IMU rows found in {path}")

    arr = np.asarray(rows, dtype=float)
    time_s = (arr[:, 0] - arr[0, 0]) * 1e-9
    accel = arr[:, 1:4]
    gyro = arr[:, 4:7]
    accel_filt = np.column_stack([_moving_average(accel[:, idx]) for idx in range(3)])
    gyro_filt = np.column_stack([_moving_average(gyro[:, idx]) for idx in range(3)])

    return {
        "time_s": time_s,
        "accel_mps2": accel,
        "gyro_radps": gyro,
        "accel_filt_mps2": accel_filt,
        "gyro_filt_radps": gyro_filt,
    }


def _fft_radar(real: np.ndarray, imag: np.ndarray) -> np.ndarray:
    spectrum = np.fft.fftshift(np.fft.fft(real + 1j * imag))
    return np.abs(spectrum)


def load_radar_spectra(dataset_dir: str | Path, sequence: str | int) -> dict[str, np.ndarray]:
    """Load radar CSV and compute first-chirp FFT magnitudes for RX1/RX2.

    The ODA CSV radar rows contain four blocks after timestamp:
    RX1 real, RX1 imaginary, RX2 real, RX2 imaginary.  Following the upstream
    visualization script, this helper uses the first chirp from each sweep and
    zero pads it before FFT.
    """

    root = dataset_root(dataset_dir)
    path = root / str(sequence) / "radar.csv"

    time_s: list[float] = []
    mag_rx1: list[np.ndarray] = []
    mag_rx2: list[np.ndarray] = []
    freq_axis: np.ndarray | None = None

    with path.open(newline="") as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if not row:
                continue
            time_s.append(float(row[0]) * 1e-9)
            block_len = int((len(row) - 1) / 4)
            chirp_len = int(block_len / 16)
            fft_len = 2 * chirp_len

            rx1_re = np.zeros(fft_len, dtype=float)
            rx1_im = np.zeros(fft_len, dtype=float)
            rx2_re = np.zeros(fft_len, dtype=float)
            rx2_im = np.zeros(fft_len, dtype=float)

            rx1_re[:chirp_len] = np.asarray(row[1 : 1 + chirp_len], dtype=float)
            rx1_im[:chirp_len] = np.asarray(
                row[1 + block_len : 1 + block_len + chirp_len], dtype=float
            )
            rx2_re[:chirp_len] = np.asarray(
                row[1 + 2 * block_len : 1 + 2 * block_len + chirp_len], dtype=float
            )
            rx2_im[:chirp_len] = np.asarray(
                row[1 + 3 * block_len : 1 + 3 * block_len + chirp_len], dtype=float
            )

            raw_freq = np.fft.fftshift(np.fft.fftfreq(fft_len, 1.0))
            positive = raw_freq >= 0
            if freq_axis is None:
                freq_axis = raw_freq[positive]

            mag_rx1.append(_fft_radar(rx1_re, rx1_im)[positive])
            mag_rx2.append(_fft_radar(rx2_re, rx2_im)[positive])

    if not time_s or freq_axis is None:
        raise ValueError(f"No radar rows found in {path}")

    time = np.asarray(time_s, dtype=float)
    time = time - time[0]
    return {
        "time_s": time,
        "freq": freq_axis,
        "mag_rx1": np.vstack(mag_rx1),
        "mag_rx2": np.vstack(mag_rx2),
    }


def summarize_dataset(dataset_dir: str | Path) -> dict[str, int]:
    """Return high-level counts from metadata and local sample folders."""

    trials = read_trial_overview(dataset_dir)
    available = available_trial_ids(dataset_dir)
    obstacle_counts = [trial.obstacle_count for trial in trials.values()]

    return {
        "metadata_unique_trials": len(trials),
        "metadata_rows": sum(max(1, count) for count in obstacle_counts),
        "local_sample_trials": len(available),
        "trials_with_0_obstacles_in_metadata": obstacle_counts.count(0),
        "trials_with_1_obstacle_in_metadata": obstacle_counts.count(1),
        "trials_with_2_obstacles_in_metadata": obstacle_counts.count(2),
        "full_light_trials": sum(1 for trial in trials.values() if trial.lux == "100"),
        "dim_light_trials": sum(1 for trial in trials.values() if trial.lux == "1"),
        "trials_with_video_flag": sum(1 for trial in trials.values() if trial.has_video),
    }


def obstacle_array(obstacles: Iterable[Obstacle]) -> np.ndarray:
    """Convert obstacles to an ``N x 2`` ground-plane array."""

    return np.asarray([(obs.x, obs.ground_y) for obs in obstacles], dtype=float)
