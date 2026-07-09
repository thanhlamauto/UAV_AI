#!/usr/bin/env python3
"""Visualize one ODA trial from OptiTrack, IMU, radar, and obstacle metadata."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from src.oda_io import (
    dataset_root,
    load_imu,
    load_optitrack,
    load_radar_spectra,
    obstacle_array,
    read_trial_overview,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--trial-id", default="345")
    parser.add_argument("--obstacle-radius-m", type=float, default=0.20)
    parser.add_argument("--safety-distance-m", type=float, default=0.50)
    parser.add_argument("--grid-resolution-m", type=float, default=0.04)
    parser.add_argument("--output", default=None)
    return parser.parse_args()


def moving_average(values: np.ndarray, window: int) -> np.ndarray:
    if len(values) < window or window <= 1:
        return values.copy()
    kernel = np.ones(window, dtype=float) / float(window)
    padded = np.pad(values, (window // 2, window - 1 - window // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def interp_at(time_s: np.ndarray, values: np.ndarray, query_t: float) -> np.ndarray:
    values = np.asarray(values)
    if values.ndim == 1:
        return np.asarray(np.interp(query_t, time_s, values))
    return np.asarray([np.interp(query_t, time_s, values[:, idx]) for idx in range(values.shape[1])])


def make_occupancy(
    xy: np.ndarray,
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float,
    safety_distance_m: float,
    resolution_m: float,
) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    pad = obstacle_radius_m + safety_distance_m + 0.70
    all_xy = np.vstack([xy, obstacles_xy])
    x_min = float(np.floor((all_xy[:, 0].min() - pad) / resolution_m) * resolution_m)
    x_max = float(np.ceil((all_xy[:, 0].max() + pad) / resolution_m) * resolution_m)
    y_min = float(np.floor((all_xy[:, 1].min() - pad) / resolution_m) * resolution_m)
    y_max = float(np.ceil((all_xy[:, 1].max() + pad) / resolution_m) * resolution_m)

    xs = np.arange(x_min, x_max + resolution_m, resolution_m)
    ys = np.arange(y_min, y_max + resolution_m, resolution_m)
    xx, yy = np.meshgrid(xs, ys)
    grid = np.zeros_like(xx, dtype=float)
    for obs in obstacles_xy:
        dist = np.hypot(xx - obs[0], yy - obs[1])
        grid = np.maximum(grid, (dist <= obstacle_radius_m).astype(float) * 2.0)
        grid = np.maximum(
            grid,
            ((dist > obstacle_radius_m) & (dist <= obstacle_radius_m + safety_distance_m)).astype(float),
        )
    return grid, (x_min, x_max, y_min, y_max)


def compute_clearance(xy: np.ndarray, obstacles_xy: np.ndarray, obstacle_radius_m: float) -> np.ndarray:
    distances = np.stack([np.hypot(xy[:, 0] - obs[0], xy[:, 1] - obs[1]) for obs in obstacles_xy], axis=1)
    return distances.min(axis=1) - obstacle_radius_m


def windowed_ground_speed(time_s: np.ndarray, xy: np.ndarray, window_s: float = 0.25) -> np.ndarray:
    """Estimate speed from displacement over a short window to reduce mocap jitter spikes."""

    half = 0.5 * window_s
    t0 = np.maximum(time_s[0], time_s - half)
    t1 = np.minimum(time_s[-1], time_s + half)
    x0 = np.interp(t0, time_s, xy[:, 0])
    y0 = np.interp(t0, time_s, xy[:, 1])
    x1 = np.interp(t1, time_s, xy[:, 0])
    y1 = np.interp(t1, time_s, xy[:, 1])
    dt = np.maximum(t1 - t0, 1e-6)
    return np.hypot(x1 - x0, y1 - y0) / dt


def draw_occupancy_map(
    ax: plt.Axes,
    trial_id: str,
    xy: np.ndarray,
    time_s: np.ndarray,
    obstacles_xy: np.ndarray,
    occupancy: np.ndarray,
    extent: tuple[float, float, float, float],
    closest_idx: int,
    obstacle_radius_m: float,
    safety_distance_m: float,
) -> None:
    from matplotlib.colors import ListedColormap

    cmap = ListedColormap(["#f8fafc", "#fde68a", "#fca5a5"])
    ax.imshow(
        occupancy,
        origin="lower",
        extent=extent,
        cmap=cmap,
        interpolation="nearest",
        alpha=0.78,
        vmin=0,
        vmax=2,
    )
    points = ax.scatter(
        xy[:, 0],
        xy[:, 1],
        c=time_s,
        s=10,
        cmap="viridis",
        linewidths=0,
        label="OptiTrack trajectory",
        zorder=4,
    )
    plt.colorbar(points, ax=ax, fraction=0.036, pad=0.02, label="time [s]")

    for idx, obs in enumerate(obstacles_xy, start=1):
        ax.add_patch(plt.Circle(obs, obstacle_radius_m, fill=False, color="#b91c1c", linewidth=2.0))
        ax.add_patch(
            plt.Circle(
                obs,
                obstacle_radius_m + safety_distance_m,
                fill=False,
                color="#dc2626",
                linestyle="--",
                linewidth=1.4,
            )
        )
        ax.text(obs[0], obs[1], f"obs {idx}", ha="center", va="center", fontsize=8, weight="bold")

    ax.scatter(xy[0, 0], xy[0, 1], marker="o", s=90, color="#16a34a", edgecolor="white", label="start", zorder=6)
    ax.scatter(xy[-1, 0], xy[-1, 1], marker="s", s=90, color="#111827", edgecolor="white", label="end", zorder=6)
    ax.scatter(
        xy[closest_idx, 0],
        xy[closest_idx, 1],
        marker="*",
        s=170,
        color="#f97316",
        edgecolor="black",
        linewidth=0.6,
        label="closest approach",
        zorder=7,
    )
    if 1 <= closest_idx < len(xy) - 1:
        direction = xy[closest_idx + 1] - xy[closest_idx - 1]
        norm = np.linalg.norm(direction)
        if norm > 1e-9:
            direction = direction / norm * 0.35
            ax.arrow(
                xy[closest_idx, 0],
                xy[closest_idx, 1],
                direction[0],
                direction[1],
                width=0.015,
                color="#f97316",
                length_includes_head=True,
                zorder=8,
            )

    ax.set_title(f"Trial {trial_id}: occupancy map + UAV ground-truth state")
    ax.set_xlabel("OptiTrack x [m]")
    ax.set_ylabel("OptiTrack z [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.18)
    ax.legend(loc="upper right", fontsize=8)


def draw_state_panel(
    ax: plt.Axes,
    time_s: np.ndarray,
    speed_mps: np.ndarray,
    height_m: np.ndarray,
    clearance_m: np.ndarray,
    closest_idx: int,
    safety_distance_m: float,
) -> None:
    ax.plot(time_s, speed_mps, color="#2563eb", linewidth=1.8, label="ground speed [m/s]")
    ax.plot(time_s, height_m, color="#7c3aed", linewidth=1.5, label="height y [m]")
    ax.plot(time_s, clearance_m, color="#dc2626", linewidth=1.8, label="clearance [m]")
    ax.axhline(safety_distance_m, color="#dc2626", linestyle="--", linewidth=1.0, alpha=0.7)
    ax.axvline(time_s[closest_idx], color="#f97316", linestyle=":", linewidth=1.6, label="closest approach")
    ax.set_title("UAV state from OptiTrack")
    ax.set_xlabel("time [s]")
    ymax = float(np.nanpercentile(np.hstack([speed_mps, height_m, clearance_m]), 99.0))
    ax.set_ylim(bottom=min(-0.2, float(np.nanmin(clearance_m)) - 0.2), top=max(2.2, ymax * 1.18))
    ax.grid(True, alpha=0.22)
    ax.legend(fontsize=8, loc="best")


def draw_imu_panel(ax: plt.Axes, imu: dict[str, np.ndarray], closest_t: float) -> tuple[float, float]:
    time_s = imu["time_s"]
    accel_norm = np.linalg.norm(imu["accel_filt_mps2"], axis=1)
    gyro_norm = np.linalg.norm(imu["gyro_filt_radps"], axis=1)
    accel_norm = moving_average(accel_norm, 51)
    gyro_norm = moving_average(gyro_norm, 51)
    ax.plot(time_s, accel_norm, color="#0f766e", linewidth=1.4, label="|accel| [m/s²]")
    ax.plot(time_s, gyro_norm, color="#ea580c", linewidth=1.4, label="|gyro| [rad/s]")
    ax.axvline(closest_t, color="#f97316", linestyle=":", linewidth=1.4)
    ax.set_title("IMU state summary")
    ax.set_xlabel("time [s]")
    ax.grid(True, alpha=0.22)
    ax.legend(fontsize=8, loc="best")
    return float(np.interp(closest_t, time_s, accel_norm)), float(np.interp(closest_t, time_s, gyro_norm))


def draw_radar_panel(ax: plt.Axes, radar: dict[str, np.ndarray], closest_t: float) -> tuple[float, int]:
    time_s = radar["time_s"]
    mag = 0.5 * (radar["mag_rx1"] + radar["mag_rx2"])
    mag_db = 20.0 * np.log10(np.maximum(mag, 1e-6))
    mag_db = mag_db - np.percentile(mag_db, 5)
    mag_db = np.clip(mag_db, 0.0, np.percentile(mag_db, 99))
    peak_by_frame = mag_db.max(axis=1)
    peak_bin = mag_db.argmax(axis=1)

    ax.imshow(
        mag_db.T,
        origin="lower",
        aspect="auto",
        extent=(time_s[0], time_s[-1], 0, mag_db.shape[1] - 1),
        cmap="magma",
        interpolation="nearest",
    )
    ax.plot(time_s, peak_bin, color="#67e8f9", linewidth=1.0, alpha=0.9, label="peak bin")
    ax.axvline(closest_t, color="#38bdf8", linestyle=":", linewidth=1.4)
    ax.set_title("Radar FFT magnitude heatmap")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("positive FFT bin")
    ax.legend(fontsize=8, loc="upper right")
    return float(np.interp(closest_t, time_s, peak_by_frame)), int(np.interp(closest_t, time_s, peak_bin))


def draw_summary_table(
    ax: plt.Axes,
    trial_id: str,
    lux: str,
    obstacle_count: int,
    closest_t: float,
    closest_xy: np.ndarray,
    speed: float,
    height: float,
    clearance: float,
    accel_norm: float,
    gyro_norm: float,
    radar_peak: float,
    radar_bin: int,
) -> None:
    ax.axis("off")
    rows = [
        ["trial", trial_id],
        ["lux", lux],
        ["obstacles", str(obstacle_count)],
        ["closest time", f"{closest_t:.3f} s"],
        ["closest x,z", f"({closest_xy[0]:.3f}, {closest_xy[1]:.3f}) m"],
        ["speed at closest", f"{speed:.3f} m/s"],
        ["height at closest", f"{height:.3f} m"],
        ["clearance at closest", f"{clearance:.3f} m"],
        ["IMU |accel|", f"{accel_norm:.3f} m/s²"],
        ["IMU |gyro|", f"{gyro_norm:.3f} rad/s"],
        ["radar peak", f"{radar_peak:.1f} dB-norm @ bin {radar_bin}"],
    ]
    table = ax.table(
        cellText=rows,
        colLabels=["quantity", "value"],
        loc="center",
        cellLoc="left",
        colLoc="left",
        bbox=[0.0, 0.0, 1.0, 0.90],
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 1.30)
    for (row_idx, _), cell in table.get_celld().items():
        if row_idx == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#e5e7eb")
        else:
            cell.set_facecolor("#ffffff" if row_idx % 2 else "#f8fafc")
    ax.text(
        0.0,
        0.98,
        "State snapshot at closest approach",
        transform=ax.transAxes,
        ha="left",
        va="top",
        fontsize=11,
        fontweight="bold",
    )


def cylinder_mesh(
    center_x: float,
    center_z: float,
    radius_m: float,
    height_m: float,
    n: int = 48,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    theta = np.linspace(0.0, 2.0 * np.pi, n)
    y = np.array([0.0, height_m])
    theta_grid, y_grid = np.meshgrid(theta, y)
    x_grid = center_x + radius_m * np.cos(theta_grid)
    z_grid = center_z + radius_m * np.sin(theta_grid)
    return x_grid, z_grid, y_grid


def draw_3d_trial(
    output: Path,
    trial_id: str,
    trial_info: object,
    xy: np.ndarray,
    height_m: np.ndarray,
    time_s: np.ndarray,
    obstacles_xy: np.ndarray,
    closest_idx: int,
    obstacle_radius_m: float,
    safety_distance_m: float,
) -> None:
    fig = plt.figure(figsize=(12.4, 9.2), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")

    colors = plt.cm.viridis((time_s - time_s.min()) / max(float(np.ptp(time_s)), 1e-9))
    for idx in range(len(xy) - 1):
        ax.plot(
            xy[idx : idx + 2, 0],
            xy[idx : idx + 2, 1],
            height_m[idx : idx + 2],
            color=colors[idx],
            linewidth=2.2,
        )

    ax.scatter(xy[0, 0], xy[0, 1], height_m[0], s=90, color="#16a34a", edgecolor="white", label="start")
    ax.scatter(xy[-1, 0], xy[-1, 1], height_m[-1], s=90, color="#111827", edgecolor="white", marker="s", label="end")
    ax.scatter(
        xy[closest_idx, 0],
        xy[closest_idx, 1],
        height_m[closest_idx],
        s=180,
        color="#f97316",
        edgecolor="black",
        marker="*",
        label="closest approach",
    )

    for idx, (obs, meta) in enumerate(zip(obstacles_xy, trial_info.obstacles), start=1):
        x_grid, z_grid, y_grid = cylinder_mesh(obs[0], obs[1], obstacle_radius_m, meta.height_y)
        ax.plot_surface(x_grid, z_grid, y_grid, color="#ef4444", alpha=0.28, linewidth=0, shade=True)
        sx, sz, sy = cylinder_mesh(obs[0], obs[1], obstacle_radius_m + safety_distance_m, meta.height_y)
        ax.plot_wireframe(sx, sz, sy, color="#f59e0b", alpha=0.24, linewidth=0.7)
        ax.text(obs[0], obs[1], meta.height_y + 0.08, f"obs {idx}", color="#991b1b", ha="center")

    floor_z = np.zeros((2, 2))
    x_min = float(min(xy[:, 0].min(), obstacles_xy[:, 0].min()) - 0.8)
    x_max = float(max(xy[:, 0].max(), obstacles_xy[:, 0].max()) + 0.8)
    g_min = float(min(xy[:, 1].min(), obstacles_xy[:, 1].min()) - 0.8)
    g_max = float(max(xy[:, 1].max(), obstacles_xy[:, 1].max()) + 0.8)
    xx, zz = np.meshgrid([x_min, x_max], [g_min, g_max])
    ax.plot_surface(xx, zz, floor_z, color="#e5e7eb", alpha=0.20, linewidth=0)

    ax.set_title(f"Trial {trial_id}: 3D OptiTrack trajectory and obstacle cylinders")
    ax.set_xlabel("OptiTrack x [m]")
    ax.set_ylabel("OptiTrack z [m]")
    ax.set_zlabel("height y [m]")
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(g_min, g_max)
    ax.set_zlim(0.0, max(float(height_m.max()), max(obs.height_y for obs in trial_info.obstacles)) + 0.4)
    ax.view_init(elev=26, azim=-58)
    ax.legend(loc="upper right")

    # Matplotlib mplot3d has no native equal aspect before recent versions;
    # setting box aspect keeps the plotted room from looking artificially tall.
    try:
        ax.set_box_aspect((x_max - x_min, g_max - g_min, ax.get_zlim()[1]))
    except Exception:
        pass

    fig.savefig(output, dpi=190, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    args = parse_args()
    trial_id = str(args.trial_id)
    dataset_dir = dataset_root(args.dataset_root)
    output = Path(args.output or f"outputs/figures/oda_trial_{trial_id}_sensor_occupancy_dashboard.png")
    output_3d = output.with_name(output.stem.replace("_sensor_occupancy_dashboard", "_3d_state_occupancy") + output.suffix)

    trial_info = read_trial_overview(dataset_dir)[trial_id]
    opt = load_optitrack(dataset_dir, trial_id)
    imu = load_imu(dataset_dir, trial_id)
    radar = load_radar_spectra(dataset_dir, trial_id)

    xy = np.column_stack([opt["ground_x_m"], opt["ground_y_m"]])
    time_s = opt["time_s"]
    height_m = opt["height_m"]
    obstacles_xy = obstacle_array(trial_info.obstacles)
    clearance_m = compute_clearance(xy, obstacles_xy, args.obstacle_radius_m)
    closest_idx = int(np.argmin(clearance_m))
    closest_t = float(time_s[closest_idx])

    speed_mps = moving_average(windowed_ground_speed(time_s, xy, window_s=0.25), 7)

    occupancy, extent = make_occupancy(
        xy,
        obstacles_xy,
        args.obstacle_radius_m,
        args.safety_distance_m,
        args.grid_resolution_m,
    )

    fig = plt.figure(figsize=(15.5, 11.2), constrained_layout=True)
    gs = fig.add_gridspec(3, 3, height_ratios=[1.25, 0.92, 0.88], width_ratios=[1.35, 1.0, 0.95])

    draw_occupancy_map(
        fig.add_subplot(gs[:, 0]),
        trial_id,
        xy,
        time_s,
        obstacles_xy,
        occupancy,
        extent,
        closest_idx,
        args.obstacle_radius_m,
        args.safety_distance_m,
    )
    draw_state_panel(
        fig.add_subplot(gs[0, 1:]),
        time_s,
        speed_mps,
        height_m,
        clearance_m,
        closest_idx,
        args.safety_distance_m,
    )
    accel_norm, gyro_norm = draw_imu_panel(fig.add_subplot(gs[1, 1]), imu, closest_t)
    radar_peak, radar_bin = draw_radar_panel(fig.add_subplot(gs[1, 2]), radar, closest_t)
    draw_summary_table(
        fig.add_subplot(gs[2, 1:]),
        trial_id,
        trial_info.lux,
        trial_info.obstacle_count,
        closest_t,
        xy[closest_idx],
        float(speed_mps[closest_idx]),
        float(height_m[closest_idx]),
        float(clearance_m[closest_idx]),
        accel_norm,
        gyro_norm,
        radar_peak,
        radar_bin,
    )

    fig.suptitle(
        "ODA CSV visualization: occupancy map, UAV state, IMU, and radar",
        fontsize=15,
        fontweight="bold",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=190, bbox_inches="tight")
    plt.close(fig)
    draw_3d_trial(
        output_3d,
        trial_id,
        trial_info,
        xy,
        height_m,
        time_s,
        obstacles_xy,
        closest_idx,
        args.obstacle_radius_m,
        args.safety_distance_m,
    )
    print(output)
    print(output_3d)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
