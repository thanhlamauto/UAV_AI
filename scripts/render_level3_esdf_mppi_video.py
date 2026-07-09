#!/usr/bin/env python3
"""Render a Level-3 3D voxel/ESDF + MPPI result video."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--npz", type=Path, default=Path("data/processed/esdf3d/indoor_demo_esdf_mppi.npz"))
    parser.add_argument("--nvblox-status", type=Path, default=Path("outputs/nvblox_esdf3d_status_echo.txt"))
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/level3_full_3d_voxel_esdf_mppi.mp4"))
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--duration-s", type=float, default=12.0)
    parser.add_argument("--dpi", type=int, default=130)
    return parser.parse_args()


def load_status(path: Path) -> dict[str, object]:
    if not path.exists():
        return {}
    text = path.read_text(errors="replace")
    match = re.search(r"data: '([^']+)'", text)
    if not match:
        return {}
    try:
        return json.loads(match.group(1))
    except json.JSONDecodeError:
        return {}


def resample_polyline(points: np.ndarray, num_points: int) -> np.ndarray:
    if len(points) <= 1:
        return np.repeat(points[:1], num_points, axis=0)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(lengths.sum())
    if total <= 1e-9:
        return np.repeat(points[:1], num_points, axis=0)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    targets = np.linspace(0.0, total, num_points)
    out = np.zeros((num_points, 3), dtype=float)
    seg = 0
    for i, target in enumerate(targets):
        while seg < len(lengths) - 1 and cumulative[seg + 1] < target:
            seg += 1
        denom = cumulative[seg + 1] - cumulative[seg]
        alpha = 0.0 if denom <= 0 else (target - cumulative[seg]) / denom
        out[i] = (1.0 - alpha) * points[seg] + alpha * points[seg + 1]
    return out


def voxel_centers(occupancy: np.ndarray, spec: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    nx, ny, nz = [int(v) for v in spec[:3]]
    resolution = float(spec[3])
    origin = spec[4:7].astype(float)
    xs = origin[0] + (np.arange(nx) + 0.5) * resolution
    ys = origin[1] + (np.arange(ny) + 0.5) * resolution
    zs = origin[2] + (np.arange(nz) + 0.5) * resolution
    idx = np.argwhere(occupancy.astype(bool))
    return xs[idx[:, 0]], ys[idx[:, 1]], zs[idx[:, 2]]


def make_metric_lines(metrics: dict[str, object], status: dict[str, object]) -> list[str]:
    lines = [
        "Level 3 result",
        "3D voxel occupancy -> ESDF -> MPPI",
        "",
        f"Map resolution: {metrics.get('map_resolution_m', 'n/a')} m",
        f"Occupied voxels: {metrics.get('occupied_voxels', 'n/a')} / {metrics.get('voxels_total', 'n/a')}",
        f"Min ESDF distance: {metrics.get('min_esdf_distance_m', 'n/a')} m",
        f"Safety violation: {metrics.get('safety_violation', 'n/a')}",
        f"Altitude change: {metrics.get('altitude_change_m', 'n/a')} m",
        f"MPPI compute: {metrics.get('planner_compute_time_ms', 'n/a')} ms",
    ]
    if status:
        lines.extend(
            [
                "",
                "ROS2/NVBlox verifier",
                f"Source: {status.get('source_mode', 'n/a')}",
                f"Grid: {status.get('grid_shape', 'n/a')}",
                f"z-span: {status.get('esdf_z_span_m', 'n/a')} m",
                f"Verifier compute: {status.get('compute_time_ms', 'n/a')} ms",
                "LiDAR ~10.1 Hz, ESDF update ~4.9 Hz",
            ]
        )
    lines.extend(["", "Scope: planner-side local 3D ESDF;", "not direct NVBlox volume query yet."])
    return lines


def main() -> int:
    args = parse_args()
    data = np.load(args.npz)
    occupancy = data["occupancy"].astype(bool)
    signed_distance = data["signed_distance_m"].astype(float)
    trajectory = data["trajectory_xyz"].astype(float)
    spec = data["spec"].astype(float)
    metrics = json.loads(str(data["metrics_json"].item()))
    status = load_status(args.nvblox_status)

    nx, ny, nz = [int(v) for v in spec[:3]]
    resolution = float(spec[3])
    origin = spec[4:7].astype(float)
    upper = origin + np.array([nx, ny, nz], dtype=float) * resolution

    frame_count = int(round(float(args.duration_s) * int(args.fps)))
    smooth_path = resample_polyline(trajectory, frame_count)
    occ_x, occ_y, occ_z = voxel_centers(occupancy, spec)

    finite = np.isfinite(signed_distance)
    max_dist = min(2.0, float(np.nanpercentile(signed_distance[finite], 96))) if np.any(finite) else 2.0
    safe_radius = float(metrics.get("safety_radius_m", 0.42))
    metric_lines = make_metric_lines(metrics, status)

    args.output.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(16, 9), facecolor="#f7fafc")
    gs = fig.add_gridspec(2, 3, width_ratios=[1.45, 1.0, 0.85], height_ratios=[1.0, 0.42])

    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_slice = fig.add_subplot(gs[0, 1])
    ax_alt = fig.add_subplot(gs[1, 1])
    ax_text = fig.add_subplot(gs[:, 2])

    fig.suptitle("Muc 3: Full 3D voxel / ESDF / MPPI obstacle avoidance", fontsize=18, fontweight="bold")

    ax3d.scatter(occ_x, occ_y, occ_z, s=7, c="#ef4444", alpha=0.20, depthshade=False, label="occupied voxels")
    ax3d.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color="#94a3b8", linewidth=1.4, alpha=0.7)
    path3d, = ax3d.plot([], [], [], color="#2563eb", linewidth=3.0, label="MPPI path")
    uav3d = ax3d.scatter([], [], [], s=70, color="#f59e0b", edgecolor="#111827", linewidth=0.8, label="MAV")
    start3d = ax3d.scatter([trajectory[0, 0]], [trajectory[0, 1]], [trajectory[0, 2]], s=65, color="#16a34a", label="A")
    goal3d = ax3d.scatter([trajectory[-1, 0]], [trajectory[-1, 1]], [trajectory[-1, 2]], s=65, color="#7c3aed", label="B")
    ax3d.set_xlim(origin[0], upper[0])
    ax3d.set_ylim(origin[1], upper[1])
    ax3d.set_zlim(origin[2], upper[2])
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("z [m]")
    ax3d.set_title("3D voxel map and continuous MPPI trajectory", pad=12)
    ax3d.legend(loc="upper left", fontsize=9)

    z_idx0 = int(np.clip(round((trajectory[0, 2] - origin[2]) / resolution - 0.5), 0, nz - 1))
    extent = [origin[0], upper[0], origin[1], upper[1]]
    slice_img = np.clip(signed_distance[:, :, z_idx0].T, -safe_radius, max_dist)
    im = ax_slice.imshow(slice_img, origin="lower", extent=extent, cmap="viridis", vmin=-safe_radius, vmax=max_dist, aspect="auto")
    slice_path, = ax_slice.plot([], [], color="#ffffff", linewidth=3.0)
    slice_path_inner, = ax_slice.plot([], [], color="#2563eb", linewidth=1.8)
    uav2d = ax_slice.scatter([], [], s=70, color="#f59e0b", edgecolor="#111827", linewidth=0.8)
    ax_slice.scatter([trajectory[0, 0], trajectory[-1, 0]], [trajectory[0, 1], trajectory[-1, 1]], c=["#16a34a", "#7c3aed"], s=55)
    ax_slice.set_title(f"ESDF slice at current altitude")
    ax_slice.set_xlabel("x [m]")
    ax_slice.set_ylabel("y [m]")
    cbar = fig.colorbar(im, ax=ax_slice, fraction=0.046, pad=0.03)
    cbar.set_label("distance to obstacle [m]")

    t = np.linspace(0, args.duration_s, frame_count)
    ax_alt.plot(t, smooth_path[:, 2], color="#0f766e", linewidth=2.2)
    alt_marker, = ax_alt.plot([], [], marker="o", color="#f59e0b", markersize=8)
    ax_alt.set_xlim(0, args.duration_s)
    ax_alt.set_ylim(max(0, float(np.min(smooth_path[:, 2])) - 0.12), float(np.max(smooth_path[:, 2])) + 0.18)
    ax_alt.set_title("Altitude profile")
    ax_alt.set_xlabel("time [s]")
    ax_alt.set_ylabel("z [m]")
    ax_alt.grid(True, alpha=0.25)

    ax_text.set_axis_off()
    ax_text.add_patch(plt.Rectangle((0.02, 0.02), 0.96, 0.96, transform=ax_text.transAxes, facecolor="#ffffff", edgecolor="#cbd5e1", linewidth=1.2))
    ax_text.text(0.07, 0.94, "\n".join(metric_lines), transform=ax_text.transAxes, va="top", ha="left", fontsize=11, linespacing=1.35, color="#111827")

    progress_text = fig.text(0.5, 0.035, "", ha="center", va="center", fontsize=12, color="#334155")
    fig.tight_layout(rect=[0, 0.055, 1, 0.95])

    writer = FFMpegWriter(fps=int(args.fps), metadata={"title": "Level 3 3D voxel ESDF MPPI"}, bitrate=4200)
    with writer.saving(fig, str(args.output), dpi=int(args.dpi)):
        for frame in range(frame_count):
            point = smooth_path[frame]
            upto = smooth_path[: frame + 1]
            path3d.set_data(upto[:, 0], upto[:, 1])
            path3d.set_3d_properties(upto[:, 2])
            uav3d._offsets3d = ([point[0]], [point[1]], [point[2]])

            z_idx = int(np.clip(round((point[2] - origin[2]) / resolution - 0.5), 0, nz - 1))
            im.set_data(np.clip(signed_distance[:, :, z_idx].T, -safe_radius, max_dist))
            ax_slice.set_title(f"ESDF slice at z={point[2]:.2f} m")
            slice_path.set_data(upto[:, 0], upto[:, 1])
            slice_path_inner.set_data(upto[:, 0], upto[:, 1])
            uav2d.set_offsets(np.array([[point[0], point[1]]]))

            alt_marker.set_data([t[frame]], [point[2]])
            ax3d.view_init(elev=24, azim=-54 + 28 * frame / max(1, frame_count - 1))
            progress_text.set_text(
                f"frame {frame + 1}/{frame_count} | current altitude {point[2]:.2f} m | safety radius {safe_radius:.2f} m"
            )
            writer.grab_frame()

    plt.close(fig)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
