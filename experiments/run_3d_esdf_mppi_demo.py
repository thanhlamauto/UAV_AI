#!/usr/bin/env python3
"""Run a lightweight indoor 3D voxel/ESDF + MPPI demonstration.

This is not a replacement for NVBlox.  It is a reproducible local proof that
the project can use a 3D distance field, not only a 2D occupancy grid, as the
collision model for MPPI.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.esdf3d import VoxelGridSpec, compute_esdf, empty_occupancy, mark_box, mark_cylinder
from src.planners.mppi_3d_esdf import MPPI3DConfig, mppi_3d_esdf_path, rollout_cost


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs"))
    parser.add_argument("--resolution", type=float, default=0.15)
    parser.add_argument("--rollouts", type=int, default=768)
    parser.add_argument("--iterations", type=int, default=12)
    parser.add_argument("--horizon-steps", type=int, default=72)
    parser.add_argument("--seed", type=int, default=31)
    parser.add_argument("--prefer-scipy", action="store_true", help="Use scipy EDT if scipy is installed.")
    return parser.parse_args()


def build_indoor_scene(resolution: float) -> tuple[VoxelGridSpec, np.ndarray, list[dict[str, object]]]:
    spec = VoxelGridSpec(
        nx=int(math.ceil(7.8 / resolution)),
        ny=int(math.ceil(6.2 / resolution)),
        nz=int(math.ceil(3.0 / resolution)),
        resolution_m=resolution,
        origin_xyz=(-0.6, -3.1, 0.0),
    )
    occ = empty_occupancy(spec)
    obstacles: list[dict[str, object]] = []

    def add_box(name: str, center: tuple[float, float, float], size: tuple[float, float, float]) -> None:
        mark_box(occ, spec, center, size)
        obstacles.append({"name": name, "type": "box", "center": center, "size": size})

    def add_cylinder(name: str, center_xy: tuple[float, float], radius: float, z_min: float, z_max: float) -> None:
        mark_cylinder(occ, spec, center_xy, radius, z_min, z_max)
        obstacles.append({"name": name, "type": "cylinder", "center_xy": center_xy, "radius": radius, "z_min": z_min, "z_max": z_max})

    # Corridor walls and lab furniture.  Doors at the start/goal remain open.
    add_box("left_corridor_wall", (3.15, -2.55, 1.35), (7.5, 0.16, 2.7))
    add_box("right_corridor_wall", (3.15, 2.55, 1.35), (7.5, 0.16, 2.7))
    add_box("lab_bench_left", (1.0, -1.88, 0.48), (1.0, 0.55, 0.95))
    add_box("lab_bench_right", (5.8, 1.85, 0.48), (1.0, 0.55, 0.95))

    # Two flight obstacles that make the direct A-to-B line unsafe.
    add_cylinder("pillar_low_lateral", (2.05, -0.38), 0.36, 0.0, 2.25)
    add_box("stacked_box_mid", (4.25, 0.44, 0.95), (0.9, 0.78, 1.9))
    # A low obstacle spanning the corridor forces the 3D planner to use altitude,
    # which distinguishes this demo from a fixed-height 2D costmap planner.
    add_box("low_crossbar_requires_climb", (3.32, 0.0, 0.48), (0.26, 3.55, 0.96))
    return spec, occ, obstacles


def path_metrics_row(args: argparse.Namespace, spec: VoxelGridSpec, occ: np.ndarray, config: MPPI3DConfig, result) -> dict[str, object]:
    return {
        "method": "esdf_mppi_3d",
        "map_resolution_m": spec.resolution_m,
        "voxels_total": int(np.prod(occ.shape)),
        "occupied_voxels": int(np.sum(occ)),
        "horizon_steps": config.horizon_steps,
        "num_rollouts": config.num_rollouts,
        "iterations": config.max_iterations,
        "safety_radius_m": config.safety_radius_m,
        "path_length_m": round(result.path_length_m, 4),
        "smoothness": round(result.smoothness, 6),
        "min_altitude_m": round(float(np.min(result.trajectory_xyz[:, 2])), 4),
        "max_altitude_m": round(float(np.max(result.trajectory_xyz[:, 2])), 4),
        "altitude_change_m": round(float(np.max(result.trajectory_xyz[:, 2]) - np.min(result.trajectory_xyz[:, 2])), 4),
        "min_esdf_distance_m": round(result.min_esdf_distance_m, 4),
        "min_safety_margin_m": round(result.min_esdf_distance_m - config.safety_radius_m, 4),
        "collision": int(result.collision),
        "safety_violation": int(result.safety_violation),
        "planner_compute_time_ms": round(result.compute_time_s * 1000.0, 3),
        "seed": args.seed,
    }


def write_csv(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)


def _draw_box(ax, center: tuple[float, float, float], size: tuple[float, float, float], color: str, alpha: float) -> None:
    cx, cy, cz = center
    sx, sy, sz = size
    x = [cx - sx / 2, cx + sx / 2]
    y = [cy - sy / 2, cy + sy / 2]
    z = [cz - sz / 2, cz + sz / 2]
    corners = np.array([[xx, yy, zz] for xx in x for yy in y for zz in z], dtype=float)
    edges = [
        (0, 1), (0, 2), (0, 4), (3, 1), (3, 2), (3, 7),
        (5, 1), (5, 4), (5, 7), (6, 2), (6, 4), (6, 7),
    ]
    for i, j in edges:
        ax.plot(*zip(corners[i], corners[j]), color=color, alpha=min(alpha + 0.25, 1.0), linewidth=1.0)
    ax.scatter(corners[:, 0], corners[:, 1], corners[:, 2], color=color, alpha=alpha, s=4)


def _draw_cylinder(ax, center_xy: tuple[float, float], radius: float, z_min: float, z_max: float, color: str, alpha: float) -> None:
    theta = np.linspace(0.0, 2.0 * math.pi, 50)
    x = center_xy[0] + radius * np.cos(theta)
    y = center_xy[1] + radius * np.sin(theta)
    for z in (z_min, z_max):
        ax.plot(x, y, np.full_like(x, z), color=color, alpha=alpha, linewidth=1.0)
    for idx in range(0, len(theta), 8):
        ax.plot([x[idx], x[idx]], [y[idx], y[idx]], [z_min, z_max], color=color, alpha=alpha, linewidth=0.8)


def plot_outputs(
    fig_path: Path,
    slice_path: Path,
    spec: VoxelGridSpec,
    occ: np.ndarray,
    esdf,
    obstacles: list[dict[str, object]],
    start: np.ndarray,
    goal: np.ndarray,
    trajectory: np.ndarray,
    config: MPPI3DConfig,
) -> None:
    fig_path.parent.mkdir(parents=True, exist_ok=True)
    slice_path.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(14, 5.6))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    for obs in obstacles:
        if obs["type"] == "box":
            color = "#64748b" if "wall" in str(obs["name"]) or "bench" in str(obs["name"]) else "#dc4f3d"
            _draw_box(ax, obs["center"], obs["size"], color, 0.36)
        else:
            _draw_cylinder(ax, obs["center_xy"], float(obs["radius"]), float(obs["z_min"]), float(obs["z_max"]), "#dc4f3d", 0.62)
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color="#2563eb", linewidth=2.4, label="3D MPPI path")
    ax.scatter([start[0]], [start[1]], [start[2]], color="#16a34a", s=55, label="A")
    ax.scatter([goal[0]], [goal[1]], [goal[2]], color="#4f46e5", s=55, label="B")
    ax.set_title("Indoor 3D ESDF MPPI trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z altitude [m]")
    ax.set_xlim(spec.origin[0], spec.upper[0])
    ax.set_ylim(spec.origin[1], spec.upper[1])
    ax.set_zlim(0, spec.upper[2])
    ax.legend(loc="upper left")

    ax2 = fig.add_subplot(1, 2, 2)
    z_idx = int(np.clip(np.floor((start[2] - spec.origin[2]) / spec.resolution_m), 0, spec.nz - 1))
    slice_dist = esdf.signed_distance_m[:, :, z_idx].T
    extent = [spec.origin[0], spec.upper[0], spec.origin[1], spec.upper[1]]
    im = ax2.imshow(
        np.clip(slice_dist, -0.2, 1.8),
        origin="lower",
        extent=extent,
        cmap="viridis",
        aspect="auto",
    )
    ax2.plot(trajectory[:, 0], trajectory[:, 1], color="#ffffff", linewidth=2.2)
    ax2.plot(trajectory[:, 0], trajectory[:, 1], color="#2563eb", linewidth=1.3)
    ax2.scatter([start[0], goal[0]], [start[1], goal[1]], c=["#16a34a", "#4f46e5"], s=48)
    ax2.set_title(f"ESDF slice at z={start[2]:.1f} m")
    ax2.set_xlabel("x [m]")
    ax2.set_ylabel("y [m]")
    fig.colorbar(im, ax=ax2, label="distance to obstacle [m]")
    fig.suptitle(f"Min ESDF distance {np.min(esdf.query_distance(trajectory)):.2f} m; safety radius {config.safety_radius_m:.2f} m")
    fig.tight_layout()
    fig.savefig(fig_path, dpi=180)
    plt.close(fig)

    fig2, ax3 = plt.subplots(figsize=(8, 5))
    im2 = ax3.imshow(np.clip(slice_dist, -0.2, 1.8), origin="lower", extent=extent, cmap="viridis", aspect="auto")
    ax3.plot(trajectory[:, 0], trajectory[:, 1], color="#ffffff", linewidth=3)
    ax3.plot(trajectory[:, 0], trajectory[:, 1], color="#2563eb", linewidth=1.6)
    ax3.set_title("Top-down ESDF slice used by MPPI")
    ax3.set_xlabel("x [m]")
    ax3.set_ylabel("y [m]")
    fig2.colorbar(im2, ax=ax3, label="distance to obstacle [m]")
    fig2.tight_layout()
    fig2.savefig(slice_path, dpi=180)
    plt.close(fig2)


def write_summary(path: Path, metrics: dict[str, object], figure: Path, slice_figure: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Indoor 3D ESDF MPPI Demo",
        "",
        "This experiment is a local Level-3 prototype: a 3D voxel occupancy map is",
        "converted into an ESDF, then MPPI optimizes a continuous `[x, y, z]`",
        "trajectory by querying metric obstacle distance.",
        "",
        "## Outputs",
        "",
        f"- Metrics CSV: `outputs/tables/indoor_3d_esdf_mppi_metrics.csv`",
        f"- 3D/path figure: `{figure}`",
        f"- ESDF slice figure: `{slice_figure}`",
        "",
        "## Key Metrics",
        "",
        "| Metric | Value |",
        "|---|---:|",
    ]
    for key in [
        "path_length_m",
        "min_esdf_distance_m",
        "min_safety_margin_m",
        "smoothness",
        "altitude_change_m",
        "planner_compute_time_ms",
        "collision",
        "safety_violation",
    ]:
        lines.append(f"| `{key}` | {metrics[key]} |")
    lines += [
        "",
        "## Interpretation",
        "",
        "This proves the project can evaluate clearance from a 3D distance field,",
        "not only from a 2D binary occupancy grid.  The next server step is to",
        "replace this synthetic ESDF with NVBlox ESDF topics generated from real",
        "depth/LiDAR streams.",
        "",
    ]
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    args = parse_args()
    spec, occ, obstacles = build_indoor_scene(args.resolution)
    esdf = compute_esdf(occ, spec, prefer_scipy=args.prefer_scipy)

    start = np.asarray([0.0, 0.0, 1.15], dtype=float)
    goal = np.asarray([6.55, 0.0, 1.15], dtype=float)
    config = MPPI3DConfig(
        num_rollouts=args.rollouts,
        horizon_steps=args.horizon_steps,
        max_iterations=args.iterations,
        seed=args.seed,
    )
    result = mppi_3d_esdf_path(start, goal, esdf, config)
    direct = np.linspace(start, goal, config.horizon_steps)
    direct_cost = float(rollout_cost(direct[None, :, :], esdf, config)[0])
    result_cost = float(rollout_cost(result.trajectory_xyz[None, :, :], esdf, config)[0])

    metrics = path_metrics_row(args, spec, occ, config, result)
    metrics["direct_line_cost"] = round(direct_cost, 4)
    metrics["optimized_cost"] = round(result_cost, 4)
    metrics["cost_reduction_pct"] = round((direct_cost - result_cost) / max(direct_cost, 1e-9) * 100.0, 2)

    table_path = args.output_dir / "tables" / "indoor_3d_esdf_mppi_metrics.csv"
    fig_path = args.output_dir / "figures" / "indoor_3d_esdf_mppi_path.png"
    slice_path = args.output_dir / "figures" / "indoor_3d_esdf_mppi_slice.png"
    summary_path = args.output_dir / "indoor_3d_esdf_mppi_summary.md"
    data_path = Path("data/processed/esdf3d/indoor_demo_esdf_mppi.npz")

    write_csv(table_path, metrics)
    plot_outputs(fig_path, slice_path, spec, occ, esdf, obstacles, start, goal, result.trajectory_xyz, config)
    write_summary(summary_path, metrics, fig_path, slice_path)
    data_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        data_path,
        occupancy=occ.astype(np.uint8),
        signed_distance_m=esdf.signed_distance_m.astype(np.float32),
        trajectory_xyz=result.trajectory_xyz.astype(np.float32),
        spec=np.asarray([spec.nx, spec.ny, spec.nz, spec.resolution_m, *spec.origin_xyz], dtype=float),
        metrics_json=json.dumps(metrics),
    )

    print(f"Wrote {table_path}")
    print(f"Wrote {fig_path}")
    print(f"Wrote {slice_path}")
    print(f"Wrote {summary_path}")
    print(f"Wrote {data_path}")
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
