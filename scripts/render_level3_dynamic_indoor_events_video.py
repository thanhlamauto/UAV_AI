#!/usr/bin/env python3
"""Render a dynamic indoor Level-3 ESDF/MPPI video with surprise events."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.esdf3d import ESDF3D, VoxelGridSpec, compute_esdf, empty_occupancy, mark_box, mark_cylinder
from src.planners.mppi_3d_esdf import MPPI3DConfig, mppi_3d_esdf_path


@dataclass(frozen=True)
class ObstacleVis:
    name: str
    kind: str
    color: str
    center: tuple[float, float, float] | None = None
    size: tuple[float, float, float] | None = None
    center_xy: tuple[float, float] | None = None
    radius: float | None = None
    z_min: float | None = None
    z_max: float | None = None


@dataclass(frozen=True)
class Stage:
    name: str
    event: str
    duration_s: float
    path_fraction: float
    occupancy: np.ndarray
    obstacles: list[ObstacleVis]
    esdf: ESDF3D
    plan: np.ndarray
    compute_time_ms: float
    min_esdf_distance_m: float
    safety_violation: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/level3_dynamic_indoor_events_esdf_mppi.mp4"))
    parser.add_argument("--metrics", type=Path, default=Path("outputs/tables/level3_dynamic_indoor_events_mppi.csv"))
    parser.add_argument("--summary", type=Path, default=Path("outputs/level3_dynamic_indoor_events_summary.md"))
    parser.add_argument("--preview", type=Path, default=Path("outputs/figures/level3_video_preview/level3_dynamic_midframe.png"))
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--duration-s", type=float, default=14.0)
    parser.add_argument("--dpi", type=int, default=130)
    parser.add_argument("--resolution", type=float, default=0.18)
    parser.add_argument("--seed", type=int, default=53)
    return parser.parse_args()


def _resample(points: np.ndarray, n: int) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) <= 1:
        return np.repeat(points[:1], n, axis=0)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(lengths.sum())
    if total <= 1e-9:
        return np.repeat(points[:1], n, axis=0)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    targets = np.linspace(0.0, total, n)
    out = np.zeros((n, 3), dtype=float)
    seg = 0
    for i, target in enumerate(targets):
        while seg < len(lengths) - 1 and cumulative[seg + 1] < target:
            seg += 1
        denom = cumulative[seg + 1] - cumulative[seg]
        alpha = 0.0 if denom <= 0.0 else (target - cumulative[seg]) / denom
        out[i] = (1.0 - alpha) * points[seg] + alpha * points[seg + 1]
    return out


def make_spec(resolution: float) -> VoxelGridSpec:
    return VoxelGridSpec(
        nx=int(math.ceil(8.2 / resolution)),
        ny=int(math.ceil(6.4 / resolution)),
        nz=int(math.ceil(3.2 / resolution)),
        resolution_m=resolution,
        origin_xyz=(-0.7, -3.2, 0.0),
    )


def add_box(
    occupancy: np.ndarray,
    spec: VoxelGridSpec,
    vis: list[ObstacleVis],
    name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    color: str,
) -> None:
    mark_box(occupancy, spec, center, size)
    vis.append(ObstacleVis(name=name, kind="box", center=center, size=size, color=color))


def add_cylinder(
    occupancy: np.ndarray,
    spec: VoxelGridSpec,
    vis: list[ObstacleVis],
    name: str,
    center_xy: tuple[float, float],
    radius: float,
    z_min: float,
    z_max: float,
    color: str,
) -> None:
    mark_cylinder(occupancy, spec, center_xy, radius, z_min, z_max)
    vis.append(
        ObstacleVis(
            name=name,
            kind="cylinder",
            center_xy=center_xy,
            radius=radius,
            z_min=z_min,
            z_max=z_max,
            color=color,
        )
    )


def build_scene(spec: VoxelGridSpec, stage_idx: int) -> tuple[np.ndarray, list[ObstacleVis]]:
    occ = empty_occupancy(spec)
    vis: list[ObstacleVis] = []

    wall_color = "#94a3b8"
    furniture = "#64748b"
    static_obs = "#ef4444"
    surprise = "#f97316"

    add_box(occ, spec, vis, "left glass wall", (3.3, -2.72, 1.35), (7.9, 0.18, 2.7), wall_color)
    add_box(occ, spec, vis, "right shelf wall", (3.3, 2.72, 1.35), (7.9, 0.18, 2.7), wall_color)
    add_box(occ, spec, vis, "lab bench left", (1.2, -1.92, 0.52), (1.15, 0.58, 1.05), furniture)
    add_box(occ, spec, vis, "server rack", (5.9, 1.82, 0.85), (1.0, 0.65, 1.7), furniture)
    add_box(occ, spec, vis, "center cabinet", (4.2, 0.72, 0.88), (0.9, 0.72, 1.76), static_obs)
    add_cylinder(occ, spec, vis, "orange pillar", (2.15, -0.55), 0.34, 0.0, 2.35, static_obs)
    add_box(occ, spec, vis, "low crossbar", (3.25, -0.25, 0.48), (0.28, 3.0, 0.96), static_obs)

    if stage_idx >= 1:
        add_cylinder(occ, spec, vis, "surprise person crossing", (2.92, 0.42), 0.30, 0.0, 1.85, surprise)
    if stage_idx >= 2:
        add_box(occ, spec, vis, "door panel closing", (4.98, -0.58, 1.1), (0.26, 1.65, 2.2), surprise)
    if stage_idx >= 3:
        add_box(occ, spec, vis, "cart appears", (5.78, 0.35, 0.7), (1.08, 0.9, 1.4), surprise)
    return occ, vis


def plan_stage(
    spec: VoxelGridSpec,
    stage_idx: int,
    start: np.ndarray,
    goal: np.ndarray,
    seed: int,
) -> Stage:
    occ, vis = build_scene(spec, stage_idx)
    esdf = compute_esdf(occ, spec, prefer_scipy=True)
    config = MPPI3DConfig(
        num_rollouts=360,
        horizon_steps=64,
        max_iterations=7,
        temperature=0.9,
        noise_sigma_m=0.32,
        safety_radius_m=0.40,
        min_altitude_m=0.55,
        max_altitude_m=2.35,
        clearance_weight=180.0,
        smoothness_weight=4.0,
        seed=seed + stage_idx * 17,
    )
    result = mppi_3d_esdf_path(start, goal, esdf, config)
    names = [
        ("Baseline", "Nominal indoor corridor"),
        ("Event 1", "Person suddenly crosses the flight corridor"),
        ("Event 2", "Door panel narrows the passage"),
        ("Event 3", "Cart/box appears near the old route"),
    ]
    fractions = [0.24, 0.33, 0.43, 1.0]
    return Stage(
        name=names[stage_idx][0],
        event=names[stage_idx][1],
        duration_s=[3.0, 3.2, 3.2, 4.6][stage_idx],
        path_fraction=fractions[stage_idx],
        occupancy=occ,
        obstacles=vis,
        esdf=esdf,
        plan=result.trajectory_xyz,
        compute_time_ms=result.compute_time_s * 1000.0,
        min_esdf_distance_m=result.min_esdf_distance_m,
        safety_violation=result.safety_violation,
    )


def build_stages(args: argparse.Namespace) -> tuple[VoxelGridSpec, list[Stage], np.ndarray]:
    spec = make_spec(float(args.resolution))
    goal = np.asarray([6.75, 0.0, 1.15], dtype=float)
    current = np.asarray([0.0, 0.0, 1.15], dtype=float)
    stages: list[Stage] = []
    for idx in range(4):
        stage = plan_stage(spec, idx, current, goal, int(args.seed))
        stages.append(stage)
        sampled = _resample(stage.plan, 160)
        current = sampled[min(len(sampled) - 1, max(1, int(stage.path_fraction * (len(sampled) - 1))))]
    return spec, stages, goal


def voxel_points(occupancy: np.ndarray, spec: VoxelGridSpec) -> np.ndarray:
    idx = np.argwhere(occupancy.astype(bool))
    if len(idx) == 0:
        return np.empty((0, 3), dtype=float)
    origin = spec.origin
    res = float(spec.resolution_m)
    return origin[None, :] + (idx.astype(float) + 0.5) * res


def draw_box(ax, obs: ObstacleVis, alpha: float = 0.26) -> list[object]:
    assert obs.center is not None and obs.size is not None
    artists: list[object] = []
    cx, cy, cz = obs.center
    sx, sy, sz = obs.size
    x = [cx - sx / 2, cx + sx / 2]
    y = [cy - sy / 2, cy + sy / 2]
    z = [cz - sz / 2, cz + sz / 2]
    corners = np.array([[xx, yy, zz] for xx in x for yy in y for zz in z], dtype=float)
    edges = [(0, 1), (0, 2), (0, 4), (3, 1), (3, 2), (3, 7), (5, 1), (5, 4), (5, 7), (6, 2), (6, 4), (6, 7)]
    for i, j in edges:
        line = ax.plot(*zip(corners[i], corners[j]), color=obs.color, alpha=min(1.0, alpha + 0.25), linewidth=1.1)[0]
        artists.append(line)
    artists.append(ax.scatter(corners[:, 0], corners[:, 1], corners[:, 2], color=obs.color, alpha=alpha, s=4))
    return artists


def draw_cylinder(ax, obs: ObstacleVis, alpha: float = 0.55) -> list[object]:
    assert obs.center_xy is not None and obs.radius is not None and obs.z_min is not None and obs.z_max is not None
    artists: list[object] = []
    theta = np.linspace(0.0, 2.0 * math.pi, 44)
    x = obs.center_xy[0] + obs.radius * np.cos(theta)
    y = obs.center_xy[1] + obs.radius * np.sin(theta)
    for z in (obs.z_min, obs.z_max):
        artists.append(ax.plot(x, y, np.full_like(x, z), color=obs.color, alpha=alpha, linewidth=1.1)[0])
    for j in range(0, len(theta), 8):
        artists.append(
            ax.plot([x[j], x[j]], [y[j], y[j]], [obs.z_min, obs.z_max], color=obs.color, alpha=alpha, linewidth=0.9)[0]
        )
    return artists


def write_metrics(path: Path, stages: list[Stage]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for idx, stage in enumerate(stages):
        rows.append(
            {
                "stage": idx,
                "name": stage.name,
                "event": stage.event,
                "compute_time_ms": round(stage.compute_time_ms, 3),
                "min_esdf_distance_m": round(stage.min_esdf_distance_m, 4),
                "safety_violation": int(stage.safety_violation),
                "planned_waypoints": len(stage.plan),
                "occupied_voxels": int(stage.occupancy.sum()),
            }
        )
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, output: Path, metrics: Path, stages: list[Stage]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Level 3 Dynamic Indoor Events Video",
        "",
        "Video:",
        f"- `{output}`",
        "",
        "Metrics:",
        f"- `{metrics}`",
        "",
        "Scenario:",
        "",
        "```text",
        "indoor corridor/lab",
        "  -> static furniture/walls/pillars",
        "  -> surprise person crossing",
        "  -> door panel closing",
        "  -> cart/box appearing",
        "  -> voxel map / ESDF update",
        "  -> MPPI replanning in x,y,z",
        "```",
        "",
        "Stage metrics:",
        "",
        "| Stage | Event | Min ESDF [m] | Compute [ms] | Safety violation |",
        "|---|---|---:|---:|---:|",
    ]
    for stage in stages:
        lines.append(
            f"| {stage.name} | {stage.event} | {stage.min_esdf_distance_m:.4f} | "
            f"{stage.compute_time_ms:.3f} | {int(stage.safety_violation)} |"
        )
    lines.extend(
        [
            "",
            "Scope note: this is a simulated dynamic indoor Level-3 visualization.",
            "It demonstrates online replanning against changing 3D occupancy/ESDF maps,",
            "but it is not yet a physical robot log or PX4 closed-loop flight.",
        ]
    )
    path.write_text("\n".join(lines) + "\n")


def render_video(args: argparse.Namespace, spec: VoxelGridSpec, stages: list[Stage], goal: np.ndarray) -> None:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    total_duration = float(sum(stage.duration_s for stage in stages))
    fps = int(args.fps)
    frame_count = int(round(total_duration * fps))

    stage_frame_ranges: list[tuple[int, int]] = []
    cursor = 0
    actual_segments = []
    for idx, stage in enumerate(stages):
        count = int(round(stage.duration_s * fps))
        if idx == len(stages) - 1:
            count = frame_count - cursor
        stage_frame_ranges.append((cursor, cursor + count))
        if idx == len(stages) - 1:
            segment = _resample(stage.plan, count)
        else:
            sampled = _resample(stage.plan, 160)
            end = max(2, int(stage.path_fraction * (len(sampled) - 1)))
            segment = _resample(sampled[: end + 1], count)
        actual_segments.append(segment)
        cursor += count
    actual_path = np.vstack(actual_segments)

    stage_voxels = [voxel_points(stage.occupancy, spec) for stage in stages]
    finite = np.isfinite(stages[0].esdf.signed_distance_m)
    max_dist = 2.0 if not np.any(finite) else min(2.0, float(np.nanpercentile(stages[0].esdf.signed_distance_m[finite], 96)))
    safe_radius = 0.40
    extent = [spec.origin[0], spec.upper[0], spec.origin[1], spec.upper[1]]

    fig = plt.figure(figsize=(16, 9), facecolor="#f8fafc")
    gs = fig.add_gridspec(2, 3, width_ratios=[1.42, 1.0, 0.92], height_ratios=[1.0, 0.42])
    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_slice = fig.add_subplot(gs[0, 1])
    ax_alt = fig.add_subplot(gs[1, 1])
    ax_status = fig.add_subplot(gs[:, 2])

    fig.suptitle("Level 3 dynamic indoor ESDF/MPPI: surprise events and replanning", fontsize=18, fontweight="bold")

    ax3d.set_xlim(spec.origin[0], spec.upper[0])
    ax3d.set_ylim(spec.origin[1], spec.upper[1])
    ax3d.set_zlim(spec.origin[2], spec.upper[2])
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("z [m]")
    ax3d.set_title("Indoor 3D voxel scene")

    initial_stage = stages[0]
    vox = stage_voxels[0]
    voxel_scatter = ax3d.scatter(vox[:, 0], vox[:, 1], vox[:, 2], s=5, c="#ef4444", alpha=0.13, depthshade=False)
    obstacle_artists: list[object] = []
    for obs in initial_stage.obstacles:
        if obs.kind == "box":
            obstacle_artists.extend(draw_box(ax3d, obs))
        else:
            obstacle_artists.extend(draw_cylinder(ax3d, obs))
    plan_line, = ax3d.plot([], [], [], color="#94a3b8", linewidth=1.7, alpha=0.9, label="current MPPI plan")
    flown_line, = ax3d.plot([], [], [], color="#2563eb", linewidth=3.0, label="executed path")
    uav = ax3d.scatter([], [], [], s=72, color="#f59e0b", edgecolor="#111827", linewidth=0.8, label="MAV")
    ax3d.scatter([actual_path[0, 0]], [actual_path[0, 1]], [actual_path[0, 2]], s=58, color="#16a34a", label="A")
    ax3d.scatter([goal[0]], [goal[1]], [goal[2]], s=58, color="#7c3aed", label="B")
    ax3d.legend(loc="upper left", fontsize=9)

    z_idx = int(np.clip(round((actual_path[0, 2] - spec.origin[2]) / spec.resolution_m - 0.5), 0, spec.nz - 1))
    im = ax_slice.imshow(
        np.clip(initial_stage.esdf.signed_distance_m[:, :, z_idx].T, -safe_radius, max_dist),
        origin="lower",
        extent=extent,
        cmap="viridis",
        vmin=-safe_radius,
        vmax=max_dist,
        aspect="auto",
    )
    slice_plan, = ax_slice.plot([], [], color="#ffffff", linewidth=3.0)
    slice_plan_inner, = ax_slice.plot([], [], color="#94a3b8", linewidth=1.6)
    slice_flown, = ax_slice.plot([], [], color="#2563eb", linewidth=2.2)
    slice_uav = ax_slice.scatter([], [], s=70, color="#f59e0b", edgecolor="#111827", linewidth=0.8)
    ax_slice.scatter([actual_path[0, 0], goal[0]], [actual_path[0, 1], goal[1]], c=["#16a34a", "#7c3aed"], s=52)
    ax_slice.set_title("ESDF slice at current altitude")
    ax_slice.set_xlabel("x [m]")
    ax_slice.set_ylabel("y [m]")
    cbar = fig.colorbar(im, ax=ax_slice, fraction=0.046, pad=0.03)
    cbar.set_label("distance to obstacle [m]")

    times = np.linspace(0.0, total_duration, len(actual_path))
    ax_alt.plot(times, actual_path[:, 2], color="#0f766e", linewidth=2.2)
    alt_marker, = ax_alt.plot([], [], marker="o", color="#f59e0b", markersize=8)
    ax_alt.set_xlim(0, total_duration)
    ax_alt.set_ylim(max(0.0, float(np.min(actual_path[:, 2])) - 0.12), float(np.max(actual_path[:, 2])) + 0.2)
    ax_alt.set_xlabel("time [s]")
    ax_alt.set_ylabel("z [m]")
    ax_alt.set_title("Altitude response")
    ax_alt.grid(True, alpha=0.25)
    for start, _end in stage_frame_ranges[1:]:
        ax_alt.axvline(times[start], color="#f97316", linestyle="--", linewidth=1.0, alpha=0.75)

    ax_status.set_axis_off()
    status_box = plt.Rectangle((0.02, 0.02), 0.96, 0.96, transform=ax_status.transAxes, facecolor="#ffffff", edgecolor="#cbd5e1", linewidth=1.2)
    ax_status.add_patch(status_box)
    status_text = ax_status.text(0.07, 0.94, "", transform=ax_status.transAxes, va="top", ha="left", fontsize=11, linespacing=1.35, color="#111827")
    bottom = fig.text(0.5, 0.035, "", ha="center", va="center", fontsize=12, color="#334155")

    def stage_for_frame(frame: int) -> tuple[int, Stage, int, int]:
        for i, (start, end) in enumerate(stage_frame_ranges):
            if start <= frame < end:
                return i, stages[i], start, end
        start, end = stage_frame_ranges[-1]
        return len(stages) - 1, stages[-1], start, end

    current_stage_idx = -1
    writer = FFMpegWriter(fps=fps, metadata={"title": "Level 3 dynamic indoor ESDF MPPI"}, bitrate=4600)
    with writer.saving(fig, str(args.output), dpi=int(args.dpi)):
        for frame in range(frame_count):
            idx, stage, start_frame, end_frame = stage_for_frame(frame)
            point = actual_path[frame]
            local_alpha = (frame - start_frame) / max(1, end_frame - start_frame - 1)

            if idx != current_stage_idx:
                current_stage_idx = idx
                vox = stage_voxels[idx]
                voxel_scatter._offsets3d = (vox[:, 0], vox[:, 1], vox[:, 2])
                for artist in obstacle_artists:
                    try:
                        artist.remove()
                    except ValueError:
                        pass
                obstacle_artists = []
                for obs in stage.obstacles:
                    if obs.kind == "box":
                        obstacle_artists.extend(draw_box(ax3d, obs))
                    else:
                        obstacle_artists.extend(draw_cylinder(ax3d, obs))

            plan = stage.plan
            plan_line.set_data(plan[:, 0], plan[:, 1])
            plan_line.set_3d_properties(plan[:, 2])
            flown = actual_path[: frame + 1]
            flown_line.set_data(flown[:, 0], flown[:, 1])
            flown_line.set_3d_properties(flown[:, 2])
            uav._offsets3d = ([point[0]], [point[1]], [point[2]])

            z_idx = int(np.clip(round((point[2] - spec.origin[2]) / spec.resolution_m - 0.5), 0, spec.nz - 1))
            im.set_data(np.clip(stage.esdf.signed_distance_m[:, :, z_idx].T, -safe_radius, max_dist))
            ax_slice.set_title(f"ESDF slice at z={point[2]:.2f} m")
            slice_plan.set_data(plan[:, 0], plan[:, 1])
            slice_plan_inner.set_data(plan[:, 0], plan[:, 1])
            slice_flown.set_data(flown[:, 0], flown[:, 1])
            slice_uav.set_offsets(np.array([[point[0], point[1]]]))
            alt_marker.set_data([times[frame]], [point[2]])

            ax3d.view_init(elev=23, azim=-56 + 32 * frame / max(1, frame_count - 1))
            status_text.set_text(
                "\n".join(
                    [
                        f"{stage.name}: {stage.event}",
                        "",
                        "Mapping/planning loop",
                        "PointCloud/LiDAR-like obstacles",
                        "-> 3D occupancy voxels",
                        "-> ESDF distance field",
                        "-> MPPI replanning in x,y,z",
                        "",
                        f"Stage compute: {stage.compute_time_ms:.1f} ms",
                        f"Min ESDF distance: {stage.min_esdf_distance_m:.3f} m",
                        f"Safety violation: {int(stage.safety_violation)}",
                        f"Current altitude: {point[2]:.2f} m",
                        f"Event progress: {100.0 * local_alpha:.0f}%",
                        "",
                        "Scope: simulated indoor surprise events;",
                        "not physical robot/PX4 log yet.",
                    ]
                )
            )
            bottom.set_text(f"t={times[frame]:.1f}s | active event: {stage.event} | orange = surprise obstacle")
            writer.grab_frame()

    plt.close(fig)


def main() -> int:
    args = parse_args()
    spec, stages, goal = build_stages(args)
    write_metrics(args.metrics, stages)
    write_summary(args.summary, args.output, args.metrics, stages)
    render_video(args, spec, stages, goal)

    args.preview.parent.mkdir(parents=True, exist_ok=True)
    # Preview extraction is handled by ffmpeg in the caller; keep paths stable in summary.
    print(args.output)
    print(args.metrics)
    print(args.summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
