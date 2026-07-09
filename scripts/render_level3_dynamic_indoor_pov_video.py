#!/usr/bin/env python3
"""Render a drone-POV Level-3 dynamic indoor ESDF/MPPI video."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Circle, Polygon, Rectangle

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_dynamic_module():
    path = REPO_ROOT / "scripts" / "render_level3_dynamic_indoor_events_video.py"
    spec = importlib.util.spec_from_file_location("level3_dynamic_events", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


dyn = _load_dynamic_module()


@dataclass(frozen=True)
class RuntimePath:
    points: np.ndarray
    stage_ranges: list[tuple[int, int]]
    stage_segments: list[np.ndarray]
    times: np.ndarray
    total_duration_s: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/level3_dynamic_indoor_pov_esdf_mppi.mp4"))
    parser.add_argument("--metrics", type=Path, default=Path("outputs/tables/level3_dynamic_indoor_events_mppi.csv"))
    parser.add_argument("--preview", type=Path, default=Path("outputs/figures/level3_video_preview/level3_dynamic_pov_midframe.png"))
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--dpi", type=int, default=130)
    parser.add_argument("--resolution", type=float, default=0.18)
    parser.add_argument("--seed", type=int, default=53)
    return parser.parse_args()


def build_runtime(stages: list[object], fps: int) -> RuntimePath:
    total_duration = float(sum(float(stage.duration_s) for stage in stages))
    frame_count = int(round(total_duration * fps))
    ranges: list[tuple[int, int]] = []
    segments: list[np.ndarray] = []
    cursor = 0
    for idx, stage in enumerate(stages):
        count = int(round(float(stage.duration_s) * fps))
        if idx == len(stages) - 1:
            count = frame_count - cursor
        ranges.append((cursor, cursor + count))
        if idx == len(stages) - 1:
            segment = dyn._resample(stage.plan, count)
        else:
            sampled = dyn._resample(stage.plan, 180)
            end = max(2, int(float(stage.path_fraction) * (len(sampled) - 1)))
            segment = dyn._resample(sampled[: end + 1], count)
        segments.append(segment)
        cursor += count
    points = np.vstack(segments)
    times = np.linspace(0.0, total_duration, len(points))
    return RuntimePath(points=points, stage_ranges=ranges, stage_segments=segments, times=times, total_duration_s=total_duration)


def stage_for_frame(runtime: RuntimePath, stages: list[object], frame: int) -> tuple[int, object, int, int]:
    for idx, (start, end) in enumerate(runtime.stage_ranges):
        if start <= frame < end:
            return idx, stages[idx], start, end
    start, end = runtime.stage_ranges[-1]
    return len(stages) - 1, stages[-1], start, end


def camera_axes(points: np.ndarray, frame: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    here = points[frame]
    look_idx = min(len(points) - 1, frame + 10)
    forward = points[look_idx] - here
    forward[2] = 0.0
    norm = float(np.linalg.norm(forward))
    if norm <= 1e-6:
        forward = np.asarray([1.0, 0.0, 0.0])
    else:
        forward = forward / norm
    right = np.asarray([forward[1], -forward[0], 0.0])
    up = np.asarray([0.0, 0.0, 1.0])
    return forward, right, up


def project(points: np.ndarray, cam: np.ndarray, forward: np.ndarray, right: np.ndarray, up: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rel = np.asarray(points, dtype=float) - cam[None, :]
    x = rel @ right
    y = rel @ up
    z = rel @ forward
    pitch_offset = 0.10
    focal = 0.90
    screen = np.empty((len(points), 2), dtype=float)
    z_safe = np.maximum(z, 0.08)
    screen[:, 0] = 0.5 + focal * x / z_safe
    screen[:, 1] = 0.54 - focal * (y - pitch_offset * z_safe) / z_safe
    return screen, z


def box_corners(center: tuple[float, float, float], size: tuple[float, float, float]) -> np.ndarray:
    cx, cy, cz = center
    sx, sy, sz = size
    return np.asarray([[cx + dx * sx / 2, cy + dy * sy / 2, cz + dz * sz / 2] for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)], dtype=float)


def box_faces() -> list[tuple[int, int, int, int]]:
    return [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]


def lighten(color: str, amount: float) -> str:
    color = color.lstrip("#")
    r, g, b = [int(color[i : i + 2], 16) for i in (0, 2, 4)]
    r = int(r + (255 - r) * amount)
    g = int(g + (255 - g) * amount)
    b = int(b + (255 - b) * amount)
    return f"#{r:02x}{g:02x}{b:02x}"


def draw_pov_background(ax) -> None:
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_axis_off()
    ax.add_patch(Rectangle((0, 0), 1, 1, facecolor="#e8eef4", edgecolor="none"))
    ax.add_patch(Polygon([(0, 0), (1, 0), (0.72, 0.48), (0.28, 0.48)], closed=True, facecolor="#cbd5c6", edgecolor="none", alpha=0.95))
    ax.add_patch(Polygon([(0, 1), (1, 1), (0.72, 0.58), (0.28, 0.58)], closed=True, facecolor="#f8fafc", edgecolor="none", alpha=0.98))
    ax.plot([0, 1], [0.54, 0.54], color="#94a3b8", linewidth=1.0, alpha=0.45)
    for x in np.linspace(-0.2, 1.2, 9):
        ax.plot([x, 0.5], [0.0, 0.54], color="#94a3b8", linewidth=0.65, alpha=0.23)
    for y in np.linspace(0.08, 0.48, 6):
        ax.plot([0.08 + 0.38 * y, 0.92 - 0.38 * y], [y, y], color="#94a3b8", linewidth=0.65, alpha=0.20)


def draw_box_pov(ax, obs, cam: np.ndarray, forward: np.ndarray, right: np.ndarray, up: np.ndarray) -> None:
    corners = box_corners(obs.center, obs.size)
    screen, depth = project(corners, cam, forward, right, up)
    if np.max(depth) <= 0.12:
        return
    faces = []
    for face in box_faces():
        d = depth[list(face)]
        if np.mean(d) <= 0.15:
            continue
        pts = screen[list(face)]
        if np.all((pts[:, 0] < -0.25) | (pts[:, 0] > 1.25)) or np.all((pts[:, 1] < -0.25) | (pts[:, 1] > 1.25)):
            continue
        faces.append((float(np.mean(d)), pts))
    for depth_mean, pts in sorted(faces, key=lambda item: item[0], reverse=True):
        shade = float(np.clip(depth_mean / 7.5, 0.0, 0.55))
        ax.add_patch(
            Polygon(
                pts,
                closed=True,
                facecolor=lighten(obs.color, shade),
                edgecolor="#334155",
                linewidth=0.8,
                alpha=0.78 if obs.color != "#f97316" else 0.88,
            )
        )


def draw_cylinder_pov(ax, obs, cam: np.ndarray, forward: np.ndarray, right: np.ndarray, up: np.ndarray) -> None:
    center = np.asarray([obs.center_xy[0], obs.center_xy[1], (obs.z_min + obs.z_max) * 0.5], dtype=float)
    radius = float(obs.radius)
    height = float(obs.z_max - obs.z_min)
    billboard_right = right * radius
    pts3 = np.asarray(
        [
            center - billboard_right + np.asarray([0.0, 0.0, -height / 2]),
            center + billboard_right + np.asarray([0.0, 0.0, -height / 2]),
            center + billboard_right + np.asarray([0.0, 0.0, height / 2]),
            center - billboard_right + np.asarray([0.0, 0.0, height / 2]),
        ]
    )
    pts, depth = project(pts3, cam, forward, right, up)
    if np.mean(depth) <= 0.15:
        return
    if np.all((pts[:, 0] < -0.25) | (pts[:, 0] > 1.25)) or np.all((pts[:, 1] < -0.25) | (pts[:, 1] > 1.25)):
        return
    ax.add_patch(
        Polygon(
            pts,
            closed=True,
            facecolor=obs.color,
            edgecolor="#7f1d1d" if obs.color != "#f97316" else "#9a3412",
            linewidth=1.1,
            alpha=0.80,
        )
    )


def draw_plan_pov(ax, plan: np.ndarray, cam: np.ndarray, forward: np.ndarray, right: np.ndarray, up: np.ndarray) -> None:
    pts, depth = project(plan, cam, forward, right, up)
    mask = (depth > 0.1) & (pts[:, 0] > -0.2) & (pts[:, 0] < 1.2) & (pts[:, 1] > -0.2) & (pts[:, 1] < 1.2)
    if np.sum(mask) >= 2:
        ax.plot(pts[mask, 0], pts[mask, 1], color="#38bdf8", linewidth=3.0, alpha=0.95)
        ax.plot(pts[mask, 0], pts[mask, 1], color="#0f172a", linewidth=1.0, alpha=0.55)


def draw_topdown_map(ax, stage, flown: np.ndarray, plan: np.ndarray, point: np.ndarray, spec) -> None:
    ax.clear()
    ax.set_facecolor("#f8fafc")
    ax.set_xlim(spec.origin[0], spec.upper[0])
    ax.set_ylim(spec.origin[1], spec.upper[1])
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Indoor object map + MPPI replan", fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.20, linewidth=0.6)
    for obs in stage.obstacles:
        if obs.kind == "box":
            cx, cy, _ = obs.center
            sx, sy, _ = obs.size
            ax.add_patch(Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, facecolor=obs.color, edgecolor="#334155", alpha=0.38))
        else:
            ax.add_patch(Circle(obs.center_xy, obs.radius, facecolor=obs.color, edgecolor="#7f1d1d", alpha=0.50))
    ax.plot(plan[:, 0], plan[:, 1], color="#94a3b8", linewidth=1.8, label="current plan")
    ax.plot(flown[:, 0], flown[:, 1], color="#2563eb", linewidth=2.6, label="executed")
    ax.scatter([point[0]], [point[1]], color="#f59e0b", edgecolor="#111827", s=65, zorder=8, label="MAV")
    ax.scatter([flown[0, 0]], [flown[0, 1]], color="#16a34a", s=45, zorder=7, label="A")
    ax.scatter([plan[-1, 0]], [plan[-1, 1]], color="#7c3aed", s=45, zorder=7, label="B")
    ax.legend(loc="upper left", fontsize=7, framealpha=0.85)


def write_metrics_copy(path: Path, stages: list[object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    for idx, stage in enumerate(stages):
        rows.append(
            {
                "stage": idx,
                "name": stage.name,
                "event": stage.event,
                "compute_time_ms": round(float(stage.compute_time_ms), 3),
                "min_esdf_distance_m": round(float(stage.min_esdf_distance_m), 4),
                "safety_violation": int(bool(stage.safety_violation)),
                "planned_waypoints": len(stage.plan),
                "occupied_voxels": int(stage.occupancy.sum()),
            }
        )
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    args = parse_args()
    spec, stages, _goal = dyn.build_stages(args)
    runtime = build_runtime(stages, int(args.fps))
    write_metrics_copy(args.metrics, stages)

    args.output.parent.mkdir(parents=True, exist_ok=True)

    fig = plt.figure(figsize=(16, 9), facecolor="#f8fafc")
    gs = fig.add_gridspec(2, 3, width_ratios=[1.45, 0.74, 0.74], height_ratios=[1.0, 0.52])
    ax_pov = fig.add_subplot(gs[:, 0])
    ax_map = fig.add_subplot(gs[0, 1:])
    ax_alt = fig.add_subplot(gs[1, 1])
    ax_status = fig.add_subplot(gs[1, 2])
    fig.suptitle("Muc 3 POV: dynamic indoor ESDF/MPPI obstacle avoidance", fontsize=18, fontweight="bold")

    ax_alt.plot(runtime.times, runtime.points[:, 2], color="#0f766e", linewidth=2.1)
    for start, _ in runtime.stage_ranges[1:]:
        ax_alt.axvline(runtime.times[start], color="#f97316", linestyle="--", linewidth=1.0, alpha=0.75)
    alt_dot, = ax_alt.plot([], [], marker="o", color="#f59e0b", markersize=7)
    ax_alt.set_xlim(0, runtime.total_duration_s)
    ax_alt.set_ylim(max(0.0, float(np.min(runtime.points[:, 2])) - 0.12), float(np.max(runtime.points[:, 2])) + 0.22)
    ax_alt.set_title("Altitude response")
    ax_alt.set_xlabel("time [s]")
    ax_alt.set_ylabel("z [m]")
    ax_alt.grid(True, alpha=0.25)

    ax_status.set_axis_off()
    status_text = ax_status.text(0.02, 0.96, "", transform=ax_status.transAxes, va="top", ha="left", fontsize=10.5, linespacing=1.28)
    bottom = fig.text(0.5, 0.035, "", ha="center", va="center", fontsize=12, color="#334155")
    fig.tight_layout(rect=[0, 0.055, 1, 0.95])

    writer = FFMpegWriter(fps=int(args.fps), metadata={"title": "Level 3 dynamic indoor drone POV ESDF MPPI"}, bitrate=5000)
    with writer.saving(fig, str(args.output), dpi=int(args.dpi)):
        for frame in range(len(runtime.points)):
            idx, stage, start, end = stage_for_frame(runtime, stages, frame)
            point = runtime.points[frame]
            forward, right, up = camera_axes(runtime.points, frame)
            cam = point + np.asarray([0.0, 0.0, -0.02])

            draw_pov_background(ax_pov)
            visible = []
            for obs in stage.obstacles:
                if obs.kind == "box":
                    center = np.asarray(obs.center)
                else:
                    center = np.asarray([obs.center_xy[0], obs.center_xy[1], (obs.z_min + obs.z_max) * 0.5])
                depth = float((center - cam) @ forward)
                visible.append((depth, obs))
            for depth, obs in sorted(visible, reverse=True):
                if depth < -0.6:
                    continue
                if obs.kind == "box":
                    draw_box_pov(ax_pov, obs, cam, forward, right, up)
                else:
                    draw_cylinder_pov(ax_pov, obs, cam, forward, right, up)
            draw_plan_pov(ax_pov, stage.plan, cam, forward, right, up)
            ax_pov.text(0.03, 0.95, "Drone POV camera", transform=ax_pov.transAxes, ha="left", va="top", fontsize=14, fontweight="bold")
            ax_pov.text(
                0.03,
                0.90,
                f"{stage.name}: {stage.event}",
                transform=ax_pov.transAxes,
                ha="left",
                va="top",
                fontsize=11,
                color="#334155",
            )
            ax_pov.plot([0.485, 0.515], [0.5, 0.5], color="#111827", linewidth=1.1, alpha=0.75)
            ax_pov.plot([0.5, 0.5], [0.485, 0.515], color="#111827", linewidth=1.1, alpha=0.75)

            flown = runtime.points[: frame + 1]
            draw_topdown_map(ax_map, stage, flown, stage.plan, point, spec)
            alt_dot.set_data([runtime.times[frame]], [point[2]])
            status_text.set_text(
                "\n".join(
                    [
                        "Level 3 loop",
                        "LiDAR/depth-like perception",
                        "-> indoor object map",
                        "-> 3D voxel occupancy",
                        "-> ESDF clearance",
                        "-> MPPI x,y,z replan",
                        "",
                        f"Min ESDF: {float(stage.min_esdf_distance_m):.3f} m",
                        f"Safety violation: {int(bool(stage.safety_violation))}",
                        f"Compute: {float(stage.compute_time_ms):.1f} ms",
                        f"Altitude: {float(point[2]):.2f} m",
                    ]
                )
            )
            bottom.set_text(f"t={runtime.times[frame]:.1f}s | POV + indoor object map | orange objects are surprise events")
            writer.grab_frame()

    plt.close(fig)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
