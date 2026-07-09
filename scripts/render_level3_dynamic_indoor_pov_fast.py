#!/usr/bin/env python3
"""Fast PIL renderer for a Level-3 dynamic indoor drone POV video."""

from __future__ import annotations

import argparse
import importlib.util
import math
import subprocess
import sys
import textwrap
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO_ROOT = Path(__file__).resolve().parents[1]


def load_dynamic_module():
    path = REPO_ROOT / "scripts" / "render_level3_dynamic_indoor_events_video.py"
    spec = importlib.util.spec_from_file_location("level3_dynamic_events_fast", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


dyn = load_dynamic_module()


@dataclass(frozen=True)
class Runtime:
    points: np.ndarray
    ranges: list[tuple[int, int]]
    times: np.ndarray
    duration_s: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/level3_dynamic_indoor_pov_esdf_mppi.mp4"))
    parser.add_argument("--preview", type=Path, default=Path("outputs/figures/level3_video_preview/level3_dynamic_pov_midframe.png"))
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=18)
    parser.add_argument("--resolution", type=float, default=0.18)
    parser.add_argument("--seed", type=int, default=53)
    return parser.parse_args()


def font(size: int, bold: bool = False):
    candidates = [
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold else "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Supplemental/Helvetica.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


FONT_TITLE = font(28, True)
FONT_H2 = font(20, True)
FONT_BODY = font(17)
FONT_SMALL = font(14)
FONT_TINY = font(12)


def resample(points: np.ndarray, n: int) -> np.ndarray:
    return dyn._resample(points, n)


def build_runtime(stages: list[object], fps: int) -> Runtime:
    duration = float(sum(float(stage.duration_s) for stage in stages))
    total = int(round(duration * fps))
    ranges: list[tuple[int, int]] = []
    segments: list[np.ndarray] = []
    cursor = 0
    for idx, stage in enumerate(stages):
        count = int(round(float(stage.duration_s) * fps))
        if idx == len(stages) - 1:
            count = total - cursor
        ranges.append((cursor, cursor + count))
        if idx == len(stages) - 1:
            segment = resample(stage.plan, count)
        else:
            sampled = resample(stage.plan, 180)
            end = max(2, int(float(stage.path_fraction) * (len(sampled) - 1)))
            segment = resample(sampled[: end + 1], count)
        segments.append(segment)
        cursor += count
    points = np.vstack(segments)
    return Runtime(points=points, ranges=ranges, times=np.linspace(0.0, duration, len(points)), duration_s=duration)


def stage_for_frame(runtime: Runtime, stages: list[object], frame: int):
    for idx, (start, end) in enumerate(runtime.ranges):
        if start <= frame < end:
            return idx, stages[idx], start, end
    start, end = runtime.ranges[-1]
    return len(stages) - 1, stages[-1], start, end


def camera_axes(points: np.ndarray, frame: int):
    here = points[frame]
    ahead = points[min(len(points) - 1, frame + 10)] - here
    ahead[2] = 0.0
    norm = float(np.linalg.norm(ahead))
    forward = np.asarray([1.0, 0.0, 0.0]) if norm <= 1e-6 else ahead / norm
    right = np.asarray([forward[1], -forward[0], 0.0])
    up = np.asarray([0.0, 0.0, 1.0])
    return forward, right, up


def project(points: np.ndarray, cam: np.ndarray, forward: np.ndarray, right: np.ndarray, up: np.ndarray, rect):
    x0, y0, w, h = rect
    rel = np.asarray(points, dtype=float) - cam[None, :]
    px = rel @ right
    py = rel @ up
    pz = rel @ forward
    z = np.maximum(pz, 0.08)
    focal = 0.92
    sx = x0 + w * (0.5 + focal * px / z)
    sy = y0 + h * (0.56 - focal * (py - 0.10 * z) / z)
    return np.stack([sx, sy], axis=1), pz


def box_corners(center, size):
    cx, cy, cz = center
    sx, sy, sz = size
    return np.asarray([[cx + dx * sx / 2, cy + dy * sy / 2, cz + dz * sz / 2] for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)])


BOX_FACES = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]


def hex_rgb(color: str):
    color = color.lstrip("#")
    return tuple(int(color[i : i + 2], 16) for i in (0, 2, 4))


def blend(color, amount: float):
    r, g, b = hex_rgb(color)
    return tuple(int(v + (255 - v) * amount) for v in (r, g, b))


def draw_pov_background(draw: ImageDraw.ImageDraw, rect):
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(231, 238, 245), outline=(203, 213, 225), width=2)
    draw.polygon([(x, y + h), (x + w, y + h), (x + int(0.72 * w), y + int(0.53 * h)), (x + int(0.28 * w), y + int(0.53 * h))], fill=(199, 213, 197))
    draw.polygon([(x, y), (x + w, y), (x + int(0.72 * w), y + int(0.45 * h)), (x + int(0.28 * w), y + int(0.45 * h))], fill=(248, 250, 252))
    draw.line([(x, y + int(0.53 * h)), (x + w, y + int(0.53 * h))], fill=(148, 163, 184), width=1)
    for i in range(9):
        xx = x - int(0.2 * w) + i * int(0.18 * w)
        draw.line([(xx, y + h), (x + w // 2, y + int(0.53 * h))], fill=(164, 176, 190), width=1)
    for j in range(6):
        yy = y + int((0.11 + 0.075 * j) * h)
        inset = int((yy - y) * 0.45)
        draw.line([(x + inset, yy), (x + w - inset, yy)], fill=(170, 181, 194), width=1)


def draw_box_pov(draw, obs, cam, forward, right, up, rect):
    corners = box_corners(obs.center, obs.size)
    pts, depth = project(corners, cam, forward, right, up, rect)
    faces = []
    for face in BOX_FACES:
        d = depth[list(face)]
        if float(np.mean(d)) <= 0.12:
            continue
        poly = pts[list(face)]
        x0, y0, w, h = rect
        if np.all((poly[:, 0] < x0 - 50) | (poly[:, 0] > x0 + w + 50)) or np.all((poly[:, 1] < y0 - 50) | (poly[:, 1] > y0 + h + 50)):
            continue
        faces.append((float(np.mean(d)), poly))
    for depth_mean, poly in sorted(faces, key=lambda item: item[0], reverse=True):
        color = blend(obs.color, min(0.55, depth_mean / 8.0))
        draw.polygon([tuple(p) for p in poly], fill=color, outline=(51, 65, 85))


def draw_cylinder_pov(draw, obs, cam, forward, right, up, rect):
    center = np.asarray([obs.center_xy[0], obs.center_xy[1], (obs.z_min + obs.z_max) * 0.5], dtype=float)
    half_h = (obs.z_max - obs.z_min) * 0.5
    pts3 = np.asarray(
        [
            center - right * obs.radius + np.asarray([0, 0, -half_h]),
            center + right * obs.radius + np.asarray([0, 0, -half_h]),
            center + right * obs.radius + np.asarray([0, 0, half_h]),
            center - right * obs.radius + np.asarray([0, 0, half_h]),
        ],
        dtype=float,
    )
    pts, depth = project(pts3, cam, forward, right, up, rect)
    if float(np.mean(depth)) <= 0.12:
        return
    draw.polygon([tuple(p) for p in pts], fill=hex_rgb(obs.color), outline=(127, 29, 29))


def draw_plan_pov(draw, plan, cam, forward, right, up, rect):
    pts, depth = project(plan, cam, forward, right, up, rect)
    x0, y0, w, h = rect
    keep = (depth > 0.1) & (pts[:, 0] >= x0) & (pts[:, 0] <= x0 + w) & (pts[:, 1] >= y0) & (pts[:, 1] <= y0 + h)
    poly = [tuple(p) for p in pts[keep]]
    if len(poly) >= 2:
        draw.line(poly, fill=(15, 23, 42), width=5)
        draw.line(poly, fill=(56, 189, 248), width=3)


def world_to_map(point, spec, rect):
    x0, y0, w, h = rect
    px = x0 + (point[0] - spec.origin[0]) / (spec.upper[0] - spec.origin[0]) * w
    py = y0 + h - (point[1] - spec.origin[1]) / (spec.upper[1] - spec.origin[1]) * h
    return int(px), int(py)


def draw_minimap(draw, stage, flown, plan, point, spec, rect):
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(248, 250, 252), outline=(203, 213, 225), width=2)
    draw.text((x + 12, y + 10), "Indoor object map + MPPI replan", fill=(15, 23, 42), font=FONT_H2)
    for obs in stage.obstacles:
        color = hex_rgb(obs.color)
        if obs.kind == "box":
            cx, cy, _ = obs.center
            sx, sy, _ = obs.size
            p1 = world_to_map((cx - sx / 2, cy - sy / 2, 0), spec, rect)
            p2 = world_to_map((cx + sx / 2, cy + sy / 2, 0), spec, rect)
            draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=color + (90,), outline=(51, 65, 85))
        else:
            cx, cy = obs.center_xy
            rpx = int(obs.radius / (spec.upper[0] - spec.origin[0]) * w)
            pc = world_to_map((cx, cy, 0), spec, rect)
            draw.ellipse([pc[0] - rpx, pc[1] - rpx, pc[0] + rpx, pc[1] + rpx], fill=color + (120,), outline=(127, 29, 29))
    for arr, color, width_line in [(plan, (148, 163, 184), 3), (flown, (37, 99, 235), 5)]:
        pts = [world_to_map(p, spec, rect) for p in arr]
        if len(pts) >= 2:
            draw.line(pts, fill=color, width=width_line)
    pc = world_to_map(point, spec, rect)
    draw.ellipse([pc[0] - 7, pc[1] - 7, pc[0] + 7, pc[1] + 7], fill=(245, 158, 11), outline=(15, 23, 42), width=2)


def draw_altitude(draw, runtime, frame, rect):
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(255, 255, 255), outline=(203, 213, 225), width=2)
    draw.text((x + 12, y + 10), "Altitude response", fill=(15, 23, 42), font=FONT_H2)
    z = runtime.points[:, 2]
    zmin, zmax = float(np.min(z)) - 0.1, float(np.max(z)) + 0.15
    pts = []
    for i, zz in enumerate(z):
        px = x + 24 + i / max(1, len(z) - 1) * (w - 48)
        py = y + h - 24 - (zz - zmin) / (zmax - zmin) * (h - 62)
        pts.append((int(px), int(py)))
    draw.line(pts, fill=(15, 118, 110), width=3)
    px, py = pts[frame]
    draw.ellipse([px - 6, py - 6, px + 6, py + 6], fill=(245, 158, 11))


def draw_status(draw, stage, point, rect):
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(255, 255, 255), outline=(203, 213, 225), width=2)
    event_lines = textwrap.wrap(str(stage.event), width=max(22, int(w / 10)))
    lines = [
        str(stage.name),
        *event_lines,
        "",
        "Level 3 loop:",
        "LiDAR/depth-like perception",
        "-> indoor object map",
        "-> 3D voxel occupancy",
        "-> ESDF clearance",
        "-> MPPI x,y,z replan",
        "",
        f"Min ESDF: {stage.min_esdf_distance_m:.3f} m",
        f"Safety violation: {int(bool(stage.safety_violation))}",
        f"Compute: {stage.compute_time_ms:.1f} ms",
        f"Altitude: {point[2]:.2f} m",
    ]
    yy = y + 12
    for idx, line in enumerate(lines):
        fnt = FONT_H2 if idx == 0 else (FONT_SMALL if idx <= len(event_lines) else FONT_BODY)
        draw.text((x + 14, yy), line, fill=(15, 23, 42), font=fnt)
        yy += 22 if line else 8


def main() -> int:
    args = parse_args()
    spec, stages, _goal = dyn.build_stages(args)
    runtime = build_runtime(stages, args.fps)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.preview.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ffmpeg",
        "-y",
        "-f",
        "rawvideo",
        "-vcodec",
        "rawvideo",
        "-s",
        f"{args.width}x{args.height}",
        "-pix_fmt",
        "rgb24",
        "-framerate",
        str(args.fps),
        "-i",
        "pipe:",
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-b:v",
        "4200k",
        str(args.output),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    preview_frame = len(runtime.points) // 2
    try:
        for frame in range(len(runtime.points)):
            idx, stage, _start, _end = stage_for_frame(runtime, stages, frame)
            point = runtime.points[frame]
            forward, right, up = camera_axes(runtime.points, frame)
            cam = point + np.asarray([0.0, 0.0, -0.02])

            img = Image.new("RGB", (args.width, args.height), (248, 250, 252))
            draw = ImageDraw.Draw(img, "RGBA")
            draw.text((28, 20), "Muc 3 POV: dynamic indoor ESDF/MPPI obstacle avoidance", fill=(15, 23, 42), font=FONT_TITLE)
            pov_rect = (24, 72, 760, 560)
            map_rect = (820, 72, 420, 305)
            status_rect = (820, 398, 420, 210)

            draw_pov_background(draw, pov_rect)
            visible = []
            for obs in stage.obstacles:
                center = np.asarray(obs.center if obs.kind == "box" else [obs.center_xy[0], obs.center_xy[1], (obs.z_min + obs.z_max) * 0.5])
                visible.append((float((center - cam) @ forward), obs))
            for depth, obs in sorted(visible, key=lambda item: item[0], reverse=True):
                if depth < -0.6:
                    continue
                if obs.kind == "box":
                    draw_box_pov(draw, obs, cam, forward, right, up, pov_rect)
                else:
                    draw_cylinder_pov(draw, obs, cam, forward, right, up, pov_rect)
            draw_plan_pov(draw, stage.plan, cam, forward, right, up, pov_rect)
            draw.text((pov_rect[0] + 18, pov_rect[1] + 16), "Drone POV camera", fill=(15, 23, 42), font=FONT_H2)
            draw.text((pov_rect[0] + 18, pov_rect[1] + 42), stage.event, fill=(51, 65, 85), font=FONT_BODY)
            cx, cy = pov_rect[0] + pov_rect[2] // 2, pov_rect[1] + pov_rect[3] // 2
            draw.line([(cx - 12, cy), (cx + 12, cy)], fill=(15, 23, 42), width=2)
            draw.line([(cx, cy - 12), (cx, cy + 12)], fill=(15, 23, 42), width=2)

            flown = runtime.points[: frame + 1]
            draw_minimap(draw, stage, flown, stage.plan, point, spec, map_rect)
            draw_status(draw, stage, point, status_rect)
            draw.text((args.width // 2 - 290, args.height - 48), f"t={runtime.times[frame]:.1f}s | POV + indoor object map | orange = surprise event", fill=(51, 65, 85), font=FONT_BODY)

            if frame == preview_frame:
                img.save(args.preview)
            assert proc.stdin is not None
            proc.stdin.write(img.tobytes())
    finally:
        if proc.stdin is not None:
            proc.stdin.close()
        proc.wait()
    if proc.returncode != 0:
        raise SystemExit(proc.returncode)
    print(args.output)
    print(args.preview)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
