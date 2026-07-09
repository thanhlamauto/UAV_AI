#!/usr/bin/env python3
"""Render an Isaac Sim indoor RGB-D/LiDAR/point-cloud UAV demo.

The script is intentionally self-contained so it can run on a rented GPU
server with the pip Isaac Sim package.  It builds a procedural indoor corridor,
reuses the local 3D ESDF/MPPI utilities for a safe route, renders an onboard RGB
and depth stream from Isaac Sim, and composes readable sensor/planner panels.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import os
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/isaacsim_indoor_rgbd_lidar_dynamic_demo.mp4"))
    parser.add_argument("--preview", type=Path, default=Path("outputs/figures/isaacsim_demo/isaacsim_indoor_midframe.png"))
    parser.add_argument("--metrics", type=Path, default=Path("outputs/tables/isaacsim_indoor_sensor_demo_metrics.csv"))
    parser.add_argument("--summary", type=Path, default=Path("outputs/isaacsim_indoor_sensor_demo_summary.md"))
    parser.add_argument("--frames", type=int, default=144)
    parser.add_argument("--fps", type=int, default=18)
    parser.add_argument("--width", type=int, default=960, help="Isaac RGB-D render width")
    parser.add_argument("--height", type=int, default=540, help="Isaac RGB-D render height")
    parser.add_argument("--resolution", type=float, default=0.18)
    parser.add_argument("--seed", type=int, default=53)
    parser.add_argument("--renderer", default="RayTracedLighting")
    parser.add_argument("--camera-mode", choices=["onboard", "chase"], default="onboard")
    parser.add_argument("--quick", action="store_true", help="Render a very short smoke-test clip")
    return parser.parse_args()


def _load_dynamic_module():
    path = REPO_ROOT / "scripts" / "render_level3_dynamic_indoor_events_video.py"
    spec = importlib.util.spec_from_file_location("level3_dynamic_events", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@dataclass(frozen=True)
class RuntimePath:
    points: np.ndarray
    stage_ranges: list[tuple[int, int]]
    total_duration_s: float


@dataclass(frozen=True)
class RuntimeObstacle:
    name: str
    kind: str
    center: tuple[float, float, float] | None
    size: tuple[float, float, float] | None
    center_xy: tuple[float, float] | None
    radius: float | None
    z_min: float | None
    z_max: float | None
    color: str


def build_stages(args: argparse.Namespace) -> tuple[Any, list[Any]]:
    dyn = _load_dynamic_module()
    spec, stages, _ = dyn.build_stages(
        argparse.Namespace(resolution=float(args.resolution), seed=int(args.seed), fps=int(args.fps))
    )
    return dyn, stages


def build_runtime(dyn: Any, stages: list[Any], frames: int) -> RuntimePath:
    stage_weights = np.asarray([float(stage.duration_s) for stage in stages], dtype=float)
    stage_counts = np.maximum(8, np.round(frames * stage_weights / stage_weights.sum()).astype(int))
    stage_counts[-1] += int(frames - int(stage_counts.sum()))

    ranges: list[tuple[int, int]] = []
    segments: list[np.ndarray] = []
    cursor = 0
    for idx, (stage, count) in enumerate(zip(stages, stage_counts, strict=True)):
        count = int(max(2, count))
        if idx == len(stages) - 1:
            segment = dyn._resample(stage.plan, count)
        else:
            sampled = dyn._resample(stage.plan, 180)
            end = max(2, int(float(stage.path_fraction) * (len(sampled) - 1)))
            segment = dyn._resample(sampled[: end + 1], count)
        segments.append(segment)
        ranges.append((cursor, cursor + count))
        cursor += count
    points = np.vstack(segments)
    if len(points) != frames:
        points = dyn._resample(points, frames)
        ranges = []
        cursor = 0
        for idx, weight in enumerate(stage_weights / stage_weights.sum()):
            end = frames if idx == len(stages) - 1 else cursor + int(round(frames * weight))
            ranges.append((cursor, max(cursor + 1, end)))
            cursor = ranges[-1][1]
        ranges[-1] = (ranges[-1][0], frames)
    return RuntimePath(points=points, stage_ranges=ranges, total_duration_s=frames / float(max(1, int(frames))))


def stage_for_frame(runtime: RuntimePath, stages: list[Any], frame_idx: int) -> tuple[int, Any, float]:
    for idx, (start, end) in enumerate(runtime.stage_ranges):
        if start <= frame_idx < end:
            local = (frame_idx - start) / max(1, end - start - 1)
            return idx, stages[idx], float(local)
    start, end = runtime.stage_ranges[-1]
    return len(stages) - 1, stages[-1], float((frame_idx - start) / max(1, end - start - 1))


def obstacle_records(stage: Any, stage_idx: int, local_t: float) -> list[RuntimeObstacle]:
    records: list[RuntimeObstacle] = []
    for obs in stage.obstacles:
        center = tuple(obs.center) if obs.center is not None else None
        center_xy = tuple(obs.center_xy) if obs.center_xy is not None else None
        size = tuple(obs.size) if obs.size is not None else None
        color = str(obs.color)

        if obs.name == "surprise person crossing" and center_xy is not None:
            # Move laterally through the corridor during the event.
            y = -1.80 + 1.20 * float(np.clip(local_t, 0.0, 1.0))
            center_xy = (float(center_xy[0]), y)
        elif obs.name == "door panel closing" and center is not None:
            # Slide the panel into the corridor.
            x, y, z = center
            center = (float(x), float(-1.55 + 0.95 * np.clip(local_t, 0.0, 1.0)), float(z))
        elif obs.name == "cart appears" and center is not None:
            x, y, z = center
            center = (float(x), float(1.45 - 1.10 * np.clip(local_t, 0.0, 1.0)), float(z))

        records.append(
            RuntimeObstacle(
                name=str(obs.name),
                kind=str(obs.kind),
                center=center,
                size=size,
                center_xy=center_xy,
                radius=float(obs.radius) if obs.radius is not None else None,
                z_min=float(obs.z_min) if obs.z_min is not None else None,
                z_max=float(obs.z_max) if obs.z_max is not None else None,
                color=color,
            )
        )
    return records


def clearance_to_obstacles(point: np.ndarray, obstacles: list[RuntimeObstacle]) -> float:
    best = float("inf")
    for obs in obstacles:
        if obs.kind == "box" and obs.center is not None and obs.size is not None:
            center = np.asarray(obs.center, dtype=float)
            half = np.asarray(obs.size, dtype=float) / 2.0
            delta = np.maximum(np.abs(point - center) - half, 0.0)
            best = min(best, float(np.linalg.norm(delta)))
        elif obs.kind == "cylinder" and obs.center_xy is not None and obs.radius is not None:
            xy = np.asarray(obs.center_xy, dtype=float)
            radial = max(0.0, float(np.linalg.norm(point[:2] - xy)) - float(obs.radius))
            low = max(0.0, float(obs.z_min or 0.0) - float(point[2]))
            high = max(0.0, float(point[2]) - float(obs.z_max or 0.0))
            best = min(best, float(math.sqrt(radial * radial + low * low + high * high)))
    return best if np.isfinite(best) else float("nan")


def risk_label(clearance_m: float) -> str:
    if clearance_m < 0.03:
        return "COLLISION"
    if clearance_m < 0.45:
        return "DANGER"
    if clearance_m < 0.75:
        return "WARNING"
    return "SAFE"


def sample_box_surface(center: tuple[float, float, float], size: tuple[float, float, float], density: int = 8) -> np.ndarray:
    c = np.asarray(center, dtype=float)
    s = np.asarray(size, dtype=float)
    grids: list[np.ndarray] = []
    lin = [np.linspace(-s[i] / 2.0, s[i] / 2.0, density) for i in range(3)]
    for axis in range(3):
        for sign in (-1.0, 1.0):
            coords = []
            for dim in range(3):
                if dim == axis:
                    coords.append(np.asarray([sign * s[dim] / 2.0]))
                else:
                    coords.append(lin[dim])
            mesh = np.meshgrid(*coords, indexing="ij")
            grids.append(np.stack([m.ravel() for m in mesh], axis=1) + c[None, :])
    return np.vstack(grids)


def sample_cylinder_surface(
    center_xy: tuple[float, float],
    radius: float,
    z_min: float,
    z_max: float,
    density: int = 18,
) -> np.ndarray:
    angles = np.linspace(0.0, 2.0 * math.pi, density, endpoint=False)
    zs = np.linspace(z_min, z_max, max(4, density // 2))
    pts = []
    for z in zs:
        pts.append(
            np.stack(
                [
                    center_xy[0] + radius * np.cos(angles),
                    center_xy[1] + radius * np.sin(angles),
                    np.full_like(angles, z),
                ],
                axis=1,
            )
        )
    return np.vstack(pts)


def lidar_points(drone: np.ndarray, obstacles: list[RuntimeObstacle], seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    clouds: list[np.ndarray] = []
    for obs in obstacles:
        if obs.kind == "box" and obs.center is not None and obs.size is not None:
            pts = sample_box_surface(obs.center, obs.size, density=7)
        elif obs.kind == "cylinder" and obs.center_xy is not None and obs.radius is not None:
            pts = sample_cylinder_surface(obs.center_xy, obs.radius, obs.z_min or 0.0, obs.z_max or 1.5, density=24)
        else:
            continue
        rel = pts - drone[None, :]
        ranges = np.linalg.norm(rel, axis=1)
        mask = (ranges > 0.15) & (ranges < 5.8)
        pts = pts[mask]
        if len(pts) > 0:
            clouds.append(pts)
    if not clouds:
        return np.zeros((0, 3), dtype=float)
    pts = np.vstack(clouds)
    pts += rng.normal(0.0, 0.015, size=pts.shape)
    if len(pts) > 1400:
        pts = pts[rng.choice(len(pts), size=1400, replace=False)]
    return pts


def depth_to_rgb(depth: np.ndarray) -> np.ndarray:
    arr = np.asarray(depth, dtype=np.float32)
    arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
    valid = arr > 0.01
    if np.any(valid):
        lo = float(np.percentile(arr[valid], 2.0))
        hi = float(np.percentile(arr[valid], 96.0))
    else:
        lo, hi = 0.0, 1.0
    norm = np.clip((arr - lo) / max(1e-6, hi - lo), 0.0, 1.0)
    # Plasma-like colormap implemented without matplotlib.
    stops = np.asarray(
        [
            [13, 8, 135],
            [84, 3, 160],
            [139, 10, 165],
            [185, 50, 137],
            [219, 92, 104],
            [244, 136, 73],
            [254, 188, 43],
            [240, 249, 33],
        ],
        dtype=np.float32,
    )
    idx = np.clip(norm * (len(stops) - 1), 0.0, len(stops) - 1 - 1e-6)
    i0 = np.floor(idx).astype(np.int32)
    frac = idx - i0
    rgb = (1.0 - frac[..., None]) * stops[i0] + frac[..., None] * stops[i0 + 1]
    rgb[~valid] = 0
    return np.clip(rgb, 0, 255).astype(np.uint8)


def get_font(size: int):
    from PIL import ImageFont

    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
    ]
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


def draw_label(draw: Any, xy: tuple[int, int], text: str, font: Any, fill: str = "white") -> None:
    x, y = xy
    bbox = draw.textbbox((x, y), text, font=font)
    draw.rectangle((bbox[0] - 7, bbox[1] - 4, bbox[2] + 7, bbox[3] + 4), fill=(8, 15, 28, 210))
    draw.text((x, y), text, font=font, fill=fill)


def draw_lidar_panel(size: tuple[int, int], drone: np.ndarray, pts: np.ndarray, obstacles: list[RuntimeObstacle], path: np.ndarray) -> Any:
    from PIL import Image, ImageDraw

    w, h = size
    img = Image.new("RGB", size, (246, 248, 252))
    draw = ImageDraw.Draw(img, "RGBA")
    font = get_font(20)
    small = get_font(15)
    pad = 28
    bounds = (-0.8, 7.2, -3.2, 3.2)

    def xy(p: np.ndarray | tuple[float, float]) -> tuple[int, int]:
        x, y = float(p[0]), float(p[1])
        sx = pad + (x - bounds[0]) / (bounds[1] - bounds[0]) * (w - 2 * pad)
        sy = h - pad - (y - bounds[2]) / (bounds[3] - bounds[2]) * (h - 2 * pad)
        return int(sx), int(sy)

    draw.rectangle((pad, pad, w - pad, h - pad), outline=(148, 163, 184, 255), width=2)
    for gx in np.linspace(bounds[0], bounds[1], 9):
        x, _ = xy((gx, bounds[2]))
        draw.line((x, pad, x, h - pad), fill=(203, 213, 225, 125), width=1)
    for gy in np.linspace(bounds[2], bounds[3], 7):
        _, y = xy((bounds[0], gy))
        draw.line((pad, y, w - pad, y), fill=(203, 213, 225, 125), width=1)

    for obs in obstacles:
        if obs.kind == "box" and obs.center is not None and obs.size is not None:
            cx, cy, _ = obs.center
            sx, sy, _ = obs.size
            a = xy((cx - sx / 2.0, cy - sy / 2.0))
            b = xy((cx + sx / 2.0, cy + sy / 2.0))
            box = (min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1]))
            color = (239, 68, 68, 80) if "surprise" not in obs.name and "cart" not in obs.name else (249, 115, 22, 115)
            draw.rectangle(box, fill=color, outline=(100, 116, 139, 185), width=1)
        elif obs.kind == "cylinder" and obs.center_xy is not None and obs.radius is not None:
            cx, cy = obs.center_xy
            c = xy((cx, cy))
            edge = xy((cx + obs.radius, cy))
            rr = abs(edge[0] - c[0])
            color = (239, 68, 68, 90) if "surprise" not in obs.name else (249, 115, 22, 130)
            draw.ellipse((c[0] - rr, c[1] - rr, c[0] + rr, c[1] + rr), fill=color, outline=(127, 29, 29, 180), width=2)

    if len(path) >= 2:
        coords = [xy(p[:2]) for p in path]
        draw.line(coords, fill=(14, 165, 233, 230), width=4)
    if len(pts):
        rel = pts - drone[None, :]
        ranges = np.linalg.norm(rel[:, :2], axis=1)
        order = np.argsort(ranges)[:: max(1, len(ranges) // 900)]
        for p, r in zip(pts[order], ranges[order], strict=False):
            c = xy(p[:2])
            alpha = int(np.clip(235 - 22 * r, 70, 230))
            draw.ellipse((c[0] - 2, c[1] - 2, c[0] + 2, c[1] + 2), fill=(37, 99, 235, alpha))

    d = xy(drone[:2])
    draw.ellipse((d[0] - 8, d[1] - 8, d[0] + 8, d[1] + 8), fill=(15, 23, 42, 255))
    draw.text((pad, 6), "LiDAR / point cloud top-down", font=font, fill=(15, 23, 42))
    draw.text((w - 190, 8), f"{len(pts):4d} pts", font=small, fill=(51, 65, 85))
    return img


def draw_timeline_panel(
    size: tuple[int, int],
    frame_idx: int,
    frames: int,
    clearance: float,
    risk: str,
    stage_idx: int,
    stage: Any,
    metrics_row: dict[str, Any],
) -> Any:
    from PIL import Image, ImageDraw

    w, h = size
    img = Image.new("RGB", size, (248, 250, 252))
    draw = ImageDraw.Draw(img, "RGBA")
    title = get_font(22)
    font = get_font(17)
    mono = get_font(15)
    draw.text((24, 18), "Fused perception -> ESDF -> MPPI-safe route", font=title, fill=(15, 23, 42))
    event = str(stage.event)
    draw.text((24, 53), f"Stage {stage_idx}: {event}", font=font, fill=(71, 85, 105))
    color = {"SAFE": (22, 163, 74), "WARNING": (234, 179, 8), "DANGER": (249, 115, 22), "COLLISION": (220, 38, 38)}[risk]
    draw.rounded_rectangle((24, 88, 176, 132), radius=8, fill=color + (235,))
    draw.text((45, 98), risk, font=title, fill="white")
    draw.text((205, 91), f"clearance {clearance:.2f} m", font=font, fill=(15, 23, 42))
    draw.text(
        (205, 119),
        f"stage MPPI compute {float(stage.compute_time_ms):.1f} ms | min ESDF {float(stage.min_esdf_distance_m):.2f} m",
        font=mono,
        fill=(71, 85, 105),
    )

    x0, y0, x1, y1 = 24, h - 48, w - 24, h - 24
    draw.rectangle((x0, y0, x1, y1), fill=(226, 232, 240, 255))
    progress = frame_idx / max(1, frames - 1)
    draw.rectangle((x0, y0, int(x0 + progress * (x1 - x0)), y1), fill=(14, 165, 233, 255))
    for k in range(1, 4):
        xx = int(x0 + k / 4.0 * (x1 - x0))
        draw.line((xx, y0 - 9, xx, y1 + 9), fill=(100, 116, 139, 220), width=2)

    lines = [
        f"RGB-D source: Isaac Sim camera annotators",
        f"LiDAR source: geometry raycast cloud over active indoor objects",
        f"Obstacle model: 3D boxes/cylinders -> ESDF safety field",
        f"Planner output: continuous 3D route, not a pre-known grid path",
    ]
    for i, line in enumerate(lines):
        draw.text((w - 630, 28 + i * 29), line, font=mono, fill=(51, 65, 85))

    draw.text((24, 144), json.dumps(metrics_row, ensure_ascii=True)[:132], font=mono, fill=(71, 85, 105))
    return img


def compose_frame(
    rgb: np.ndarray,
    depth_rgb: np.ndarray,
    drone: np.ndarray,
    lidar: np.ndarray,
    obstacles: list[RuntimeObstacle],
    planned_path: np.ndarray,
    frame_idx: int,
    frames: int,
    clearance: float,
    risk: str,
    stage_idx: int,
    stage: Any,
    metrics_row: dict[str, Any],
    camera_mode: str,
) -> Any:
    from PIL import Image, ImageDraw

    canvas = Image.new("RGB", (1920, 1080), (241, 245, 249))
    draw = ImageDraw.Draw(canvas, "RGBA")
    title_font = get_font(24)
    small_font = get_font(16)

    rgb_img = Image.fromarray(rgb.astype(np.uint8)).resize((1180, 664), Image.Resampling.LANCZOS)
    depth_img = Image.fromarray(depth_rgb.astype(np.uint8)).resize((640, 300), Image.Resampling.LANCZOS)
    lidar_img = draw_lidar_panel((640, 330), drone, lidar, obstacles, planned_path)
    timeline = draw_timeline_panel((1870, 330), frame_idx, frames, clearance, risk, stage_idx, stage, metrics_row)

    canvas.paste(rgb_img, (24, 50))
    canvas.paste(depth_img, (1255, 50))
    canvas.paste(lidar_img, (1255, 384))
    canvas.paste(timeline, (24, 730))
    camera_title = "third-person chase RGB-D" if camera_mode == "chase" else "onboard RGB-D"
    camera_label = "THIRD-PERSON / CHASE - Isaac Sim headless" if camera_mode == "chase" else "ONBOARD RGB - Isaac Sim headless"
    draw.text((24, 16), f"Isaac Sim indoor UAV demo: {camera_title} + LiDAR/point cloud + dynamic obstacles", font=title_font, fill=(15, 23, 42))
    draw.text((1255, 18), "Depth input", font=title_font, fill=(15, 23, 42))
    draw.text((1268, 62), "relative distance-to-camera", font=small_font, fill=(248, 250, 252))
    draw_label(draw, (44, 70), camera_label, get_font(18))
    draw_label(draw, (44, 104), f"frame {frame_idx + 1:03d}/{frames:03d}", get_font(16))
    return canvas


def normalize_rgb(data: Any, width: int, height: int) -> np.ndarray:
    if isinstance(data, dict):
        data = data.get("data", data)
    arr = np.asarray(data)
    if arr.size == 0:
        raise RuntimeError("empty RGB annotator frame")
    arr = arr.reshape((height, width, -1))
    if arr.shape[-1] >= 3:
        arr = arr[..., :3]
    arr = np.clip(arr.astype(np.float32) / 255.0, 0.0, 1.0)
    arr = np.clip((arr ** 0.74) * 1.16, 0.0, 1.0)
    return np.clip(arr * 255.0, 0, 255).astype(np.uint8)


def normalize_depth(data: Any, width: int, height: int) -> np.ndarray:
    if isinstance(data, dict):
        data = data.get("data", data)
    arr = np.asarray(data, dtype=np.float32)
    arr = arr.reshape((height, width))
    return arr


def make_dirs(*paths: Path) -> None:
    for path in paths:
        if path.suffix:
            path.parent.mkdir(parents=True, exist_ok=True)
        else:
            path.mkdir(parents=True, exist_ok=True)


def main() -> int:
    args = parse_args()
    if args.quick:
        args.frames = min(args.frames, 24)
        args.fps = min(args.fps, 12)

    make_dirs(args.output, args.preview, args.metrics, args.summary)
    dyn, stages = build_stages(args)
    runtime = build_runtime(dyn, stages, int(args.frames))

    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    os.environ.setdefault("ACCEPT_EULA", "Y")

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": True,
            "renderer": str(args.renderer),
            "width": int(args.width),
            "height": int(args.height),
        }
    )

    completed = False
    try:
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

        ctx = omni.usd.get_context()
        ctx.new_stage()
        simulation_app.update()
        stage_usd = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage_usd, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage_usd, 1.0)

        materials: dict[str, Any] = {}

        def material(color_hex: str):
            if color_hex in materials:
                return materials[color_hex]
            color_hex_clean = color_hex.lstrip("#")
            rgb = tuple(int(color_hex_clean[i : i + 2], 16) / 255.0 for i in (0, 2, 4))
            path = f"/World/Looks/Mat_{color_hex_clean}"
            mat = UsdShade.Material.Define(stage_usd, path)
            shader = UsdShade.Shader.Define(stage_usd, f"{path}/Shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.68)
            shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            materials[color_hex] = mat
            return mat

        def bind(prim: Any, color: str) -> None:
            UsdShade.MaterialBindingAPI(prim).Bind(material(color))

        def set_cube(path: str, center: tuple[float, float, float], size: tuple[float, float, float], color: str):
            cube = UsdGeom.Cube.Define(stage_usd, path)
            cube.CreateSizeAttr(1.0)
            prim = cube.GetPrim()
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*center))
            xform.AddScaleOp().Set(Gf.Vec3f(*size))
            bind(prim, color)
            return prim

        def set_cylinder(
            path: str,
            center_xy: tuple[float, float],
            radius: float,
            z_min: float,
            z_max: float,
            color: str,
        ):
            cyl = UsdGeom.Cylinder.Define(stage_usd, path)
            cyl.CreateRadiusAttr(float(radius))
            cyl.CreateHeightAttr(float(z_max - z_min))
            prim = cyl.GetPrim()
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(float(center_xy[0]), float(center_xy[1]), float((z_min + z_max) / 2.0)))
            bind(prim, color)
            return prim

        def update_cube(prim: Any, center: tuple[float, float, float], size: tuple[float, float, float]) -> None:
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*center))
            xform.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdGeom.Imageable(prim).MakeVisible()

        def update_cylinder(prim: Any, center_xy: tuple[float, float], radius: float, z_min: float, z_max: float) -> None:
            cyl = UsdGeom.Cylinder(prim)
            cyl.CreateRadiusAttr(float(radius))
            cyl.CreateHeightAttr(float(z_max - z_min))
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(float(center_xy[0]), float(center_xy[1]), float((z_min + z_max) / 2.0)))
            UsdGeom.Imageable(prim).MakeVisible()

        # Environment shell and furniture not used by the planner.
        set_cube("/World/Floor", (3.25, 0.0, -0.04), (8.6, 6.6, 0.08), "#d7dde5")
        set_cube("/World/Ceiling", (3.25, 0.0, 2.92), (8.6, 6.6, 0.08), "#eef2f7")
        set_cube("/World/BackWall", (-0.85, 0.0, 1.45), (0.12, 6.6, 2.9), "#cbd5e1")
        set_cube("/World/EndWall", (7.42, 0.0, 1.45), (0.12, 6.6, 2.9), "#cbd5e1")
        set_cube("/World/GlassPanelA", (1.6, -2.50, 1.42), (1.2, 0.05, 1.7), "#b7d7f0")
        set_cube("/World/GlassPanelB", (4.4, -2.50, 1.42), (1.2, 0.05, 1.7), "#b7d7f0")
        set_cube("/World/Sign", (6.5, -2.48, 1.75), (0.8, 0.04, 0.35), "#0ea5e9")

        # Lights.
        dome = UsdLux.DomeLight.Define(stage_usd, "/World/DomeLight")
        dome.CreateIntensityAttr(950.0)
        key = UsdLux.DistantLight.Define(stage_usd, "/World/KeyLight")
        key.CreateIntensityAttr(4200.0)
        key.CreateAngleAttr(0.45)
        for idx, x in enumerate(np.linspace(0.0, 6.5, 4)):
            light = UsdLux.RectLight.Define(stage_usd, f"/World/CeilingPanelLight{idx}")
            light.CreateIntensityAttr(2600.0)
            light.CreateWidthAttr(1.1)
            light.CreateHeightAttr(0.35)
            prim = light.GetPrim()
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(float(x), 0.0, 2.82))
            xform.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))

        # Planner obstacles, created once and updated/hidden per frame.
        unique: dict[str, RuntimeObstacle] = {}
        for stage in stages:
            for obs in obstacle_records(stage, 0, 0.5):
                unique[obs.name] = obs

        obstacle_prims: dict[str, Any] = {}
        for idx, obs in enumerate(unique.values()):
            safe_name = "".join(ch if ch.isalnum() else "_" for ch in obs.name)
            path = f"/World/PlannerObstacles/Obs_{idx:02d}_{safe_name}"
            if obs.kind == "box" and obs.center is not None and obs.size is not None:
                obstacle_prims[obs.name] = set_cube(path, obs.center, obs.size, obs.color)
            elif obs.kind == "cylinder" and obs.center_xy is not None and obs.radius is not None:
                obstacle_prims[obs.name] = set_cylinder(path, obs.center_xy, obs.radius, obs.z_min or 0.0, obs.z_max or 1.5, obs.color)

        # Simple drone body, visible in the LiDAR/map panel and sometimes at frame edge.
        drone_parts = [
            set_cube("/World/Drone/Body", (0.0, 0.0, 1.15), (0.32, 0.20, 0.10), "#111827"),
            set_cube("/World/Drone/ArmX", (0.0, 0.0, 1.15), (0.72, 0.05, 0.04), "#334155"),
            set_cube("/World/Drone/ArmY", (0.0, 0.0, 1.15), (0.05, 0.72, 0.04), "#334155"),
        ]
        headlamp = UsdLux.SphereLight.Define(stage_usd, "/World/Drone/Headlamp")
        headlamp.CreateIntensityAttr(1650.0)
        headlamp.CreateRadiusAttr(0.45)
        headlamp_xform = UsdGeom.Xformable(headlamp.GetPrim())

        camera = UsdGeom.Camera.Define(stage_usd, "/World/OnboardCamera")
        camera.CreateFocalLengthAttr(18.0)
        camera.CreateHorizontalApertureAttr(25.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.03, 80.0))
        camera_xform = UsdGeom.Xformable(camera.GetPrim())
        camera_xform.ClearXformOpOrder()
        camera_transform_op = camera_xform.AddTransformOp()

        render_product = rep.create.render_product(str(camera.GetPath()), (int(args.width), int(args.height)))
        rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
        depth_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
        try:
            rgb_annot.attach([render_product])
            depth_annot.attach([render_product])
        except TypeError:
            rgb_annot.attach(render_product)
            depth_annot.attach(render_product)

        def set_camera(eye: np.ndarray, target: np.ndarray) -> None:
            view = Gf.Matrix4d(1.0)
            view.SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, target)), Gf.Vec3d(0.0, 0.0, 1.0))
            camera_transform_op.Set(view.GetInverse())

        def update_drone(drone: np.ndarray, heading: np.ndarray) -> None:
            update_cube(drone_parts[0], tuple(drone), (0.32, 0.20, 0.10))
            update_cube(drone_parts[1], tuple(drone + np.asarray([0.0, 0.0, 0.01])), (0.72, 0.05, 0.04))
            update_cube(drone_parts[2], tuple(drone + np.asarray([0.0, 0.0, 0.012])), (0.05, 0.72, 0.04))
            headlamp_xform.ClearXformOpOrder()
            headlamp_xform.AddTranslateOp().Set(Gf.Vec3d(*map(float, drone + heading * 0.45 + np.asarray([0.0, 0.0, 0.20]))))

        for _ in range(12):
            simulation_app.update()

        metrics_rows: list[dict[str, Any]] = []
        render_status = {"rgb_depth": "isaac_replicator", "lidar": "geometry_raycast_panel"}
        start_time = time.perf_counter()

        with tempfile.TemporaryDirectory(prefix="isaacsim_frames_") as tmpdir:
            frame_dir = Path(tmpdir)
            for frame_idx in range(int(args.frames)):
                stage_idx, stage, local_t = stage_for_frame(runtime, stages, frame_idx)
                active = obstacle_records(stage, stage_idx, local_t)
                active_names = {obs.name for obs in active}

                for name, prim in obstacle_prims.items():
                    if name not in active_names:
                        UsdGeom.Imageable(prim).MakeInvisible()
                        continue
                    obs = next(item for item in active if item.name == name)
                    if obs.kind == "box" and obs.center is not None and obs.size is not None:
                        update_cube(prim, obs.center, obs.size)
                    elif obs.kind == "cylinder" and obs.center_xy is not None and obs.radius is not None:
                        update_cylinder(prim, obs.center_xy, obs.radius, obs.z_min or 0.0, obs.z_max or 1.5)

                drone = runtime.points[frame_idx]
                look_idx = min(len(runtime.points) - 1, frame_idx + 9)
                heading = runtime.points[look_idx] - drone
                if np.linalg.norm(heading[:2]) < 1e-4:
                    heading = np.asarray([1.0, 0.0, 0.0])
                heading = heading / max(1e-6, float(np.linalg.norm(heading)))
                camera_heading = heading.copy()
                camera_heading[2] = 0.0
                if np.linalg.norm(camera_heading[:2]) < 1e-4:
                    camera_heading = np.asarray([1.0, 0.0, 0.0])
                camera_heading = camera_heading / max(1e-6, float(np.linalg.norm(camera_heading)))
                update_drone(drone, heading)

                if args.camera_mode == "chase":
                    camera_right = np.asarray([camera_heading[1], -camera_heading[0], 0.0])
                    eye = drone - camera_heading * 2.15 + camera_right * 0.55 + np.asarray([0.0, 0.0, 0.78])
                    target = drone + camera_heading * 1.10 + np.asarray([0.0, 0.0, 0.03])
                else:
                    eye = drone + np.asarray([-0.18, 0.0, 0.06])
                    target = drone + camera_heading * 2.35 + np.asarray([0.0, 0.0, -0.24])
                set_camera(eye, target)

                try:
                    rep.orchestrator.step()
                except Exception:
                    simulation_app.update()
                simulation_app.update()

                rgb = normalize_rgb(rgb_annot.get_data(), int(args.width), int(args.height))
                depth = normalize_depth(depth_annot.get_data(), int(args.width), int(args.height))
                depth_rgb = depth_to_rgb(depth)
                cloud = lidar_points(drone, active, seed=int(args.seed) + frame_idx)
                clearance = clearance_to_obstacles(drone, active)
                risk = risk_label(clearance)
                row = {
                    "frame": frame_idx,
                    "time_s": round(frame_idx / float(args.fps), 3),
                    "stage": stage_idx,
                    "risk": risk,
                    "clearance_m": round(float(clearance), 4),
                    "lidar_points": int(len(cloud)),
                    "drone_x": round(float(drone[0]), 4),
                    "drone_y": round(float(drone[1]), 4),
                    "drone_z": round(float(drone[2]), 4),
                }
                metrics_rows.append(row)

                frame_img = compose_frame(
                    rgb,
                    depth_rgb,
                    drone,
                    cloud,
                    active,
                    stage.plan,
                    frame_idx,
                    int(args.frames),
                    clearance,
                    risk,
                    stage_idx,
                    stage,
                    row,
                    str(args.camera_mode),
                )
                out_png = frame_dir / f"frame_{frame_idx:04d}.png"
                frame_img.save(out_png)
                if frame_idx == int(args.frames) // 2:
                    frame_img.save(args.preview)
                if frame_idx % max(1, int(args.frames) // 8) == 0:
                    print(f"[isaac-demo] rendered frame {frame_idx + 1}/{args.frames}", flush=True)

            if not args.preview.exists():
                (frame_dir / f"frame_{int(args.frames) // 2:04d}.png").replace(args.preview)

            cmd = [
                "ffmpeg",
                "-y",
                "-framerate",
                str(int(args.fps)),
                "-i",
                str(frame_dir / "frame_%04d.png"),
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                "-crf",
                "19",
                str(args.output),
            ]
            subprocess.run(cmd, check=True)

        with args.metrics.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(metrics_rows[0].keys()))
            writer.writeheader()
            writer.writerows(metrics_rows)

        elapsed = time.perf_counter() - start_time
        summary = {
            "output": str(args.output),
            "preview": str(args.preview),
            "metrics": str(args.metrics),
            "frames": int(args.frames),
            "fps": int(args.fps),
            "render_size": [int(args.width), int(args.height)],
            "elapsed_s": round(float(elapsed), 3),
            "isaac_renderer": str(args.renderer),
            "camera_mode": str(args.camera_mode),
            "sensor_status": render_status,
            "min_clearance_m": round(float(min(row["clearance_m"] for row in metrics_rows)), 4),
            "risk_counts": {label: sum(1 for row in metrics_rows if row["risk"] == label) for label in ["SAFE", "WARNING", "DANGER", "COLLISION"]},
        }
        args.summary.write_text(
            "# Isaac Sim Indoor RGB-D/LiDAR Demo\n\n"
            "This artifact is a headless Isaac Sim render for the indoor UAV obstacle-avoidance story.\n\n"
            "It shows an onboard RGB stream, distance-to-camera depth, a LiDAR/point-cloud top-down panel, "
            "dynamic indoor obstacles, and the fused obstacle-field/planner timeline.\n\n"
            "```json\n"
            + json.dumps(summary, indent=2, ensure_ascii=False)
            + "\n```\n",
            encoding="utf-8",
        )
        print(json.dumps(summary, indent=2), flush=True)
        completed = True
    except Exception:
        import traceback

        traceback.print_exc()
        return 1
    finally:
        if completed:
            simulation_app.close(skip_cleanup=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
