#!/usr/bin/env python3
"""Render an indoor doorway-turn ESDF/MPPI demonstration.

This is a lightweight visual artifact for the "harder than open-room" case:
the UAV must pass through a door opening under a low ceiling instead of escaping
by increasing altitude.  The scene is procedural, but the planning contract is
the same Level-3 story used elsewhere in the project:

    indoor geometry -> voxel occupancy/ESDF -> seeded MPPI path -> UAV motion
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
import subprocess
import sys
import tempfile
import textwrap
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.esdf3d import ESDF3D, VoxelGridSpec, compute_esdf, empty_occupancy, mark_box
from src.planners.mppi_3d_esdf import MPPI3DConfig, rollout_cost


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[int, int, int]
    alpha: int = 210


@dataclass(frozen=True)
class Result:
    path: np.ndarray
    costs: list[float]
    min_clearance_m: float
    path_length_m: float
    smoothness: float
    door_cross_y_m: float
    door_cross_z_m: float
    max_altitude_m: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/level3_indoor_doorway_turn_esdf_mppi.mp4"))
    parser.add_argument("--preview", type=Path, default=Path("outputs/figures/level3_video_preview/level3_doorway_turn_midframe.png"))
    parser.add_argument("--metrics", type=Path, default=Path("outputs/tables/level3_indoor_doorway_turn_mppi.csv"))
    parser.add_argument("--summary", type=Path, default=Path("outputs/level3_indoor_doorway_turn_summary.md"))
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--fps", type=int, default=18)
    parser.add_argument("--duration-s", type=float, default=12.0)
    parser.add_argument("--resolution", type=float, default=0.12)
    parser.add_argument("--seed", type=int, default=84)
    return parser.parse_args()


def font(size: int, bold: bool = False):
    candidates = [
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold else "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Supplemental/Helvetica.ttc",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


FONT_TITLE = font(30, True)
FONT_H2 = font(20, True)
FONT_BODY = font(16)
FONT_SMALL = font(13)

WALL = (107, 114, 128)
FRAME = (71, 85, 105)
FURNITURE = (101, 116, 139)
OBSTACLE = (239, 68, 68)
PATH = (37, 99, 235)
MPPI = (14, 165, 233)
DRONE = (245, 158, 11)
SAFE = (22, 163, 74)
WARN = (245, 158, 11)
DANGER = (220, 38, 38)


def make_spec(resolution: float) -> VoxelGridSpec:
    return VoxelGridSpec(
        nx=int(math.ceil(9.0 / resolution)),
        ny=int(math.ceil(6.2 / resolution)),
        nz=int(math.ceil(2.45 / resolution)),
        resolution_m=resolution,
        origin_xyz=(-0.7, -3.1, 0.0),
    )


def add_box(occ: np.ndarray, spec: VoxelGridSpec, boxes: list[Box], name: str, center, size, color, alpha=210) -> None:
    mark_box(occ, spec, center, size)
    boxes.append(Box(name=name, center=tuple(center), size=tuple(size), color=color, alpha=alpha))


def build_doorway_scene(spec: VoxelGridSpec) -> tuple[np.ndarray, list[Box]]:
    occ = empty_occupancy(spec)
    boxes: list[Box] = []

    # Boundary walls and a low ceiling.  The ceiling is the important constraint:
    # a high-altitude escape is not free space.
    add_box(occ, spec, boxes, "left boundary wall", (3.9, -2.9, 1.05), (8.9, 0.18, 2.1), WALL)
    add_box(occ, spec, boxes, "right boundary wall", (3.9, 2.9, 1.05), (8.9, 0.18, 2.1), WALL)
    add_box(occ, spec, boxes, "back wall", (-0.58, 0.0, 1.05), (0.18, 5.8, 2.1), WALL)
    add_box(occ, spec, boxes, "front wall", (8.28, 0.0, 1.05), (0.18, 5.8, 2.1), WALL)
    add_box(occ, spec, boxes, "low ceiling occupied slab", (3.9, 0.0, 2.18), (8.9, 5.8, 0.34), (148, 163, 184), 105)

    # Partition wall with only one door opening: y in [-0.62, 0.62], z below 1.62.
    wall_x = 3.65
    add_box(occ, spec, boxes, "door wall lower segment", (wall_x, -1.78, 1.02), (0.24, 2.24, 2.04), WALL)
    add_box(occ, spec, boxes, "door wall upper segment", (wall_x, 1.78, 1.02), (0.24, 2.24, 2.04), WALL)
    add_box(occ, spec, boxes, "door lintel", (wall_x, 0.0, 1.86), (0.24, 1.24, 0.42), FRAME)
    add_box(occ, spec, boxes, "left door frame", (wall_x, -0.71, 0.82), (0.30, 0.12, 1.64), FRAME)
    add_box(occ, spec, boxes, "right door frame", (wall_x, 0.71, 0.82), (0.30, 0.12, 1.64), FRAME)

    # Indoor clutter that makes the direct room-to-room line a poor route.
    add_box(occ, spec, boxes, "left-room bench", (1.40, -0.48, 0.45), (1.35, 0.76, 0.9), FURNITURE)
    add_box(occ, spec, boxes, "left-room cabinet", (2.22, 1.38, 0.80), (1.00, 0.78, 1.6), FURNITURE)
    add_box(occ, spec, boxes, "right-room shelf", (5.32, 1.62, 0.78), (1.15, 0.62, 1.56), FURNITURE)
    add_box(occ, spec, boxes, "right-room rolling cart", (6.25, -0.72, 0.58), (1.00, 0.72, 1.16), OBSTACLE)
    add_box(occ, spec, boxes, "goal-side pillar", (7.05, 0.92, 0.95), (0.44, 0.44, 1.9), OBSTACLE)

    return occ, boxes


def resample_polyline(points: np.ndarray, n: int) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(lengths.sum())
    targets = np.linspace(0.0, total, n)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    out = np.zeros((n, 3), dtype=float)
    seg = 0
    for i, target in enumerate(targets):
        while seg < len(lengths) - 1 and cumulative[seg + 1] < target:
            seg += 1
        denom = max(1e-9, cumulative[seg + 1] - cumulative[seg])
        alpha = (target - cumulative[seg]) / denom
        out[i] = (1.0 - alpha) * points[seg] + alpha * points[seg + 1]
    return out


def path_length(path: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def smoothness(path: np.ndarray) -> float:
    second = path[2:] - 2.0 * path[1:-1] + path[:-2]
    return float(np.sum(np.linalg.norm(second, axis=1) ** 2))


def optimize_seeded_mppi(esdf: ESDF3D, seed_path: np.ndarray, base_seed: int) -> Result:
    config = MPPI3DConfig(
        num_rollouts=520,
        horizon_steps=len(seed_path),
        max_iterations=9,
        temperature=0.85,
        noise_sigma_m=0.07,
        safety_radius_m=0.30,
        min_altitude_m=0.72,
        max_altitude_m=1.46,
        clearance_weight=260.0,
        smoothness_weight=7.5,
        path_length_weight=1.4,
        altitude_weight=220.0,
        collision_weight=9000.0,
        bounds_weight=9000.0,
        seed=base_seed,
    )
    rng = np.random.default_rng(base_seed)
    mean = seed_path.copy()
    start = mean[0].copy()
    goal = mean[-1].copy()
    door_x = 3.65
    costs: list[float] = []

    for _ in range(config.max_iterations):
        noise = rng.normal(0.0, config.noise_sigma_m, size=(config.num_rollouts, len(mean), 3))
        progress = np.sin(np.linspace(0.0, math.pi, len(mean)))
        noise *= progress[None, :, None]
        noise[:, :, 2] *= 0.35
        rollouts = mean[None, :, :] + noise
        rollouts[:, 0, :] = start
        rollouts[:, -1, :] = goal

        base_cost = rollout_cost(rollouts, esdf, config)
        guide_cost = 260.0 * np.sum((rollouts - seed_path[None, :, :]) ** 2, axis=(1, 2))
        cross_idx = np.argmin(np.abs(rollouts[:, :, 0] - door_x), axis=1)
        cross = rollouts[np.arange(len(rollouts)), cross_idx]
        door_cost = 4200.0 * cross[:, 1] ** 2
        door_cost += 1600.0 * (cross[:, 2] - 1.05) ** 2
        total_cost = base_cost + guide_cost + door_cost

        shifted = total_cost - float(np.min(total_cost))
        weights = np.exp(-shifted / max(config.temperature, 1e-6))
        weights /= max(float(np.sum(weights)), 1e-12)
        mean = mean + np.sum(weights[:, None, None] * noise, axis=0)
        mean[0] = start
        mean[-1] = goal
        for _smooth in range(2):
            mean[1:-1] = 0.22 * mean[:-2] + 0.56 * mean[1:-1] + 0.22 * mean[2:]
            mean[0] = start
            mean[-1] = goal
        mean[:, 2] = np.clip(mean[:, 2], config.min_altitude_m, config.max_altitude_m)
        costs.append(float(rollout_cost(mean[None, :, :], esdf, config)[0]))

    distances = esdf.query_distance(mean)
    cross_idx = int(np.argmin(np.abs(mean[:, 0] - door_x)))
    return Result(
        path=mean,
        costs=costs,
        min_clearance_m=float(np.min(distances)),
        path_length_m=path_length(mean),
        smoothness=smoothness(mean),
        door_cross_y_m=float(mean[cross_idx, 1]),
        door_cross_z_m=float(mean[cross_idx, 2]),
        max_altitude_m=float(np.max(mean[:, 2])),
    )


def box_corners(box: Box) -> np.ndarray:
    cx, cy, cz = box.center
    sx, sy, sz = box.size
    return np.asarray([[cx + dx * sx / 2, cy + dy * sy / 2, cz + dz * sz / 2] for dx in (-1, 1) for dy in (-1, 1) for dz in (-1, 1)])


FACES = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]


def camera_basis(path: np.ndarray, idx: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    here = path[idx]
    ahead = path[min(len(path) - 1, idx + 8)] - here
    ahead[2] = 0.0
    if np.linalg.norm(ahead) < 1e-6:
        ahead = np.asarray([1.0, 0.0, 0.0])
    forward = ahead / np.linalg.norm(ahead)
    right = np.asarray([forward[1], -forward[0], 0.0])
    up = np.asarray([0.0, 0.0, 1.0])
    cam = here - forward * 2.4 + right * 0.85 + up * 0.95
    return cam, forward, right, up


def project(points: np.ndarray, cam: np.ndarray, forward: np.ndarray, right: np.ndarray, up: np.ndarray, rect):
    x0, y0, w, h = rect
    rel = np.asarray(points, dtype=float) - cam[None, :]
    px = rel @ right
    py = rel @ up
    pz = rel @ forward
    z = np.maximum(pz, 0.10)
    focal = 0.88
    sx = x0 + w * (0.5 + focal * px / z)
    sy = y0 + h * (0.55 - focal * (py - 0.04 * z) / z)
    return np.stack([sx, sy], axis=1), pz


def shade(color: tuple[int, int, int], depth: float) -> tuple[int, int, int]:
    amount = min(0.50, max(0.0, depth / 12.0))
    return tuple(int(c + (255 - c) * amount) for c in color)


def draw_box_3d(draw: ImageDraw.ImageDraw, box: Box, cam, forward, right, up, rect) -> None:
    corners = box_corners(box)
    pts, depth = project(corners, cam, forward, right, up, rect)
    faces = []
    for face in FACES:
        mean_depth = float(np.mean(depth[list(face)]))
        if mean_depth <= 0.12:
            continue
        poly = pts[list(face)]
        x0, y0, w, h = rect
        if np.all(poly[:, 0] < x0 - 80) or np.all(poly[:, 0] > x0 + w + 80):
            continue
        if np.all(poly[:, 1] < y0 - 80) or np.all(poly[:, 1] > y0 + h + 80):
            continue
        faces.append((mean_depth, poly))
    for depth_mean, poly in sorted(faces, key=lambda item: item[0], reverse=True):
        draw.polygon([tuple(p) for p in poly], fill=shade(box.color, depth_mean), outline=(51, 65, 85))


def draw_drone(draw: ImageDraw.ImageDraw, pos: np.ndarray, cam, forward, right, up, rect) -> None:
    pts3 = np.asarray(
        [
            pos + np.asarray([0.00, 0.00, 0.00]),
            pos + np.asarray([0.34, 0.00, 0.00]),
            pos + np.asarray([-0.28, -0.22, 0.00]),
            pos + np.asarray([-0.28, 0.22, 0.00]),
            pos + np.asarray([0.00, 0.00, 0.18]),
        ]
    )
    pts, depth = project(pts3, cam, forward, right, up, rect)
    if float(np.mean(depth)) <= 0.12:
        return
    body = pts[0]
    draw.ellipse([body[0] - 13, body[1] - 13, body[0] + 13, body[1] + 13], fill=DRONE, outline=(15, 23, 42), width=2)
    for rotor in pts[1:4]:
        draw.line([tuple(body), tuple(rotor)], fill=(15, 23, 42), width=3)
        draw.ellipse([rotor[0] - 8, rotor[1] - 8, rotor[0] + 8, rotor[1] + 8], outline=(15, 23, 42), width=2)
    tip = pts[4]
    draw.line([tuple(body), tuple(tip)], fill=(15, 23, 42), width=2)


def map_point(point: np.ndarray, spec: VoxelGridSpec, rect) -> tuple[int, int]:
    x0, y0, w, h = rect
    px = x0 + (point[0] - spec.origin[0]) / (spec.upper[0] - spec.origin[0]) * w
    py = y0 + h - (point[1] - spec.origin[1]) / (spec.upper[1] - spec.origin[1]) * h
    return int(px), int(py)


def draw_topdown(draw: ImageDraw.ImageDraw, boxes: list[Box], path: np.ndarray, flown: np.ndarray, pos: np.ndarray, spec: VoxelGridSpec, rect) -> None:
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(248, 250, 252), outline=(203, 213, 225), width=2)
    draw.text((x + 12, y + 10), "Top-down map: only door gap is free", fill=(15, 23, 42), font=FONT_H2)
    for box in boxes:
        cx, cy, _ = box.center
        sx, sy, _ = box.size
        p1 = map_point(np.asarray([cx - sx / 2, cy - sy / 2, 0.0]), spec, rect)
        p2 = map_point(np.asarray([cx + sx / 2, cy + sy / 2, 0.0]), spec, rect)
        fill = box.color
        draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=fill, outline=(51, 65, 85))
    draw.rectangle([x + 2, y + 2, x + w - 2, y + h - 2], outline=(15, 23, 42), width=1)
    for arr, color, width_line in [(path, MPPI, 4), (flown, PATH, 6)]:
        pts = [map_point(p, spec, rect) for p in arr]
        if len(pts) >= 2:
            draw.line(pts, fill=color, width=width_line)
    pc = map_point(pos, spec, rect)
    draw.ellipse([pc[0] - 8, pc[1] - 8, pc[0] + 8, pc[1] + 8], fill=DRONE, outline=(15, 23, 42), width=2)
    door = map_point(np.asarray([3.65, 0.0, 0.0]), spec, rect)
    draw.ellipse([door[0] - 6, door[1] - 6, door[0] + 6, door[1] + 6], outline=SAFE, width=3)
    draw.text((x + 12, y + h - 28), "green circle = door waypoint / no fly-over route", fill=(51, 65, 85), font=FONT_SMALL)


def ray_box_intersection(origin: np.ndarray, direction: np.ndarray, box: Box) -> float | None:
    low = np.asarray(box.center) - np.asarray(box.size) / 2.0
    high = np.asarray(box.center) + np.asarray(box.size) / 2.0
    inv = 1.0 / np.where(np.abs(direction) < 1e-9, 1e-9, direction)
    t1 = (low - origin) * inv
    t2 = (high - origin) * inv
    tmin = float(np.max(np.minimum(t1, t2)))
    tmax = float(np.min(np.maximum(t1, t2)))
    if tmax >= max(tmin, 0.0):
        return max(tmin, 0.0)
    return None


def lidar_scan(pos: np.ndarray, boxes: list[Box], num: int = 96, max_range: float = 5.0) -> tuple[np.ndarray, np.ndarray]:
    angles = np.linspace(-math.pi, math.pi, num, endpoint=False)
    points = []
    ranges = []
    origin = pos.copy()
    origin[2] = pos[2]
    for angle in angles:
        direction = np.asarray([math.cos(angle), math.sin(angle), 0.0])
        best = max_range
        for box in boxes:
            hit = ray_box_intersection(origin, direction, box)
            if hit is not None and 0.02 < hit < best:
                best = hit
        ranges.append(best)
        points.append(origin + direction * best)
    return np.asarray(points), np.asarray(ranges)


def draw_sensor_panel(draw: ImageDraw.ImageDraw, pos: np.ndarray, boxes: list[Box], spec: VoxelGridSpec, rect) -> tuple[int, float]:
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(255, 255, 255), outline=(203, 213, 225), width=2)
    draw.text((x + 12, y + 10), "Sensor inputs: depth + LiDAR rays", fill=(15, 23, 42), font=FONT_H2)
    points, ranges = lidar_scan(pos, boxes, num=140, max_range=5.2)
    depth_rect = (x + 16, y + 46, w - 32, 62)
    for i, rng in enumerate(ranges[35:105]):
        alpha = i / max(1, 70)
        col = int(255 - min(1.0, rng / 5.2) * 190)
        xx0 = depth_rect[0] + int(alpha * depth_rect[2])
        xx1 = depth_rect[0] + int((i + 1) / 70 * depth_rect[2]) + 1
        draw.rectangle([xx0, depth_rect[1], xx1, depth_rect[1] + depth_rect[3]], fill=(col, 38, 130 + int(rng / 5.2 * 80)))
    draw.rectangle([depth_rect[0], depth_rect[1], depth_rect[0] + depth_rect[2], depth_rect[1] + depth_rect[3]], outline=(51, 65, 85))
    draw.text((depth_rect[0], depth_rect[1] + depth_rect[3] + 4), "relative depth strip: closer = brighter", fill=(51, 65, 85), font=FONT_SMALL)

    cloud_rect = (x + 16, y + 138, w - 32, h - 154)
    draw.rectangle([cloud_rect[0], cloud_rect[1], cloud_rect[0] + cloud_rect[2], cloud_rect[1] + cloud_rect[3]], fill=(241, 245, 249), outline=(203, 213, 225))
    center = (cloud_rect[0] + cloud_rect[2] // 2, cloud_rect[1] + cloud_rect[3] // 2)
    scale = min(cloud_rect[2], cloud_rect[3]) / 11.0
    for point, rng in zip(points, ranges):
        dx, dy = point[0] - pos[0], point[1] - pos[1]
        px = int(center[0] + dx * scale)
        py = int(center[1] - dy * scale)
        color = DANGER if rng < 1.0 else WARN if rng < 2.0 else (59, 130, 246)
        draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill=color)
    draw.ellipse([center[0] - 7, center[1] - 7, center[0] + 7, center[1] + 7], fill=DRONE, outline=(15, 23, 42))
    return len(points), float(np.min(ranges))


def draw_altitude(draw: ImageDraw.ImageDraw, path: np.ndarray, idx: int, rect) -> None:
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(255, 255, 255), outline=(203, 213, 225), width=2)
    draw.text((x + 12, y + 10), "Altitude constraint", fill=(15, 23, 42), font=FONT_H2)
    y_lintel = y + 52
    y_ceiling = y + 30
    draw.line([(x + 20, y_lintel), (x + w - 20, y_lintel)], fill=(239, 68, 68), width=2)
    draw.text((x + 22, y_lintel + 4), "door lintel / low ceiling blocks fly-over", fill=(127, 29, 29), font=FONT_SMALL)
    zmin, zmax = 0.65, 2.15
    pts = []
    for i, p in enumerate(path):
        px = x + 20 + i / max(1, len(path) - 1) * (w - 40)
        py = y + h - 18 - (p[2] - zmin) / (zmax - zmin) * (h - 64)
        pts.append((int(px), int(py)))
    draw.line(pts, fill=(13, 148, 136), width=4)
    px, py = pts[idx]
    draw.ellipse([px - 6, py - 6, px + 6, py + 6], fill=DRONE, outline=(15, 23, 42))
    draw.text((x + w - 118, y_ceiling), f"z={path[idx,2]:.2f} m", fill=(15, 23, 42), font=FONT_BODY)


def risk_color(clearance: float) -> tuple[int, int, int]:
    if clearance < 0.30:
        return DANGER
    if clearance < 0.45:
        return WARN
    return SAFE


def draw_scene(draw: ImageDraw.ImageDraw, boxes: list[Box], path: np.ndarray, idx: int, spec: VoxelGridSpec, rect) -> None:
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(226, 232, 240), outline=(203, 213, 225), width=2)
    draw.text((x + 16, y + 12), "Third-person constrained indoor doorway turn", fill=(15, 23, 42), font=FONT_H2)
    cam, forward, right, up = camera_basis(path, idx)
    visible_boxes = [
        box
        for box in boxes
        if box.name
        not in {
            "left boundary wall",
            "right boundary wall",
            "back wall",
            "front wall",
            "low ceiling occupied slab",
            "door wall lower segment",
            "door wall upper segment",
        }
    ]
    for box in sorted(visible_boxes, key=lambda b: float(np.asarray(b.center) @ forward), reverse=True):
        draw_box_3d(draw, box, cam, forward, right, up, rect)
    pts, depth = project(path, cam, forward, right, up, rect)
    visible = [(tuple(p), d) for p, d in zip(pts, depth) if d > 0.1 and x <= p[0] <= x + w and y <= p[1] <= y + h]
    if len(visible) > 2:
        draw.line([p for p, _ in visible], fill=(15, 23, 42), width=7)
        draw.line([p for p, _ in visible], fill=MPPI, width=4)
    flown = path[: idx + 1]
    pts2, depth2 = project(flown, cam, forward, right, up, rect)
    vis2 = [tuple(p) for p, d in zip(pts2, depth2) if d > 0.1 and x <= p[0] <= x + w and y <= p[1] <= y + h]
    if len(vis2) > 2:
        draw.line(vis2, fill=PATH, width=6)
    draw_drone(draw, path[idx], cam, forward, right, up, rect)
    draw.text((x + 16, y + h - 34), "Low ceiling + wall partition: valid route is through the door opening, not over it.", fill=(15, 23, 42), font=FONT_BODY)


def render_frame(
    width: int,
    height: int,
    boxes: list[Box],
    spec: VoxelGridSpec,
    esdf: ESDF3D,
    result: Result,
    frame: int,
    total_frames: int,
) -> tuple[Image.Image, dict[str, object]]:
    img = Image.new("RGB", (width, height), (241, 245, 249))
    draw = ImageDraw.Draw(img)
    draw.text((28, 18), "Level 3 indoor door scenario: MPPI must turn through doorway", fill=(15, 23, 42), font=FONT_TITLE)
    draw.text(
        (28, 54),
        "Constraint: low ceiling + partition wall + narrow door.  Planner output should not solve the task by climbing over obstacles.",
        fill=(71, 85, 105),
        font=FONT_BODY,
    )

    idx = min(len(result.path) - 1, int(frame / max(1, total_frames - 1) * (len(result.path) - 1)))
    point = result.path[idx]
    clearance = float(esdf.query_distance(point[None, :])[0])
    status = "DANGER" if clearance < 0.30 else "WARNING" if clearance < 0.45 else "SAFE"

    scene_rect = (28, 94, 930, 604)
    map_rect = (986, 94, 576, 300)
    sensor_rect = (986, 414, 576, 292)
    alt_rect = (28, 720, 930, 146)
    metric_rect = (986, 728, 576, 138)

    draw_scene(draw, boxes, result.path, idx, spec, scene_rect)
    draw_topdown(draw, boxes, result.path, result.path[: idx + 1], point, spec, map_rect)
    lidar_points, min_lidar_range = draw_sensor_panel(draw, point, boxes, spec, sensor_rect)
    draw_altitude(draw, result.path, idx, alt_rect)

    draw.rectangle([metric_rect[0], metric_rect[1], metric_rect[0] + metric_rect[2], metric_rect[1] + metric_rect[3]], fill=(255, 255, 255), outline=(203, 213, 225), width=2)
    draw.text((metric_rect[0] + 12, metric_rect[1] + 10), "Planner / safety readout", fill=(15, 23, 42), font=FONT_H2)
    pill = [metric_rect[0] + metric_rect[2] - 140, metric_rect[1] + 12, metric_rect[0] + metric_rect[2] - 20, metric_rect[1] + 42]
    draw.rounded_rectangle(pill, radius=10, fill=risk_color(clearance))
    draw.text((pill[0] + 20, pill[1] + 6), status, fill=(255, 255, 255), font=FONT_SMALL)
    lines = [
        f"t={frame / 18.0:4.1f}s  clearance={clearance:.3f} m  min LiDAR={min_lidar_range:.2f} m",
        f"door crossing y={result.door_cross_y_m:+.2f} m, z={result.door_cross_z_m:.2f} m",
        f"max altitude={result.max_altitude_m:.2f} m  path length={result.path_length_m:.2f} m",
    ]
    for i, line in enumerate(lines):
        draw.text((metric_rect[0] + 14, metric_rect[1] + 50 + i * 24), line, fill=(51, 65, 85), font=FONT_BODY)

    return img, {
        "frame": frame,
        "time_s": frame / 18.0,
        "x": float(point[0]),
        "y": float(point[1]),
        "z": float(point[2]),
        "clearance_m": clearance,
        "risk": status,
        "lidar_points": lidar_points,
        "min_lidar_range_m": min_lidar_range,
    }


def write_video(frames_dir: Path, output: Path, fps: int) -> None:
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg is required to render MP4 video")
    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-framerate",
        str(fps),
        "-i",
        str(frames_dir / "frame_%04d.png"),
        "-vf",
        "scale=trunc(iw/2)*2:trunc(ih/2)*2",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        "18",
        str(output),
    ]
    subprocess.run(cmd, check=True)


def main() -> int:
    args = parse_args()
    spec = make_spec(float(args.resolution))
    occ, boxes = build_doorway_scene(spec)
    esdf = compute_esdf(occ, spec, prefer_scipy=True)

    seed_waypoints = np.asarray(
        [
            [0.34, -2.12, 1.05],
            [1.15, -2.02, 1.05],
            [2.50, -0.98, 1.04],
            [3.33, -0.20, 1.04],
            [3.66, 0.00, 1.04],
            [4.16, 0.18, 1.04],
            [5.35, 0.34, 1.05],
            [6.62, 1.36, 1.06],
            [7.66, 2.08, 1.06],
        ],
        dtype=float,
    )
    seed_path = resample_polyline(seed_waypoints, 88)
    result = optimize_seeded_mppi(esdf, seed_path, int(args.seed))

    total_frames = int(round(float(args.duration_s) * int(args.fps)))
    rows: list[dict[str, object]] = []
    mid_frame = total_frames // 2
    with tempfile.TemporaryDirectory(prefix="doorway_turn_frames_") as tmp:
        frame_dir = Path(tmp)
        for frame in range(total_frames):
            img, row = render_frame(args.width, args.height, boxes, spec, esdf, result, frame, total_frames)
            rows.append(row)
            img.save(frame_dir / f"frame_{frame:04d}.png")
            if frame == mid_frame:
                args.preview.parent.mkdir(parents=True, exist_ok=True)
                img.save(args.preview)
        write_video(frame_dir, args.output, int(args.fps))

    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    with args.metrics.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    summary = textwrap.dedent(
        f"""\
        # Level 3 Indoor Doorway-Turn ESDF/MPPI Demo

        This demo addresses the failure mode where the UAV solves obstacle
        avoidance by climbing above obstacles.  The simulated indoor environment
        has a low ceiling, a full partition wall, and only one valid door
        opening.  The path therefore has to turn through the doorway.

        - Video: `{args.output}`
        - Preview: `{args.preview}`
        - Metrics: `{args.metrics}`
        - Door opening: `x=3.65 m`, `y in [-0.62, 0.62] m`, usable height below the lintel.
        - Minimum ESDF clearance: `{result.min_clearance_m:.3f} m`
        - Door crossing: `y={result.door_cross_y_m:+.3f} m`, `z={result.door_cross_z_m:.3f} m`
        - Maximum altitude: `{result.max_altitude_m:.3f} m`
        - Path length: `{result.path_length_m:.3f} m`
        - Smoothness: `{result.smoothness:.4f}`

        Scope note: this is a lightweight procedural ESDF/MPPI visualization,
        not a PX4/Gazebo closed-loop flight.  It is meant to demonstrate the
        constrained indoor planner story before moving the same scenario into
        Isaac Sim or Gazebo.
        """
    )
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(summary)

    print(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
