#!/usr/bin/env python3
"""Render a sensor-ablation planner-decision demo.

The video is a lightweight, deterministic visualization of the perception to
planner contract:

    sensor evidence -> obstacle/cost map -> planner decision -> safety result

Each segment compares a full sensor map against the same planner with one
sensor removed.  The goal is to make the value of LiDAR, depth and radar visible
without requiring a ROS/Gazebo/Isaac runtime.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import math
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))


@dataclass(frozen=True)
class RectObstacle:
    name: str
    sensor: str
    center: tuple[float, float]
    size: tuple[float, float]
    color: tuple[int, int, int]


@dataclass(frozen=True)
class GridSpec2D:
    origin: tuple[float, float] = (0.0, -2.8)
    upper: tuple[float, float] = (8.3, 2.8)
    resolution_m: float = 0.07

    @property
    def shape(self) -> tuple[int, int]:
        width = int(math.ceil((self.upper[0] - self.origin[0]) / self.resolution_m)) + 1
        height = int(math.ceil((self.upper[1] - self.origin[1]) / self.resolution_m)) + 1
        return width, height


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    removed_sensor: str
    sensor_story: str
    failure_story: str
    true_obstacles: tuple[RectObstacle, ...]
    full_obstacles: tuple[RectObstacle, ...]
    ablated_obstacles: tuple[RectObstacle, ...]


@dataclass(frozen=True)
class PlanResult:
    path: np.ndarray
    min_clearance_m: float
    path_length_m: float
    collision: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/sensor_ablation_planner_decision_demo.mp4"))
    parser.add_argument("--preview", type=Path, default=Path("outputs/figures/level3_video_preview/sensor_ablation_planner_decision_midframe.png"))
    parser.add_argument("--metrics", type=Path, default=Path("outputs/tables/sensor_ablation_planner_decision_metrics.csv"))
    parser.add_argument("--summary", type=Path, default=Path("outputs/sensor_ablation_planner_decision_summary.md"))
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=18)
    parser.add_argument("--phase-duration-s", type=float, default=5.0)
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


FONT_TITLE = font(34, True)
FONT_H2 = font(22, True)
FONT_BODY = font(17)
FONT_SMALL = font(14)
FONT_TINY = font(12)

LIDAR = (37, 99, 235)
DEPTH = (168, 85, 247)
RADAR = (249, 115, 22)
WALL = (100, 116, 139)
FULL = (5, 150, 105)
ABLATE = (220, 38, 38)
DRONE = (245, 158, 11)
GRID = (226, 232, 240)
TEXT = (15, 23, 42)
MUTED = (71, 85, 105)

START = np.asarray([0.62, -2.10], dtype=float)
GOAL = np.asarray([7.65, 2.08], dtype=float)
SPEC = GridSpec2D()


def base_lidar_obstacles() -> tuple[RectObstacle, ...]:
    return (
        RectObstacle("partition lower wall", "lidar", (3.55, -1.70), (0.22, 2.22), WALL),
        RectObstacle("partition upper wall", "lidar", (3.55, 1.70), (0.22, 2.22), WALL),
        RectObstacle("door left frame", "lidar", (3.55, -0.63), (0.30, 0.13), WALL),
        RectObstacle("door right frame", "lidar", (3.55, 0.63), (0.30, 0.13), WALL),
    )


def scenarios() -> list[Scenario]:
    lidar_obs = base_lidar_obstacles()
    glass = RectObstacle("depth-only glass panel", "depth", (4.55, 0.52), (0.78, 0.88), DEPTH)
    low_box = RectObstacle("depth low box", "depth", (5.05, 0.10), (0.72, 0.48), DEPTH)
    radar_sweep = RectObstacle("radar predicted moving person sweep", "radar", (5.45, 0.83), (0.74, 1.44), RADAR)
    return [
        Scenario(
            key="without_lidar",
            title="LiDAR ablation: structure and doorway disappear",
            removed_sensor="lidar",
            sensor_story="LiDAR returns wall/door-frame points, creating the structural occupancy map.",
            failure_story="Without LiDAR, the planner cuts through the partition wall instead of using the door.",
            true_obstacles=lidar_obs,
            full_obstacles=lidar_obs,
            ablated_obstacles=tuple(),
        ),
        Scenario(
            key="without_depth",
            title="Depth ablation: transparent/low obstacle is missed",
            removed_sensor="depth",
            sensor_story="Depth marks near glass/low obstacles after the doorway, changing the route after entry.",
            failure_story="Without depth, the planner uses the doorway but drives through the glass/low obstacle.",
            true_obstacles=lidar_obs + (glass, low_box),
            full_obstacles=lidar_obs + (glass, low_box),
            ablated_obstacles=lidar_obs,
        ),
        Scenario(
            key="without_radar",
            title="Radar ablation: moving obstacle prediction is missed",
            removed_sensor="radar",
            sensor_story="Radar contributes a Doppler/predicted sweep for a moving person crossing the route.",
            failure_story="Without radar, the static map looks free and the planner enters the future crossing zone.",
            true_obstacles=lidar_obs + (radar_sweep,),
            full_obstacles=lidar_obs + (radar_sweep,),
            ablated_obstacles=lidar_obs,
        ),
    ]


def xy_to_idx(xy: np.ndarray, spec: GridSpec2D = SPEC) -> tuple[int, int]:
    idx = np.round((xy - np.asarray(spec.origin)) / spec.resolution_m).astype(int)
    return int(idx[0]), int(idx[1])


def idx_to_xy(idx: tuple[int, int], spec: GridSpec2D = SPEC) -> np.ndarray:
    return np.asarray(spec.origin) + np.asarray(idx, dtype=float) * spec.resolution_m


def rect_bounds(obs: RectObstacle, inflate_m: float = 0.0) -> tuple[float, float, float, float]:
    cx, cy = obs.center
    sx, sy = obs.size
    return cx - sx / 2 - inflate_m, cx + sx / 2 + inflate_m, cy - sy / 2 - inflate_m, cy + sy / 2 + inflate_m


def build_occupancy(obstacles: tuple[RectObstacle, ...], inflate_m: float = 0.18, spec: GridSpec2D = SPEC) -> np.ndarray:
    nx, ny = spec.shape
    xs = spec.origin[0] + np.arange(nx) * spec.resolution_m
    ys = spec.origin[1] + np.arange(ny) * spec.resolution_m
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    occ = np.zeros((nx, ny), dtype=bool)
    for obs in obstacles:
        xmin, xmax, ymin, ymax = rect_bounds(obs, inflate_m)
        occ |= (xx >= xmin) & (xx <= xmax) & (yy >= ymin) & (yy <= ymax)
    return occ


def nearest_free(occ: np.ndarray, idx: tuple[int, int]) -> tuple[int, int]:
    nx, ny = occ.shape
    x0 = min(max(idx[0], 0), nx - 1)
    y0 = min(max(idx[1], 0), ny - 1)
    if not occ[x0, y0]:
        return x0, y0
    for radius in range(1, max(nx, ny)):
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                x = x0 + dx
                y = y0 + dy
                if 0 <= x < nx and 0 <= y < ny and not occ[x, y]:
                    return x, y
    raise RuntimeError("no free cell")


def astar_path(obstacles: tuple[RectObstacle, ...]) -> np.ndarray:
    occ = build_occupancy(obstacles)
    start_idx = nearest_free(occ, xy_to_idx(START))
    goal_idx = nearest_free(occ, xy_to_idx(GOAL))
    neighbors = [
        (-1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (1, -1, math.sqrt(2.0)),
        (1, 0, 1.0),
        (1, 1, math.sqrt(2.0)),
    ]
    open_heap: list[tuple[float, tuple[int, int]]] = [(0.0, start_idx)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start_idx: 0.0}
    visited: set[tuple[int, int]] = set()

    def heuristic(a: tuple[int, int]) -> float:
        return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(goal_idx, dtype=float)))

    nx, ny = occ.shape
    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)
        if current == goal_idx:
            out = [current]
            while current in came_from:
                current = came_from[current]
                out.append(current)
            out.reverse()
            return resample_polyline(np.asarray([idx_to_xy(p) for p in out], dtype=float), 180)
        for dx, dy, step in neighbors:
            nb = (current[0] + dx, current[1] + dy)
            if not (0 <= nb[0] < nx and 0 <= nb[1] < ny) or occ[nb]:
                continue
            cand = g_score[current] + step * SPEC.resolution_m
            if cand < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = cand
                heapq.heappush(open_heap, (cand + heuristic(nb) * SPEC.resolution_m, nb))
    raise RuntimeError("A* failed")


def resample_polyline(points: np.ndarray, n: int) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) <= 1:
        return np.repeat(points[:1], n, axis=0)
    seg_len = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(np.sum(seg_len))
    if total <= 1e-9:
        return np.repeat(points[:1], n, axis=0)
    cumulative = np.concatenate([[0.0], np.cumsum(seg_len)])
    targets = np.linspace(0.0, total, n)
    out = np.zeros((n, 2), dtype=float)
    seg = 0
    for i, target in enumerate(targets):
        while seg < len(seg_len) - 1 and cumulative[seg + 1] < target:
            seg += 1
        alpha = (target - cumulative[seg]) / max(cumulative[seg + 1] - cumulative[seg], 1e-9)
        out[i] = (1.0 - alpha) * points[seg] + alpha * points[seg + 1]
    return out


def signed_distance_to_rect(point: np.ndarray, obs: RectObstacle, inflate_m: float = 0.0) -> float:
    center = np.asarray(obs.center, dtype=float)
    half = np.asarray(obs.size, dtype=float) / 2.0 + inflate_m
    q = np.abs(point - center) - half
    outside = float(np.linalg.norm(np.maximum(q, 0.0)))
    inside = float(min(max(q[0], q[1]), 0.0))
    return outside + inside


def evaluate_path(path: np.ndarray, true_obstacles: tuple[RectObstacle, ...]) -> PlanResult:
    distances = [min(signed_distance_to_rect(p, obs, 0.0) for obs in true_obstacles) if true_obstacles else 99.0 for p in path]
    length = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
    min_dist = float(min(distances))
    return PlanResult(path=path, min_clearance_m=min_dist, path_length_m=length, collision=bool(min_dist <= 0.0))


def world_to_px(point: np.ndarray | tuple[float, float], rect: tuple[int, int, int, int]) -> tuple[int, int]:
    x0, y0, w, h = rect
    p = np.asarray(point, dtype=float)
    px = x0 + (p[0] - SPEC.origin[0]) / (SPEC.upper[0] - SPEC.origin[0]) * w
    py = y0 + h - (p[1] - SPEC.origin[1]) / (SPEC.upper[1] - SPEC.origin[1]) * h
    return int(round(px)), int(round(py))


def draw_rect_obstacle(draw: ImageDraw.ImageDraw, obs: RectObstacle, rect, outline=(51, 65, 85), fill_alpha: float = 1.0) -> None:
    xmin, xmax, ymin, ymax = rect_bounds(obs, 0.0)
    p1 = world_to_px((xmin, ymin), rect)
    p2 = world_to_px((xmax, ymax), rect)
    fill = tuple(int(c * fill_alpha + 255 * (1.0 - fill_alpha)) for c in obs.color)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=fill, outline=outline, width=2)


def draw_map_frame(draw: ImageDraw.ImageDraw, rect, title: str) -> None:
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(248, 250, 252), outline=(203, 213, 225), width=2)
    for gx in np.arange(0.0, 8.4, 0.7):
        p1 = world_to_px((gx, SPEC.origin[1]), rect)
        p2 = world_to_px((gx, SPEC.upper[1]), rect)
        draw.line([p1, p2], fill=GRID, width=1)
    for gy in np.arange(-2.8, 2.81, 0.7):
        p1 = world_to_px((SPEC.origin[0], gy), rect)
        p2 = world_to_px((SPEC.upper[0], gy), rect)
        draw.line([p1, p2], fill=GRID, width=1)


def draw_panel_title(draw: ImageDraw.ImageDraw, rect, title: str) -> None:
    x, y, w, _ = rect
    draw.rounded_rectangle([x + 10, y + 10, x + w - 10, y + 48], radius=8, fill=(255, 255, 255), outline=(226, 232, 240))
    draw.text((x + 20, y + 18), title, fill=TEXT, font=FONT_H2)


def draw_path(draw: ImageDraw.ImageDraw, path: np.ndarray, rect, color: tuple[int, int, int], width: int = 6) -> None:
    pts = [world_to_px(p, rect) for p in path]
    if len(pts) >= 2:
        draw.line(pts, fill=(15, 23, 42), width=width + 3, joint="curve")
        draw.line(pts, fill=color, width=width, joint="curve")


def draw_drone(draw: ImageDraw.ImageDraw, point: np.ndarray, rect, color=DRONE) -> None:
    px, py = world_to_px(point, rect)
    draw.ellipse([px - 10, py - 10, px + 10, py + 10], fill=color, outline=(15, 23, 42), width=2)
    draw.line([(px - 18, py), (px + 18, py)], fill=(15, 23, 42), width=2)
    draw.line([(px, py - 18), (px, py + 18)], fill=(15, 23, 42), width=2)


def draw_collision_mark(draw: ImageDraw.ImageDraw, path: np.ndarray, true_obstacles: tuple[RectObstacle, ...], rect) -> None:
    if not true_obstacles:
        return
    dists = np.asarray([min(signed_distance_to_rect(p, obs, 0.0) for obs in true_obstacles) for p in path])
    idx = int(np.argmin(dists))
    if dists[idx] <= 0.0:
        px, py = world_to_px(path[idx], rect)
        draw.line([(px - 14, py - 14), (px + 14, py + 14)], fill=ABLATE, width=5)
        draw.line([(px - 14, py + 14), (px + 14, py - 14)], fill=ABLATE, width=5)


def sample_obstacle_edges(obs: RectObstacle, count_per_edge: int = 18) -> np.ndarray:
    xmin, xmax, ymin, ymax = rect_bounds(obs, 0.0)
    xs = np.linspace(xmin, xmax, count_per_edge)
    ys = np.linspace(ymin, ymax, count_per_edge)
    pts = []
    pts.extend((x, ymin) for x in xs)
    pts.extend((x, ymax) for x in xs)
    pts.extend((xmin, y) for y in ys)
    pts.extend((xmax, y) for y in ys)
    return np.asarray(pts, dtype=float)


def draw_sensor_evidence(draw: ImageDraw.ImageDraw, scenario: Scenario, active_path: np.ndarray, progress: float, rect) -> None:
    drone = active_path[min(len(active_path) - 1, int(progress * (len(active_path) - 1)))]
    for obs in scenario.true_obstacles:
        draw_rect_obstacle(draw, obs, rect, fill_alpha=0.22)
    draw_path(draw, active_path, rect, FULL, width=5)
    draw_drone(draw, drone, rect)

    if scenario.removed_sensor == "lidar":
        for obs in scenario.full_obstacles:
            if obs.sensor != "lidar":
                continue
            pts = sample_obstacle_edges(obs, 10)
            for p in pts[::2]:
                px, py = world_to_px(p, rect)
                draw.ellipse([px - 3, py - 3, px + 3, py + 3], fill=LIDAR)
            for p in pts[::12]:
                draw.line([world_to_px(drone, rect), world_to_px(p, rect)], fill=(147, 197, 253), width=1)
        draw.text((rect[0] + 16, rect[1] + rect[3] - 54), "LiDAR points define wall/door-frame occupancy", fill=LIDAR, font=FONT_BODY)
    elif scenario.removed_sensor == "depth":
        cone = [world_to_px(drone, rect), world_to_px((4.25, -0.35), rect), world_to_px((5.15, 1.10), rect)]
        draw.polygon(cone, outline=DEPTH, fill=None)
        for obs in scenario.full_obstacles:
            if obs.sensor == "depth":
                draw_rect_obstacle(draw, obs, rect, outline=DEPTH, fill_alpha=0.35)
        x, y, w, _ = rect
        strip_x = x + 18
        strip_y = y + rect[3] - 78
        for i in range(120):
            alpha = i / 119
            near = 0.5 + 0.5 * math.sin(alpha * math.pi * 3.0)
            col = (int(85 + 145 * near), 24, int(150 + 70 * (1.0 - near)))
            draw.rectangle([strip_x + i * 3, strip_y, strip_x + i * 3 + 3, strip_y + 28], fill=col)
        draw.text((strip_x, strip_y + 34), "Depth strip: bright patch = near obstacle after door", fill=DEPTH, font=FONT_SMALL)
    else:
        person_y = -0.20 + 1.90 * progress
        person = np.asarray([5.45, person_y])
        for obs in scenario.full_obstacles:
            if obs.sensor == "radar":
                draw_rect_obstacle(draw, obs, rect, outline=RADAR, fill_alpha=0.25)
        px, py = world_to_px(person, rect)
        draw.ellipse([px - 13, py - 13, px + 13, py + 13], fill=RADAR, outline=(124, 45, 18), width=2)
        draw.line([world_to_px((5.18, -0.35), rect), world_to_px((5.18, 1.55), rect)], fill=RADAR, width=3)
        for k in range(4):
            draw.arc([px - 28 - k * 9, py - 28 - k * 9, px + 28 + k * 9, py + 28 + k * 9], 300, 60, fill=RADAR, width=2)
        draw.text((rect[0] + 16, rect[1] + rect[3] - 54), "Radar Doppler predicts a moving-person swept zone", fill=RADAR, font=FONT_BODY)


def draw_sensor_chips(draw: ImageDraw.ImageDraw, x: int, y: int, removed: str) -> None:
    chips = [("LiDAR", "lidar", LIDAR), ("Depth", "depth", DEPTH), ("Radar", "radar", RADAR)]
    for i, (label, key, color) in enumerate(chips):
        x0 = x + i * 126
        enabled = key != removed
        fill = color if enabled else (226, 232, 240)
        text = "ON" if enabled else "OFF"
        draw.rounded_rectangle([x0, y, x0 + 112, y + 34], radius=10, fill=fill, outline=(148, 163, 184), width=2)
        draw.text((x0 + 10, y + 8), f"{label} {text}", fill=(255, 255, 255) if enabled else MUTED, font=FONT_SMALL)


def render_frame(scenario: Scenario, full: PlanResult, ablated: PlanResult, progress: float, frame: int, fps: int, size: tuple[int, int]) -> Image.Image:
    width, height = size
    img = Image.new("RGB", (width, height), (241, 245, 249))
    draw = ImageDraw.Draw(img)

    draw.text((30, 20), "Sensor-input ablation: how perception changes the planner decision", fill=TEXT, font=FONT_TITLE)
    draw.text((30, 60), scenario.title, fill=MUTED, font=FONT_BODY)
    draw_sensor_chips(draw, width - 430, 24, scenario.removed_sensor)

    top = 102
    panel_h = 720
    margin = 28
    gap = 22
    panel_w = (width - 2 * margin - 2 * gap) // 3
    scene_rect = (margin, top, panel_w, panel_h)
    full_rect = (margin + panel_w + gap, top, panel_w, panel_h)
    ablate_rect = (margin + 2 * (panel_w + gap), top, panel_w, panel_h)

    draw_map_frame(draw, scene_rect, "1. Sensor evidence / true scene")
    draw_map_frame(draw, full_rect, "2. Full fusion -> safe map")
    draw_map_frame(draw, ablate_rect, f"3. Missing {scenario.removed_sensor} -> wrong map")

    draw_sensor_evidence(draw, scenario, full.path, progress, scene_rect)

    for obs in scenario.full_obstacles:
        draw_rect_obstacle(draw, obs, full_rect, outline=obs.color, fill_alpha=0.38)
    draw_path(draw, full.path, full_rect, FULL)
    draw_drone(draw, full.path[min(len(full.path) - 1, int(progress * (len(full.path) - 1)))], full_rect)

    for obs in scenario.ablated_obstacles:
        draw_rect_obstacle(draw, obs, ablate_rect, outline=obs.color, fill_alpha=0.38)
    missing = [obs for obs in scenario.true_obstacles if obs not in scenario.ablated_obstacles]
    for obs in missing:
        draw_rect_obstacle(draw, obs, ablate_rect, outline=(148, 163, 184), fill_alpha=0.88)
        bx, by = world_to_px(obs.center, ablate_rect)
        draw.text((bx - 32, by - 8), "hidden", fill=(100, 116, 139), font=FONT_TINY)
    draw_path(draw, ablated.path, ablate_rect, ABLATE)
    draw_drone(draw, ablated.path[min(len(ablated.path) - 1, int(progress * (len(ablated.path) - 1)))], ablate_rect, color=(252, 165, 165))
    draw_collision_mark(draw, ablated.path, scenario.true_obstacles, ablate_rect)
    draw_panel_title(draw, scene_rect, "1. Sensor evidence / true scene")
    draw_panel_title(draw, full_rect, "2. Full fusion -> safe map")
    draw_panel_title(draw, ablate_rect, f"3. Missing {scenario.removed_sensor} -> wrong map")

    footer_y = top + panel_h + 26
    draw.rectangle([margin, footer_y, width - margin, height - 28], fill=(255, 255, 255), outline=(203, 213, 225), width=2)
    draw.text((margin + 18, footer_y + 14), "Decision explanation", fill=TEXT, font=FONT_H2)
    wrapped = textwrap_lines(scenario.sensor_story + " " + scenario.failure_story, 82)
    for i, line in enumerate(wrapped[:3]):
        draw.text((margin + 18, footer_y + 48 + i * 24), line, fill=MUTED, font=FONT_BODY)

    metric_x = width - 610
    draw.text((metric_x, footer_y + 14), "Planner comparison", fill=TEXT, font=FONT_H2)
    rows = [
        ("Full fusion", full.min_clearance_m, full.path_length_m, full.collision, FULL),
        (f"No {scenario.removed_sensor}", ablated.min_clearance_m, ablated.path_length_m, ablated.collision, ABLATE),
    ]
    for i, (label, clearance, length, collision, color) in enumerate(rows):
        yy = footer_y + 52 + i * 38
        draw.rounded_rectangle([metric_x, yy, metric_x + 150, yy + 28], radius=9, fill=color)
        draw.text((metric_x + 12, yy + 6), label, fill=(255, 255, 255), font=FONT_SMALL)
        status = "COLLISION" if collision else "CLEAR"
        status_color = ABLATE if collision else FULL
        draw.text((metric_x + 174, yy + 5), f"{status}", fill=status_color, font=FONT_BODY)
        draw.text((metric_x + 300, yy + 5), f"clearance={clearance:.2f} m", fill=MUTED, font=FONT_BODY)
        draw.text((metric_x + 470, yy + 5), f"L={length:.2f} m", fill=MUTED, font=FONT_BODY)

    t = frame / max(fps, 1)
    draw.text((width - 160, height - 52), f"t={t:04.1f}s", fill=MUTED, font=FONT_BODY)
    return img


def textwrap_lines(text: str, width: int) -> list[str]:
    words = text.split()
    lines: list[str] = []
    line: list[str] = []
    for word in words:
        trial = " ".join(line + [word])
        if len(trial) > width and line:
            lines.append(" ".join(line))
            line = [word]
        else:
            line.append(word)
    if line:
        lines.append(" ".join(line))
    return lines


def write_video(frame_dir: Path, output: Path, fps: int) -> None:
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg is required")
    output.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-framerate",
            str(fps),
            "-i",
            str(frame_dir / "frame_%04d.png"),
            "-vf",
            "scale=trunc(iw/2)*2:trunc(ih/2)*2",
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-crf",
            "18",
            str(output),
        ],
        check=True,
    )


def main() -> int:
    args = parse_args()
    scens = scenarios()
    plans: dict[str, tuple[PlanResult, PlanResult]] = {}
    rows: list[dict[str, object]] = []
    for scenario in scens:
        full_path = astar_path(scenario.full_obstacles)
        ablated_path = astar_path(scenario.ablated_obstacles)
        full = evaluate_path(full_path, scenario.true_obstacles)
        ablated = evaluate_path(ablated_path, scenario.true_obstacles)
        plans[scenario.key] = (full, ablated)
        rows.append(
            {
                "scenario": scenario.key,
                "removed_sensor": scenario.removed_sensor,
                "full_min_clearance_m": f"{full.min_clearance_m:.4f}",
                "full_collision": int(full.collision),
                "full_path_length_m": f"{full.path_length_m:.4f}",
                "ablated_min_clearance_m": f"{ablated.min_clearance_m:.4f}",
                "ablated_collision": int(ablated.collision),
                "ablated_path_length_m": f"{ablated.path_length_m:.4f}",
            }
        )

    phase_frames = int(round(float(args.phase_duration_s) * int(args.fps)))
    total_frames = phase_frames * len(scens)
    preview_frame = total_frames // 2
    args.preview.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="sensor_ablation_frames_") as tmp:
        frame_dir = Path(tmp)
        for frame in range(total_frames):
            phase = min(len(scens) - 1, frame // phase_frames)
            local = frame - phase * phase_frames
            progress = local / max(phase_frames - 1, 1)
            scenario = scens[phase]
            full, ablated = plans[scenario.key]
            img = render_frame(scenario, full, ablated, progress, frame, int(args.fps), (int(args.width), int(args.height)))
            img.save(frame_dir / f"frame_{frame:04d}.png")
            if frame == preview_frame:
                img.save(args.preview)
        write_video(frame_dir, args.output, int(args.fps))

    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    with args.metrics.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    lines = [
        "# Sensor Ablation Planner Decision Demo",
        "",
        "This artifact compares the planner decision with full sensor fusion against the same planner when one sensor input is removed.",
        "",
        f"- Video: `{args.output}`",
        f"- Preview: `{args.preview}`",
        f"- Metrics: `{args.metrics}`",
        "",
        "| Removed sensor | Full collision | Ablated collision | Full clearance m | Ablated clearance m |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['removed_sensor']} | {row['full_collision']} | {row['ablated_collision']} | "
            f"{row['full_min_clearance_m']} | {row['ablated_min_clearance_m']} |"
        )
    lines.extend(
        [
            "",
            "Scope note: the sensor panels are simulated sensor-derived obstacle maps. This is an explanatory ablation video, not a raw ROS/Gazebo screen recording.",
        ]
    )
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
