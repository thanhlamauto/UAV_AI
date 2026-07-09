#!/usr/bin/env python3
"""Render a multi-panel sensor dashboard for the ROS2/Gazebo UAV demo.

The output is a qualitative video for reports and mentor demos. It does not
require ROS at render time; it uses the same lightweight geometry, planners,
and depth projection helpers as the ROS2 nodes.
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from render_ros2_costmap_demo_video import (
    _draw_circle_px,
    _draw_grid_scene,
    _draw_line,
    _draw_polyline,
    _draw_rect,
    _draw_text,
    _import_planners,
    _make_demo_grid,
    _resample_path,
)


@dataclass(frozen=True)
class Panel:
    x: int
    y: int
    w: int
    h: int
    title: str


@dataclass(frozen=True)
class Obstacle:
    x: float
    y: float
    radius: float
    height: float = 2.2


class WorldProjector:
    def __init__(self, panel: Panel, x_min: float, x_max: float, y_min: float, y_max: float, pad: int = 38) -> None:
        self.panel = panel
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.left = panel.x + pad
        self.right = panel.x + panel.w - pad
        self.top = panel.y + 58
        self.bottom = panel.y + panel.h - pad

    def world(self, x: float, y: float) -> tuple[int, int]:
        px = self.left + int(round((x - self.x_min) / max(self.x_max - self.x_min, 1e-6) * (self.right - self.left)))
        py = self.bottom - int(round((y - self.y_min) / max(self.y_max - self.y_min, 1e-6) * (self.bottom - self.top)))
        return px, py

    def radius_px(self, radius_m: float) -> int:
        sx = (self.right - self.left) / max(self.x_max - self.x_min, 1e-6)
        sy = (self.bottom - self.top) / max(self.y_max - self.y_min, 1e-6)
        return max(2, int(round(radius_m * 0.5 * (sx + sy))))


def _draw_panel(img: np.ndarray, panel: Panel) -> None:
    _draw_rect(img, panel.x, panel.y, panel.x + panel.w, panel.y + panel.h, (248, 250, 252))
    _draw_rect(img, panel.x, panel.y, panel.x + panel.w, panel.y + 42, (226, 232, 240))
    _draw_rect(img, panel.x, panel.y, panel.x + panel.w, panel.y + 2, (51, 65, 85))
    _draw_rect(img, panel.x, panel.y + panel.h - 2, panel.x + panel.w, panel.y + panel.h, (51, 65, 85))
    _draw_rect(img, panel.x, panel.y, panel.x + 2, panel.y + panel.h, (51, 65, 85))
    _draw_rect(img, panel.x + panel.w - 2, panel.y, panel.x + panel.w, panel.y + panel.h, (51, 65, 85))
    _draw_text(img, panel.x + 16, panel.y + 14, panel.title, (15, 23, 42), 2)


def _draw_world_grid(img: np.ndarray, proj: WorldProjector) -> None:
    _draw_rect(img, proj.left, proj.top, proj.right, proj.bottom, (241, 245, 249))
    for x in np.arange(math.ceil(proj.x_min), math.floor(proj.x_max) + 1):
        _draw_line(img, proj.world(float(x), proj.y_min), proj.world(float(x), proj.y_max), (220, 226, 235), 1)
    for y in np.arange(math.ceil(proj.y_min), math.floor(proj.y_max) + 1):
        _draw_line(img, proj.world(proj.x_min, float(y)), proj.world(proj.x_max, float(y)), (220, 226, 235), 1)


def _draw_uav(img: np.ndarray, proj: WorldProjector, pose: np.ndarray, yaw: float, color: tuple[int, int, int]) -> None:
    px, py = proj.world(float(pose[0]), float(pose[1]))
    _draw_circle_px(img, px, py, 12, color)
    hx, hy = proj.world(float(pose[0] + 0.42 * math.cos(yaw)), float(pose[1] + 0.42 * math.sin(yaw)))
    _draw_line(img, (px, py), (hx, hy), (15, 23, 42), 3)


def _draw_obstacles(img: np.ndarray, proj: WorldProjector, obstacles: list[Obstacle]) -> None:
    for obstacle in obstacles:
        px, py = proj.world(obstacle.x, obstacle.y)
        _draw_circle_px(img, px, py, proj.radius_px(obstacle.radius + 0.28), (255, 226, 199))
        _draw_circle_px(img, px, py, proj.radius_px(obstacle.radius), (222, 84, 62))


def _path_yaws(points: np.ndarray) -> np.ndarray:
    yaws = np.zeros(len(points), dtype=float)
    for idx in range(len(points)):
        if idx < len(points) - 1:
            delta = points[idx + 1] - points[idx]
        else:
            delta = points[idx] - points[idx - 1]
        yaws[idx] = math.atan2(float(delta[1]), float(delta[0])) if np.linalg.norm(delta) > 1e-6 else 0.0
    return yaws


def _make_point_cloud(obstacles: list[Obstacle], n_per_obstacle: int = 500) -> np.ndarray:
    points: list[tuple[float, float, float]] = []
    for obs_idx, obstacle in enumerate(obstacles):
        for i in range(n_per_obstacle):
            theta = 2.0 * math.pi * ((i * 37 + obs_idx * 19) % n_per_obstacle) / max(n_per_obstacle, 1)
            z = obstacle.height * ((i * 53 + obs_idx * 11) % n_per_obstacle) / max(n_per_obstacle - 1, 1)
            r = obstacle.radius * (0.88 + 0.12 * math.sin(i * 0.37))
            points.append((obstacle.x + r * math.cos(theta), obstacle.y + r * math.sin(theta), z))
    return np.asarray(points, dtype=float)


def _visible_points(points: np.ndarray, pose: np.ndarray, yaw: float, fov: float, max_range: float) -> np.ndarray:
    rel = points[:, :2] - pose[None, :2]
    rng = np.linalg.norm(rel, axis=1)
    angles = np.arctan2(rel[:, 1], rel[:, 0]) - yaw
    angles = (angles + math.pi) % (2 * math.pi) - math.pi
    return (rng < max_range) & (np.abs(angles) < fov * 0.5)


def _draw_pointcloud_panel(
    img: np.ndarray,
    panel: Panel,
    points: np.ndarray,
    pose: np.ndarray,
    yaw: float,
    path: np.ndarray,
) -> None:
    proj = WorldProjector(panel, -1.0, 7.0, -4.0, 4.0)
    _draw_world_grid(img, proj)
    visible = _visible_points(points, pose, yaw, math.radians(120.0), 6.5)
    for idx, point in enumerate(points[::2]):
        src_idx = idx * 2
        px, py = proj.world(float(point[0]), float(point[1]))
        if visible[src_idx]:
            z_norm = float(np.clip(point[2] / 2.2, 0.0, 1.0))
            color = (int(28 + 160 * z_norm), int(110 + 80 * (1.0 - z_norm)), int(180 + 55 * z_norm))
            _draw_circle_px(img, px, py, 2, color)
        else:
            _draw_circle_px(img, px, py, 1, (190, 198, 211))
    _draw_polyline(img, [proj.world(float(x), float(y)) for x, y in path], (37, 99, 235), 2)
    _draw_uav(img, proj, pose, yaw, (15, 118, 110))
    _draw_text(img, panel.x + 18, panel.y + panel.h - 30, f"VISIBLE {int(visible.sum())} PTS", (51, 65, 85), 2)


def _ray_circle_intersection(origin: np.ndarray, angle: float, obstacle: Obstacle, max_range: float) -> float | None:
    direction = np.asarray([math.cos(angle), math.sin(angle)], dtype=float)
    center = np.asarray([obstacle.x, obstacle.y], dtype=float)
    oc = origin - center
    b = 2.0 * float(np.dot(direction, oc))
    c = float(np.dot(oc, oc) - obstacle.radius * obstacle.radius)
    disc = b * b - 4.0 * c
    if disc < 0:
        return None
    root = math.sqrt(disc)
    candidates = [(-b - root) / 2.0, (-b + root) / 2.0]
    valid = [value for value in candidates if 0.08 <= value <= max_range]
    return min(valid) if valid else None


def _simulate_lidar(pose: np.ndarray, yaw: float, obstacles: list[Obstacle], n: int = 181, max_range: float = 8.0) -> tuple[np.ndarray, np.ndarray]:
    rel_angles = np.linspace(-math.pi / 2.0, math.pi / 2.0, n)
    ranges = np.full(n, max_range, dtype=float)
    for idx, rel_angle in enumerate(rel_angles):
        angle = yaw + float(rel_angle)
        hits = [_ray_circle_intersection(pose, angle, obstacle, max_range) for obstacle in obstacles]
        finite = [hit for hit in hits if hit is not None]
        if finite:
            ranges[idx] = min(finite)
    return rel_angles, ranges


def _draw_lidar_panel(img: np.ndarray, panel: Panel, pose: np.ndarray, yaw: float, obstacles: list[Obstacle]) -> None:
    cx = panel.x + panel.w // 2
    cy = panel.y + panel.h - 62
    radius_px = min(panel.w // 2 - 52, panel.h - 122)
    _draw_circle_px(img, cx, cy, 5, (15, 118, 110))
    for frac in [0.25, 0.5, 0.75, 1.0]:
        r = int(radius_px * frac)
        for a in np.linspace(-math.pi / 2, math.pi / 2, 80):
            x = cx + int(round(r * math.sin(a)))
            y = cy - int(round(r * math.cos(a)))
            _draw_circle_px(img, x, y, 1, (222, 226, 235))
    rel_angles, ranges = _simulate_lidar(pose, yaw, obstacles)
    hit_count = int((ranges < 7.99).sum())
    prev: tuple[int, int] | None = None
    for rel_angle, rng in zip(rel_angles, ranges):
        x = cx + int(round((rng / 8.0) * radius_px * math.sin(float(rel_angle))))
        y = cy - int(round((rng / 8.0) * radius_px * math.cos(float(rel_angle))))
        color = (222, 84, 62) if rng < 7.99 else (148, 163, 184)
        _draw_circle_px(img, x, y, 2 if rng < 7.99 else 1, color)
        if prev is not None and rng < 7.99:
            _draw_line(img, prev, (x, y), (222, 84, 62), 1)
        prev = (x, y)
    _draw_line(img, (cx, cy), (cx - radius_px, cy), (203, 213, 225), 1)
    _draw_line(img, (cx, cy), (cx + radius_px, cy), (203, 213, 225), 1)
    _draw_text(img, panel.x + 18, panel.y + panel.h - 30, f"HITS {hit_count} / {len(ranges)}", (51, 65, 85), 2)


def _make_depth_image(
    pose: np.ndarray,
    yaw: float,
    obstacles: list[Obstacle],
    width: int = 160,
    height: int = 96,
    fov_deg: float = 74.0,
) -> np.ndarray:
    depth = np.full((height, width), 7.5, dtype=np.float32)
    fov = math.radians(fov_deg)
    xs = np.linspace(0, math.tau, width, dtype=np.float32)
    depth += 0.04 * np.sin(xs[None, :] + float(pose[0]) * 0.7)
    for obstacle in obstacles:
        dx = obstacle.x - float(pose[0])
        dy = obstacle.y - float(pose[1])
        distance_center = math.hypot(dx, dy)
        angle = math.atan2(dy, dx) - yaw
        angle = (angle + math.pi) % (2 * math.pi) - math.pi
        if abs(angle) > fov * 0.65 or distance_center > 8.0:
            continue
        distance = max(0.15, distance_center - obstacle.radius)
        center_col = int(round((angle / fov + 0.5) * (width - 1)))
        angular_radius = math.atan2(obstacle.radius, max(distance, 1e-6))
        half_width = max(3, int(round(angular_radius / fov * width)))
        center_row = int(height * (0.58 + 0.04 * math.sin(angle)))
        half_height = max(7, int(height * 0.24 / max(distance, 0.7)))
        for row in range(max(0, center_row - half_height), min(height, center_row + half_height + 1)):
            for col in range(max(0, center_col - half_width), min(width, center_col + half_width + 1)):
                rx = (col - center_col) / max(half_width, 1)
                ry = (row - center_row) / max(half_height, 1)
                if rx * rx + ry * ry <= 1.0:
                    depth[row, col] = min(depth[row, col], distance)
    return depth


def _depth_to_rgb(depth: np.ndarray) -> np.ndarray:
    norm = np.clip((depth - 0.2) / 7.3, 0.0, 1.0)
    near = 1.0 - norm
    rgb = np.zeros((*depth.shape, 3), dtype=np.uint8)
    rgb[..., 0] = np.clip(35 + 210 * near, 0, 255).astype(np.uint8)
    rgb[..., 1] = np.clip(70 + 130 * norm + 40 * near, 0, 255).astype(np.uint8)
    rgb[..., 2] = np.clip(95 + 135 * norm, 0, 255).astype(np.uint8)
    return rgb


def _draw_image_nearest(img: np.ndarray, panel: Panel, rgb: np.ndarray, pad: int = 24) -> None:
    x0 = panel.x + pad
    y0 = panel.y + 58
    x1 = panel.x + panel.w - pad
    y1 = panel.y + panel.h - 46
    out_h = max(1, y1 - y0)
    out_w = max(1, x1 - x0)
    src_h, src_w = rgb.shape[:2]
    ys = (np.linspace(0, src_h - 1, out_h)).astype(int)
    xs = (np.linspace(0, src_w - 1, out_w)).astype(int)
    img[y0:y1, x0:x1] = rgb[ys[:, None], xs[None, :]]
    _draw_rect(img, x0, y0, x1, y0 + 2, (51, 65, 85))
    _draw_rect(img, x0, y1 - 2, x1, y1, (51, 65, 85))
    _draw_rect(img, x0, y0, x0 + 2, y1, (51, 65, 85))
    _draw_rect(img, x1 - 2, y0, x1, y1, (51, 65, 85))


def _draw_depth_panel(img: np.ndarray, panel: Panel, depth: np.ndarray) -> None:
    rgb = _depth_to_rgb(depth)
    _draw_image_nearest(img, panel, rgb)
    finite = depth[np.isfinite(depth)]
    _draw_text(
        img,
        panel.x + 18,
        panel.y + panel.h - 30,
        f"MIN {finite.min():.1f}M  P10 {np.percentile(finite, 10):.1f}M",
        (51, 65, 85),
        2,
    )


def _draw_depth_costmap_panel(img: np.ndarray, panel: Panel, depth_grid: np.ndarray, pose: np.ndarray, yaw: float, path: np.ndarray) -> None:
    proj = WorldProjector(panel, -1.0, 7.0, -4.0, 4.0)
    _draw_world_grid(img, proj)
    occupied = np.argwhere(depth_grid >= 50)
    for row, col in occupied[:: max(1, len(occupied) // 900)]:
        x = -1.0 + float(col) * 0.05
        y = -4.0 + float(row) * 0.05
        px, py = proj.world(x, y)
        _draw_rect(img, px - 1, py - 1, px + 2, py + 2, (124, 58, 237))
    _draw_polyline(img, [proj.world(float(x), float(y)) for x, y in path], (37, 99, 235), 2)
    _draw_uav(img, proj, pose, yaw, (15, 118, 110))
    _draw_text(img, panel.x + 18, panel.y + panel.h - 30, f"OCC {len(occupied)} CELLS", (51, 65, 85), 2)


def _draw_flight_panel(
    img: np.ndarray,
    panel: Panel,
    obstacles: list[Obstacle],
    path: np.ndarray,
    sampled: np.ndarray,
    idx: int,
    yaw: float,
) -> None:
    proj = WorldProjector(panel, -1.0, 7.0, -4.0, 4.0)
    _draw_world_grid(img, proj)
    _draw_obstacles(img, proj, obstacles)
    path_px = [proj.world(float(x), float(y)) for x, y in path]
    _draw_polyline(img, path_px, (37, 99, 235), 3)
    trail_px = [proj.world(float(x), float(y)) for x, y in sampled[: idx + 1]]
    if len(trail_px) > 1:
        _draw_polyline(img, trail_px, (15, 23, 42), 4)
    _draw_circle_px(img, *proj.world(float(path[0, 0]), float(path[0, 1])), 9, (34, 197, 94))
    _draw_circle_px(img, *proj.world(float(path[-1, 0]), float(path[-1, 1])), 11, (79, 70, 229))
    _draw_uav(img, proj, sampled[idx], yaw, (15, 118, 110))
    length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    _draw_text(img, panel.x + 18, panel.y + panel.h - 30, f"PATH {length:.1f}M", (51, 65, 85), 2)


def _draw_planner_panel(
    img: np.ndarray,
    panel: Panel,
    grid: np.ndarray,
    inflated: np.ndarray,
    spec: object,
    path: np.ndarray,
    sampled: np.ndarray,
    idx: int,
    yaw: float,
) -> None:
    class PanelSpecProjector:
        def __init__(self, panel: Panel, spec: object) -> None:
            self.spec = spec
            self.width = panel.x + panel.w
            self.height = panel.y + panel.h
            self.left = panel.x + 36
            self.right = 36
            self.top = panel.y + 58
            self.bottom = 42
            self.x_min = spec.origin_x
            self.x_max = spec.origin_x + (spec.width - 1) * spec.resolution
            self.y_min = spec.origin_y
            self.y_max = spec.origin_y + (spec.height - 1) * spec.resolution
            self.map_w = self.width - self.left - self.right
            self.map_h = self.height - self.top - self.bottom

        def world(self, x: float, y: float) -> tuple[int, int]:
            px = self.left + int(round((x - self.x_min) / max(self.x_max - self.x_min, 1e-6) * self.map_w))
            py = self.height - self.bottom - int(
                round((y - self.y_min) / max(self.y_max - self.y_min, 1e-6) * self.map_h)
            )
            return px, py

        def cell_rect(self, row: int, col: int) -> tuple[int, int, int, int]:
            x0, y0 = self.world(spec.origin_x + col * spec.resolution, spec.origin_y + row * spec.resolution)
            x1, y1 = self.world(spec.origin_x + (col + 1) * spec.resolution, spec.origin_y + (row + 1) * spec.resolution)
            return x0, y0, x1, y1

    proj = PanelSpecProjector(panel, spec)
    _draw_grid_scene(img, grid, inflated, spec, proj)
    _draw_polyline(img, [proj.world(float(x), float(y)) for x, y in path], (37, 99, 235), 3)
    _draw_uav(img, proj, sampled[idx], yaw, (15, 118, 110))
    _draw_text(img, panel.x + 18, panel.y + panel.h - 30, "A* COSTMAP OUTPUT", (51, 65, 85), 2)


def _render(args: argparse.Namespace) -> None:
    GridSpec, PlannerConfig, inflate_grid, plan_path = _import_planners()
    repo_root = Path(__file__).resolve().parents[1]
    ros_pkg = repo_root / "ros2_ws" / "src" / "uav_oda_ros2_demo"
    sys.path.insert(0, str(ros_pkg))
    from uav_oda_ros2_demo.costmap_converters import DepthProjectionConfig, depth_image_to_grid

    grid, spec, start, goal = _make_demo_grid()
    obstacles = [Obstacle(2.0, 1.0, 0.35), Obstacle(4.2, -0.8, 0.45)]
    config = PlannerConfig(robot_radius_m=0.15, safety_distance_m=0.20, seed=11)
    path = plan_path(args.planner, grid, spec, start, goal, config)
    inflated = inflate_grid(grid, spec, config.inflation_radius_m, config.occupied_threshold)
    n_frames = max(2, int(round(args.fps * args.duration_s)))
    sampled = _resample_path(path, n_frames)
    yaws = _path_yaws(sampled)
    point_cloud = _make_point_cloud(obstacles)

    margin = 22
    header_h = 58
    gap = 18
    panel_w = (args.width - 2 * margin - 2 * gap) // 3
    panel_h = (args.height - header_h - 2 * margin - gap) // 2
    panels = [
        Panel(margin, header_h + margin, panel_w, panel_h, "UAV FLIGHT"),
        Panel(margin + panel_w + gap, header_h + margin, panel_w, panel_h, "POINTCLOUD XYZ"),
        Panel(margin + 2 * (panel_w + gap), header_h + margin, panel_w, panel_h, "GAZEBO LIDAR"),
        Panel(margin, header_h + margin + panel_h + gap, panel_w, panel_h, "DEPTH IMAGE"),
        Panel(margin + panel_w + gap, header_h + margin + panel_h + gap, panel_w, panel_h, "DEPTH COSTMAP"),
        Panel(margin + 2 * (panel_w + gap), header_h + margin + panel_h + gap, panel_w, panel_h, "PLANNER OUTPUT"),
    ]

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        ffmpeg,
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{args.width}x{args.height}",
        "-r",
        str(args.fps),
        "-i",
        "-",
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(args.output),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None

    try:
        for idx in range(n_frames):
            pose = sampled[idx]
            yaw = float(yaws[idx])
            img = np.full((args.height, args.width, 3), (255, 255, 255), dtype=np.uint8)
            _draw_rect(img, 0, 0, args.width - 1, header_h - 1, (226, 232, 240))
            _draw_text(img, 26, 21, "ROS2 GAZEBO SENSOR DASHBOARD", (15, 23, 42), 2)
            _draw_text(img, args.width - 360, 21, f"PLANNER {args.planner}", (51, 65, 85), 2)
            for panel in panels:
                _draw_panel(img, panel)

            _draw_flight_panel(img, panels[0], obstacles, path, sampled, idx, yaw)
            _draw_pointcloud_panel(img, panels[1], point_cloud, pose, yaw, path)
            _draw_lidar_panel(img, panels[2], pose, yaw, obstacles)

            depth = _make_depth_image(pose, yaw, obstacles)
            _draw_depth_panel(img, panels[3], depth)
            depth_config = DepthProjectionConfig(
                resolution_m=0.05,
                origin_x=-1.0,
                origin_y=-4.0,
                width_m=8.0,
                height_m=8.0,
                camera_x=float(pose[0]),
                camera_y=float(pose[1]),
                camera_yaw_rad=yaw,
                horizontal_fov_deg=74.0,
                row_min_fraction=0.30,
                row_max_fraction=0.85,
                sample_stride_px=4,
                hit_dilation_cells=2,
            )
            depth_grid, _, _ = depth_image_to_grid(depth, "32FC1", depth_config)
            _draw_depth_costmap_panel(img, panels[4], depth_grid, pose, yaw, path)
            _draw_planner_panel(img, panels[5], grid, inflated, spec, path, sampled, idx, yaw)

            progress = int((idx + 1) / n_frames * (args.width - 2 * margin))
            _draw_rect(img, margin, args.height - 16, args.width - margin, args.height - 10, (226, 232, 240))
            _draw_rect(img, margin, args.height - 16, margin + progress, args.height - 10, (15, 118, 110))
            _draw_text(img, args.width - 170, args.height - 38, f"T {idx / args.fps:04.1f}S", (51, 65, 85), 2)
            proc.stdin.write(img.tobytes())
    finally:
        proc.stdin.close()
    stderr = proc.stderr.read() if proc.stderr is not None else b""
    if proc.stdout is not None:
        proc.stdout.read()
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg failed with code {proc.returncode}: {stderr.decode(errors='replace')[-2000:]}")


def _write_summary(output: Path, video: Path) -> None:
    lines = [
        "# ROS2 Sensor Dashboard Video",
        "",
        f"Video: `{video}`",
        "",
        "| Panel | What it shows | Source |",
        "|---|---|---|",
        "| UAV FLIGHT | UAV marker moving along A* path with obstacle/safety footprints | Gazebo demo geometry + planner helper |",
        "| POINTCLOUD XYZ | Synthetic PointCloud2-style obstacle points visible from the moving UAV | Same geometry as ROS2 synthetic point cloud publisher |",
        "| GAZEBO LIDAR | 180-degree LiDAR scan fan and obstacle hits | Same range model as Gazebo LaserScan runtime evidence |",
        "| DEPTH IMAGE | Metric depth image changing with UAV pose | Same depth projection convention as `depth_image_costmap` |",
        "| DEPTH COSTMAP | Depth-derived occupied cells in world coordinates | `depth_image_to_grid` helper |",
        "| PLANNER OUTPUT | Inflated occupancy grid and A* path consumed by follower | `costmap_planner` helper |",
        "",
        "This qualitative video complements the ROS2 runtime evidence folders. It is rendered offline so it can be viewed without RViz/Gazebo GUI, while preserving the same perception-to-costmap-to-planner story.",
        "",
    ]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", default="astar", choices=["astar", "rrt", "mppi"])
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/ros2_sensor_dashboard_flight_astar.mp4"))
    parser.add_argument("--summary", type=Path, default=Path("outputs/ros2_sensor_dashboard_video.md"))
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    args = parser.parse_args()

    _render(args)
    _write_summary(args.summary, args.output)
    print(f"Wrote {args.output}")
    print(f"Wrote {args.summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
