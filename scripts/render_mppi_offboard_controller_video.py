#!/usr/bin/env python3
"""Render a server-side MPPI Offboard controller evidence video.

This is a lightweight closed-loop dynamics visualization for the upgraded
controller path:

    fused occupancy map -> MPPI local controller -> velocity/acceleration setpoint

It intentionally avoids ROS/PX4 imports so it can run on the rented GPU server
even before the PX4 SITL workspace is installed.  The runtime claim is therefore
"controller evidence", not raw PX4/Gazebo screen recording.
"""

from __future__ import annotations

import argparse
import csv
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
ROS_PKG = REPO_ROOT / "ros2_ws" / "src" / "uav_oda_ros2_demo"
sys.path.insert(0, str(ROS_PKG))

from uav_oda_ros2_demo.grid_planners import GridSpec
from uav_oda_ros2_demo.mppi_local_controller import MPPIControllerConfig, mppi_velocity_command


@dataclass(frozen=True)
class Obstacle:
    name: str
    sensor: str
    center: tuple[float, float]
    size: tuple[float, float]
    color: tuple[int, int, int]


@dataclass(frozen=True)
class RuntimeState:
    position: np.ndarray
    velocity: np.ndarray
    target_idx: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/mppi_offboard_controller_setpoint_demo.mp4"))
    parser.add_argument("--preview", type=Path, default=Path("outputs/figures/level3_video_preview/mppi_offboard_controller_midframe.png"))
    parser.add_argument("--metrics", type=Path, default=Path("outputs/tables/mppi_offboard_controller_setpoint_metrics.csv"))
    parser.add_argument("--summary", type=Path, default=Path("outputs/mppi_offboard_controller_setpoint_summary.md"))
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=int, default=16)
    parser.add_argument("--frames", type=int, default=192)
    parser.add_argument("--rollouts", type=int, default=192)
    parser.add_argument("--horizon", type=int, default=22)
    parser.add_argument("--sim-dt", type=float, default=0.10)
    parser.add_argument("--seed", type=int, default=41)
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

TEXT = (15, 23, 42)
MUTED = (71, 85, 105)
GRID = (226, 232, 240)
WALL = (100, 116, 139)
LIDAR = (37, 99, 235)
DEPTH = (168, 85, 247)
RADAR = (249, 115, 22)
FUSION = (5, 150, 105)
MPPI = (14, 165, 233)
DRONE = (245, 158, 11)
DANGER = (220, 38, 38)
WARNING = (245, 158, 11)
SAFE = (22, 163, 74)


def make_spec() -> GridSpec:
    return GridSpec(width=60, height=44, resolution=0.15, origin_x=0.0, origin_y=-3.15)


def obstacles_for_time(t: float) -> list[Obstacle]:
    obstacles = [
        Obstacle("left boundary", "lidar", (4.2, -2.95), (8.3, 0.18), WALL),
        Obstacle("right boundary", "lidar", (4.2, 2.95), (8.3, 0.18), WALL),
        Obstacle("partition lower wall", "lidar", (3.55, -1.78), (0.22, 2.34), WALL),
        Obstacle("partition upper wall", "lidar", (3.55, 1.78), (0.22, 2.34), WALL),
        Obstacle("door left frame", "lidar", (3.55, -0.66), (0.30, 0.13), LIDAR),
        Obstacle("door right frame", "lidar", (3.55, 0.66), (0.30, 0.13), LIDAR),
        Obstacle("depth glass panel", "depth", (4.72, 0.52), (0.70, 0.92), DEPTH),
        Obstacle("depth low box", "depth", (5.28, -0.48), (0.82, 0.58), DEPTH),
        Obstacle("goal-side shelf", "lidar", (6.92, 1.08), (0.74, 0.72), WALL),
    ]
    if t >= 4.2:
        sweep_y = 0.20 + 0.72 * math.sin(min(1.0, (t - 4.2) / 4.0) * math.pi)
        obstacles.append(Obstacle("radar moving-person sweep", "radar", (5.82, sweep_y), (0.72, 1.28), RADAR))
    return obstacles


def mark_obstacle(grid: np.ndarray, spec: GridSpec, obstacle: Obstacle, inflate: float = 0.0) -> None:
    cx, cy = obstacle.center
    sx, sy = obstacle.size
    xmin = cx - sx / 2.0 - inflate
    xmax = cx + sx / 2.0 + inflate
    ymin = cy - sy / 2.0 - inflate
    ymax = cy + sy / 2.0 + inflate
    c0 = max(0, int(math.floor((xmin - spec.origin_x) / spec.resolution)))
    c1 = min(spec.width - 1, int(math.ceil((xmax - spec.origin_x) / spec.resolution)))
    r0 = max(0, int(math.floor((ymin - spec.origin_y) / spec.resolution)))
    r1 = min(spec.height - 1, int(math.ceil((ymax - spec.origin_y) / spec.resolution)))
    if r1 >= r0 and c1 >= c0:
        grid[r0 : r1 + 1, c0 : c1 + 1] = 100


def build_grid(spec: GridSpec, obstacles: list[Obstacle]) -> np.ndarray:
    grid = np.zeros((spec.height, spec.width), dtype=np.int8)
    for obstacle in obstacles:
        mark_obstacle(grid, spec, obstacle)
    return grid


def rect_distance(point: np.ndarray, obstacle: Obstacle) -> float:
    center = np.asarray(obstacle.center, dtype=float)
    half = np.asarray(obstacle.size, dtype=float) / 2.0
    q = np.abs(point - center) - half
    return float(np.linalg.norm(np.maximum(q, 0.0)) + min(max(float(q[0]), float(q[1])), 0.0))


def min_clearance(point: np.ndarray, obstacles: list[Obstacle]) -> float:
    return min(rect_distance(point, obs) for obs in obstacles)


def global_route() -> np.ndarray:
    return np.asarray(
        [
            [0.58, -2.18],
            [1.38, -1.72],
            [2.28, -0.90],
            [3.54, 0.00],
            [4.24, -0.34],
            [5.16, -1.46],
            [6.55, -1.74],
            [7.84, -0.18],
            [7.62, 2.12],
        ],
        dtype=float,
    )


def clip_speed(v: np.ndarray, max_speed: float) -> np.ndarray:
    speed = float(np.linalg.norm(v))
    if speed > max_speed:
        return v / max(speed, 1e-9) * max_speed
    return v


def world_to_px(point: np.ndarray | tuple[float, float], rect: tuple[int, int, int, int], spec: GridSpec) -> tuple[int, int]:
    x0, y0, w, h = rect
    p = np.asarray(point, dtype=float)
    upper_x = spec.origin_x + (spec.width - 1) * spec.resolution
    upper_y = spec.origin_y + (spec.height - 1) * spec.resolution
    px = x0 + (p[0] - spec.origin_x) / (upper_x - spec.origin_x) * w
    py = y0 + h - (p[1] - spec.origin_y) / (upper_y - spec.origin_y) * h
    return int(round(px)), int(round(py))


def draw_obstacle(draw: ImageDraw.ImageDraw, obs: Obstacle, rect, spec: GridSpec, alpha: float = 1.0) -> None:
    cx, cy = obs.center
    sx, sy = obs.size
    p1 = world_to_px((cx - sx / 2, cy - sy / 2), rect, spec)
    p2 = world_to_px((cx + sx / 2, cy + sy / 2), rect, spec)
    fill = tuple(int(c * alpha + 255 * (1.0 - alpha)) for c in obs.color)
    draw.rectangle([min(p1[0], p2[0]), min(p1[1], p2[1]), max(p1[0], p2[0]), max(p1[1], p2[1])], fill=fill, outline=(51, 65, 85), width=2)


def draw_map_grid(draw: ImageDraw.ImageDraw, rect, spec: GridSpec) -> None:
    x, y, w, h = rect
    draw.rectangle([x, y, x + w, y + h], fill=(248, 250, 252), outline=(203, 213, 225), width=2)
    upper_x = spec.origin_x + (spec.width - 1) * spec.resolution
    upper_y = spec.origin_y + (spec.height - 1) * spec.resolution
    for gx in np.arange(spec.origin_x, upper_x + 1e-6, 0.75):
        draw.line([world_to_px((gx, spec.origin_y), rect, spec), world_to_px((gx, upper_y), rect, spec)], fill=GRID, width=1)
    for gy in np.arange(spec.origin_y, upper_y + 1e-6, 0.75):
        draw.line([world_to_px((spec.origin_x, gy), rect, spec), world_to_px((upper_x, gy), rect, spec)], fill=GRID, width=1)


def draw_polyline(draw: ImageDraw.ImageDraw, points: np.ndarray, rect, spec: GridSpec, color, width: int, outline: bool = True) -> None:
    pts = [world_to_px(p, rect, spec) for p in points]
    if len(pts) >= 2:
        if outline:
            draw.line(pts, fill=(15, 23, 42), width=width + 3, joint="curve")
        draw.line(pts, fill=color, width=width, joint="curve")


def draw_drone(draw: ImageDraw.ImageDraw, pos: np.ndarray, vel: np.ndarray, rect, spec: GridSpec) -> None:
    px, py = world_to_px(pos, rect, spec)
    draw.ellipse([px - 13, py - 13, px + 13, py + 13], fill=DRONE, outline=TEXT, width=2)
    angle = math.atan2(float(vel[1]), float(vel[0])) if np.linalg.norm(vel) > 1e-3 else 0.0
    nose = (px + int(math.cos(angle) * 24), py - int(math.sin(angle) * 24))
    draw.line([(px, py), nose], fill=TEXT, width=3)
    draw.line([(px - 20, py), (px + 20, py)], fill=TEXT, width=2)
    draw.line([(px, py - 20), (px, py + 20)], fill=TEXT, width=2)


def draw_bar(draw: ImageDraw.ImageDraw, x: int, y: int, w: int, h: int, value: float, limit: float, color, label: str) -> None:
    draw.rectangle([x, y, x + w, y + h], fill=(226, 232, 240), outline=(203, 213, 225))
    center = x + w // 2
    draw.line([(center, y), (center, y + h)], fill=(100, 116, 139), width=1)
    span = int(max(-1.0, min(1.0, value / max(limit, 1e-6))) * (w / 2 - 4))
    if span >= 0:
        draw.rectangle([center, y + 3, center + span, y + h - 3], fill=color)
    else:
        draw.rectangle([center + span, y + 3, center, y + h - 3], fill=color)
    draw.text((x, y - 20), f"{label}: {value:+.2f}", fill=MUTED, font=FONT_SMALL)


def risk_label(clearance: float) -> tuple[str, tuple[int, int, int]]:
    if clearance < 0.0:
        return "COLLISION", DANGER
    if clearance < 0.30:
        return "DANGER", DANGER
    if clearance < 0.55:
        return "WARNING", WARNING
    return "SAFE", SAFE


def render_frame(
    width: int,
    height: int,
    spec: GridSpec,
    obstacles: list[Obstacle],
    route: np.ndarray,
    flown: np.ndarray,
    state: RuntimeState,
    predicted: np.ndarray,
    command_v: np.ndarray,
    command_a: np.ndarray,
    telemetry: list[dict[str, float | str | int]],
    frame: int,
    fps: int,
    command_clearance: float,
    compute_ms: float,
) -> Image.Image:
    img = Image.new("RGB", (width, height), (241, 245, 249))
    draw = ImageDraw.Draw(img)
    t = frame / fps
    clearance = float(telemetry[-1]["true_clearance_m"])
    status, status_color = risk_label(clearance)

    draw.text((32, 22), "MPPI Offboard controller evidence: fused map -> velocity/acceleration setpoint", fill=TEXT, font=FONT_TITLE)
    draw.text((32, 62), "Server render; controller loop simulates closed-loop dynamics while PX4/Gazebo SITL is the next runtime verifier.", fill=MUTED, font=FONT_BODY)

    map_rect = (32, 108, 1210, 748)
    draw_map_grid(draw, map_rect, spec)
    for obs in obstacles:
        draw_obstacle(draw, obs, map_rect, spec, alpha=0.72)
    draw_polyline(draw, route, map_rect, spec, (148, 163, 184), 3, outline=False)
    draw_polyline(draw, predicted, map_rect, spec, MPPI, 5, outline=True)
    draw_polyline(draw, flown, map_rect, spec, FUSION, 6, outline=True)
    draw_drone(draw, state.position, state.velocity, map_rect, spec)
    draw.rounded_rectangle([map_rect[0] + 14, map_rect[1] + 14, map_rect[0] + 650, map_rect[1] + 54], radius=8, fill=(255, 255, 255), outline=(226, 232, 240))
    draw.text((map_rect[0] + 26, map_rect[1] + 23), "Indoor fused costmap + MPPI receding-horizon prediction", fill=TEXT, font=FONT_H2)
    target = route[state.target_idx]
    tx, ty = world_to_px(target, map_rect, spec)
    draw.ellipse([tx - 10, ty - 10, tx + 10, ty + 10], outline=(15, 23, 42), width=3)
    draw.text((tx + 14, ty - 10), f"local target {state.target_idx}", fill=TEXT, font=FONT_SMALL)

    right_x = 1272
    panel_w = width - right_x - 32
    sensor_rect = (right_x, 108, panel_w, 236)
    setpoint_rect = (right_x, 368, panel_w, 286)
    timeline_rect = (right_x, 678, panel_w, 178)
    for rect in [sensor_rect, setpoint_rect, timeline_rect]:
        draw.rectangle([rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3]], fill=(255, 255, 255), outline=(203, 213, 225), width=2)

    draw.text((sensor_rect[0] + 18, sensor_rect[1] + 14), "Sensor inputs in fused map", fill=TEXT, font=FONT_H2)
    sensors = [("LiDAR structure", LIDAR, "walls / door frame"), ("Depth", DEPTH, "glass + low box"), ("Radar", RADAR, "moving-person sweep")]
    for i, (name, color, desc) in enumerate(sensors):
        yy = sensor_rect[1] + 58 + i * 52
        draw.rounded_rectangle([sensor_rect[0] + 18, yy, sensor_rect[0] + 156, yy + 34], radius=10, fill=color)
        draw.text((sensor_rect[0] + 30, yy + 8), name, fill=(255, 255, 255), font=FONT_SMALL)
        draw.text((sensor_rect[0] + 176, yy + 8), desc, fill=MUTED, font=FONT_BODY)
    draw.text((sensor_rect[0] + 18, sensor_rect[1] + sensor_rect[3] - 34), "costmap_mux -> /perception/occupancy_grid", fill=FUSION, font=FONT_BODY)

    draw.text((setpoint_rect[0] + 18, setpoint_rect[1] + 14), "PX4 Offboard setpoint generated by MPPI", fill=TEXT, font=FONT_H2)
    draw_bar(draw, setpoint_rect[0] + 26, setpoint_rect[1] + 74, 250, 28, float(command_v[0]), 2.0, FUSION, "vx m/s")
    draw_bar(draw, setpoint_rect[0] + 306, setpoint_rect[1] + 74, 250, 28, float(command_v[1]), 2.0, FUSION, "vy m/s")
    draw_bar(draw, setpoint_rect[0] + 26, setpoint_rect[1] + 156, 250, 28, float(command_a[0]), 2.4, MPPI, "ax m/s2")
    draw_bar(draw, setpoint_rect[0] + 306, setpoint_rect[1] + 156, 250, 28, float(command_a[1]), 2.4, MPPI, "ay m/s2")
    draw.rounded_rectangle([setpoint_rect[0] + 26, setpoint_rect[1] + 222, setpoint_rect[0] + 160, setpoint_rect[1] + 254], radius=10, fill=status_color)
    draw.text((setpoint_rect[0] + 44, setpoint_rect[1] + 230), status, fill=(255, 255, 255), font=FONT_SMALL)
    draw.text((setpoint_rect[0] + 184, setpoint_rect[1] + 226), f"clearance={clearance:.2f} m  MPPI min={command_clearance:.2f} m", fill=MUTED, font=FONT_BODY)

    draw.text((timeline_rect[0] + 18, timeline_rect[1] + 14), "Telemetry timeline", fill=TEXT, font=FONT_H2)
    max_points = min(len(telemetry), 180)
    xs = np.linspace(timeline_rect[0] + 24, timeline_rect[0] + timeline_rect[2] - 24, max_points)
    clearances = np.asarray([float(row["true_clearance_m"]) for row in telemetry[-max_points:]], dtype=float)
    speeds = np.asarray([float(row["speed_mps"]) for row in telemetry[-max_points:]], dtype=float)
    cmin, cmax = -0.1, 1.7
    smin, smax = 0.0, 2.1
    cpts = [(int(x), int(timeline_rect[1] + 132 - (c - cmin) / (cmax - cmin) * 78)) for x, c in zip(xs, clearances)]
    spts = [(int(x), int(timeline_rect[1] + 132 - (s - smin) / (smax - smin) * 78)) for x, s in zip(xs, speeds)]
    if len(cpts) > 1:
        draw.line(cpts, fill=FUSION, width=3)
        draw.line(spts, fill=MPPI, width=3)
    draw.text((timeline_rect[0] + 24, timeline_rect[1] + 146), "green=clearance, blue=speed", fill=MUTED, font=FONT_SMALL)
    draw.text((timeline_rect[0] + timeline_rect[2] - 250, timeline_rect[1] + 146), f"compute={compute_ms:.1f} ms  t={t:.1f}s", fill=MUTED, font=FONT_SMALL)

    footer_y = 888
    draw.rectangle([32, footer_y, width - 32, height - 28], fill=(255, 255, 255), outline=(203, 213, 225), width=2)
    draw.text((54, footer_y + 18), "Runtime claim", fill=TEXT, font=FONT_H2)
    draw.text(
        (54, footer_y + 54),
        "This server-rendered video runs the MPPI controller code and a closed-loop kinematic UAV model. "
        "It demonstrates velocity/acceleration setpoint generation; PX4/Gazebo SITL recording remains the next verifier.",
        fill=MUTED,
        font=FONT_BODY,
    )
    draw.text((54, footer_y + 90), "TrajectorySetpoint fields: velocity=[vy, vx, 0] NED, acceleration=[ay, ax, 0] NED, z held at fixed altitude.", fill=MUTED, font=FONT_SMALL)
    return img


def write_video(frame_dir: Path, output: Path, fps: int) -> None:
    if not shutil.which("ffmpeg"):
        raise RuntimeError("ffmpeg is required to render MP4")
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
    spec = make_spec()
    route = global_route()
    state = RuntimeState(position=route[0].copy(), velocity=np.zeros(2, dtype=float), target_idx=1)
    flown = [state.position.copy()]
    rows: list[dict[str, float | str | int]] = []
    sim_dt = float(args.sim_dt)
    last_command_v = np.zeros(2, dtype=float)
    last_command_a = np.zeros(2, dtype=float)
    last_predicted = np.repeat(state.position[None, :], 4, axis=0)
    last_command_clearance = 0.0
    last_compute_ms = 0.0

    args.preview.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="mppi_offboard_frames_") as tmp:
        frame_dir = Path(tmp)
        for frame in range(int(args.frames)):
            t = frame / float(args.fps)
            obstacles = obstacles_for_time(t)
            grid = build_grid(spec, obstacles)
            if state.target_idx < len(route) - 1 and np.linalg.norm(route[state.target_idx] - state.position) < 0.42:
                state = RuntimeState(position=state.position, velocity=state.velocity, target_idx=state.target_idx + 1)
            goal = route[state.target_idx]
            command = mppi_velocity_command(
                grid,
                spec,
                state.position,
                state.velocity,
                goal,
                MPPIControllerConfig(
                    dt_s=sim_dt,
                    horizon_steps=int(args.horizon),
                    num_rollouts=int(args.rollouts),
                    temperature=0.8,
                    max_speed_mps=2.0,
                    max_accel_mps2=2.4,
                    accel_noise_sigma_mps2=0.75,
                    robot_radius_m=0.18,
                    safety_distance_m=0.36,
                    clearance_weight=120.0,
                    terminal_goal_weight=28.0,
                    collision_weight=9000.0,
                    stop_distance_m=0.24,
                    seed=int(args.seed) + frame,
                ),
            )
            # Simulate PX4-like velocity tracking with acceleration feed-forward.
            vel = state.velocity + command.acceleration_sp_mps2 * sim_dt
            vel = 0.62 * vel + 0.38 * command.velocity_sp_mps
            vel = clip_speed(vel, 2.0)
            pos = state.position + vel * sim_dt
            clearance = min_clearance(pos, obstacles)
            speed = float(np.linalg.norm(vel))
            rows.append(
                {
                    "frame": frame,
                    "time_s": t,
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "vx_mps": float(vel[0]),
                    "vy_mps": float(vel[1]),
                    "ax_sp_mps2": float(command.acceleration_sp_mps2[0]),
                    "ay_sp_mps2": float(command.acceleration_sp_mps2[1]),
                    "speed_mps": speed,
                    "true_clearance_m": clearance,
                    "mppi_min_clearance_m": float(command.min_clearance_m),
                    "compute_ms": float(command.compute_time_s * 1000.0),
                    "target_idx": state.target_idx,
                    "collision": int(clearance <= 0.0),
                }
            )
            state = RuntimeState(position=pos, velocity=vel, target_idx=state.target_idx)
            flown.append(pos.copy())
            last_command_v = command.velocity_sp_mps
            last_command_a = command.acceleration_sp_mps2
            last_predicted = command.predicted_xy
            last_command_clearance = float(command.min_clearance_m)
            last_compute_ms = float(command.compute_time_s * 1000.0)

            img = render_frame(
                int(args.width),
                int(args.height),
                spec,
                obstacles,
                route,
                np.asarray(flown, dtype=float),
                state,
                last_predicted,
                last_command_v,
                last_command_a,
                rows,
                frame,
                int(args.fps),
                last_command_clearance,
                last_compute_ms,
            )
            img.save(frame_dir / f"frame_{frame:04d}.png")
            if frame == int(args.frames) // 2:
                img.save(args.preview)
        write_video(frame_dir, args.output, int(args.fps))

    args.metrics.parent.mkdir(parents=True, exist_ok=True)
    with args.metrics.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    min_clear = min(float(row["true_clearance_m"]) for row in rows)
    max_speed = max(float(row["speed_mps"]) for row in rows)
    mean_compute = sum(float(row["compute_ms"]) for row in rows) / len(rows)
    collisions = sum(int(row["collision"]) for row in rows)
    summary = (
        "# MPPI Offboard Controller Setpoint Demo\n\n"
        "Server-rendered evidence for the upgraded controller path.\n\n"
        f"- Video: `{args.output}`\n"
        f"- Preview: `{args.preview}`\n"
        f"- Metrics: `{args.metrics}`\n"
        f"- Frames: `{len(rows)}`\n"
        f"- Minimum true clearance: `{min_clear:.3f} m`\n"
        f"- Maximum speed: `{max_speed:.3f} m/s`\n"
        f"- Mean MPPI compute time: `{mean_compute:.2f} ms`\n"
        f"- Collision frames: `{collisions}`\n\n"
        "Scope note: this runs the MPPI controller and a kinematic UAV dynamics model. "
        "PX4/Gazebo SITL recording is still required for a real flight-stack runtime claim.\n"
    )
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(summary)
    print(summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
