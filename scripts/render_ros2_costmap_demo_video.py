#!/usr/bin/env python3
"""Render a lightweight MP4 demo for the ROS2 costmap planner.

The renderer does not require ROS, RViz, OpenCV, or matplotlib. It reuses the
pure Python occupancy-grid planner helpers, draws RGB frames with numpy, and
encodes them through ffmpeg. The output is a reproducible demo video showing:

  obstacle costmap -> planned path -> UAV marker following the path
"""

from __future__ import annotations

import argparse
import csv
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np


FONT_5X7 = {
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
    "-": ["00000", "00000", "00000", "11111", "00000", "00000", "00000"],
    ">": ["10000", "01000", "00100", "00010", "00100", "01000", "10000"],
    "/": ["00001", "00010", "00100", "01000", "10000", "00000", "00000"],
    ":": ["00000", "00100", "00100", "00000", "00100", "00100", "00000"],
    ".": ["00000", "00000", "00000", "00000", "00000", "01100", "01100"],
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    "6": ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00010", "11100"],
}

LETTER_PATTERNS = {
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "G": ["01110", "10001", "10000", "10111", "10001", "10001", "01110"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "J": ["00111", "00010", "00010", "00010", "00010", "10010", "01100"],
    "K": ["10001", "10010", "10100", "11000", "10100", "10010", "10001"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "Q": ["01110", "10001", "10001", "10001", "10101", "10010", "01101"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "W": ["10001", "10001", "10001", "10101", "10101", "10101", "01010"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "Z": ["11111", "00001", "00010", "00100", "01000", "10000", "11111"],
}
FONT_5X7.update(LETTER_PATTERNS)


def _import_planners() -> tuple[object, object, object, object]:
    repo_root = Path(__file__).resolve().parents[1]
    ros_pkg = repo_root / "ros2_ws" / "src" / "uav_oda_ros2_demo"
    sys.path.insert(0, str(ros_pkg))
    from uav_oda_ros2_demo.grid_planners import GridSpec, PlannerConfig, inflate_grid, plan_path

    return GridSpec, PlannerConfig, inflate_grid, plan_path


def _mark_circle(grid: np.ndarray, spec: object, cx: float, cy: float, radius: float) -> None:
    for row in range(grid.shape[0]):
        y = spec.origin_y + row * spec.resolution
        for col in range(grid.shape[1]):
            x = spec.origin_x + col * spec.resolution
            if (x - cx) ** 2 + (y - cy) ** 2 <= radius**2:
                grid[row, col] = 100


def _make_demo_grid() -> tuple[np.ndarray, object, np.ndarray, np.ndarray]:
    GridSpec, _, _, _ = _import_planners()
    spec = GridSpec(width=90, height=80, resolution=0.10, origin_x=-1.0, origin_y=-4.0)
    grid = np.zeros((spec.height, spec.width), dtype=np.int8)
    _mark_circle(grid, spec, 2.0, 1.0, 0.35)
    _mark_circle(grid, spec, 4.2, -0.8, 0.45)
    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([6.0, 0.0], dtype=float)
    return grid, spec, start, goal


def _read_bbox_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(path)
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _load_cached_depth_frame(path: Path, frame_index: int = 0) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(path)
    cache = np.load(path)
    depth = np.asarray(cache["depth_u8"])
    if depth.ndim != 3:
        raise ValueError(f"Expected depth_u8 with shape [frames,h,w], got {depth.shape}")
    return depth[frame_index].astype(np.float32)


def _make_cached_mux_grid(
    bbox_csv: Path,
    depth_cache: Path,
    depth_frame_index: int,
) -> tuple[np.ndarray, object, np.ndarray, np.ndarray]:
    repo_root = Path(__file__).resolve().parents[1]
    ros_pkg = repo_root / "ros2_ws" / "src" / "uav_oda_ros2_demo"
    sys.path.insert(0, str(ros_pkg))
    from uav_oda_ros2_demo.costmap_converters import (
        DepthProjectionConfig,
        bbox_rows_to_grid,
        depth_image_to_grid,
        merge_occupancy_grids,
        select_bbox_rows,
    )

    rows = select_bbox_rows(_read_bbox_rows(bbox_csv), frame_offset=0, min_point_count=50)
    bbox_grid, bbox_spec = bbox_rows_to_grid(rows, resolution_m=0.20, margin_m=1.0)
    bbox_goal = np.asarray([bbox_spec.origin_x + (bbox_spec.width - 2) * bbox_spec.resolution, 4.0], dtype=float)

    depth_config = DepthProjectionConfig(resolution_m=0.10, sample_stride_px=3, hit_dilation_cells=2)
    cached_depth = _load_cached_depth_frame(depth_cache, depth_frame_index)
    cached_grid, cached_spec, _ = depth_image_to_grid(cached_depth, "mono8", depth_config)
    mux_grid, mux_spec = merge_occupancy_grids(
        [(bbox_grid, bbox_spec), (cached_grid, cached_spec)],
        occupied_threshold=50,
        resolution_m=0.20,
        padding_m=0.25,
    )
    start = np.asarray([0.0, 0.0], dtype=float)
    return mux_grid, mux_spec, start, bbox_goal


def _resample_path(points: np.ndarray, n: int) -> np.ndarray:
    if len(points) <= 1:
        return np.repeat(points[:1], n, axis=0)
    seg_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    total = float(seg_lengths.sum())
    if total <= 0:
        return np.repeat(points[:1], n, axis=0)
    cumulative = np.concatenate([[0.0], np.cumsum(seg_lengths)])
    targets = np.linspace(0.0, total, n)
    out = np.zeros((n, 2), dtype=float)
    seg = 0
    for idx, target in enumerate(targets):
        while seg < len(seg_lengths) - 1 and cumulative[seg + 1] < target:
            seg += 1
        denom = cumulative[seg + 1] - cumulative[seg]
        alpha = 0.0 if denom == 0 else (target - cumulative[seg]) / denom
        out[idx] = (1.0 - alpha) * points[seg] + alpha * points[seg + 1]
    return out


def _draw_line(img: np.ndarray, p0: tuple[int, int], p1: tuple[int, int], color: tuple[int, int, int], width: int = 2) -> None:
    x0, y0 = p0
    x1, y1 = p1
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        _draw_circle_px(img, x, y, max(1, width), color)
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def _draw_polyline(img: np.ndarray, points: list[tuple[int, int]], color: tuple[int, int, int], width: int = 2) -> None:
    for a, b in zip(points[:-1], points[1:]):
        _draw_line(img, a, b, color, width)


def _draw_circle_px(img: np.ndarray, cx: int, cy: int, radius: int, color: tuple[int, int, int]) -> None:
    h, w = img.shape[:2]
    r2 = radius * radius
    y0 = max(0, cy - radius)
    y1 = min(h - 1, cy + radius)
    x0 = max(0, cx - radius)
    x1 = min(w - 1, cx + radius)
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            if (x - cx) ** 2 + (y - cy) ** 2 <= r2:
                img[y, x] = color


def _draw_rect(img: np.ndarray, x0: int, y0: int, x1: int, y1: int, color: tuple[int, int, int]) -> None:
    h, w = img.shape[:2]
    xa, xb = sorted((max(0, x0), min(w - 1, x1)))
    ya, yb = sorted((max(0, y0), min(h - 1, y1)))
    img[ya : yb + 1, xa : xb + 1] = color


def _draw_text(
    img: np.ndarray,
    x: int,
    y: int,
    text: str,
    color: tuple[int, int, int] = (15, 23, 42),
    scale: int = 2,
) -> None:
    cursor = x
    for char in text.upper():
        pattern = FONT_5X7.get(char, FONT_5X7[" "])
        for row, line in enumerate(pattern):
            for col, bit in enumerate(line):
                if bit == "1":
                    _draw_rect(
                        img,
                        cursor + col * scale,
                        y + row * scale,
                        cursor + (col + 1) * scale - 1,
                        y + (row + 1) * scale - 1,
                        color,
                    )
        cursor += 6 * scale


def _draw_label_with_swatch(
    img: np.ndarray,
    x: int,
    y: int,
    label: str,
    color: tuple[int, int, int],
    text_color: tuple[int, int, int] = (30, 41, 59),
) -> int:
    _draw_rect(img, x, y + 3, x + 18, y + 17, color)
    _draw_text(img, x + 26, y, label, text_color, 2)
    return x + 26 + len(label) * 12 + 18


class Projector:
    def __init__(self, spec: object, width: int, height: int, margin: int = 64) -> None:
        self.spec = spec
        self.width = width
        self.height = height
        self.margin = margin
        self.left = margin
        self.right = margin
        self.top = 108
        self.bottom = margin
        self.x_min = spec.origin_x
        self.x_max = spec.origin_x + (spec.width - 1) * spec.resolution
        self.y_min = spec.origin_y
        self.y_max = spec.origin_y + (spec.height - 1) * spec.resolution
        self.map_w = width - self.left - self.right
        self.map_h = height - self.top - self.bottom

    def world(self, x: float, y: float) -> tuple[int, int]:
        px = self.left + int(round((x - self.x_min) / max(self.x_max - self.x_min, 1e-6) * self.map_w))
        py = self.height - self.bottom - int(round((y - self.y_min) / max(self.y_max - self.y_min, 1e-6) * self.map_h))
        return px, py

    def cell_rect(self, row: int, col: int) -> tuple[int, int, int, int]:
        x0, y0 = self.world(self.spec.origin_x + col * self.spec.resolution, self.spec.origin_y + row * self.spec.resolution)
        x1, y1 = self.world(
            self.spec.origin_x + (col + 1) * self.spec.resolution,
            self.spec.origin_y + (row + 1) * self.spec.resolution,
        )
        return x0, y0, x1, y1


def _draw_grid_scene(
    img: np.ndarray,
    grid: np.ndarray,
    inflated: np.ndarray,
    spec: object,
    projector: Projector,
) -> None:
    _draw_rect(img, projector.left, projector.top, projector.width - projector.right, projector.height - projector.bottom, (248, 250, 252))
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            if inflated[row, col] >= 50:
                _draw_rect(img, *projector.cell_rect(row, col), (255, 226, 199))
            if grid[row, col] >= 50:
                _draw_rect(img, *projector.cell_rect(row, col), (222, 84, 62))

    for x in np.arange(np.ceil(projector.x_min), np.floor(projector.x_max) + 1):
        p0 = projector.world(float(x), projector.y_min)
        p1 = projector.world(float(x), projector.y_max)
        _draw_line(img, p0, p1, (226, 232, 240), 1)
    for y in np.arange(np.ceil(projector.y_min), np.floor(projector.y_max) + 1):
        p0 = projector.world(projector.x_min, float(y))
        p1 = projector.world(projector.x_max, float(y))
        _draw_line(img, p0, p1, (226, 232, 240), 1)
    _draw_rect(img, projector.left, projector.top, projector.width - projector.right, projector.top + 2, (30, 41, 59))
    _draw_rect(img, projector.left, projector.height - projector.bottom - 2, projector.width - projector.right, projector.height - projector.bottom, (30, 41, 59))
    _draw_rect(img, projector.left, projector.top, projector.left + 2, projector.height - projector.bottom, (30, 41, 59))
    _draw_rect(img, projector.width - projector.right - 2, projector.top, projector.width - projector.right, projector.height - projector.bottom, (30, 41, 59))


def _render_frames(
    output: Path,
    planner: str,
    fps: int,
    duration_s: float,
    width: int,
    height: int,
    title: str,
    status_text: str,
    scene: str,
    bbox_csv: Path,
    depth_cache: Path,
    depth_frame_index: int,
) -> None:
    _, PlannerConfig, inflate_grid, plan_path = _import_planners()
    if scene == "cached_mux":
        grid, spec, start, goal = _make_cached_mux_grid(bbox_csv, depth_cache, depth_frame_index)
    else:
        grid, spec, start, goal = _make_demo_grid()
    config = PlannerConfig(
        robot_radius_m=0.10 if scene == "cached_mux" else 0.15,
        safety_distance_m=0.15 if scene == "cached_mux" else 0.20,
        rrt_max_iterations=1500,
        rrt_step_size_m=0.35,
        mppi_rollouts=128,
        mppi_iterations=4,
        seed=11,
    )
    path = plan_path(planner, grid, spec, start, goal, config)
    inflated = inflate_grid(grid, spec, config.inflation_radius_m, config.occupied_threshold)
    n_frames = max(2, int(round(fps * duration_s)))
    sampled = _resample_path(path, n_frames)
    projector = Projector(spec, width, height)

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise RuntimeError("ffmpeg is required to encode MP4 video")
    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        ffmpeg,
        "-y",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{width}x{height}",
        "-r",
        str(fps),
        "-i",
        "-",
        "-an",
        "-vcodec",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(output),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdin is not None
    full_path_px = [projector.world(float(x), float(y)) for x, y in path]
    start_px = projector.world(float(start[0]), float(start[1]))
    goal_px = projector.world(float(goal[0]), float(goal[1]))
    path_length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())

    try:
        for idx, uav_xy in enumerate(sampled):
            img = np.full((height, width, 3), (255, 255, 255), dtype=np.uint8)
            _draw_rect(img, 0, 0, width - 1, 52, (241, 245, 249))
            _draw_text(img, 28, 18, title, (15, 23, 42), 2)
            _draw_text(img, width - 320, 18, f"PLANNER: {planner}", (51, 65, 85), 2)

            _draw_grid_scene(img, grid, inflated, spec, projector)
            _draw_polyline(img, full_path_px, (61, 117, 211), 2)
            trail_px = [projector.world(float(x), float(y)) for x, y in sampled[: idx + 1]]
            if len(trail_px) >= 2:
                _draw_polyline(img, trail_px, (17, 24, 39), 4)
            _draw_circle_px(img, start_px[0], start_px[1], 9, (34, 197, 94))
            _draw_circle_px(img, goal_px[0], goal_px[1], 11, (79, 70, 229))
            uav_px = projector.world(float(uav_xy[0]), float(uav_xy[1]))
            _draw_circle_px(img, uav_px[0], uav_px[1], 13, (15, 118, 110))
            _draw_circle_px(img, uav_px[0], uav_px[1], 6, (240, 253, 250))

            legend_y = 66
            lx = 72
            lx = _draw_label_with_swatch(img, lx, legend_y, "OBSTACLE", (222, 84, 62))
            lx = _draw_label_with_swatch(img, lx, legend_y, "SAFETY", (255, 226, 199))
            lx = _draw_label_with_swatch(img, lx, legend_y, "PATH", (61, 117, 211))
            lx = _draw_label_with_swatch(img, lx, legend_y, "UAV", (15, 118, 110))
            _draw_text(img, 72, height - 62, f"PATH {path_length:.1f} M", (51, 65, 85), 2)
            if status_text:
                _draw_text(img, 72, height - 86, status_text, (51, 65, 85), 2)
            _draw_text(img, width - 220, height - 62, f"T {idx / fps:04.1f} S", (51, 65, 85), 2)
            progress_w = int((idx + 1) / n_frames * projector.map_w)
            _draw_rect(img, projector.left, height - 35, width - projector.right, height - 27, (226, 232, 240))
            _draw_rect(img, projector.left, height - 35, projector.left + progress_w, height - 27, (15, 118, 110))
            proc.stdin.write(img.tobytes())
    finally:
        proc.stdin.close()
    stdout = proc.stdout.read() if proc.stdout is not None else b""
    stderr = proc.stderr.read() if proc.stderr is not None else b""
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg failed with code {proc.returncode}: {stderr.decode(errors='replace')[-2000:]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner", default="astar", choices=["astar", "rrt", "mppi"])
    parser.add_argument("--output", type=Path, default=Path("outputs/videos/ros2_costmap_demo_astar.mp4"))
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--duration-s", type=float, default=8.0)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--title", default="ROS2 COSTMAP PLANNER DEMO")
    parser.add_argument("--status-text", default="")
    parser.add_argument("--scene", choices=["demo", "cached_mux"], default="demo")
    parser.add_argument(
        "--bbox-csv",
        type=Path,
        default=Path("outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv"),
    )
    parser.add_argument("--depth-cache", type=Path, default=Path("data/processed/depth_sample_3_5fps.npz"))
    parser.add_argument("--depth-frame-index", type=int, default=0)
    args = parser.parse_args()

    _render_frames(
        args.output,
        args.planner,
        args.fps,
        args.duration_s,
        args.width,
        args.height,
        args.title,
        args.status_text,
        args.scene,
        args.bbox_csv,
        args.depth_cache,
        args.depth_frame_index,
    )
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
