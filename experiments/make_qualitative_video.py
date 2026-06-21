#!/usr/bin/env python3
"""Create a lightweight qualitative ODA video.

The output mirrors the common qualitative-results layout: camera input on the
left, obstacle-risk map in the middle, and MAV trajectory/localization on the
right.  It uses only RGB video, trial metadata, and OptiTrack CSV.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import imageio
import matplotlib
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from src.metrics import compute_trial_metrics, pairwise_ground_distances
from src.oda_io import (
    dataset_root,
    load_imu,
    load_optitrack,
    load_radar_spectra,
    obstacle_array,
    read_trial_overview,
)


PANEL_W = 480
PANEL_H = 480
TITLE_H = 38
PLOT_PAD = 46


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", default="data/raw/ODA_Dataset/dataset")
    parser.add_argument("--trial-id", default="345")
    parser.add_argument("--output", default="outputs/videos/qualitative_sample_345.mp4")
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument(
        "--max-duration",
        type=float,
        default=None,
        help="Optional duration cap in seconds.",
    )
    parser.add_argument(
        "--include-sensors",
        action="store_true",
        help="Add radar FFT and IMU time-series panels.",
    )
    parser.add_argument(
        "--include-depth",
        action="store_true",
        help="Add cached monocular predicted-depth panel.",
    )
    parser.add_argument(
        "--depth-cache",
        default=None,
        help="Path to .npz cache from cache_monocular_depth.py.",
    )
    parser.add_argument(
        "--sensor-window",
        type=float,
        default=3.0,
        help="Rolling time window in seconds for the IMU panel.",
    )
    return parser.parse_args()


def font(size: int) -> ImageFont.ImageFont:
    candidates = [
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
    ]
    for path in candidates:
        if Path(path).exists():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


FONT_TITLE = font(18)
FONT_SMALL = font(13)
FONT_TINY = font(11)


def cover_resize(image: Image.Image, size: tuple[int, int]) -> Image.Image:
    target_w, target_h = size
    src_w, src_h = image.size
    scale = max(target_w / src_w, target_h / src_h)
    new_size = (math.ceil(src_w * scale), math.ceil(src_h * scale))
    resized = image.resize(new_size, Image.Resampling.LANCZOS)
    left = (resized.width - target_w) // 2
    top = (resized.height - target_h) // 2
    return resized.crop((left, top, left + target_w, top + target_h))


def draw_title(draw: ImageDraw.ImageDraw, title: str, subtitle: str | None = None) -> None:
    draw.rectangle((0, 0, PANEL_W, TITLE_H), fill=(250, 250, 250))
    draw.text((12, 6), title, fill=(20, 20, 20), font=FONT_TITLE)
    if subtitle:
        draw.text((PANEL_W - 145, 12), subtitle, fill=(70, 70, 70), font=FONT_TINY)


def make_camera_panel(frame: np.ndarray, title: str, time_s: float) -> Image.Image:
    panel = Image.new("RGB", (PANEL_W, PANEL_H), "white")
    image = Image.fromarray(frame).convert("RGB")
    image = cover_resize(image, (PANEL_W, PANEL_H - TITLE_H))
    panel.paste(image, (0, TITLE_H))
    draw = ImageDraw.Draw(panel)
    draw_title(draw, title, f"t={time_s:4.1f}s")
    return panel


def default_depth_cache_path(sequence: str) -> Path:
    return Path("data/processed") / f"depth_sample_{sequence}_5fps.npz"


def load_depth_cache(path: str | Path) -> dict[str, np.ndarray]:
    cache_path = Path(path)
    with np.load(cache_path, allow_pickle=False) as data:
        return {
            "times": data["times"],
            "depth_u8": data["depth_u8"],
            "model_id": data["model_id"],
            "depth_fps": data["depth_fps"],
        }


def make_depth_panel(depth_cache: dict[str, np.ndarray], depth_index: int, time_s: float) -> Image.Image:
    panel = Image.new("RGB", (PANEL_W, PANEL_H), "white")
    depth = depth_cache["depth_u8"][depth_index]
    rgba = matplotlib.colormaps["magma"](depth.astype(float) / 255.0)
    rgb = (rgba[:, :, :3] * 255).astype(np.uint8)
    depth_image = Image.fromarray(rgb)
    depth_image = cover_resize(depth_image, (PANEL_W, PANEL_H - TITLE_H))
    panel.paste(depth_image, (0, TITLE_H))

    draw = ImageDraw.Draw(panel)
    draw_title(draw, "Predicted relative depth", f"t={time_s:4.1f}s")
    draw.rectangle((10, PANEL_H - 58, PANEL_W - 10, PANEL_H - 10), fill=(0, 0, 0))
    draw.text(
        (18, PANEL_H - 50),
        "monocular DPT/MiDaS; qualitative only",
        fill=(255, 255, 255),
        font=FONT_SMALL,
    )
    draw.text(
        (18, PANEL_H - 30),
        "brighter = larger relative depth response",
        fill=(230, 230, 230),
        font=FONT_TINY,
    )
    return panel


def world_to_px(
    xy: np.ndarray | tuple[float, float],
    bounds: tuple[float, float, float, float],
    plot_box: tuple[int, int, int, int],
) -> tuple[int, int]:
    x_min, x_max, y_min, y_max = bounds
    left, top, right, bottom = plot_box
    x, y = float(xy[0]), float(xy[1])
    px = left + (x - x_min) / (x_max - x_min) * (right - left)
    py = bottom - (y - y_min) / (y_max - y_min) * (bottom - top)
    return int(round(px)), int(round(py))


def meters_to_px_x(radius_m: float, bounds: tuple[float, float, float, float], plot_box) -> int:
    x_min, x_max, _, _ = bounds
    left, _, right, _ = plot_box
    return max(1, int(round(radius_m / (x_max - x_min) * (right - left))))


def compute_bounds(trajectory_xy: np.ndarray, obstacles_xy: np.ndarray, margin: float = 0.9):
    all_xy = np.vstack([trajectory_xy, obstacles_xy])
    x_min, y_min = all_xy.min(axis=0) - margin
    x_max, y_max = all_xy.max(axis=0) + margin
    span_x = x_max - x_min
    span_y = y_max - y_min
    if span_x > span_y:
        extra = (span_x - span_y) / 2
        y_min -= extra
        y_max += extra
    else:
        extra = (span_y - span_x) / 2
        x_min -= extra
        x_max += extra
    return float(x_min), float(x_max), float(y_min), float(y_max)


def make_risk_background(
    bounds: tuple[float, float, float, float],
    obstacles_xy: np.ndarray,
    obstacle_radius_m: float,
    safety_distance_m: float,
    size: tuple[int, int],
) -> Image.Image:
    w, h = size
    x_min, x_max, y_min, y_max = bounds
    xs = np.linspace(x_min, x_max, w)
    ys = np.linspace(y_max, y_min, h)
    grid_x, grid_y = np.meshgrid(xs, ys)
    grid = np.stack([grid_x, grid_y], axis=2).reshape(-1, 2)
    d = pairwise_ground_distances(grid, obstacles_xy).min(axis=1)
    clearance = d - obstacle_radius_m
    risk = np.clip(1.0 - clearance / safety_distance_m, 0.0, 1.0)
    risk = risk.reshape(h, w)
    rgba = matplotlib.colormaps["YlOrRd"](risk)
    rgb = (rgba[:, :, :3] * 255).astype(np.uint8)
    return Image.fromarray(rgb)


def draw_axes(draw: ImageDraw.ImageDraw, bounds, plot_box, label_color=(85, 85, 85)) -> None:
    left, top, right, bottom = plot_box
    draw.rectangle(plot_box, outline=(220, 220, 220), width=1)
    x_min, x_max, y_min, y_max = bounds
    for frac in [0.25, 0.5, 0.75]:
        x = int(left + frac * (right - left))
        y = int(top + frac * (bottom - top))
        draw.line((x, top, x, bottom), fill=(235, 235, 235), width=1)
        draw.line((left, y, right, y), fill=(235, 235, 235), width=1)
    draw.text((left, bottom + 8), f"x {x_min:.1f}..{x_max:.1f} m", fill=label_color, font=FONT_TINY)
    draw.text((left, bottom + 22), f"z {y_min:.1f}..{y_max:.1f} m", fill=label_color, font=FONT_TINY)


def draw_plot_frame(
    draw: ImageDraw.ImageDraw,
    plot_box: tuple[int, int, int, int],
    label: str,
    y_label: str | None = None,
) -> None:
    left, top, right, bottom = plot_box
    draw.rectangle(plot_box, outline=(220, 220, 220), width=1)
    for frac in [0.25, 0.5, 0.75]:
        x = int(left + frac * (right - left))
        y = int(top + frac * (bottom - top))
        draw.line((x, top, x, bottom), fill=(238, 238, 238), width=1)
        draw.line((left, y, right, y), fill=(238, 238, 238), width=1)
    draw.text((left, top - 14), label, fill=(45, 45, 45), font=FONT_TINY)
    if y_label:
        draw.text((right - 72, top - 14), y_label, fill=(85, 85, 85), font=FONT_TINY)


def plot_line(
    draw: ImageDraw.ImageDraw,
    x: np.ndarray,
    y: np.ndarray,
    plot_box: tuple[int, int, int, int],
    x_range: tuple[float, float],
    y_range: tuple[float, float],
    color: tuple[int, int, int],
    width: int = 2,
) -> None:
    if len(x) < 2:
        return
    left, top, right, bottom = plot_box
    x_min, x_max = x_range
    y_min, y_max = y_range
    if x_max <= x_min or y_max <= y_min:
        return
    px = left + (x - x_min) / (x_max - x_min) * (right - left)
    py = bottom - (y - y_min) / (y_max - y_min) * (bottom - top)
    pts = [
        (int(round(px_i)), int(round(py_i)))
        for px_i, py_i in zip(px, py)
        if np.isfinite(px_i) and np.isfinite(py_i)
    ]
    if len(pts) > 1:
        draw.line(pts, fill=color, width=width)


def make_radar_panel(
    radar: dict[str, np.ndarray],
    radar_index: int,
    y_max: float,
    title: str = "24 GHz radar FFT",
) -> Image.Image:
    panel = Image.new("RGB", (PANEL_W, PANEL_H), "white")
    draw = ImageDraw.Draw(panel)
    t = float(radar["time_s"][radar_index])
    draw_title(draw, title, f"t={t:4.1f}s")

    plot_box = (42, TITLE_H + 32, PANEL_W - 24, PANEL_H - 62)
    draw_plot_frame(draw, plot_box, "first chirp, log10(1 + FFT magnitude)", "power")

    freq = radar["freq"]
    rx1 = np.log10(1.0 + radar["mag_rx1"][radar_index])
    rx2 = np.log10(1.0 + radar["mag_rx2"][radar_index])
    x_range = (0.0, float(freq.max()))
    y_range = (0.0, y_max)
    plot_line(draw, freq, rx1, plot_box, x_range, y_range, color=(35, 110, 190), width=2)
    plot_line(draw, freq, rx2, plot_box, x_range, y_range, color=(230, 120, 20), width=2)

    peak_i = int(np.argmax(np.maximum(rx1, rx2)))
    px, py = world_to_px(
        (freq[peak_i], max(rx1[peak_i], rx2[peak_i])),
        (x_range[0], x_range[1], y_range[0], y_range[1]),
        plot_box,
    )
    draw.ellipse((px - 4, py - 4, px + 4, py + 4), fill=(130, 20, 160))
    draw.text((14, PANEL_H - 44), "RX1", fill=(35, 110, 190), font=FONT_SMALL)
    draw.text((58, PANEL_H - 44), "RX2", fill=(230, 120, 20), font=FONT_SMALL)
    draw.text((14, PANEL_H - 25), f"peak normalized freq: {freq[peak_i]:.3f}", fill=(45, 45, 45), font=FONT_SMALL)
    return panel


def make_imu_panel(
    imu: dict[str, np.ndarray],
    imu_index: int,
    t_now: float,
    window_s: float,
    accel_range: tuple[float, float],
    gyro_range: tuple[float, float],
) -> Image.Image:
    panel = Image.new("RGB", (PANEL_W, PANEL_H), "white")
    draw = ImageDraw.Draw(panel)
    draw_title(draw, "6-axis IMU", f"t={t_now:4.1f}s")

    time = imu["time_s"]
    start_t = max(0.0, t_now - window_s)
    mask = (time >= start_t) & (time <= t_now)
    if mask.sum() < 2:
        mask[: imu_index + 1] = True

    x = time[mask]
    accel = imu["accel_filt_mps2"][mask]
    gyro = imu["gyro_filt_radps"][mask]
    if len(x) >= 2:
        x_range = (float(x.min()), float(max(x.max(), x.min() + 0.1)))
    else:
        x_range = (max(0.0, t_now - window_s), t_now + 0.1)

    accel_box = (42, TITLE_H + 36, PANEL_W - 24, TITLE_H + 178)
    gyro_box = (42, TITLE_H + 234, PANEL_W - 24, PANEL_H - 58)
    draw_plot_frame(draw, accel_box, "linear acceleration", "m/s^2")
    draw_plot_frame(draw, gyro_box, "angular velocity", "rad/s")

    colors = [(35, 110, 190), (40, 160, 70), (220, 70, 70)]
    labels = ["x", "y", "z"]
    for idx, color in enumerate(colors):
        plot_line(draw, x, accel[:, idx], accel_box, x_range, accel_range, color=color, width=2)
        plot_line(draw, x, gyro[:, idx], gyro_box, x_range, gyro_range, color=color, width=2)
        draw.text((14 + idx * 42, PANEL_H - 25), labels[idx], fill=color, font=FONT_SMALL)

    current_accel_norm = float(np.linalg.norm(imu["accel_filt_mps2"][imu_index]))
    current_gyro_norm = float(np.linalg.norm(imu["gyro_filt_radps"][imu_index]))
    draw.text(
        (128, PANEL_H - 25),
        f"|a|={current_accel_norm:.1f}, |w|={current_gyro_norm:.2f}",
        fill=(45, 45, 45),
        font=FONT_SMALL,
    )
    return panel


def make_risk_panel(
    risk_bg: Image.Image,
    current_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    bounds,
    obstacle_radius_m: float,
    safety_distance_m: float,
    clearance_m: float,
) -> Image.Image:
    panel = Image.new("RGB", (PANEL_W, PANEL_H), "white")
    draw = ImageDraw.Draw(panel)
    draw_title(draw, "Obstacle safety-distance map", f"clear={clearance_m:.2f}m")
    plot_box = (PLOT_PAD, TITLE_H + 18, PANEL_W - 22, PANEL_H - 48)
    bg = risk_bg.resize((plot_box[2] - plot_box[0], plot_box[3] - plot_box[1]))
    panel.paste(bg, plot_box[:2])
    draw = ImageDraw.Draw(panel)
    draw_axes(draw, bounds, plot_box)

    safety_px = meters_to_px_x(obstacle_radius_m + safety_distance_m, bounds, plot_box)
    radius_px = meters_to_px_x(obstacle_radius_m, bounds, plot_box)
    for idx, obstacle in enumerate(obstacles_xy):
        cx, cy = world_to_px(obstacle, bounds, plot_box)
        draw.ellipse((cx - safety_px, cy - safety_px, cx + safety_px, cy + safety_px), outline=(180, 35, 35), width=2)
        draw.ellipse((cx - radius_px, cy - radius_px, cx + radius_px, cy + radius_px), fill=(130, 20, 20), outline=(255, 255, 255), width=1)
        draw.text((cx + 8, cy - 14), f"obs {idx}", fill=(80, 0, 0), font=FONT_TINY)

    ux, uy = world_to_px(current_xy, bounds, plot_box)
    draw.ellipse((ux - 6, uy - 6, ux + 6, uy + 6), fill=(30, 90, 220), outline=(255, 255, 255), width=2)
    draw.text((12, PANEL_H - 25), "yellow=far, red=inside/near safety boundary", fill=(45, 45, 45), font=FONT_SMALL)
    return panel


def make_trajectory_panel(
    trajectory_xy: np.ndarray,
    current_index: int,
    obstacles_xy: np.ndarray,
    bounds,
    obstacle_radius_m: float,
    safety_distance_m: float,
    metrics,
) -> Image.Image:
    panel = Image.new("RGB", (PANEL_W, PANEL_H), "white")
    draw = ImageDraw.Draw(panel)
    draw_title(draw, "OptiTrack MAV trajectory", f"sample {metrics.sequence}")
    plot_box = (PLOT_PAD, TITLE_H + 18, PANEL_W - 22, PANEL_H - 48)
    draw_axes(draw, bounds, plot_box)

    safety_px = meters_to_px_x(obstacle_radius_m + safety_distance_m, bounds, plot_box)
    radius_px = meters_to_px_x(obstacle_radius_m, bounds, plot_box)
    for idx, obstacle in enumerate(obstacles_xy):
        cx, cy = world_to_px(obstacle, bounds, plot_box)
        draw.ellipse((cx - safety_px, cy - safety_px, cx + safety_px, cy + safety_px), outline=(220, 60, 60), width=2)
        draw.ellipse((cx - radius_px, cy - radius_px, cx + radius_px, cy + radius_px), fill=(250, 180, 180), outline=(220, 60, 60), width=2)
        draw.text((cx + 8, cy - 14), f"obs {idx}", fill=(120, 20, 20), font=FONT_TINY)

    full_pts = [world_to_px(p, bounds, plot_box) for p in trajectory_xy]
    past_pts = full_pts[: current_index + 1]
    if len(full_pts) > 1:
        draw.line(full_pts, fill=(200, 200, 200), width=2)
    if len(past_pts) > 1:
        draw.line(past_pts, fill=(40, 110, 190), width=4)

    sx, sy = full_pts[0]
    ex, ey = full_pts[-1]
    ux, uy = full_pts[current_index]
    draw.ellipse((sx - 6, sy - 6, sx + 6, sy + 6), fill=(35, 160, 65))
    draw.ellipse((ex - 6, ey - 6, ex + 6, ey + 6), fill=(20, 20, 20))
    draw.ellipse((ux - 7, uy - 7, ux + 7, uy + 7), fill=(255, 130, 20), outline=(255, 255, 255), width=2)

    closest = full_pts[metrics.closest_index]
    draw.ellipse(
        (closest[0] - 5, closest[1] - 5, closest[0] + 5, closest[1] + 5),
        outline=(140, 0, 160),
        width=2,
    )
    text = (
        f"min clearance: {metrics.min_boundary_clearance_m:.2f} m | "
        f"violation: {'yes' if metrics.safety_violation else 'no'}"
    )
    draw.text((12, PANEL_H - 25), text, fill=(45, 45, 45), font=FONT_SMALL)
    return panel


def main() -> None:
    args = parse_args()
    dataset_dir = dataset_root(args.dataset_root)
    sequence = str(args.trial_id)
    trial = read_trial_overview(dataset_dir)[sequence]
    optitrack = load_optitrack(dataset_dir, sequence)
    trajectory_xy = np.column_stack([optitrack["ground_x_m"], optitrack["ground_y_m"]])
    obstacles_xy = obstacle_array(trial.obstacles)
    metrics = compute_trial_metrics(
        sequence,
        optitrack["time_s"],
        trajectory_xy,
        obstacles_xy,
        obstacle_radius_m=args.obstacle_radius,
        safety_distance_m=args.safety_distance,
    )
    distances = pairwise_ground_distances(trajectory_xy, obstacles_xy)
    nearest_clearance = distances.min(axis=1) - args.obstacle_radius

    depth_cache = None
    if args.include_depth:
        depth_path = Path(args.depth_cache) if args.depth_cache else default_depth_cache_path(sequence)
        if not depth_path.exists():
            raise FileNotFoundError(
                f"Missing depth cache {depth_path}. Create it with: "
                f"python3 experiments/cache_monocular_depth.py --trial-id {sequence} --output {depth_path}"
            )
        depth_cache = load_depth_cache(depth_path)

    imu = None
    radar = None
    radar_y_max = 1.0
    accel_range = (-20.0, 20.0)
    gyro_range = (-3.0, 3.0)
    if args.include_sensors:
        try:
            imu = load_imu(dataset_dir, sequence)
            radar = load_radar_spectra(dataset_dir, sequence)
        except FileNotFoundError as exc:
            raise FileNotFoundError(
                f"{exc}\nFetch sensor CSV first with: "
                f"scripts/fetch_oda_sensor_sample.sh data/raw/ODA_Dataset {sequence}"
            ) from exc

        radar_log = np.log10(
            1.0 + np.vstack([radar["mag_rx1"].ravel(), radar["mag_rx2"].ravel()])
        )
        radar_y_max = float(max(1.0, np.percentile(radar_log, 99.5)))
        accel_low, accel_high = np.percentile(imu["accel_filt_mps2"], [1.0, 99.0])
        gyro_low, gyro_high = np.percentile(imu["gyro_filt_radps"], [1.0, 99.0])
        accel_pad = max(1.0, 0.10 * (accel_high - accel_low))
        gyro_pad = max(0.2, 0.10 * (gyro_high - gyro_low))
        accel_range = (float(accel_low - accel_pad), float(accel_high + accel_pad))
        gyro_range = (float(gyro_low - gyro_pad), float(gyro_high + gyro_pad))

    video_path = dataset_dir / sequence / f"{sequence}.avi"
    if not video_path.exists():
        raise FileNotFoundError(
            f"Missing RGB video {video_path}. Fetch it with: "
            f"git -C data/raw/ODA_Dataset sparse-checkout add /dataset/{sequence}/{sequence}.avi"
        )

    reader = imageio.get_reader(video_path)
    meta = reader.get_meta_data()
    source_fps = float(meta.get("fps", 29.97))
    source_duration = float(meta.get("duration", optitrack["time_s"][-1]))
    duration = min(source_duration, float(optitrack["time_s"][-1]))
    if args.max_duration is not None:
        duration = min(duration, args.max_duration)

    bounds = compute_bounds(trajectory_xy, obstacles_xy)
    risk_bg = make_risk_background(
        bounds,
        obstacles_xy,
        obstacle_radius_m=args.obstacle_radius,
        safety_distance_m=args.safety_distance,
        size=(360, 360),
    )

    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(out_path, fps=args.fps, codec="libx264", quality=8)

    n_frames = int(math.floor(duration * args.fps))
    for frame_i in range(n_frames):
        t = frame_i / args.fps
        source_i = min(int(round(t * source_fps)), int(source_duration * source_fps) - 1)
        rgb = reader.get_data(source_i)
        opt_i = int(np.searchsorted(optitrack["time_s"], t, side="left"))
        opt_i = min(max(opt_i, 0), len(trajectory_xy) - 1)
        if args.include_depth:
            assert depth_cache is not None
            depth_i = int(np.searchsorted(depth_cache["times"], t, side="left"))
            depth_i = min(max(depth_i, 0), len(depth_cache["times"]) - 1)
        if args.include_sensors:
            assert imu is not None
            assert radar is not None
            imu_i = int(np.searchsorted(imu["time_s"], t, side="left"))
            imu_i = min(max(imu_i, 0), len(imu["time_s"]) - 1)
            radar_i = int(np.searchsorted(radar["time_s"], t, side="left"))
            radar_i = min(max(radar_i, 0), len(radar["time_s"]) - 1)

        camera_panel = make_camera_panel(rgb, "Camera input", t)
        risk_panel = make_risk_panel(
            risk_bg,
            trajectory_xy[opt_i],
            obstacles_xy,
            bounds,
            obstacle_radius_m=args.obstacle_radius,
            safety_distance_m=args.safety_distance,
            clearance_m=float(nearest_clearance[opt_i]),
        )
        trajectory_panel = make_trajectory_panel(
            trajectory_xy,
            opt_i,
            obstacles_xy,
            bounds,
            obstacle_radius_m=args.obstacle_radius,
            safety_distance_m=args.safety_distance,
            metrics=metrics,
        )

        panels = [camera_panel]
        if args.include_depth:
            panels.append(make_depth_panel(depth_cache, depth_i, t))
        if args.include_sensors:
            panels.append(
                make_radar_panel(
                    radar=radar,
                    radar_index=radar_i,
                    y_max=radar_y_max,
                )
            )
            panels.append(
                make_imu_panel(
                    imu=imu,
                    imu_index=imu_i,
                    t_now=t,
                    window_s=args.sensor_window,
                    accel_range=accel_range,
                    gyro_range=gyro_range,
                )
            )
        panels.extend([risk_panel, trajectory_panel])

        canvas = Image.new("RGB", (PANEL_W * len(panels), PANEL_H), "white")
        for panel_idx, panel in enumerate(panels):
            canvas.paste(panel, (PANEL_W * panel_idx, 0))
        writer.append_data(np.asarray(canvas))

    writer.close()
    reader.close()
    print(f"Wrote {out_path}")
    print(f"Duration: {duration:.2f}s, fps: {args.fps:g}, frames: {n_frames}")
    panel_names = ["RGB"]
    if args.include_depth:
        panel_names.append("predicted depth")
    if args.include_sensors:
        panel_names.extend(["radar", "IMU"])
    panel_names.extend(["safety map", "trajectory"])
    print(f"Panels: {' + '.join(panel_names)}")
    print(
        f"Min clearance: {metrics.min_boundary_clearance_m:.3f} m, "
        f"safety violation: {metrics.safety_violation}"
    )


if __name__ == "__main__":
    main()
