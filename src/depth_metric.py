"""Metric depth, point-cloud, occupancy, and ESDF helpers.

Standard Depth Anything V2 checkpoints produce relative monocular depth.  The
planner-facing path in this module is intended for the metric indoor checkpoint:
``depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf``.  Monocular metric
depth can still have scale and domain error, so downstream safety claims should
compare it against ODA ground-truth metadata/OptiTrack before treating it as a
robot-ready map.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image

from src.esdf3d import ESDF3D, VoxelGridSpec, compute_esdf


METRIC_INDOOR_SMALL_MODEL_ID = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class MetricDepthResult:
    depth_m: np.ndarray
    inference_ms: float
    model_id: str
    device: str


def choose_torch_device(requested: str = "auto") -> str:
    import torch

    if requested != "auto":
        return requested
    if torch.cuda.is_available():
        return "cuda"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def intrinsics_from_fov(width: int, height: int, horizontal_fov_deg: float = 70.0) -> CameraIntrinsics:
    fov_rad = np.deg2rad(float(horizontal_fov_deg))
    fx = float(width) / (2.0 * np.tan(fov_rad / 2.0))
    fy = fx
    return CameraIntrinsics(fx=fx, fy=fy, cx=(float(width) - 1.0) / 2.0, cy=(float(height) - 1.0) / 2.0)


class MetricDepthProvider:
    """Depth Anything V2 Metric Indoor Small provider using Transformers."""

    def __init__(
        self,
        model_id: str = METRIC_INDOOR_SMALL_MODEL_ID,
        device: str = "auto",
    ) -> None:
        try:
            import torch
            from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        except Exception as exc:  # pragma: no cover - depends on optional runtime deps.
            raise RuntimeError(
                "Metric depth requires torch and transformers. Install project requirements or use MockDepthProvider."
            ) from exc

        self.model_id = model_id
        self.device = choose_torch_device(device)
        self._torch = torch
        try:
            self.processor = AutoImageProcessor.from_pretrained(model_id)
            self.model = AutoModelForDepthEstimation.from_pretrained(model_id)
        except Exception as exc:  # pragma: no cover - depends on network/cache.
            raise RuntimeError(
                f"Could not load metric depth model '{model_id}'. "
                "Use --cache-depth with an existing cache, allow Hugging Face download, or use tests with MockDepthProvider."
            ) from exc
        self.model.to(self.device)
        self.model.eval()

    def predict(self, image: Image.Image) -> MetricDepthResult:
        image = image.convert("RGB")
        started = time.perf_counter()
        inputs = self.processor(images=image, return_tensors="pt")
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        with self._torch.no_grad():
            predicted = self.model(**inputs).predicted_depth
            predicted = self._torch.nn.functional.interpolate(
                predicted.unsqueeze(1),
                size=(image.height, image.width),
                mode="bicubic",
                align_corners=False,
            ).squeeze(1)
        depth = predicted.detach().float().cpu().numpy()[0].astype(np.float32)
        return MetricDepthResult(
            depth_m=depth,
            inference_ms=(time.perf_counter() - started) * 1000.0,
            model_id=self.model_id,
            device=self.device,
        )


class MockDepthProvider:
    """Deterministic metric-depth provider for tests and offline contract checks."""

    def __init__(self, depth_m: float = 2.0, model_id: str = "mock_metric_depth", device: str = "cpu") -> None:
        self.depth_m = float(depth_m)
        self.model_id = model_id
        self.device = device

    def predict(self, image: Image.Image) -> MetricDepthResult:
        started = time.perf_counter()
        depth = np.full((image.height, image.width), self.depth_m, dtype=np.float32)
        return MetricDepthResult(
            depth_m=depth,
            inference_ms=(time.perf_counter() - started) * 1000.0,
            model_id=self.model_id,
            device=self.device,
        )


def depth_to_point_cloud(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_min_m: float = 0.20,
    depth_max_m: float = 8.0,
    stride: int = 4,
) -> np.ndarray:
    """Back-project a metric depth image to camera-frame xyz points."""

    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth_m must be a 2D array")
    step = max(1, int(stride))
    vv, uu = np.mgrid[0 : depth.shape[0] : step, 0 : depth.shape[1] : step]
    z = depth[::step, ::step]
    valid = np.isfinite(z) & (z >= depth_min_m) & (z <= depth_max_m)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32)
    z_valid = z[valid].astype(np.float32)
    x = ((uu[valid].astype(np.float32) - float(intrinsics.cx)) / float(intrinsics.fx)) * z_valid
    y = ((vv[valid].astype(np.float32) - float(intrinsics.cy)) / float(intrinsics.fy)) * z_valid
    return np.column_stack([x, y, z_valid]).astype(np.float32)


def point_cloud_to_occupancy(
    points_xyz: np.ndarray,
    voxel_size_m: float = 0.15,
    safety_radius_m: float = 0.50,
    x_limits: tuple[float, float] = (-3.0, 3.0),
    y_limits: tuple[float, float] = (-2.0, 2.0),
    z_limits: tuple[float, float] = (0.0, 8.0),
) -> tuple[np.ndarray, VoxelGridSpec]:
    """Voxelize camera-frame xyz points and inflate occupied cells."""

    resolution = float(voxel_size_m)
    origin = np.asarray([x_limits[0], y_limits[0], z_limits[0]], dtype=float)
    upper = np.asarray([x_limits[1], y_limits[1], z_limits[1]], dtype=float)
    shape = np.ceil((upper - origin) / resolution).astype(int)
    spec = VoxelGridSpec(
        nx=int(shape[0]),
        ny=int(shape[1]),
        nz=int(shape[2]),
        resolution_m=resolution,
        origin_xyz=(float(origin[0]), float(origin[1]), float(origin[2])),
    )
    occupancy = np.zeros(spec.shape, dtype=bool)
    points = np.asarray(points_xyz, dtype=float)
    if points.size == 0:
        return occupancy, spec
    inside = np.all((points >= origin) & (points < upper), axis=1)
    if not np.any(inside):
        return occupancy, spec
    indices = np.floor((points[inside] - origin) / resolution).astype(int)
    occupancy[indices[:, 0], indices[:, 1], indices[:, 2]] = True

    inflate = int(np.ceil(float(safety_radius_m) / resolution))
    if inflate > 0 and np.any(occupancy):
        try:
            from scipy.ndimage import binary_dilation

            grid = np.arange(-inflate, inflate + 1)
            dx, dy, dz = np.meshgrid(grid, grid, grid, indexing="ij")
            structure = (dx * dx + dy * dy + dz * dz) <= inflate * inflate
            occupancy = binary_dilation(occupancy, structure=structure)
        except Exception:
            occupied = np.argwhere(occupancy)
            for ix, iy, iz in occupied:
                xs = slice(max(0, ix - inflate), min(spec.nx, ix + inflate + 1))
                ys = slice(max(0, iy - inflate), min(spec.ny, iy + inflate + 1))
                zs = slice(max(0, iz - inflate), min(spec.nz, iz + inflate + 1))
                occupancy[xs, ys, zs] = True
    return occupancy, spec


def depth_to_esdf(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    depth_min_m: float = 0.20,
    depth_max_m: float = 8.0,
    voxel_size_m: float = 0.15,
    safety_radius_m: float = 0.50,
    stride: int = 4,
) -> tuple[ESDF3D, dict[str, float | int]]:
    """Convert a metric depth image to local point cloud, occupancy, and ESDF."""

    point_started = time.perf_counter()
    points = depth_to_point_cloud(depth_m, intrinsics, depth_min_m=depth_min_m, depth_max_m=depth_max_m, stride=stride)
    point_ms = (time.perf_counter() - point_started) * 1000.0
    occupancy_started = time.perf_counter()
    occupancy, spec = point_cloud_to_occupancy(
        points,
        voxel_size_m=voxel_size_m,
        safety_radius_m=safety_radius_m,
        z_limits=(0.0, depth_max_m),
    )
    occupancy_ms = (time.perf_counter() - occupancy_started) * 1000.0
    esdf_started = time.perf_counter()
    esdf = compute_esdf(occupancy, spec)
    esdf_ms = (time.perf_counter() - esdf_started) * 1000.0
    return esdf, {
        "point_count": int(len(points)),
        "occupied_voxels": int(np.sum(occupancy)),
        "pointcloud_ms": round(point_ms, 3),
        "occupancy_update_ms": round(occupancy_ms, 3),
        "esdf_update_ms": round(esdf_ms, 3),
    }


def load_cached_metric_depth(path: str | Path) -> tuple[np.ndarray, np.ndarray, dict[str, object]]:
    cache_path = Path(path)
    with np.load(cache_path, allow_pickle=False) as data:
        if "depth_m" not in data:
            raise ValueError(f"{cache_path} does not contain metric depth_m")
        times = np.asarray(data["times"], dtype=np.float32)
        depth = np.asarray(data["depth_m"], dtype=np.float32)
        meta = {key: data[key].item() if data[key].shape == () else data[key] for key in data.files if key not in {"times", "depth_m"}}
    return times, depth, meta

