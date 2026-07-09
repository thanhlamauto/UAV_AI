"""Lightweight 3D voxel occupancy and ESDF utilities.

The project uses this module as a local, dependency-light stand-in for the map
that NVBlox should provide on a GPU/ROS2 server.  The API is intentionally
small: build an occupied voxel grid, compute a signed distance field, and query
that field at continuous xyz positions.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VoxelGridSpec:
    """Metric definition for a 3D voxel grid.

    The occupancy array shape is always `(nx, ny, nz)`.  Coordinates are in
    meters, with `z` representing altitude.
    """

    nx: int
    ny: int
    nz: int
    resolution_m: float
    origin_xyz: tuple[float, float, float]

    @property
    def shape(self) -> tuple[int, int, int]:
        return (int(self.nx), int(self.ny), int(self.nz))

    @property
    def origin(self) -> np.ndarray:
        return np.asarray(self.origin_xyz, dtype=float)

    @property
    def upper(self) -> np.ndarray:
        return self.origin + np.asarray(self.shape, dtype=float) * float(self.resolution_m)

    def voxel_centers(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        x = self.origin[0] + (np.arange(self.nx, dtype=float) + 0.5) * self.resolution_m
        y = self.origin[1] + (np.arange(self.ny, dtype=float) + 0.5) * self.resolution_m
        z = self.origin[2] + (np.arange(self.nz, dtype=float) + 0.5) * self.resolution_m
        return x, y, z


@dataclass(frozen=True)
class ESDF3D:
    """Signed distance field over a voxel grid.

    Positive values are free-space distance to the nearest occupied voxel.
    Occupied voxels have non-positive values.  Querying outside the grid returns
    `outside_value_m`.
    """

    spec: VoxelGridSpec
    signed_distance_m: np.ndarray
    occupancy: np.ndarray
    outside_value_m: float = -1.0

    def contains(self, points_xyz: np.ndarray) -> np.ndarray:
        points = np.asarray(points_xyz, dtype=float)
        lower = self.spec.origin
        upper = self.spec.upper
        return np.all((points >= lower) & (points <= upper), axis=-1)

    def query_distance(self, points_xyz: np.ndarray) -> np.ndarray:
        """Trilinearly interpolate ESDF distance at continuous xyz points."""

        points = np.asarray(points_xyz, dtype=float)
        original_shape = points.shape[:-1]
        flat = points.reshape((-1, 3))
        idx = (flat - self.spec.origin) / float(self.spec.resolution_m) - 0.5
        i0 = np.floor(idx).astype(int)
        frac = idx - i0
        out = np.full(flat.shape[0], float(self.outside_value_m), dtype=float)

        valid = (
            (i0[:, 0] >= 0)
            & (i0[:, 1] >= 0)
            & (i0[:, 2] >= 0)
            & (i0[:, 0] < self.spec.nx - 1)
            & (i0[:, 1] < self.spec.ny - 1)
            & (i0[:, 2] < self.spec.nz - 1)
        )
        if not np.any(valid):
            return out.reshape(original_shape)

        ii = i0[valid]
        ff = frac[valid]
        values = np.zeros(ii.shape[0], dtype=float)
        field = self.signed_distance_m
        for dx in (0, 1):
            wx = (1.0 - ff[:, 0]) if dx == 0 else ff[:, 0]
            for dy in (0, 1):
                wy = (1.0 - ff[:, 1]) if dy == 0 else ff[:, 1]
                for dz in (0, 1):
                    wz = (1.0 - ff[:, 2]) if dz == 0 else ff[:, 2]
                    values += wx * wy * wz * field[ii[:, 0] + dx, ii[:, 1] + dy, ii[:, 2] + dz]
        out[valid] = values
        return out.reshape(original_shape)

    def query(self, points_xyz: np.ndarray) -> np.ndarray:
        """Alias used by planner-facing ESDF contracts."""

        return self.query_distance(points_xyz)


def empty_occupancy(spec: VoxelGridSpec) -> np.ndarray:
    return np.zeros(spec.shape, dtype=bool)


def _index_bounds(spec: VoxelGridSpec, low: np.ndarray, high: np.ndarray) -> tuple[slice, slice, slice]:
    lo = np.floor((low - spec.origin) / spec.resolution_m).astype(int)
    hi = np.ceil((high - spec.origin) / spec.resolution_m).astype(int)
    lo = np.maximum(lo, 0)
    hi = np.minimum(hi, np.asarray(spec.shape, dtype=int))
    return (slice(lo[0], hi[0]), slice(lo[1], hi[1]), slice(lo[2], hi[2]))


def mark_box(
    occupancy: np.ndarray,
    spec: VoxelGridSpec,
    center_xyz: tuple[float, float, float],
    size_xyz: tuple[float, float, float],
) -> None:
    center = np.asarray(center_xyz, dtype=float)
    size = np.asarray(size_xyz, dtype=float)
    low = center - size / 2.0
    high = center + size / 2.0
    xs, ys, zs = _index_bounds(spec, low, high)
    if xs.stop <= xs.start or ys.stop <= ys.start or zs.stop <= zs.start:
        return
    x_centers, y_centers, z_centers = spec.voxel_centers()
    x = x_centers[xs][:, None, None]
    y = y_centers[ys][None, :, None]
    z = z_centers[zs][None, None, :]
    mask = (np.abs(x - center[0]) <= size[0] / 2.0) & (np.abs(y - center[1]) <= size[1] / 2.0) & (
        np.abs(z - center[2]) <= size[2] / 2.0
    )
    occupancy[xs, ys, zs] |= mask


def mark_cylinder(
    occupancy: np.ndarray,
    spec: VoxelGridSpec,
    center_xy: tuple[float, float],
    radius_m: float,
    z_min_m: float,
    z_max_m: float,
) -> None:
    low = np.asarray([center_xy[0] - radius_m, center_xy[1] - radius_m, z_min_m], dtype=float)
    high = np.asarray([center_xy[0] + radius_m, center_xy[1] + radius_m, z_max_m], dtype=float)
    xs, ys, zs = _index_bounds(spec, low, high)
    if xs.stop <= xs.start or ys.stop <= ys.start or zs.stop <= zs.start:
        return
    x_centers, y_centers, z_centers = spec.voxel_centers()
    x = x_centers[xs][:, None, None]
    y = y_centers[ys][None, :, None]
    z = z_centers[zs][None, None, :]
    radial = (x - center_xy[0]) ** 2 + (y - center_xy[1]) ** 2 <= radius_m**2
    vertical = (z >= z_min_m) & (z <= z_max_m)
    occupancy[xs, ys, zs] |= radial & vertical


def _chamfer_distance_transform(source: np.ndarray, resolution_m: float) -> np.ndarray:
    """Approximate Euclidean distance to source voxels using 26-neighbor Dijkstra."""

    source = np.asarray(source, dtype=bool)
    dist = np.full(source.shape, np.inf, dtype=np.float32)
    heap: list[tuple[float, int, int, int]] = []
    for x, y, z in np.argwhere(source):
        dist[x, y, z] = 0.0
        heapq.heappush(heap, (0.0, int(x), int(y), int(z)))

    if not heap:
        return dist

    offsets: list[tuple[int, int, int, float]] = []
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                step = float(resolution_m) * math.sqrt(dx * dx + dy * dy + dz * dz)
                offsets.append((dx, dy, dz, step))

    nx, ny, nz = source.shape
    while heap:
        current, x, y, z = heapq.heappop(heap)
        if current > float(dist[x, y, z]) + 1e-8:
            continue
        for dx, dy, dz, step in offsets:
            xx = x + dx
            yy = y + dy
            zz = z + dz
            if not (0 <= xx < nx and 0 <= yy < ny and 0 <= zz < nz):
                continue
            candidate = current + step
            if candidate < float(dist[xx, yy, zz]):
                dist[xx, yy, zz] = candidate
                heapq.heappush(heap, (candidate, xx, yy, zz))
    return dist


def compute_esdf(occupancy: np.ndarray, spec: VoxelGridSpec, prefer_scipy: bool = True) -> ESDF3D:
    occupancy = np.asarray(occupancy, dtype=bool)
    if occupancy.shape != spec.shape:
        raise ValueError(f"occupancy shape {occupancy.shape} does not match spec {spec.shape}")
    if not np.any(occupancy):
        field = np.full(spec.shape, np.inf, dtype=np.float32)
        return ESDF3D(spec=spec, signed_distance_m=field, occupancy=occupancy)
    if np.all(occupancy):
        field = np.full(spec.shape, -np.inf, dtype=np.float32)
        return ESDF3D(spec=spec, signed_distance_m=field, occupancy=occupancy)

    if prefer_scipy:
        try:
            from scipy.ndimage import distance_transform_edt  # type: ignore

            outside = distance_transform_edt(~occupancy, sampling=spec.resolution_m).astype(np.float32)
            inside = distance_transform_edt(occupancy, sampling=spec.resolution_m).astype(np.float32)
            signed = outside
            signed[occupancy] = -inside[occupancy]
            return ESDF3D(spec=spec, signed_distance_m=signed, occupancy=occupancy)
        except Exception:
            pass

    outside = _chamfer_distance_transform(occupancy, spec.resolution_m)
    inside = _chamfer_distance_transform(~occupancy, spec.resolution_m)
    signed = outside.astype(np.float32)
    signed[occupancy] = -inside[occupancy]
    return ESDF3D(spec=spec, signed_distance_m=signed, occupancy=occupancy)
