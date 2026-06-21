#!/usr/bin/env python3
"""Segment ARCO PointCloud2 LiDAR frames and export 3D bounding boxes.

This script intentionally avoids ROS runtime dependencies.  It reads ROS2 bag
SQLite ``.db3`` files directly, decodes ``sensor_msgs/msg/PointCloud2`` CDR
messages for the common Ouster XYZ layout, removes a simple ground estimate,
clusters occupied voxels, and writes per-cluster 3D axis-aligned bounding boxes.
"""

from __future__ import annotations

import argparse
import csv
import sqlite3
import struct
import sys
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np


POINTFIELD_DTYPES = {
    1: ("i1", 1),
    2: ("u1", 1),
    3: ("<i2", 2),
    4: ("<u2", 2),
    5: ("<i4", 4),
    6: ("<u4", 4),
    7: ("<f4", 4),
    8: ("<f8", 8),
}


@dataclass(frozen=True)
class PointField:
    name: str
    offset: int
    datatype: int
    count: int


@dataclass
class PointCloud2:
    timestamp_ns: int
    frame_id: str
    height: int
    width: int
    fields: list[PointField]
    point_step: int
    row_step: int
    data: memoryview
    is_dense: bool


class CdrReader:
    """Minimal little-endian CDR reader for PointCloud2 messages."""

    def __init__(self, data: bytes):
        if len(data) < 4:
            raise ValueError("CDR payload too short")
        self.data = data
        # ROS2 stores a 4-byte CDR encapsulation header before message fields.
        if data[:2] not in {b"\x00\x01", b"\x01\x00"}:
            raise ValueError(f"Unexpected CDR encapsulation header: {data[:4].hex()}")
        self.offset = 4

    def align(self, size: int) -> None:
        self.offset += (-self.offset) % size

    def read_u8(self) -> int:
        value = self.data[self.offset]
        self.offset += 1
        return value

    def read_bool(self) -> bool:
        return bool(self.read_u8())

    def read_i32(self) -> int:
        self.align(4)
        value = struct.unpack_from("<i", self.data, self.offset)[0]
        self.offset += 4
        return value

    def read_u32(self) -> int:
        self.align(4)
        value = struct.unpack_from("<I", self.data, self.offset)[0]
        self.offset += 4
        return value

    def read_string(self) -> str:
        length = self.read_u32()
        raw = self.data[self.offset : self.offset + length]
        self.offset += length
        if raw.endswith(b"\x00"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")

    def read_bytes(self) -> memoryview:
        length = self.read_u32()
        start = self.offset
        self.offset += length
        return memoryview(self.data)[start : start + length]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--db3",
        default="data/processed/arco_rosbag_probe/TrafficMonitoring/TrafficMonitoring/rosbag2_2024_05_23-15_55_45_0.db3",
        help="Path to an ARCO ROS2 bag SQLite file.",
    )
    parser.add_argument("--topic", default="/ouster/points")
    parser.add_argument("--sample-name", default=None)
    parser.add_argument("--frame-count", type=int, default=6)
    parser.add_argument("--start-offset", type=int, default=0)
    parser.add_argument("--max-range", type=float, default=35.0)
    parser.add_argument("--x-min", type=float, default=1.0)
    parser.add_argument("--x-max", type=float, default=30.0)
    parser.add_argument("--y-abs", type=float, default=12.0)
    parser.add_argument("--z-min", type=float, default=-2.0)
    parser.add_argument("--z-max", type=float, default=4.0)
    parser.add_argument("--ground-percentile", type=float, default=5.0)
    parser.add_argument("--ground-margin", type=float, default=0.30)
    parser.add_argument("--voxel-size", type=float, default=0.45)
    parser.add_argument("--min-cluster-points", type=int, default=45)
    parser.add_argument("--min-cluster-voxels", type=int, default=4)
    parser.add_argument("--max-cluster-extent", type=float, default=9.0)
    parser.add_argument("--max-clusters-per-frame", type=int, default=18)
    parser.add_argument("--output", default="outputs/tables/arco_pointcloud_3d_bboxes.csv")
    parser.add_argument("--summary-output", default="outputs/tables/arco_pointcloud_segmentation_summary.csv")
    parser.add_argument("--figure-output", default="outputs/figures/arco_pointcloud_3d_bboxes.png")
    return parser.parse_args()


def parse_pointcloud2(blob: bytes, timestamp_ns: int) -> PointCloud2:
    reader = CdrReader(blob)
    reader.read_i32()  # header.stamp.sec
    reader.read_u32()  # header.stamp.nanosec
    frame_id = reader.read_string()
    height = reader.read_u32()
    width = reader.read_u32()
    field_count = reader.read_u32()
    fields: list[PointField] = []
    for _ in range(field_count):
        fields.append(
            PointField(
                name=reader.read_string(),
                offset=reader.read_u32(),
                datatype=reader.read_u8(),
                count=reader.read_u32(),
            )
        )
    is_bigendian = reader.read_bool()
    if is_bigendian:
        raise ValueError("Big-endian PointCloud2 is not supported by this lightweight parser")
    point_step = reader.read_u32()
    row_step = reader.read_u32()
    raw_data = reader.read_bytes()
    is_dense = reader.read_bool()
    return PointCloud2(
        timestamp_ns=timestamp_ns,
        frame_id=frame_id,
        height=height,
        width=width,
        fields=fields,
        point_step=point_step,
        row_step=row_step,
        data=raw_data,
        is_dense=is_dense,
    )


def field_array(cloud: PointCloud2, name: str) -> np.ndarray:
    fields = {field.name: field for field in cloud.fields}
    if name not in fields:
        raise ValueError(f"PointCloud2 field {name!r} not found; fields={sorted(fields)}")
    field = fields[name]
    dtype, _ = POINTFIELD_DTYPES[field.datatype]
    point_count = len(cloud.data) // cloud.point_step
    return np.ndarray(
        shape=(point_count,),
        dtype=np.dtype(dtype),
        buffer=cloud.data,
        offset=field.offset,
        strides=(cloud.point_step,),
    ).copy()


def xyz_points(cloud: PointCloud2) -> np.ndarray:
    x = field_array(cloud, "x").astype(np.float32, copy=False)
    y = field_array(cloud, "y").astype(np.float32, copy=False)
    z = field_array(cloud, "z").astype(np.float32, copy=False)
    points = np.column_stack([x, y, z]).astype(np.float32, copy=False)
    finite = np.isfinite(points).all(axis=1)
    nonzero = np.linalg.norm(points, axis=1) > 0.05
    return points[finite & nonzero]


def foreground_points(points: np.ndarray, args: argparse.Namespace) -> tuple[np.ndarray, float]:
    ranges = np.linalg.norm(points, axis=1)
    roi = (
        (ranges <= args.max_range)
        & (points[:, 0] >= args.x_min)
        & (points[:, 0] <= args.x_max)
        & (np.abs(points[:, 1]) <= args.y_abs)
        & (points[:, 2] >= args.z_min)
        & (points[:, 2] <= args.z_max)
    )
    cropped = points[roi]
    if len(cropped) == 0:
        return cropped, float("nan")
    ground_z = float(np.percentile(cropped[:, 2], args.ground_percentile))
    foreground = cropped[cropped[:, 2] > ground_z + args.ground_margin]
    return foreground, ground_z


def voxelize(points: np.ndarray, voxel_size: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.int32), np.empty(0, dtype=np.int64), np.empty(0, dtype=np.int64)
    keys = np.floor(points / voxel_size).astype(np.int32)
    unique, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    return unique, inverse, counts


def connected_voxel_components(voxels: np.ndarray) -> list[list[int]]:
    voxel_index = {tuple(voxel.tolist()): idx for idx, voxel in enumerate(voxels)}
    visited = np.zeros(len(voxels), dtype=bool)
    components: list[list[int]] = []
    neighbor_offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]
    for start in range(len(voxels)):
        if visited[start]:
            continue
        visited[start] = True
        component: list[int] = []
        queue: deque[int] = deque([start])
        while queue:
            idx = queue.popleft()
            component.append(idx)
            vx, vy, vz = voxels[idx]
            for dx, dy, dz in neighbor_offsets:
                neighbor = voxel_index.get((int(vx + dx), int(vy + dy), int(vz + dz)))
                if neighbor is not None and not visited[neighbor]:
                    visited[neighbor] = True
                    queue.append(neighbor)
        components.append(component)
    return components


def cluster_points(points: np.ndarray, args: argparse.Namespace) -> list[dict[str, object]]:
    voxels, inverse, voxel_counts = voxelize(points, args.voxel_size)
    components = connected_voxel_components(voxels)
    clusters: list[dict[str, object]] = []
    for component in components:
        if len(component) < args.min_cluster_voxels:
            continue
        component_arr = np.asarray(component, dtype=np.int64)
        point_mask = np.isin(inverse, component_arr)
        cluster = points[point_mask]
        if len(cluster) < args.min_cluster_points:
            continue
        mins = cluster.min(axis=0)
        maxs = cluster.max(axis=0)
        dims = maxs - mins
        if float(np.max(dims)) > args.max_cluster_extent:
            continue
        if float(np.prod(np.maximum(dims, 1e-3))) <= 0.0:
            continue
        clusters.append(
            {
                "points": cluster,
                "point_count": int(len(cluster)),
                "voxel_count": int(len(component)),
                "weighted_voxel_points": int(voxel_counts[component_arr].sum()),
                "min": mins,
                "max": maxs,
                "center": 0.5 * (mins + maxs),
                "dims": dims,
                "volume": float(np.prod(np.maximum(dims, 1e-3))),
            }
        )
    clusters.sort(key=lambda item: int(item["point_count"]), reverse=True)
    return clusters[: args.max_clusters_per_frame]


def selected_messages(db3: Path, topic: str, frame_count: int, start_offset: int) -> list[tuple[int, int, bytes]]:
    con = sqlite3.connect(db3)
    topic_row = con.execute("select id from topics where name = ?", (topic,)).fetchone()
    if topic_row is None:
        available = [row[0] for row in con.execute("select name from topics order by id")]
        raise ValueError(f"Topic {topic!r} not found in {db3}; available={available}")
    topic_id = int(topic_row[0])
    total = int(con.execute("select count(*) from messages where topic_id = ?", (topic_id,)).fetchone()[0])
    if total == 0:
        raise ValueError(f"No messages found for topic {topic!r} in {db3}")
    if frame_count <= 1:
        offsets = [min(max(start_offset, 0), total - 1)]
    else:
        max_offset = max(start_offset, total - 1)
        offsets = np.linspace(start_offset, max_offset, num=frame_count, dtype=int)
        offsets = sorted({int(min(max(offset, 0), total - 1)) for offset in offsets})
    rows: list[tuple[int, int, bytes]] = []
    for frame_index, offset in enumerate(offsets):
        row = con.execute(
            "select timestamp, data from messages where topic_id = ? order by timestamp limit 1 offset ?",
            (topic_id, offset),
        ).fetchone()
        if row is None:
            continue
        rows.append((int(offset), int(row[0]), row[1]))
    con.close()
    return rows


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"No rows to write to {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def cluster_rows(
    clusters: list[dict[str, object]],
    *,
    sample: str,
    topic: str,
    frame_offset: int,
    timestamp_ns: int,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for cluster_id, cluster in enumerate(clusters):
        mins = np.asarray(cluster["min"], dtype=float)
        maxs = np.asarray(cluster["max"], dtype=float)
        center = np.asarray(cluster["center"], dtype=float)
        dims = np.asarray(cluster["dims"], dtype=float)
        rows.append(
            {
                "dataset": "ARCO",
                "sample": sample,
                "topic": topic,
                "frame_offset": frame_offset,
                "timestamp_ns": timestamp_ns,
                "cluster_id": cluster_id,
                "point_count": cluster["point_count"],
                "voxel_count": cluster["voxel_count"],
                "bbox_min_x_m": round(float(mins[0]), 4),
                "bbox_min_y_m": round(float(mins[1]), 4),
                "bbox_min_z_m": round(float(mins[2]), 4),
                "bbox_max_x_m": round(float(maxs[0]), 4),
                "bbox_max_y_m": round(float(maxs[1]), 4),
                "bbox_max_z_m": round(float(maxs[2]), 4),
                "bbox_center_x_m": round(float(center[0]), 4),
                "bbox_center_y_m": round(float(center[1]), 4),
                "bbox_center_z_m": round(float(center[2]), 4),
                "bbox_size_x_m": round(float(dims[0]), 4),
                "bbox_size_y_m": round(float(dims[1]), 4),
                "bbox_size_z_m": round(float(dims[2]), 4),
                "bbox_volume_m3": round(float(cluster["volume"]), 4),
            }
        )
    return rows


def maybe_plot(path: Path, points: np.ndarray, clusters: list[dict[str, object]], title: str) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except Exception as exc:
        print(f"Warning: matplotlib unavailable, skip figure: {exc}", file=sys.stderr)
        return

    path.parent.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.7), constrained_layout=True)
    if len(points):
        stride = max(1, len(points) // 8000)
        show = points[::stride]
        axes[0].scatter(show[:, 0], show[:, 1], s=1.0, c="#a7a7a7", alpha=0.35, linewidths=0)
        axes[1].scatter(show[:, 0], show[:, 2], s=1.0, c="#a7a7a7", alpha=0.35, linewidths=0)

    colors = plt.cm.tab20(np.linspace(0, 1, max(1, min(len(clusters), 20))))
    for idx, cluster in enumerate(clusters[:20]):
        pts = np.asarray(cluster["points"])
        color = colors[idx % len(colors)]
        axes[0].scatter(pts[:, 0], pts[:, 1], s=2.0, color=color, alpha=0.85, linewidths=0)
        axes[1].scatter(pts[:, 0], pts[:, 2], s=2.0, color=color, alpha=0.85, linewidths=0)
        mins = np.asarray(cluster["min"], dtype=float)
        dims = np.asarray(cluster["dims"], dtype=float)
        axes[0].add_patch(Rectangle((mins[0], mins[1]), dims[0], dims[1], fill=False, color=color, linewidth=1.2))
        axes[1].add_patch(Rectangle((mins[0], mins[2]), dims[0], dims[2], fill=False, color=color, linewidth=1.2))
        center = np.asarray(cluster["center"], dtype=float)
        axes[0].text(center[0], center[1], str(idx), color="black", fontsize=7)
        axes[1].text(center[0], center[2], str(idx), color="black", fontsize=7)

    axes[0].set_title("Top-down XY clusters + 3D bbox footprint")
    axes[0].set_xlabel("x forward (m)")
    axes[0].set_ylabel("y lateral (m)")
    axes[0].grid(alpha=0.25)
    axes[0].set_aspect("equal", adjustable="box")
    axes[1].set_title("Side XZ clusters + bbox height")
    axes[1].set_xlabel("x forward (m)")
    axes[1].set_ylabel("z height (m)")
    axes[1].grid(alpha=0.25)
    fig.suptitle(title)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def infer_sample_name(db3: Path, sample_name: str | None) -> str:
    if sample_name:
        return sample_name
    parts = db3.parts
    for candidate in ("Trajectory1", "Trajectory2", "TrafficMonitoring"):
        if candidate in parts or candidate in str(db3):
            return candidate
    return db3.stem


def main() -> None:
    args = parse_args()
    db3 = Path(args.db3)
    sample = infer_sample_name(db3, args.sample_name)
    messages = selected_messages(db3, args.topic, args.frame_count, args.start_offset)
    all_cluster_rows: list[dict[str, object]] = []
    summary_rows: list[dict[str, object]] = []
    first_foreground: np.ndarray | None = None
    first_clusters: list[dict[str, object]] | None = None

    for frame_order, (frame_offset, timestamp_ns, blob) in enumerate(messages):
        cloud = parse_pointcloud2(blob, timestamp_ns)
        points = xyz_points(cloud)
        foreground, ground_z = foreground_points(points, args)
        clusters = cluster_points(foreground, args)
        if first_foreground is None:
            first_foreground = foreground
            first_clusters = clusters
        all_cluster_rows.extend(
            cluster_rows(
                clusters,
                sample=sample,
                topic=args.topic,
                frame_offset=frame_offset,
                timestamp_ns=timestamp_ns,
            )
        )
        summary_rows.append(
            {
                "dataset": "ARCO",
                "sample": sample,
                "topic": args.topic,
                "frame_order": frame_order,
                "frame_offset": frame_offset,
                "timestamp_ns": timestamp_ns,
                "height": cloud.height,
                "width": cloud.width,
                "raw_points": int(len(points)),
                "foreground_points": int(len(foreground)),
                "ground_z_estimate_m": round(float(ground_z), 4) if np.isfinite(ground_z) else "",
                "cluster_count": len(clusters),
                "largest_cluster_points": int(clusters[0]["point_count"]) if clusters else 0,
                "voxel_size_m": args.voxel_size,
                "x_min_m": args.x_min,
                "ground_margin_m": args.ground_margin,
                "method": "ground_percentile_voxel_connected_components",
            }
        )
        print(
            f"frame_offset={frame_offset} raw={len(points)} foreground={len(foreground)} "
            f"clusters={len(clusters)}"
        )

    write_csv(Path(args.summary_output), summary_rows)
    if all_cluster_rows:
        write_csv(Path(args.output), all_cluster_rows)
    else:
        write_csv(
            Path(args.output),
            [
                {
                    "dataset": "ARCO",
                    "sample": sample,
                    "topic": args.topic,
                    "frame_offset": "",
                    "timestamp_ns": "",
                    "cluster_id": "",
                    "point_count": 0,
                    "voxel_count": 0,
                    "bbox_min_x_m": "",
                    "bbox_min_y_m": "",
                    "bbox_min_z_m": "",
                    "bbox_max_x_m": "",
                    "bbox_max_y_m": "",
                    "bbox_max_z_m": "",
                    "bbox_center_x_m": "",
                    "bbox_center_y_m": "",
                    "bbox_center_z_m": "",
                    "bbox_size_x_m": "",
                    "bbox_size_y_m": "",
                    "bbox_size_z_m": "",
                    "bbox_volume_m3": "",
                }
            ],
        )
    if first_foreground is not None and first_clusters is not None:
        maybe_plot(
            Path(args.figure_output),
            first_foreground,
            first_clusters,
            f"ARCO {sample} {args.topic}: voxel clusters and 3D AABBs",
        )
    print(f"Wrote {args.summary_output}")
    print(f"Wrote {args.output}")
    print(f"Wrote {args.figure_output}")


if __name__ == "__main__":
    main()
