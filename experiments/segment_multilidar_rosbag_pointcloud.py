#!/usr/bin/env python3
"""Segment Multi-LiDAR ROS1 point-cloud frames and export 3D boxes.

This script handles the ROS1 ``.bag`` files from the Multi-LiDAR Multi-UAV
dataset without launching ROS.  It uses ``rosbags`` to deserialize
``sensor_msgs/msg/PointCloud2`` and Livox ``CustomMsg`` messages, then reuses
the lightweight voxel clustering and 3D axis-aligned bounding-box utilities
from the ARCO point-cloud stress test.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from experiments.segment_arco_pointcloud import (
    cluster_points,
    foreground_points,
    maybe_plot,
    write_csv,
    xyz_points,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", default="data/raw/multi_lidar/Calibration.bag")
    parser.add_argument("--topic", default="/ouster/points")
    parser.add_argument("--sample-name", default="Calibration")
    parser.add_argument("--frame-count", type=int, default=8)
    parser.add_argument("--start-offset", type=int, default=0)
    parser.add_argument("--max-range", type=float, default=35.0)
    parser.add_argument("--x-min", type=float, default=0.5)
    parser.add_argument("--x-max", type=float, default=30.0)
    parser.add_argument("--y-abs", type=float, default=12.0)
    parser.add_argument("--z-min", type=float, default=-2.5)
    parser.add_argument("--z-max", type=float, default=5.0)
    parser.add_argument("--ground-percentile", type=float, default=5.0)
    parser.add_argument("--ground-margin", type=float, default=0.25)
    parser.add_argument("--voxel-size", type=float, default=0.35)
    parser.add_argument("--min-cluster-points", type=int, default=35)
    parser.add_argument("--min-cluster-voxels", type=int, default=4)
    parser.add_argument("--max-cluster-extent", type=float, default=8.0)
    parser.add_argument("--max-clusters-per-frame", type=int, default=20)
    parser.add_argument(
        "--output",
        default="outputs/tables/multilidar_pointcloud_3d_bboxes.csv",
    )
    parser.add_argument(
        "--summary-output",
        default="outputs/tables/multilidar_pointcloud_segmentation_summary.csv",
    )
    parser.add_argument(
        "--figure-output",
        default="outputs/figures/multilidar_pointcloud_3d_bboxes.png",
    )
    return parser.parse_args()


SUPPORTED_POINT_CLOUD_TYPES = {
    "sensor_msgs/msg/PointCloud2",
    "livox_ros_driver/msg/CustomMsg",
    "livox_ros_driver2/msg/CustomMsg",
}


def topic_connections(reader: AnyReader, topic: str):
    connections = [conn for conn in reader.connections if conn.topic == topic]
    if not connections:
        available = sorted((conn.topic, conn.msgtype) for conn in reader.connections)
        raise ValueError(f"Topic {topic!r} not found. Available topics: {available}")
    msgtypes = sorted({conn.msgtype for conn in connections})
    unsupported = [msgtype for msgtype in msgtypes if msgtype not in SUPPORTED_POINT_CLOUD_TYPES]
    if unsupported:
        supported = sorted(SUPPORTED_POINT_CLOUD_TYPES)
        raise ValueError(f"Topic {topic!r} has unsupported point-cloud type(s) {unsupported}. Supported: {supported}")
    return connections


def message_xyz_points(message: object, msgtype: str) -> np.ndarray:
    if msgtype == "sensor_msgs/msg/PointCloud2":
        return xyz_points(message)
    if msgtype in {"livox_ros_driver/msg/CustomMsg", "livox_ros_driver2/msg/CustomMsg"}:
        points = np.asarray([(point.x, point.y, point.z) for point in message.points], dtype=float)
        if points.size == 0:
            return np.empty((0, 3), dtype=float)
        return points[np.isfinite(points).all(axis=1)]
    raise ValueError(f"Unsupported point-cloud message type: {msgtype}")


def message_shape(message: object, msgtype: str) -> tuple[object, object]:
    if msgtype == "sensor_msgs/msg/PointCloud2":
        return int(message.height), int(message.width)
    if msgtype in {"livox_ros_driver/msg/CustomMsg", "livox_ros_driver2/msg/CustomMsg"}:
        return "", int(getattr(message, "point_num", len(getattr(message, "points", []))))
    return "", ""


def count_messages(bag: Path, topic: str) -> int:
    with AnyReader([bag]) as reader:
        connections = topic_connections(reader, topic)
        return sum(1 for _ in reader.messages(connections=connections))


def selected_offsets(total: int, frame_count: int, start_offset: int) -> list[int]:
    if total <= 0:
        return []
    start = min(max(start_offset, 0), total - 1)
    if frame_count <= 1:
        return [start]
    offsets = np.linspace(start, total - 1, num=frame_count, dtype=int)
    return sorted({int(min(max(offset, 0), total - 1)) for offset in offsets})


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
                "dataset": "Multi-LiDAR Multi-UAV",
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


def write_topic_inventory(bag: Path, path: Path) -> None:
    rows: list[dict[str, object]] = []
    with AnyReader([bag]) as reader:
        for conn in sorted(reader.connections, key=lambda item: item.topic):
            rows.append(
                {
                    "dataset": "Multi-LiDAR Multi-UAV",
                    "bag": bag.name,
                    "topic": conn.topic,
                    "msgtype": conn.msgtype,
                    "offered_qos_profiles": getattr(conn, "offered_qos_profiles", ""),
                }
            )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    bag = Path(args.bag)
    total = count_messages(bag, args.topic)
    targets = set(selected_offsets(total, args.frame_count, args.start_offset))
    if not targets:
        raise ValueError(f"No messages found for {args.topic!r} in {bag}")

    inventory_output = Path("outputs/tables/multilidar_rosbag_topic_inventory.csv")
    write_topic_inventory(bag, inventory_output)

    all_cluster_rows: list[dict[str, object]] = []
    summary_rows: list[dict[str, object]] = []
    first_foreground: np.ndarray | None = None
    first_clusters: list[dict[str, object]] | None = None

    with AnyReader([bag]) as reader:
        connections = topic_connections(reader, args.topic)
        for offset, (conn, timestamp_ns, rawdata) in enumerate(reader.messages(connections=connections)):
            if offset not in targets:
                continue
            cloud = reader.deserialize(rawdata, conn.msgtype)
            points = message_xyz_points(cloud, conn.msgtype)
            height, width = message_shape(cloud, conn.msgtype)
            foreground, ground_z = foreground_points(points, args)
            clusters = cluster_points(foreground, args)
            if first_foreground is None:
                first_foreground = foreground
                first_clusters = clusters
            all_cluster_rows.extend(
                cluster_rows(
                    clusters,
                    sample=args.sample_name,
                    topic=args.topic,
                    frame_offset=offset,
                    timestamp_ns=int(timestamp_ns),
                )
            )
            summary_rows.append(
                {
                    "dataset": "Multi-LiDAR Multi-UAV",
                    "sample": args.sample_name,
                    "bag": bag.name,
                    "topic": args.topic,
                    "msgtype": conn.msgtype,
                    "frame_offset": offset,
                    "timestamp_ns": int(timestamp_ns),
                    "height": height,
                    "width": width,
                    "raw_points": int(len(points)),
                    "foreground_points": int(len(foreground)),
                    "ground_z_estimate_m": round(float(ground_z), 4) if np.isfinite(ground_z) else "",
                    "cluster_count": len(clusters),
                    "largest_cluster_points": int(clusters[0]["point_count"]) if clusters else 0,
                    "voxel_size_m": args.voxel_size,
                    "x_min_m": args.x_min,
                    "ground_margin_m": args.ground_margin,
                    "method": "ros1_pointcloud_ground_percentile_voxel_connected_components",
                }
            )
            print(
                f"frame_offset={offset} raw={len(points)} foreground={len(foreground)} "
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
                    "dataset": "Multi-LiDAR Multi-UAV",
                    "sample": args.sample_name,
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
            f"Multi-LiDAR {args.sample_name} {args.topic}: voxel clusters and 3D AABBs",
        )
    print(f"Wrote {inventory_output}")
    print(f"Wrote {args.summary_output}")
    print(f"Wrote {args.output}")
    print(f"Wrote {args.figure_output}")


if __name__ == "__main__":
    main()
