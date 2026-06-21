#!/usr/bin/env python3
"""Summarize external LiDAR/point-cloud stress evidence from probe tables."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arco", default="outputs/tables/arco_rosbag_topic_probe.csv")
    parser.add_argument("--multilidar", default="outputs/tables/multilidar_download_link_probe.csv")
    parser.add_argument("--output", default="outputs/tables/external_lidar_stress_summary.csv")
    parser.add_argument("--markdown-output", default="outputs/external_lidar_stress_summary.md")
    return parser.parse_args()


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        raise ValueError(f"No rows to write to {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def summarize_arco(rows: list[dict[str, str]]) -> dict[str, object] | None:
    if not rows:
        return None
    samples = sorted({row["sample"] for row in rows})
    pointcloud = [
        row
        for row in rows
        if row.get("topic_type") == "sensor_msgs/msg/PointCloud2"
        or "PointCloud" in row.get("topic_name", "")
    ]
    pointcloud2 = [row for row in rows if row.get("topic_type") == "sensor_msgs/msg/PointCloud2"]
    detection = [
        row
        for row in rows
        if row.get("topic_name") in {"/PointCloudObject", "/PointCloudDetection"}
    ]
    total_messages = sum(int(float(row.get("message_count", 0) or 0)) for row in rows)
    pc_messages = sum(int(float(row.get("message_count", 0) or 0)) for row in pointcloud)
    pc2_messages = sum(int(float(row.get("message_count", 0) or 0)) for row in pointcloud2)
    det_messages = sum(int(float(row.get("message_count", 0) or 0)) for row in detection)
    db_size_by_sample: dict[str, float] = {}
    duration_by_sample: dict[str, float] = {}
    for row in rows:
        sample = row["sample"]
        db_size_by_sample[sample] = max(
            db_size_by_sample.get(sample, 0.0), float(row.get("db_size_gib", 0) or 0.0)
        )
        duration_by_sample[sample] = max(
            duration_by_sample.get(sample, 0.0), float(row.get("duration_s_estimated", 0) or 0.0)
        )
    return {
        "source": "ARCO Dataset",
        "probe_status": "downloaded_rosbag_sqlite_topics",
        "samples": len(samples),
        "total_probe_rows": len(rows),
        "total_messages": total_messages,
        "pointcloud_topics": len({row["topic_name"] for row in pointcloud}),
        "pointcloud_messages": pc_messages,
        "pointcloud2_messages": pc2_messages,
        "pointcloud_detection_messages": det_messages,
        "estimated_duration_s": round(sum(duration_by_sample.values()), 2),
        "estimated_db_size_gib": round(sum(db_size_by_sample.values()), 4),
        "directly_comparable_to_oda": "no_ground_robot_reference_only",
        "current_scope": "topic_and_message_stress_probe_not_segmentation",
    }


def summarize_multilidar(rows: list[dict[str, str]]) -> dict[str, object] | None:
    if not rows:
        return None
    links = len(rows)
    login_required = sum(1 for row in rows if row.get("access_status") == "login_required")
    scenarios = len({row.get("scenario", "") for row in rows})
    sequences = len({row.get("sequence", "") for row in rows})
    return {
        "source": "Multi-LiDAR Multi-UAV Dataset",
        "probe_status": "link_probe_login_required",
        "samples": sequences,
        "total_probe_rows": links,
        "total_messages": 0,
        "pointcloud_topics": 0,
        "pointcloud_messages": 0,
        "pointcloud2_messages": 0,
        "pointcloud_detection_messages": 0,
        "estimated_duration_s": 0.0,
        "estimated_db_size_gib": 0.0,
        "directly_comparable_to_oda": f"partial_uav_sensing_tracking_{scenarios}_scenarios",
        "current_scope": f"{login_required}/{links}_links_require_login_no_local_pointcloud_processing",
    }


def write_markdown(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# External LiDAR / Point-Cloud Stress Summary",
        "",
        "ODA remains the main UAV obstacle-avoidance benchmark. These probes only document how external LiDAR or point-cloud sources can stress the sensing side later.",
        "",
        "| Source | Evidence | Point-cloud messages | Scope |",
        "|---|---:|---:|---|",
    ]
    for row in rows:
        lines.append(
            "| {source} | {probe_status}, {samples} sample(s), {total_probe_rows} rows | {pointcloud_messages} | {current_scope} |".format(
                **row
            )
        )
    lines.extend(
        [
            "",
            "Current limitation: no point-cloud segmentation, clustering, 3D bounding boxes, or voxel grid has been integrated into the ODA benchmark yet.",
            "",
            "Next experiment: download one accessible point-cloud sequence, run ground removal plus DBSCAN/Euclidean clustering, and export cluster centroids/bounding boxes as a perception stress test. Keep ODA as the planner benchmark.",
        ]
    )
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    args = parse_args()
    rows: list[dict[str, object]] = []
    arco = summarize_arco(read_csv(Path(args.arco)))
    if arco:
        rows.append(arco)
    multilidar = summarize_multilidar(read_csv(Path(args.multilidar)))
    if multilidar:
        rows.append(multilidar)
    write_csv(Path(args.output), rows)
    write_markdown(Path(args.markdown_output), rows)
    print(f"Wrote {args.output} ({len(rows)} rows)")
    print(f"Wrote {args.markdown_output}")


if __name__ == "__main__":
    main()
