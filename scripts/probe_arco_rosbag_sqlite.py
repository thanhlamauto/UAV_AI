#!/usr/bin/env python3
"""Probe downloaded ARCO ROS2 bag ZIPs with sqlite3, without ROS.

The ARCO samples are ROS2 bags stored as sqlite databases inside ZIP archives.
This script extracts only metadata/YAML and *.db3 files, then reports topic
message counts. It is meant as a lightweight sensing-ingestion stress test, not
as a planner benchmark.
"""

from __future__ import annotations

import argparse
import csv
import shutil
import sqlite3
import zipfile
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="ARCO ZIP archives or extracted bag directories.")
    parser.add_argument("--extract-root", default="data/processed/arco_rosbag_probe")
    parser.add_argument("--output", default="outputs/tables/arco_rosbag_topic_probe.csv")
    parser.add_argument("--summary-output", default="outputs/arco_rosbag_stress_probe.md")
    parser.add_argument("--force-extract", action="store_true")
    return parser.parse_args()


def sample_name(path: Path) -> str:
    name = path.stem if path.suffix.lower() == ".zip" else path.name
    return name.replace("Trayectory", "Trajectory")


def extract_selected(zip_path: Path, extract_root: Path, force: bool) -> Path:
    target = extract_root / sample_name(zip_path)
    marker = target / ".extract_complete"
    if marker.exists() and not force:
        return target
    if target.exists() and force:
        shutil.rmtree(target)
    target.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(zip_path) as zf:
        members = [
            item
            for item in zf.infolist()
            if item.filename.endswith("/")
            or item.filename.lower().endswith((".db3", ".yaml", ".yml"))
        ]
        for member in members:
            zf.extract(member, target)
    marker.write_text("ok\n")
    return target


def db_size_gib(path: Path) -> float:
    return path.stat().st_size / (1024**3)


def query_db(sample: str, archive: Path, db_path: Path) -> tuple[list[dict[str, object]], dict[str, object]]:
    with sqlite3.connect(str(db_path)) as conn:
        topic_rows = conn.execute("SELECT id, name, type FROM topics ORDER BY id").fetchall()
        counts = dict(conn.execute("SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id").fetchall())
        min_ts, max_ts, total_messages = conn.execute(
            "SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages"
        ).fetchone()

    duration_s = ""
    if min_ts is not None and max_ts is not None:
        # ROS2 bag timestamps are normally nanoseconds.
        duration_s = round((float(max_ts) - float(min_ts)) / 1e9, 4)

    rows = []
    for topic_id, topic_name, topic_type in topic_rows:
        rows.append(
            {
                "dataset": "ARCO",
                "sample": sample,
                "archive": str(archive),
                "db3": str(db_path),
                "db_size_gib": round(db_size_gib(db_path), 4),
                "duration_s_estimated": duration_s,
                "topic_id": int(topic_id),
                "topic_name": topic_name,
                "topic_type": topic_type,
                "message_count": int(counts.get(topic_id, 0)),
            }
        )
    summary = {
        "sample": sample,
        "archive": str(archive),
        "db_count": 1,
        "topic_count": len(topic_rows),
        "total_messages": int(total_messages or 0),
        "duration_s_estimated": duration_s,
        "db_size_gib": round(db_size_gib(db_path), 4),
    }
    return rows, summary


def probe_input(path: Path, extract_root: Path, force: bool) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    archive = path
    root = extract_selected(path, extract_root, force) if path.suffix.lower() == ".zip" else path
    db_paths = sorted(root.rglob("*.db3"))
    if not db_paths:
        return [], [
            {
                "sample": sample_name(path),
                "archive": str(archive),
                "db_count": 0,
                "topic_count": 0,
                "total_messages": 0,
                "duration_s_estimated": "",
                "db_size_gib": 0.0,
                "note": "no db3 files found",
            }
        ]
    all_rows: list[dict[str, object]] = []
    summaries: list[dict[str, object]] = []
    for db_path in db_paths:
        rows, summary = query_db(sample_name(path), archive, db_path)
        all_rows.extend(rows)
        summaries.append(summary)
    return all_rows, summaries


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fields = list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def important_topics(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    keys = ("detection", "object", "pointcloud", "imu", "/tf")
    return [
        row
        for row in rows
        if any(key in str(row["topic_name"]).lower() for key in keys)
    ]


def write_summary(path: Path, topic_rows: list[dict[str, object]], summaries: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# ARCO ROS2 Bag Stress Probe",
        "",
        "Purpose: validate that the project can inspect radar/LiDAR/IMU ROS2 bag samples without adding ROS to the ODA benchmark pipeline.",
        "",
        "ARCO is a ground-robot dataset, so these counts support sensing generalization only. They are not directly comparable to ODA UAV obstacle-avoidance metrics.",
        "",
        "## Samples",
        "",
        "| Sample | DB files | Topics | Messages | Duration est. (s) | DB size (GiB) |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summaries:
        lines.append(
            f"| {row['sample']} | {row['db_count']} | {row['topic_count']} | "
            f"{row['total_messages']} | {row.get('duration_s_estimated', '')} | {row['db_size_gib']} |"
        )
    lines.extend(["", "## Sensor Topics", ""])
    for row in important_topics(topic_rows):
        lines.append(
            f"- `{row['sample']}` `{row['topic_name']}` ({row['topic_type']}): "
            f"{row['message_count']} messages"
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            "- Use these samples to stress-test radar/LiDAR/IMU parsing and feature extraction assumptions.",
            "- Keep ODA as the primary UAV avoidance benchmark because it provides MAV trajectory and obstacle ground truth.",
            "- Do not use ARCO planner/path metrics as head-to-head results against ODA.",
            "",
        ]
    )
    path.write_text("\n".join(lines))


def main() -> None:
    args = parse_args()
    extract_root = Path(args.extract_root)
    all_topic_rows: list[dict[str, object]] = []
    summaries: list[dict[str, object]] = []
    for input_value in args.inputs:
        rows, summary_rows = probe_input(Path(input_value), extract_root, args.force_extract)
        all_topic_rows.extend(rows)
        summaries.extend(summary_rows)
    write_csv(Path(args.output), all_topic_rows)
    write_summary(Path(args.summary_output), all_topic_rows, summaries)
    print(f"Wrote {args.output} with {len(all_topic_rows)} topic rows")
    print(f"Wrote {args.summary_output}")


if __name__ == "__main__":
    main()
