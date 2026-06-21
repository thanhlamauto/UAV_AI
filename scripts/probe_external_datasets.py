#!/usr/bin/env python3
"""Probe mentor-suggested external datasets without pulling large rosbags.

The intent is to decide which dataset is worth trying next after ODA, while
keeping the current project lightweight and reproducible.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import urllib.error
import urllib.request
from pathlib import Path


MULTI_LIDAR_URL = "https://tiers.github.io/multi_lidar_multi_uav_dataset/"
ARCO_URL = "https://robotics.upo.es/datasets/ArcoDataset/main.html"
HEPP_URL = "https://arxiv.org/abs/2505.17438"

ARCO_DOWNLOADS = [
    {
        "sequence": "Trajectory1",
        "scenario": "dynamic outdoor/open-building trajectory",
        "duration_s": "193.746",
        "contents_url": "https://robotics.upo.es/datasets/ArcoDataset/Trayectory1",
        "download_url": "https://robotics.upo.es/datasets/ArcoDataset/bags/Trayectory1.zip",
    },
    {
        "sequence": "Trajectory2",
        "scenario": "dynamic outdoor/open-building trajectory",
        "duration_s": "216.55",
        "contents_url": "https://robotics.upo.es/datasets/ArcoDataset/Trayectory2",
        "download_url": "https://robotics.upo.es/datasets/ArcoDataset/bags/Trayectory2.zip",
    },
    {
        "sequence": "TrafficMonitoring",
        "scenario": "static traffic-monitoring radar/LiDAR recording",
        "duration_s": "121",
        "contents_url": "https://robotics.upo.es/datasets/ArcoDataset/Traffic",
        "download_url": "https://robotics.upo.es/datasets/ArcoDataset/bags/TrafficMonitoring.zip",
    },
]


MULTI_LIDAR_ROWS = [
    ("HolybroStnd01", "structured indoor up/down", "8.5", "31.6", "Easy"),
    ("HolybroStnd02", "structured indoor square", "24.4", "90", "Easy"),
    ("HolybroStnd03", "structured indoor circle", "20.6", "76", "Easy"),
    ("HolybroStnd04", "structured indoor spiral", "26.5", "98", "Easy"),
    ("Holybro01", "unstructured indoor", "18.5", "68", "Easy"),
    ("Holybro02", "unstructured indoor", "19.4", "72", "Easy"),
    ("Holybro03", "unstructured indoor", "21.8", "81", "Easy"),
    ("Holybro04", "unstructured indoor", "20.8", "77", "Medium"),
    ("Holybro05", "unstructured indoor", "25.0", "93", "Medium"),
    ("HolybroOut01", "unstructured outdoor", "10.1", "37.8", "Medium"),
    ("HolybroOut02", "unstructured outdoor", "10.9", "40.6", "Medium"),
    ("Autel01", "unstructured indoor", "11.1", "41.4", "Easy"),
    ("Autel02", "unstructured indoor", "16.3", "60", "Easy"),
    ("Autel03", "unstructured indoor", "13.0", "48.6", "Easy"),
    ("Autel04", "unstructured indoor", "12.7", "47.3", "Medium"),
    ("Autel05", "unstructured indoor", "13.5", "50.2", "Hard"),
    ("AutelOut01", "unstructured outdoor", "10.1", "37.6", "Hard"),
    ("AutelOut02", "unstructured outdoor", "11.1", "41.3", "Hard"),
    ("Tello01", "unstructured indoor", "13.4", "49.9", "Medium"),
    ("Tello02", "unstructured indoor", "15.5", "57.8", "Medium"),
    ("Tello03", "unstructured indoor", "15.5", "57.8", "Hard"),
    ("Tello04", "unstructured indoor", "18.7", "69", "Hard"),
    ("Tello05", "unstructured indoor", "14.5", "54.1", "Hard"),
    ("TelloOut01", "unstructured outdoor", "10.3", "38.4", "Hard"),
    ("TelloOut02", "unstructured outdoor", "7.1", "26.4", "Hard"),
    ("Calibration", "calibration office indoor", "1.3", "9.3", ""),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="outputs/tables/external_dataset_probe.csv")
    parser.add_argument("--summary-output", default="outputs/external_dataset_extension_plan.md")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--skip-network", action="store_true")
    return parser.parse_args()


def url_text(url: str, timeout: float) -> str:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read().decode("utf-8", "replace")


def head_url(url: str, timeout: float) -> tuple[str, int | None, str]:
    request = urllib.request.Request(url, method="HEAD")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            content_length = response.headers.get("Content-Length")
            content_type = response.headers.get("Content-Type", "")
            return str(response.status), int(content_length) if content_length else None, content_type
    except urllib.error.HTTPError as exc:
        return str(exc.code), None, ""
    except Exception as exc:
        return f"error:{type(exc).__name__}", None, ""


def gib(size_bytes: int | None) -> str:
    if size_bytes is None:
        return ""
    return f"{size_bytes / (1024 ** 3):.3f}"


def arco_topic_summary(text: str) -> str:
    topics = re.findall(r"Topic: ([^|]+) \| Type: ([^|]+) \| Count: ([0-9]+)", text)
    keep = []
    for topic, msg_type, count in topics:
        if any(key in topic.lower() for key in ["detection", "object", "pointcloud", "imu", "/tf"]):
            keep.append(f"{topic.strip()}:{count}")
    return "; ".join(keep[:10])


def build_rows(skip_network: bool, timeout: float) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []

    multi_page_status = "not_checked"
    if not skip_network:
        try:
            html = url_text(MULTI_LIDAR_URL, timeout)
            multi_page_status = "ok" if "Multi-LiDAR" in html or "Holybro" in html else "unexpected_page"
        except Exception as exc:
            multi_page_status = f"error:{type(exc).__name__}"

    for sequence, scenario, size_gb, duration_s, difficulty in MULTI_LIDAR_ROWS:
        rows.append(
            {
                "dataset": "Multi-LiDAR Multi-UAV",
                "sequence": sequence,
                "scenario": scenario,
                "vehicle": "UAV target tracked by external LiDAR rig",
                "modalities": "OS1-64 LiDAR; Livox Mid-360; Livox Avia; RGB-D/RGB; IMU; MOCAP",
                "format": "ROS bag",
                "size_gb_advertised": size_gb,
                "size_gib_probe": "",
                "duration_s": duration_s,
                "difficulty": difficulty,
                "download_url": MULTI_LIDAR_URL,
                "probe_status": multi_page_status,
                "recommended_use": "cross-dataset UAV tracking/perception stress test",
                "directly_comparable_to_oda": "partial: UAV + MOCAP, but tracking dataset not obstacle-avoidance benchmark",
            }
        )

    for item in ARCO_DOWNLOADS:
        status, size_bytes, content_type = ("not_checked", None, "")
        topic_summary = ""
        if not skip_network:
            status, size_bytes, content_type = head_url(item["download_url"], timeout)
            try:
                topic_summary = arco_topic_summary(url_text(item["contents_url"], timeout))
            except Exception as exc:
                topic_summary = f"contents_error:{type(exc).__name__}"
        rows.append(
            {
                "dataset": "ARCO",
                "sequence": item["sequence"],
                "scenario": item["scenario"],
                "vehicle": "ground robot",
                "modalities": "Ouster LiDAR; radar detections/objects; IMU; TF",
                "format": "ROS2 bag sqlite3 zip",
                "size_gb_advertised": "",
                "size_gib_probe": gib(size_bytes),
                "duration_s": item["duration_s"],
                "difficulty": "",
                "download_url": item["download_url"],
                "probe_status": f"HEAD {status} {content_type}".strip(),
                "recommended_use": "radar/LiDAR/IMU parsing and sensing-context validation",
                "directly_comparable_to_oda": "no: ground robot localization/mapping, not UAV obstacle avoidance",
                "topic_summary": topic_summary,
            }
        )

    return rows


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row.keys()})
    priority = [
        "dataset",
        "sequence",
        "scenario",
        "vehicle",
        "modalities",
        "format",
        "size_gb_advertised",
        "size_gib_probe",
        "duration_s",
        "difficulty",
        "probe_status",
        "recommended_use",
        "directly_comparable_to_oda",
        "download_url",
        "topic_summary",
    ]
    fieldnames = [field for field in priority if field in fields] + [field for field in fields if field not in priority]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    multi_rows = [row for row in rows if row["dataset"] == "Multi-LiDAR Multi-UAV"]
    arco_rows = [row for row in rows if row["dataset"] == "ARCO"]
    hard_multi = [row for row in multi_rows if row.get("difficulty") == "Hard"]
    small_arco = sorted(arco_rows, key=lambda row: float(row.get("size_gib_probe") or "999"))[:1]

    lines = [
        "# External Dataset Extension Plan",
        "",
        "## Why Add External Data",
        "",
        "The 300-trial ODA benchmark is useful for statistical confidence, but ODA remains relatively controlled: indoor MAV flights with one or two obstacles and OptiTrack ground truth. External datasets should therefore be used as stress/positioning evidence, not as a replacement benchmark.",
        "",
        "## Probe Result",
        "",
        f"- Multi-LiDAR Multi-UAV: {len(multi_rows)} advertised sequences, including {len(hard_multi)} hard sequences. Best next probe: `Tello03`, `Tello04`, `Tello05`, `TelloOut01`, or `TelloOut02` because they are hard UAV tracking cases with LiDAR/MOCAP context.",
        f"- ARCO: {len(arco_rows)} direct ROS2 bag ZIP entries probed. Smallest detected candidate: `{small_arco[0]['sequence'] if small_arco else 'n/a'}` at about `{small_arco[0].get('size_gib_probe', '') if small_arco else ''}` GiB.",
        "",
        "## Recommended Next Experiments",
        "",
        "1. Keep ODA as the planner benchmark and scale it to 300 trials.",
        "2. Download one ARCO ZIP first only to validate radar/LiDAR/IMU ingestion from ROS2 sqlite bags. Do not compare ARCO path-planning metrics against ODA.",
        "3. Use Multi-LiDAR as a later UAV perception/tracking stress dataset. Its rosbags are much larger, so start with one hard short sequence only after the ODA 300-trial table is stable.",
        "4. In the report, present this as generalization pressure: ODA answers avoidance metrics; ARCO tests radar/LiDAR sensing context; Multi-LiDAR tests UAV tracking/perception complexity.",
        "",
        "## Sources",
        "",
        f"- Multi-LiDAR Multi-UAV Dataset: {MULTI_LIDAR_URL}",
        f"- ARCO Dataset: {ARCO_URL}",
        f"- HEPP paper for high-speed UAV motivation: {HEPP_URL}",
        "",
    ]
    path.write_text("\n".join(lines))


def main() -> None:
    args = parse_args()
    rows = build_rows(skip_network=args.skip_network, timeout=args.timeout)
    write_csv(Path(args.output), rows)
    write_summary(Path(args.summary_output), rows)
    print(f"Wrote {args.output}")
    print(f"Wrote {args.summary_output}")


if __name__ == "__main__":
    main()
