#!/usr/bin/env python3
"""Probe Multi-LiDAR Multi-UAV dataset download links.

The dataset page points to SharePoint-hosted rosbags. This script checks which
links are still reachable and records the advertised sequence context. It does
not download large rosbags by default.
"""

from __future__ import annotations

import argparse
import csv
import re
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


DATASET_URL = "https://tiers.github.io/multi_lidar_multi_uav_dataset/"

SEQUENCES = [
    ("HolybroStnd01", "structured indoor up/down", "Easy"),
    ("HolybroStnd02", "structured indoor square", "Easy"),
    ("HolybroStnd03", "structured indoor circle", "Easy"),
    ("HolybroStnd04", "structured indoor spiral", "Easy"),
    ("Holybro01", "unstructured indoor", "Easy"),
    ("Holybro02", "unstructured indoor", "Easy"),
    ("Holybro03", "unstructured indoor", "Easy"),
    ("Holybro04", "unstructured indoor", "Medium"),
    ("Holybro05", "unstructured indoor", "Medium"),
    ("HolybroOut01", "unstructured outdoor", "Medium"),
    ("HolybroOut02", "unstructured outdoor", "Medium"),
    ("Autel01", "unstructured indoor", "Easy"),
    ("Autel02", "unstructured indoor", "Easy"),
    ("Autel03", "unstructured indoor", "Easy"),
    ("Autel04", "unstructured indoor", "Medium"),
    ("Autel05", "unstructured indoor", "Hard"),
    ("AutelOut01", "unstructured outdoor", "Hard"),
    ("AutelOut02", "unstructured outdoor", "Hard"),
    ("Tello01", "unstructured indoor", "Medium"),
    ("Tello02", "unstructured indoor", "Medium"),
    ("Tello03", "unstructured indoor", "Hard"),
    ("Tello04", "unstructured indoor", "Hard"),
    ("Tello05", "unstructured indoor", "Hard"),
    ("TelloOut01", "unstructured outdoor", "Hard"),
    ("TelloOut02", "unstructured outdoor", "Hard"),
    ("Calibration", "calibration office indoor", ""),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--page-url", default=DATASET_URL)
    parser.add_argument("--output", default="outputs/tables/multilidar_download_link_probe.csv")
    parser.add_argument("--summary-output", default="outputs/multilidar_download_probe.md")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--max-links", type=int, default=0, help="0 means all links.")
    return parser.parse_args()


def fetch_text(url: str, timeout: float) -> str:
    with urllib.request.urlopen(url, timeout=timeout) as response:
        return response.read().decode("utf-8", "replace")


def extract_sharepoint_links(html: str) -> list[str]:
    links = re.findall(r'href=["\']([^"\']+)["\']', html)
    sharepoint = []
    seen = set()
    for link in links:
        if "sharepoint.com" not in link:
            continue
        link = urllib.parse.unquote(link)
        if link not in seen:
            seen.add(link)
            sharepoint.append(link)
    return sharepoint


def probe_link(url: str, timeout: float) -> dict[str, object]:
    request = urllib.request.Request(url, method="HEAD", headers={"User-Agent": "Mozilla/5.0"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            final_url = response.geturl()
            content_type = response.headers.get("Content-Type", "")
            content_length = response.headers.get("Content-Length", "")
            requires_auth = "login.microsoftonline.com" in final_url or "text/html" in content_type.lower()
            return {
                "http_status": response.status,
                "final_url": final_url,
                "content_type": content_type,
                "content_length_bytes": content_length,
                "requires_auth": int(requires_auth),
                "access_status": "login_required" if requires_auth else "download_ready",
                "error": "",
            }
    except urllib.error.HTTPError as exc:
        final_url = exc.geturl()
        content_type = exc.headers.get("Content-Type", "") if exc.headers else ""
        requires_auth = "login.microsoftonline.com" in final_url or exc.code in {401, 403}
        return {
            "http_status": exc.code,
            "final_url": final_url,
            "content_type": content_type,
            "content_length_bytes": "",
            "requires_auth": int(requires_auth),
            "access_status": "login_required" if requires_auth else "http_error",
            "error": str(exc.reason),
        }
    except Exception as exc:
        return {
            "http_status": "",
            "final_url": "",
            "content_type": "",
            "content_length_bytes": "",
            "requires_auth": "",
            "access_status": "probe_error",
            "error": f"{type(exc).__name__}: {exc}",
        }


def sequence_for_index(index: int) -> tuple[str, str, str]:
    if index < len(SEQUENCES):
        return SEQUENCES[index]
    return (f"extra_link_{index + 1}", "", "")


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["note"])
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, rows: list[dict[str, object]]) -> None:
    ok_rows = [row for row in rows if row.get("access_status") == "download_ready"]
    login_rows = [row for row in rows if row.get("access_status") == "login_required"]
    hard_ok = [row for row in ok_rows if row.get("difficulty") == "Hard"]
    lines = [
        "# Multi-LiDAR Download Link Probe",
        "",
        "Purpose: check whether the SharePoint-hosted Multi-LiDAR Multi-UAV samples are currently reachable before treating them as downloadable stress-test data.",
        "",
        f"- Probed links: {len(rows)}",
        f"- Direct download-ready links: {len(ok_rows)}",
        f"- Login-required links: {len(login_rows)}",
        f"- Download-ready hard-sequence links: {len(hard_ok)}",
        "",
        "## Recommended Use",
        "",
        "- Use Multi-LiDAR as UAV perception/tracking generalization evidence only.",
        "- Do not replace ODA as the primary obstacle-avoidance benchmark.",
        "- If SharePoint links fail, cite the dataset page and keep this as a documented access blocker rather than spending the main project budget on data wrangling.",
        "",
        "## Hard Sequences To Try First",
        "",
    ]
    for row in rows:
        if row.get("difficulty") == "Hard":
            lines.append(
                f"- `{row['sequence']}`: `{row.get('access_status')}` "
                f"(HTTP `{row.get('http_status') or row.get('error')}`), "
                f"{row['scenario']}"
            )
    lines.append("")
    path.write_text("\n".join(lines))


def main() -> None:
    args = parse_args()
    html = fetch_text(args.page_url, args.timeout)
    links = extract_sharepoint_links(html)
    if args.max_links > 0:
        links = links[: args.max_links]
    rows = []
    for index, link in enumerate(links):
        sequence, scenario, difficulty = sequence_for_index(index)
        probed = probe_link(link, args.timeout)
        rows.append(
            {
                "dataset": "Multi-LiDAR Multi-UAV",
                "sequence": sequence,
                "scenario": scenario,
                "difficulty": difficulty,
                "link_index": index + 1,
                "download_url": link,
                **probed,
                "directly_comparable_to_oda": "partial: UAV sensing/tracking, not ODA-style obstacle-avoidance benchmark",
            }
        )
    write_csv(Path(args.output), rows)
    write_summary(Path(args.summary_output), rows)
    print(f"Wrote {args.output} with {len(rows)} rows")
    print(f"Wrote {args.summary_output}")


if __name__ == "__main__":
    main()
