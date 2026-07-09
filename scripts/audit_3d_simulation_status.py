#!/usr/bin/env python3
"""Audit the lightweight 3D UAV simulation deliverable."""

from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any


REQUIRED_FILES = [
    "sim3d/index.html",
    "sim3d/uav_sim_data.json",
    "docs/3d_simulation.md",
    "outputs/figures/uav_3d_sim_desktop.png",
    "outputs/figures/uav_3d_sim_mobile.png",
    "outputs/figures/uav_3d_sim_render_check.json",
    "outputs/videos/uav_3d_simulation_astar.mp4",
    "outputs/3d_simulation_artifacts.tar.gz",
    "scripts/export_3d_simulation_data.py",
    "scripts/serve_3d_simulation.sh",
    "scripts/verify_3d_simulation_render.js",
    "scripts/record_3d_simulation_video.js",
    "scripts/bundle_3d_simulation_artifacts.py",
    "scripts/audit_3d_simulation_status.py",
]

REQUIRED_PLANNERS = {"astar", "rrt", "mppi"}


@dataclass(frozen=True)
class Check:
    label: str
    ok: bool
    evidence: str


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _scene_check(path: Path) -> Check:
    if not path.exists():
        return Check("3D scene JSON is valid", False, "missing")
    try:
        data = _load_json(path)
    except Exception as exc:  # noqa: BLE001 - report parse failure.
        return Check("3D scene JSON is valid", False, str(exc))

    planners = data.get("planners", {})
    planner_keys = set(planners)
    missing = REQUIRED_PLANNERS - planner_keys
    if missing:
        return Check("3D scene contains required planners", False, f"missing={sorted(missing)}")

    failures = []
    for planner in sorted(REQUIRED_PLANNERS):
        entry = planners.get(planner, {})
        metrics = entry.get("metrics", {})
        if entry.get("status") != "ok":
            failures.append(f"{planner}: status={entry.get('status')}")
        if not entry.get("path"):
            failures.append(f"{planner}: empty path")
        if metrics.get("collision") is not False:
            failures.append(f"{planner}: collision={metrics.get('collision')}")
        if metrics.get("safety_violation") is not False:
            failures.append(f"{planner}: safety_violation={metrics.get('safety_violation')}")
    if failures:
        return Check("3D planner paths are safe", False, "; ".join(failures[:4]))

    obstacle_count = len(data.get("obstacles", []))
    occupied = len(data.get("grid", {}).get("occupied_cells", []))
    inflated = len(data.get("grid", {}).get("inflated_cells", []))
    evidence = f"planners={sorted(planner_keys)}, obstacles={obstacle_count}, occupied={occupied}, inflated={inflated}"
    return Check("3D planner paths are safe", True, evidence)


def _render_check(path: Path) -> Check:
    if not path.exists():
        return Check("3D render verifier output exists", False, "missing")
    try:
        rows = _load_json(path)
    except Exception as exc:  # noqa: BLE001
        return Check("3D render verifier output exists", False, str(exc))
    if not isinstance(rows, list) or not rows:
        return Check("3D render verifier output exists", False, "empty/non-list")
    failed = [row for row in rows if not row.get("ok")]
    if failed:
        return Check("3D render verifier passed", False, f"{len(failed)} failed row(s)")
    viewports = [row.get("viewport", {}).get("name") for row in rows]
    stats = [f"{row.get('viewport', {}).get('name')}:std={row.get('luminanceStd')},nonBg={row.get('nonBackgroundRatio')}" for row in rows]
    return Check("3D render verifier passed", set(viewports) >= {"desktop", "mobile"}, "; ".join(stats))


def _ffprobe_video(path: Path) -> Check:
    if not path.exists() or path.stat().st_size == 0:
        return Check("3D MP4 video exists and is readable", False, "missing/empty")
    cmd = [
        "ffprobe",
        "-v",
        "error",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=width,height,nb_frames,r_frame_rate",
        "-show_entries",
        "format=duration,size",
        "-of",
        "json",
        str(path),
    ]
    try:
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        data = json.loads(result.stdout)
    except Exception as exc:  # noqa: BLE001
        return Check("3D MP4 video exists and is readable", False, str(exc))
    stream = (data.get("streams") or [{}])[0]
    fmt = data.get("format", {})
    width = int(stream.get("width", 0))
    height = int(stream.get("height", 0))
    frames = int(stream.get("nb_frames", 0))
    duration = float(fmt.get("duration", 0.0))
    ok = width >= 1280 and height >= 720 and frames >= 200 and duration >= 9.5
    evidence = f"{width}x{height}, frames={frames}, duration={duration:.2f}s, size={fmt.get('size', path.stat().st_size)}"
    return Check("3D MP4 video exists and is readable", ok, evidence)


def _docs_check() -> Check:
    docs = Path("docs/3d_simulation.md")
    readme = Path("README.md")
    if not docs.exists() or not readme.exists():
        return Check("3D simulation docs mention run/verify/video", False, "docs or README missing")
    docs_text = docs.read_text(errors="replace")
    readme_text = readme.read_text(errors="replace")
    tokens = [
        "scripts/serve_3d_simulation.sh",
        "scripts/verify_3d_simulation_render.js",
        "scripts/record_3d_simulation_video.js",
        "outputs/videos/uav_3d_simulation_astar.mp4",
    ]
    missing = [token for token in tokens if token not in docs_text or token not in readme_text]
    return Check(
        "3D simulation docs mention run/verify/video",
        not missing,
        "all commands documented" if not missing else f"missing={missing}",
    )


def build_checks() -> list[Check]:
    missing_files = [path for path in REQUIRED_FILES if not Path(path).exists() or Path(path).stat().st_size == 0]
    checks = [
        Check(
            "3D simulation required files exist",
            not missing_files,
            "all present" if not missing_files else ", ".join(missing_files),
        ),
        _scene_check(Path("sim3d/uav_sim_data.json")),
        _render_check(Path("outputs/figures/uav_3d_sim_render_check.json")),
        _ffprobe_video(Path("outputs/videos/uav_3d_simulation_astar.mp4")),
        _docs_check(),
    ]
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fail-on-incomplete", action="store_true")
    args = parser.parse_args()

    checks = build_checks()
    all_ok = True
    print("3D simulation audit")
    print("===================")
    for check in checks:
        status = "PASS" if check.ok else "MISSING"
        print(f"{status:7} {check.label}: {check.evidence}")
        all_ok = all_ok and check.ok
    print()
    print("COMPLETE" if all_ok else "INCOMPLETE")
    return 1 if args.fail_on_incomplete and not all_ok else 0


if __name__ == "__main__":
    raise SystemExit(main())
