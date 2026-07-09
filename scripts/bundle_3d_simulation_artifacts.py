#!/usr/bin/env python3
"""Bundle lightweight 3D simulation artifacts.

This excludes datasets and keeps only the browser demo, scene JSON, docs,
screenshots, video, and scripts needed to reproduce the visual simulation.
"""

from __future__ import annotations

import argparse
import tarfile
from pathlib import Path


DEFAULT_INCLUDES = [
    "sim3d/index.html",
    "sim3d/uav_sim_data.json",
    "docs/3d_simulation.md",
    "outputs/figures/uav_3d_sim_desktop.png",
    "outputs/figures/uav_3d_sim_mobile.png",
    "outputs/figures/uav_3d_sim_render_check.json",
    "outputs/videos/uav_3d_simulation_astar.mp4",
    "scripts/export_3d_simulation_data.py",
    "scripts/serve_3d_simulation.sh",
    "scripts/verify_3d_simulation_render.js",
    "scripts/record_3d_simulation_video.js",
    "scripts/bundle_3d_simulation_artifacts.py",
    "scripts/audit_3d_simulation_status.py",
]


def bundle(output_path: Path, includes: list[str]) -> list[Path]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    added: list[Path] = []
    with tarfile.open(output_path, "w:gz") as tar:
        for include in includes:
            path = Path(include)
            if not path.exists() or not path.is_file():
                continue
            tar.add(path, arcname=str(path))
            added.append(path)
    return added


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/3d_simulation_artifacts.tar.gz"))
    parser.add_argument("--include", action="append", default=[], help="Additional file to include.")
    args = parser.parse_args()

    added = bundle(args.output, DEFAULT_INCLUDES + args.include)
    print(f"Wrote {args.output} with {len(added)} file(s)")
    for path in added:
        print(f"  {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
