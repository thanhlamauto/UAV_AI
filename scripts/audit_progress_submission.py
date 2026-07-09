#!/usr/bin/env python3
"""Audit the current progress-submission package."""

from __future__ import annotations

import argparse
import csv
import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path


REQUIRED_FILES = [
    "reports/uav_oda_progress_technical_report.pdf",
    "reports/uav_oda_progress_technical_report.tex",
    "outputs/progress_submission_index.md",
    "outputs/videos/uav_3d_simulation_astar.mp4",
    "outputs/videos/level3_full_3d_voxel_esdf_mppi.mp4",
    "outputs/videos/level3_dynamic_indoor_events_esdf_mppi.mp4",
    "outputs/videos/level3_dynamic_indoor_pov_esdf_mppi.mp4",
    "outputs/videos/level3_realistic_indoor_chase_fused_esdf_mppi.mp4",
    "outputs/videos/level3_realistic_indoor_pov_esdf_mppi.mp4",
    "outputs/videos/isaacsim_indoor_rgbd_lidar_dynamic_demo.mp4",
    "outputs/videos/isaacsim_indoor_third_person_rgbd_lidar_dynamic_demo.mp4",
    "outputs/videos/level3_indoor_doorway_turn_esdf_mppi.mp4",
    "outputs/videos/sensor_ablation_planner_decision_demo.mp4",
    "outputs/videos/mppi_offboard_controller_setpoint_demo.mp4",
    "outputs/figures/isaacsim_demo/isaacsim_indoor_midframe.png",
    "outputs/figures/isaacsim_demo/isaacsim_indoor_third_person_midframe.png",
    "outputs/figures/level3_video_preview/level3_doorway_turn_midframe.png",
    "outputs/figures/level3_video_preview/sensor_ablation_planner_decision_midframe.png",
    "outputs/figures/level3_video_preview/mppi_offboard_controller_midframe.png",
    "outputs/tables/isaacsim_indoor_sensor_demo_metrics.csv",
    "outputs/tables/isaacsim_indoor_third_person_sensor_demo_metrics.csv",
    "outputs/tables/level3_indoor_doorway_turn_mppi.csv",
    "outputs/tables/sensor_ablation_planner_decision_metrics.csv",
    "outputs/tables/mppi_offboard_controller_setpoint_metrics.csv",
    "outputs/isaacsim_indoor_sensor_demo_summary.md",
    "outputs/isaacsim_indoor_third_person_sensor_demo_summary.md",
    "outputs/level3_indoor_doorway_turn_summary.md",
    "outputs/sensor_ablation_planner_decision_summary.md",
    "outputs/mppi_offboard_controller_setpoint_summary.md",
    "outputs/px4_mppi_offboard_controller_status.md",
    "outputs/3d_simulation_artifacts.tar.gz",
    "outputs/figures/uav_3d_sim_desktop.png",
    "outputs/figures/uav_3d_sim_mobile.png",
    "outputs/tables/planner_comparison_summary_300.csv",
    "outputs/tables/batch_planner_metrics_300.csv",
    "outputs/tables/perception_risk_features_depth_anything_v2_small_50.csv",
    "outputs/tables/multilidar_tello03_ouster_pointcloud_3d_bboxes.csv",
    "outputs/figures/multilidar_tello03_ouster_pointcloud_3d_bboxes.png",
    "scripts/audit_3d_simulation_status.py",
]

PDF_REQUIRED_TEXT = [
    "Nguyễn Thanh Lâm",
    "Hoàng Công Phát",
    "Viettel High Tech",
    "Thí nghiệm 7: Lightweight 3D simulation",
    "PointCloud2 LiDAR",
    "Isaac Sim",
]

PDF_FORBIDDEN_TEXT = [
    "Hoàng Khánh Đồng",
    "Nguyễn Thành Lâm",
    "Dự án đã chuyển từ prototype",
    "prototype trực quan hóa nhỏ",
]


@dataclass(frozen=True)
class Check:
    label: str
    ok: bool
    evidence: str


def _count_csv_rows(path: Path) -> int:
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(newline="") as f:
        return sum(1 for _ in csv.DictReader(f))


def _pdf_text(path: Path) -> str:
    if not shutil.which("pdftotext"):
        return ""
    result = subprocess.run(["pdftotext", str(path), "-"], check=True, capture_output=True, text=True)
    return result.stdout


def _check_pdf() -> list[Check]:
    path = Path("reports/uav_oda_progress_technical_report.pdf")
    if not path.exists() or path.stat().st_size == 0:
        return [Check("Progress PDF exists", False, "missing/empty")]
    checks = [Check("Progress PDF exists", True, f"{path.stat().st_size} bytes")]
    try:
        text = _pdf_text(path)
    except Exception as exc:  # noqa: BLE001
        checks.append(Check("Progress PDF text can be extracted", False, str(exc)))
        return checks
    if not text:
        checks.append(Check("Progress PDF text can be extracted", False, "pdftotext unavailable or empty"))
        return checks
    missing = [token for token in PDF_REQUIRED_TEXT if token not in text]
    forbidden = [token for token in PDF_FORBIDDEN_TEXT if token in text]
    checks.append(
        Check(
            "Progress PDF has correct identity and 3D section",
            not missing,
            "all required text present" if not missing else f"missing={missing}",
        )
    )
    checks.append(
        Check(
            "Progress PDF has no stale identity/prototype wording",
            not forbidden,
            "no stale text found" if not forbidden else f"forbidden={forbidden}",
        )
    )
    return checks


def _check_video(path: Path, label: str, min_frames: int = 200, min_duration_s: float = 9.5) -> Check:
    if not path.exists() or path.stat().st_size == 0:
        return Check(label, False, "missing/empty")
    if not shutil.which("ffprobe"):
        return Check(label, True, f"{path.stat().st_size} bytes; ffprobe unavailable")
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
        data = json.loads(subprocess.run(cmd, check=True, capture_output=True, text=True).stdout)
    except Exception as exc:  # noqa: BLE001
        return Check(label, False, str(exc))
    stream = (data.get("streams") or [{}])[0]
    fmt = data.get("format", {})
    width = int(stream.get("width", 0))
    height = int(stream.get("height", 0))
    frames = int(stream.get("nb_frames", 0))
    duration = float(fmt.get("duration", 0))
    ok = width >= 1280 and height >= 720 and frames >= min_frames and duration >= min_duration_s
    return Check(label, ok, f"{width}x{height}, {frames} frames, {duration:.2f}s")


def _check_index() -> Check:
    path = Path("outputs/progress_submission_index.md")
    if not path.exists():
        return Check("Submission index exists and references deliverables", False, "missing")
    text = path.read_text(errors="replace")
    tokens = [
        "reports/uav_oda_progress_technical_report.pdf",
        "outputs/videos/uav_3d_simulation_astar.mp4",
        "outputs/videos/level3_full_3d_voxel_esdf_mppi.mp4",
        "outputs/videos/level3_dynamic_indoor_pov_esdf_mppi.mp4",
        "outputs/videos/level3_realistic_indoor_chase_fused_esdf_mppi.mp4",
        "outputs/videos/level3_realistic_indoor_pov_esdf_mppi.mp4",
        "outputs/videos/isaacsim_indoor_rgbd_lidar_dynamic_demo.mp4",
        "outputs/videos/isaacsim_indoor_third_person_rgbd_lidar_dynamic_demo.mp4",
        "outputs/videos/level3_indoor_doorway_turn_esdf_mppi.mp4",
        "outputs/videos/sensor_ablation_planner_decision_demo.mp4",
        "outputs/videos/mppi_offboard_controller_setpoint_demo.mp4",
        "outputs/tables/isaacsim_indoor_sensor_demo_metrics.csv",
        "outputs/tables/isaacsim_indoor_third_person_sensor_demo_metrics.csv",
        "outputs/tables/level3_indoor_doorway_turn_mppi.csv",
        "outputs/tables/sensor_ablation_planner_decision_metrics.csv",
        "outputs/tables/mppi_offboard_controller_setpoint_metrics.csv",
        "outputs/px4_mppi_offboard_controller_status.md",
        "outputs/3d_simulation_artifacts.tar.gz",
        "outputs/tables/planner_comparison_summary_300.csv",
        "Hoàng Công Phát",
    ]
    missing = [token for token in tokens if token not in text]
    return Check(
        "Submission index exists and references deliverables",
        not missing,
        "all references present" if not missing else f"missing={missing}",
    )


def build_checks() -> list[Check]:
    checks: list[Check] = []
    missing = [path for path in REQUIRED_FILES if not Path(path).exists() or Path(path).stat().st_size == 0]
    checks.append(Check("Required submission files exist", not missing, "all present" if not missing else ", ".join(missing)))
    checks.extend(_check_pdf())
    checks.append(_check_video(Path("outputs/videos/uav_3d_simulation_astar.mp4"), "3D simulation MP4 is readable"))
    checks.append(
        _check_video(
            Path("outputs/videos/level3_full_3d_voxel_esdf_mppi.mp4"),
            "Level-3 ESDF/MPPI MP4 is readable",
            min_frames=250,
            min_duration_s=11.5,
        )
    )
    checks.append(
        _check_video(
            Path("outputs/videos/level3_dynamic_indoor_pov_esdf_mppi.mp4"),
            "Level-3 dynamic indoor POV MP4 is readable",
            min_frames=240,
            min_duration_s=13.5,
        )
    )
    checks.append(
        _check_video(
            Path("outputs/videos/level3_realistic_indoor_chase_fused_esdf_mppi.mp4"),
            "Level-3 realistic indoor chase MP4 is readable",
            min_frames=300,
            min_duration_s=14.5,
        )
    )
    checks.append(
        _check_video(
            Path("outputs/videos/level3_realistic_indoor_pov_esdf_mppi.mp4"),
            "Level-3 realistic indoor POV MP4 is readable",
            min_frames=300,
            min_duration_s=14.5,
        )
    )
    checks.append(
        _check_video(
            Path("outputs/videos/isaacsim_indoor_rgbd_lidar_dynamic_demo.mp4"),
            "Isaac Sim indoor RGB-D/LiDAR MP4 is readable",
            min_frames=180,
            min_duration_s=9.5,
        )
    )
    checks.append(
        _check_video(
            Path("outputs/videos/isaacsim_indoor_third_person_rgbd_lidar_dynamic_demo.mp4"),
            "Isaac Sim third-person/chase MP4 is readable",
            min_frames=180,
            min_duration_s=9.5,
        )
    )
    checks.append(
        _check_video(
            Path("outputs/videos/level3_indoor_doorway_turn_esdf_mppi.mp4"),
            "Level-3 doorway-turn MPPI MP4 is readable",
            min_frames=200,
            min_duration_s=11.5,
        )
    )
    checks.append(
        _check_video(
            Path("outputs/videos/sensor_ablation_planner_decision_demo.mp4"),
            "Sensor-ablation planner decision MP4 is readable",
            min_frames=250,
            min_duration_s=14.5,
        )
    )
    checks.append(
        _check_video(
            Path("outputs/videos/mppi_offboard_controller_setpoint_demo.mp4"),
            "MPPI Offboard controller setpoint MP4 is readable",
            min_frames=120,
            min_duration_s=11.5,
        )
    )
    checks.append(_check_index())
    isaac_rows = _count_csv_rows(Path("outputs/tables/isaacsim_indoor_sensor_demo_metrics.csv"))
    checks.append(
        Check(
            "Isaac Sim sensor demo metrics exist",
            isaac_rows >= 180,
            f"{isaac_rows} rows",
        )
    )
    third_person_rows = _count_csv_rows(Path("outputs/tables/isaacsim_indoor_third_person_sensor_demo_metrics.csv"))
    checks.append(
        Check(
            "Isaac Sim third-person metrics exist",
            third_person_rows >= 180,
            f"{third_person_rows} rows",
        )
    )
    doorway_rows = _count_csv_rows(Path("outputs/tables/level3_indoor_doorway_turn_mppi.csv"))
    checks.append(
        Check(
            "Level-3 doorway-turn metrics exist",
            doorway_rows >= 200,
            f"{doorway_rows} rows",
        )
    )
    ablation_rows = _count_csv_rows(Path("outputs/tables/sensor_ablation_planner_decision_metrics.csv"))
    checks.append(
        Check(
            "Sensor-ablation planner decision metrics exist",
            ablation_rows >= 3,
            f"{ablation_rows} rows",
        )
    )
    mppi_controller_rows = _count_csv_rows(Path("outputs/tables/mppi_offboard_controller_setpoint_metrics.csv"))
    checks.append(
        Check(
            "MPPI Offboard controller metrics exist",
            mppi_controller_rows >= 120,
            f"{mppi_controller_rows} rows",
        )
    )
    checks.append(
        Check(
            "300-trial planner summary exists",
            _count_csv_rows(Path("outputs/tables/planner_comparison_summary_300.csv")) >= 8,
            f"{_count_csv_rows(Path('outputs/tables/planner_comparison_summary_300.csv'))} rows",
        )
    )
    checks.append(
        Check(
            "300-trial planner detail rows exist",
            _count_csv_rows(Path("outputs/tables/batch_planner_metrics_300.csv")) >= 2100,
            f"{_count_csv_rows(Path('outputs/tables/batch_planner_metrics_300.csv'))} rows",
        )
    )
    checks.append(
        Check(
            "50-trial perception-risk features exist",
            _count_csv_rows(Path("outputs/tables/perception_risk_features_depth_anything_v2_small_50.csv")) >= 2500,
            f"{_count_csv_rows(Path('outputs/tables/perception_risk_features_depth_anything_v2_small_50.csv'))} rows",
        )
    )
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fail-on-incomplete", action="store_true")
    args = parser.parse_args()

    checks = build_checks()
    all_ok = True
    print("Progress submission audit")
    print("=========================")
    for check in checks:
        status = "PASS" if check.ok else "MISSING"
        print(f"{status:7} {check.label}: {check.evidence}")
        all_ok = all_ok and check.ok
    print()
    print("COMPLETE" if all_ok else "INCOMPLETE")
    return 1 if args.fail_on_incomplete and not all_ok else 0


if __name__ == "__main__":
    raise SystemExit(main())
