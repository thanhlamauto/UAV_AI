#!/usr/bin/env python3
"""Audit current evidence against the active ODA 300-trial project goal."""

from __future__ import annotations

import csv
from pathlib import Path


def count_rows(path: str | Path) -> int:
    path = Path(path)
    if not path.exists() or path.stat().st_size == 0:
        return 0
    with path.open(newline="") as f:
        return sum(1 for _ in csv.DictReader(f))


def csv_values(path: str | Path, column: str) -> list[str]:
    with Path(path).open(newline="") as f:
        return [row[column] for row in csv.DictReader(f)]


def ready_count(path: str | Path) -> tuple[int, int]:
    path = Path(path)
    if not path.exists() or path.stat().st_size == 0:
        return 0, 0
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    return sum(int(row.get("ready", 0)) for row in rows), len(rows)


def has_column(path: str | Path, column: str) -> bool:
    path = Path(path)
    if not path.exists() or path.stat().st_size == 0:
        return False
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        return column in (reader.fieldnames or [])


def main() -> None:
    checks = []

    readiness_300 = Path("outputs/tables/target_300_trials_readiness.csv")
    manifest_300 = Path("outputs/tables/target_300_trials_manifest.csv")
    metrics_300 = Path("outputs/tables/batch_planner_metrics_300.csv")
    summary_300 = Path("outputs/tables/planner_comparison_summary_300.csv")
    balanced = Path("outputs/tables/perception_risk_ablation_balanced_metrics_dpt.csv")
    recall_tuned = Path("outputs/tables/perception_risk_ablation_recall_tuned_metrics_dpt.csv")
    onnx_timing = Path("outputs/tables/depth_onnx_timing_dpt_probe.csv")
    external_probe = Path("outputs/tables/external_dataset_probe.csv")
    depth50_timing = Path("outputs/tables/depth_batch_timing_depth_anything_v2_small_50.csv")
    depth50_features = Path("outputs/tables/perception_risk_features_depth_anything_v2_small_50.csv")
    depth50_balanced = Path("outputs/tables/perception_risk_ablation_balanced_metrics_depth_anything_v2_small_50.csv")
    depth50_recall = Path("outputs/tables/perception_risk_ablation_recall_tuned_metrics_depth_anything_v2_small_50.csv")
    depth50_calibration = Path("outputs/tables/depth_stability_calibration_depth_anything_v2_small_50.csv")
    arco_probe = Path("outputs/tables/arco_rosbag_topic_probe.csv")
    multilidar_probe = Path("outputs/tables/multilidar_download_link_probe.csv")
    radar_rd = Path("outputs/tables/radar_range_doppler_summary.csv")
    obstacle_repr = Path("outputs/tables/obstacle_representation_aggregate.csv")
    trajectory_feasibility = Path("outputs/tables/trajectory_feasibility_summary.csv")
    lidar_stress = Path("outputs/tables/external_lidar_stress_summary.csv")

    manifest_rows = count_rows(manifest_300)
    ready_rows, readiness_rows = ready_count(readiness_300)

    methods = set(csv_values(metrics_300, "method")) if metrics_300.exists() and metrics_300.stat().st_size else set()
    required_methods = {"human", "straight_line", "astar_grid", "rrt", "rrt_star", "mppi"}
    has_geometric = any(method.startswith("geometric_bypass") for method in methods)

    checks.append(("300-trial manifest exists", manifest_rows >= 300, f"{manifest_rows} rows"))
    checks.append(("300 trials fully downloaded", ready_rows >= 300 and readiness_rows >= 300, f"{ready_rows}/{readiness_rows} ready"))
    checks.append(("300-trial batch benchmark output exists", count_rows(metrics_300) >= 2100, f"{count_rows(metrics_300)} metric rows"))
    checks.append(("risk labels included", has_column(metrics_300, "future_risk_count"), "future_risk_count column"))
    checks.append(("straight-line baseline included", "straight_line" in methods, f"methods={sorted(methods)}"))
    checks.append(("geometric bypass baseline included", has_geometric, f"methods={sorted(methods)}"))
    checks.append(("human trajectory included", "human" in methods, f"methods={sorted(methods)}"))
    checks.append(("required planner baselines included", required_methods.issubset(methods), f"methods={sorted(methods)}"))
    checks.append(("300-trial planner summary exists", count_rows(summary_300) >= 8, f"{count_rows(summary_300)} rows"))
    checks.append(("imbalance macro-F1 tuning output exists", count_rows(balanced) >= 4 and has_column(balanced, "model_balanced_accuracy"), f"{count_rows(balanced)} rows"))
    checks.append(("imbalance recall tuning output exists", count_rows(recall_tuned) >= 4 and has_column(recall_tuned, "recall_future_risk"), f"{count_rows(recall_tuned)} rows"))
    checks.append(("ONNX depth timing output exists", count_rows(onnx_timing) >= 3 and has_column(onnx_timing, "inference_seconds_per_frame"), f"{count_rows(onnx_timing)} rows"))
    checks.append(("external dataset probe exists", count_rows(external_probe) >= 3, f"{count_rows(external_probe)} rows"))
    checks.append(("Depth Anything V2 Small 50-trial timing exists", count_rows(depth50_timing) >= 50 and has_column(depth50_timing, "inference_seconds_per_frame"), f"{count_rows(depth50_timing)} rows"))
    checks.append(("Depth Anything V2 Small 50-trial features exist", count_rows(depth50_features) >= 2500 and has_column(depth50_features, "future_risk_label"), f"{count_rows(depth50_features)} rows"))
    checks.append(("Depth Anything V2 Small 50-trial balanced ablation exists", count_rows(depth50_balanced) >= 4 and has_column(depth50_balanced, "model_balanced_accuracy"), f"{count_rows(depth50_balanced)} rows"))
    checks.append(("Depth Anything V2 Small 50-trial recall ablation exists", count_rows(depth50_recall) >= 4 and has_column(depth50_recall, "recall_future_risk"), f"{count_rows(depth50_recall)} rows"))
    checks.append(("Depth Anything V2 Small 50-trial calibration exists", count_rows(depth50_calibration) >= 1 and has_column(depth50_calibration, "spearman_depth_median_clearance"), f"{count_rows(depth50_calibration)} rows"))
    checks.append(("ARCO ROS2 bag topic stress probe exists", count_rows(arco_probe) >= 30 and has_column(arco_probe, "message_count"), f"{count_rows(arco_probe)} rows"))
    checks.append(("Multi-LiDAR download link probe exists", count_rows(multilidar_probe) >= 20 and has_column(multilidar_probe, "access_status"), f"{count_rows(multilidar_probe)} rows"))
    checks.append(("radar range-Doppler summary exists", count_rows(radar_rd) >= 3 and has_column(radar_rd, "mean_peak_doppler_bin"), f"{count_rows(radar_rd)} rows"))
    checks.append(("obstacle representation aggregate exists", count_rows(obstacle_repr) >= 2 and has_column(obstacle_repr, "mean_occupancy_area_m2"), f"{count_rows(obstacle_repr)} rows"))
    checks.append(("trajectory feasibility summary exists", count_rows(trajectory_feasibility) >= 8 and has_column(trajectory_feasibility, "p95_smoothness_heading_change"), f"{count_rows(trajectory_feasibility)} rows"))
    checks.append(("external LiDAR stress summary exists", count_rows(lidar_stress) >= 2 and has_column(lidar_stress, "pointcloud_messages"), f"{count_rows(lidar_stress)} rows"))

    print("Goal completion audit")
    print("=====================")
    all_ok = True
    for label, ok, evidence in checks:
        status = "PASS" if ok else "MISSING"
        print(f"{status:7} {label}: {evidence}")
        all_ok = all_ok and ok
    print()
    print("COMPLETE" if all_ok else "INCOMPLETE")


if __name__ == "__main__":
    main()
