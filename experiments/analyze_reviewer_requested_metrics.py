#!/usr/bin/env python3
"""Generate reviewer-requested analyses from existing ODA-Bench artifacts.

This script does not rerun planners, policies, or simulators.  It recomputes
statistics from saved per-case CSVs and reloads the saved risk MLP only to
evaluate operating points on the existing test split.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np

from experiments.train_risk_predictor import BinaryMLP, matrix, ttc_distance_score
from src.oda_bench_downstream import FEATURE_COLUMNS, binary_metrics, read_csv, write_csv


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--planner-detail", default="outputs/tables/batch_planner_metrics_300.csv")
    parser.add_argument("--bc-detail", default="outputs/tables/oda_bc_rollout_detail.csv")
    parser.add_argument("--pybullet-detail", default="outputs/tables/pybullet_validation_detail.csv")
    parser.add_argument("--risk-samples", default="outputs/tables/oda_risk_labeled_samples.csv")
    parser.add_argument("--risk-model", default="outputs/models/small_mlp_risk_predictor.npz")
    parser.add_argument("--outputs-dir", default="outputs")
    parser.add_argument("--base-obstacle-radius", type=float, default=0.20)
    parser.add_argument("--obstacle-radii", nargs="*", type=float, default=[0.15, 0.20, 0.25, 0.30])
    parser.add_argument("--safety-margins", nargs="*", type=float, default=[0.30, 0.40, 0.50, 0.60])
    parser.add_argument("--target-recalls", nargs="*", type=float, default=[0.95, 0.99])
    return parser.parse_args()


def wilson_interval(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n <= 0:
        return 0.0, 0.0
    p = k / n
    denom = 1.0 + z * z / n
    center = (p + z * z / (2.0 * n)) / denom
    half = z * np.sqrt((p * (1.0 - p) + z * z / (4.0 * n)) / n) / denom
    return float(max(0.0, center - half)), float(min(1.0, center + half))


def rate_ci_rows(rows: list[dict[str, str]], source: str) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    methods = sorted({row["method"] for row in rows})
    for method in methods:
        group = [row for row in rows if row["method"] == method]
        n = len(group)
        for metric, column in [
            ("collision_rate", "collision"),
            ("safety_violation_rate", "safety_violation"),
            ("success_rate", "success"),
            ("latency_violation_rate", "latency_violation"),
        ]:
            if not group or column not in group[0]:
                continue
            values = [int(float(row[column])) for row in group if str(row.get(column, "")) != ""]
            if not values:
                continue
            k = sum(values)
            lo, hi = wilson_interval(k, len(values))
            out.append(
                {
                    "source": source,
                    "method": method,
                    "metric": metric,
                    "n": len(values),
                    "count": k,
                    "rate": round(k / len(values), 6),
                    "wilson95_low": round(lo, 6),
                    "wilson95_high": round(hi, 6),
                }
            )
    return out


def sensitivity_rows(
    planner_rows: list[dict[str, str]],
    obstacle_radii: list[float],
    safety_margins: list[float],
    base_obstacle_radius: float,
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    methods = sorted({row["method"] for row in planner_rows})
    for method in methods:
        group = [row for row in planner_rows if row["method"] == method]
        if not group:
            continue
        base_boundary = np.asarray([float(row["min_boundary_clearance_m"]) for row in group], dtype=float)
        if "min_center_distance_m" in group[0]:
            center_dist = np.asarray([float(row["min_center_distance_m"]) for row in group], dtype=float)
        else:
            center_dist = base_boundary + base_obstacle_radius
        for radius in obstacle_radii:
            # Use the saved boundary clearance at the nominal radius so margin
            # sensitivity exactly matches the published evaluator rows.
            if abs(radius - base_obstacle_radius) < 1e-12:
                boundary = base_boundary
            else:
                boundary = center_dist - radius
            for margin in safety_margins:
                out.append(
                    {
                        "method": method,
                        "obstacle_radius_m": radius,
                        "safety_margin_m": margin,
                        "trials": len(group),
                        "collision_rate": round(float(np.mean(boundary < -1e-9)), 6),
                        "violation_rate": round(float(np.mean(boundary < margin - 1e-9)), 6),
                        "mean_min_clearance_m": round(float(np.mean(boundary)), 6),
                    }
                )
    return out


def load_risk_model(path: str | Path) -> tuple[BinaryMLP, np.ndarray, np.ndarray]:
    payload = np.load(path)
    model = BinaryMLP(input_dim=payload["w1"].shape[0])
    for key in ["w1", "b1", "w2", "b2"]:
        setattr(model, key, payload[key].astype(np.float32))
    return model, payload["mean"].astype(np.float32), payload["std"].astype(np.float32)


def threshold_for_recall(y_true: np.ndarray, score: np.ndarray, target_recall: float) -> float:
    candidates = np.unique(np.concatenate([[0.0, 1.0], score.astype(float)]))
    best_threshold = 0.0
    best_precision = -1.0
    for threshold in candidates:
        metrics = binary_metrics(y_true.astype(int), score, threshold=float(threshold))
        if metrics["risk_recall"] + 1e-12 >= target_recall and metrics["precision"] > best_precision:
            best_precision = metrics["precision"]
            best_threshold = float(threshold)
    return best_threshold


def risk_operating_point_rows(samples_path: str, model_path: str, target_recalls: list[float]) -> list[dict[str, object]]:
    rows = read_csv(samples_path)
    test_rows = [row for row in rows if row["split"] == "test"]
    x_test, y_test = matrix(test_rows)
    ttc_score = ttc_distance_score(test_rows)
    model, mean, std = load_risk_model(model_path)
    x_test_n = (x_test - mean) / std
    mlp_score = model.predict_proba(x_test_n)

    out: list[dict[str, object]] = []
    for model_name, score in [("ttc_distance_threshold", ttc_score), ("small_mlp_risk_predictor", mlp_score)]:
        for target in target_recalls:
            threshold = threshold_for_recall(y_test, score, target)
            metrics = binary_metrics(y_test.astype(int), score, threshold=threshold)
            out.append(
                {
                    "method": model_name,
                    "target_recall": target,
                    "threshold": round(float(threshold), 6),
                    "risk_recall": round(float(metrics["risk_recall"]), 6),
                    "false_negative_rate": round(float(metrics["false_negative_rate"]), 6),
                    "precision": round(float(metrics["precision"]), 6),
                    "balanced_accuracy": round(float(metrics["balanced_accuracy"]), 6),
                    "pr_auc": round(float(metrics["pr_auc"]), 6),
                    "ece": round(float(metrics["ece"]), 6),
                    "tp": int(metrics["tp"]),
                    "tn": int(metrics["tn"]),
                    "fp": int(metrics["fp"]),
                    "fn": int(metrics["fn"]),
                }
            )
    return out


def main() -> None:
    args = parse_args()
    tables = Path(args.outputs_dir) / "tables"
    planner_rows = read_csv(args.planner_detail)
    bc_rows = read_csv(args.bc_detail) if Path(args.bc_detail).exists() else []
    pybullet_rows = read_csv(args.pybullet_detail) if Path(args.pybullet_detail).exists() else []

    ci_rows = rate_ci_rows(planner_rows, "planner_300")
    if bc_rows:
        ci_rows.extend(rate_ci_rows(bc_rows, "bc_test"))
    if pybullet_rows:
        ci_rows.extend(rate_ci_rows(pybullet_rows, "pybullet_validation"))
    write_csv(tables / "reviewer_rate_confidence_intervals.csv", ci_rows)

    write_csv(
        tables / "reviewer_radius_margin_sensitivity.csv",
        sensitivity_rows(planner_rows, args.obstacle_radii, args.safety_margins, args.base_obstacle_radius),
    )

    write_csv(
        tables / "reviewer_risk_operating_points.csv",
        risk_operating_point_rows(args.risk_samples, args.risk_model, args.target_recalls),
    )

    print(f"Wrote {tables / 'reviewer_rate_confidence_intervals.csv'}")
    print(f"Wrote {tables / 'reviewer_radius_margin_sensitivity.csv'}")
    print(f"Wrote {tables / 'reviewer_risk_operating_points.csv'}")


if __name__ == "__main__":
    main()
