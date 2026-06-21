#!/usr/bin/env python3
"""Run sensor-feature ablations for the ODA perception-risk classifier."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


FEATURE_GROUPS = {
    "radar": ["radar_peak", "radar_peak_bin", "radar_energy"],
    "camera_depth": ["depth_min", "depth_p10", "depth_median"],
    "radar_imu": ["radar_peak", "radar_peak_bin", "radar_energy", "imu_acc_norm", "imu_gyro_norm"],
    "depth_radar_imu": [
        "depth_min",
        "depth_p10",
        "depth_median",
        "radar_peak",
        "radar_peak_bin",
        "radar_energy",
        "imu_acc_norm",
        "imu_gyro_norm",
    ],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--features", default="outputs/tables/perception_risk_features.csv")
    parser.add_argument("--target", default="future_risk_label", choices=["future_risk_label", "risk_label"])
    parser.add_argument("--output", default="outputs/tables/perception_risk_ablation_metrics.csv")
    parser.add_argument("--figure-output", default="outputs/figures/perception_risk_ablation.png")
    parser.add_argument("--test-fraction", type=float, default=0.30)
    parser.add_argument("--model", default="random_forest", choices=["logistic", "random_forest", "centroid"])
    parser.add_argument(
        "--groups",
        nargs="*",
        default=list(FEATURE_GROUPS),
        choices=list(FEATURE_GROUPS),
    )
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        raise FileNotFoundError(f"Missing or empty feature table: {path}")
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def trial_split(rows: list[dict[str, str]], test_fraction: float) -> tuple[list[str], list[str]]:
    trials = sorted({row["sequence"] for row in rows}, key=lambda value: int(value) if value.isdigit() else value)
    if len(trials) < 2:
        raise ValueError("Need at least two trials for a trial-level train/test split")
    test_count = max(1, int(math.ceil(len(trials) * test_fraction)))
    train_trials = trials[:-test_count]
    test_trials = trials[-test_count:]
    if not train_trials:
        train_trials = trials[:1]
        test_trials = trials[1:]
    return train_trials, test_trials


def labels_from_rows(rows: list[dict[str, str]], target: str) -> list[str]:
    labels = sorted({row[target] for row in rows})
    if "no_future_risk" in labels and "future_risk" in labels:
        return ["no_future_risk", "future_risk"]
    return labels


def matrix_from_rows(
    rows: list[dict[str, str]],
    feature_columns: list[str],
    target: str,
    labels: list[str],
) -> tuple[np.ndarray, np.ndarray]:
    x = np.asarray([[float(row[col]) for col in feature_columns] for row in rows], dtype=float)
    label_index = {label: idx for idx, label in enumerate(labels)}
    y = np.asarray([label_index[row[target]] for row in rows], dtype=int)
    return x, y


def majority_predict(y_train: np.ndarray, count: int) -> np.ndarray:
    majority = Counter(y_train.tolist()).most_common(1)[0][0]
    return np.full(count, majority, dtype=int)


def centroid_predict(x_train: np.ndarray, y_train: np.ndarray, x_test: np.ndarray) -> np.ndarray:
    classes = sorted(set(y_train.tolist()))
    centroids = np.asarray([x_train[y_train == cls].mean(axis=0) for cls in classes], dtype=float)
    distances = np.linalg.norm(x_test[:, None, :] - centroids[None, :, :], axis=2)
    return np.asarray([classes[int(idx)] for idx in np.argmin(distances, axis=1)], dtype=int)


def sklearn_predict(model_name: str, x_train: np.ndarray, y_train: np.ndarray, x_test: np.ndarray) -> np.ndarray | None:
    if len(set(y_train.tolist())) < 2:
        return None
    try:
        from sklearn.ensemble import RandomForestClassifier
        from sklearn.linear_model import LogisticRegression
        from sklearn.pipeline import make_pipeline
        from sklearn.preprocessing import StandardScaler
    except Exception:
        return None

    if model_name == "random_forest":
        model = RandomForestClassifier(n_estimators=200, random_state=7, class_weight="balanced")
    elif model_name == "logistic":
        model = make_pipeline(
            StandardScaler(),
            LogisticRegression(max_iter=1000, random_state=7, class_weight="balanced"),
        )
    else:
        return None
    model.fit(x_train, y_train)
    return np.asarray(model.predict(x_test), dtype=int)


def confusion(y_true: np.ndarray, y_pred: np.ndarray, labels: list[str]) -> np.ndarray:
    cm = np.zeros((len(labels), len(labels)), dtype=int)
    for true, pred in zip(y_true, y_pred):
        cm[int(true), int(pred)] += 1
    return cm


def metrics_from_predictions(y_true: np.ndarray, y_pred: np.ndarray, labels: list[str]) -> dict[str, float]:
    cm = confusion(y_true, y_pred, labels)
    accuracy = float(np.mean(y_true == y_pred)) if len(y_true) else 0.0
    f1_values = []
    recalls = {}
    for idx, label in enumerate(labels):
        tp = float(cm[idx, idx])
        fp = float(cm[:, idx].sum() - tp)
        fn = float(cm[idx, :].sum() - tp)
        precision = 0.0 if tp + fp == 0 else tp / (tp + fp)
        recall = 0.0 if tp + fn == 0 else tp / (tp + fn)
        f1 = 0.0 if precision + recall == 0 else 2.0 * precision * recall / (precision + recall)
        f1_values.append(f1)
        recalls[f"recall_{label}"] = recall
    return {
        "accuracy": accuracy,
        "macro_f1": float(np.mean(f1_values)) if f1_values else 0.0,
        **recalls,
    }


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def plot_ablation(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    labels = [str(row["feature_group"]) for row in rows]
    x = np.arange(len(labels))
    accuracy = [float(row["model_accuracy"]) for row in rows]
    macro_f1 = [float(row["model_macro_f1"]) for row in rows]
    majority = [float(row["majority_accuracy"]) for row in rows]

    fig, ax = plt.subplots(figsize=(8.0, 4.6), constrained_layout=True)
    ax.bar(x - 0.22, accuracy, width=0.22, label="accuracy", color="#4c78a8")
    ax.bar(x, macro_f1, width=0.22, label="macro-F1", color="#f58518")
    ax.bar(x + 0.22, majority, width=0.22, label="majority acc.", color="#bab0ac")
    ax.set_xticks(x, labels=labels, rotation=25, ha="right")
    ax.set_ylim(0.0, 1.0)
    ax.set_ylabel("Score")
    ax.set_title("Perception-risk sensor ablation")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(loc="best")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def run_group(
    group_name: str,
    feature_columns: list[str],
    rows: list[dict[str, str]],
    train_trials: list[str],
    test_trials: list[str],
    labels: list[str],
    target: str,
    model_name: str,
) -> dict[str, object]:
    train_rows = [row for row in rows if row["sequence"] in train_trials]
    test_rows = [row for row in rows if row["sequence"] in test_trials]
    x_train, y_train = matrix_from_rows(train_rows, feature_columns, target, labels)
    x_test, y_test = matrix_from_rows(test_rows, feature_columns, target, labels)
    majority_pred = majority_predict(y_train, len(y_test))
    majority_metrics = metrics_from_predictions(y_test, majority_pred, labels)

    model_pred = None if model_name == "centroid" else sklearn_predict(model_name, x_train, y_train, x_test)
    actual_model = model_name
    if model_pred is None:
        model_pred = centroid_predict(x_train, y_train, x_test) if len(set(y_train.tolist())) >= 2 else majority_pred
        actual_model = "centroid" if len(set(y_train.tolist())) >= 2 else "majority_only"
    model_metrics = metrics_from_predictions(y_test, model_pred, labels)

    out: dict[str, object] = {
        "feature_group": group_name,
        "features": " ".join(feature_columns),
        "model": actual_model,
        "target": target,
        "feature_rows": len(rows),
        "train_trials": " ".join(train_trials),
        "test_trials": " ".join(test_trials),
        "majority_accuracy": round(majority_metrics["accuracy"], 4),
        "model_accuracy": round(model_metrics["accuracy"], 4),
        "model_macro_f1": round(model_metrics["macro_f1"], 4),
        "beats_majority": int(model_metrics["accuracy"] > majority_metrics["accuracy"]),
    }
    for key, value in sorted(model_metrics.items()):
        if key.startswith("recall_"):
            out[key] = round(value, 4)
    return out


def main() -> None:
    args = parse_args()
    rows = read_rows(Path(args.features))
    labels = labels_from_rows(rows, args.target)
    train_trials, test_trials = trial_split(rows, args.test_fraction)
    output_rows = [
        run_group(
            group_name=group,
            feature_columns=FEATURE_GROUPS[group],
            rows=rows,
            train_trials=train_trials,
            test_trials=test_trials,
            labels=labels,
            target=args.target,
            model_name=args.model,
        )
        for group in args.groups
    ]
    write_csv(Path(args.output), output_rows)
    plot_ablation(Path(args.figure_output), output_rows)
    print(f"Wrote {args.output}")
    print(f"Wrote {args.figure_output}")
    for row in output_rows:
        print(row)


if __name__ == "__main__":
    main()
