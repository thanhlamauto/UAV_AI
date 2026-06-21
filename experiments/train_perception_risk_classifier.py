#!/usr/bin/env python3
"""Train a small frame-level perception-risk classifier."""

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


FEATURE_COLUMNS = [
    "depth_min",
    "depth_p10",
    "depth_median",
    "radar_peak",
    "radar_peak_bin",
    "radar_energy",
    "imu_acc_norm",
    "imu_gyro_norm",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--features", default="outputs/tables/perception_risk_features.csv")
    parser.add_argument("--target", default="future_risk_label", choices=["future_risk_label", "risk_label"])
    parser.add_argument("--metrics-output", default="outputs/tables/perception_risk_metrics.csv")
    parser.add_argument("--figure-output", default="outputs/figures/perception_risk_confusion_matrix.png")
    parser.add_argument("--test-fraction", type=float, default=0.30)
    parser.add_argument("--model", default="logistic", choices=["logistic", "random_forest", "centroid"])
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


def matrix_from_rows(rows: list[dict[str, str]], target: str) -> tuple[np.ndarray, np.ndarray, list[str]]:
    x = np.asarray([[float(row[col]) for col in FEATURE_COLUMNS] for row in rows], dtype=float)
    y_raw = [row[target] for row in rows]
    labels = sorted(set(y_raw))
    if "no_future_risk" in labels and "future_risk" in labels:
        labels = ["no_future_risk", "future_risk"]
    y = np.asarray([labels.index(label) for label in y_raw], dtype=int)
    return x, y, labels


def majority_predict(y_train: np.ndarray, count: int) -> np.ndarray:
    majority = Counter(y_train.tolist()).most_common(1)[0][0]
    return np.full(count, majority, dtype=int)


def centroid_predict(x_train: np.ndarray, y_train: np.ndarray, x_test: np.ndarray) -> np.ndarray:
    classes = sorted(set(y_train.tolist()))
    centroids = []
    for cls in classes:
        centroids.append(x_train[y_train == cls].mean(axis=0))
    centroids_arr = np.asarray(centroids, dtype=float)
    distances = np.linalg.norm(x_test[:, None, :] - centroids_arr[None, :, :], axis=2)
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


def write_metrics(path: Path, row: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)


def plot_confusion(path: Path, cm: np.ndarray, labels: list[str], title: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(5.5, 4.8), constrained_layout=True)
    im = ax.imshow(cm, cmap="Blues")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    ax.set_xticks(range(len(labels)), labels=labels, rotation=30, ha="right")
    ax.set_yticks(range(len(labels)), labels=labels)
    ax.set_xlabel("Predicted")
    ax.set_ylabel("Ground truth")
    ax.set_title(title)
    for y in range(cm.shape[0]):
        for x in range(cm.shape[1]):
            ax.text(x, y, str(cm[y, x]), ha="center", va="center", color="black")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    rows = read_rows(Path(args.features))
    train_trials, test_trials = trial_split(rows, args.test_fraction)
    train_rows = [row for row in rows if row["sequence"] in train_trials]
    test_rows = [row for row in rows if row["sequence"] in test_trials]
    if not train_rows or not test_rows:
        raise ValueError("Train/test split produced an empty side")

    x_train, y_train, labels = matrix_from_rows(train_rows, args.target)
    x_test, y_test, test_labels = matrix_from_rows(test_rows, args.target)
    if set(labels) != set(test_labels):
        labels = sorted(set(labels).union(test_labels))
        if "no_future_risk" in labels and "future_risk" in labels:
            labels = ["no_future_risk", "future_risk"]
        label_index = {label: idx for idx, label in enumerate(labels)}
        y_train = np.asarray([label_index[row[args.target]] for row in train_rows], dtype=int)
        y_test = np.asarray([label_index[row[args.target]] for row in test_rows], dtype=int)

    majority_pred = majority_predict(y_train, len(y_test))
    majority_metrics = metrics_from_predictions(y_test, majority_pred, labels)

    model_pred = None if args.model == "centroid" else sklearn_predict(args.model, x_train, y_train, x_test)
    model_name = args.model
    if model_pred is None:
        model_pred = centroid_predict(x_train, y_train, x_test) if len(set(y_train.tolist())) >= 2 else majority_pred
        model_name = "centroid" if len(set(y_train.tolist())) >= 2 else "majority_only"

    model_metrics = metrics_from_predictions(y_test, model_pred, labels)
    cm = confusion(y_test, model_pred, labels)
    row: dict[str, object] = {
        "model": model_name,
        "target": args.target,
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
            row[key] = round(value, 4)

    write_metrics(Path(args.metrics_output), row)
    plot_confusion(Path(args.figure_output), cm, labels, f"{model_name} {args.target}")
    print(f"Wrote {args.metrics_output}")
    print(f"Wrote {args.figure_output}")
    print(row)


if __name__ == "__main__":
    main()
