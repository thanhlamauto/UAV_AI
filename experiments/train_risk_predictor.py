#!/usr/bin/env python3
"""Train TTC/distance and small numpy-MLP future-risk predictors."""

from __future__ import annotations

import argparse
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from src.oda_bench_downstream import FEATURE_COLUMNS, binary_metrics, pr_auc_score, read_csv, write_csv


class BinaryMLP:
    def __init__(self, input_dim: int, hidden_dim: int = 48, seed: int = 7):
        rng = np.random.default_rng(seed)
        self.w1 = (rng.normal(0, 1 / np.sqrt(input_dim), size=(input_dim, hidden_dim))).astype(np.float32)
        self.b1 = np.zeros(hidden_dim, dtype=np.float32)
        self.w2 = (rng.normal(0, 1 / np.sqrt(hidden_dim), size=(hidden_dim, 1))).astype(np.float32)
        self.b2 = np.zeros(1, dtype=np.float32)

    def logits(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        z1 = x @ self.w1 + self.b1
        h1 = np.maximum(z1, 0.0)
        z2 = (h1 @ self.w2 + self.b2).reshape(-1)
        return z2, z1, h1

    def predict_proba(self, x: np.ndarray) -> np.ndarray:
        logits, _, _ = self.logits(x.astype(np.float32))
        return 1.0 / (1.0 + np.exp(-np.clip(logits, -30, 30)))

    def train_epoch(
        self,
        x: np.ndarray,
        y: np.ndarray,
        lr: float,
        batch_size: int,
        pos_weight: float,
        rng: np.random.Generator,
    ) -> None:
        order = rng.permutation(len(x))
        for start in range(0, len(order), batch_size):
            idx = order[start : start + batch_size]
            xb = x[idx].astype(np.float32)
            yb = y[idx].astype(np.float32)
            logits, z1, h1 = self.logits(xb)
            prob = 1.0 / (1.0 + np.exp(-np.clip(logits, -30, 30)))
            weights = np.where(yb > 0.5, pos_weight, 1.0)
            grad_logits = ((prob - yb) * weights / max(1, len(xb))).astype(np.float32)
            gw2 = h1.T @ grad_logits[:, None]
            gb2 = np.asarray([grad_logits.sum()], dtype=np.float32)
            gh1 = grad_logits[:, None] @ self.w2.T
            gz1 = gh1 * (z1 > 0)
            gw1 = xb.T @ gz1
            gb1 = gz1.sum(axis=0)
            self.w1 -= lr * gw1
            self.b1 -= lr * gb1
            self.w2 -= lr * gw2
            self.b2 -= lr * gb2

    def state(self) -> dict[str, np.ndarray]:
        return {"w1": self.w1, "b1": self.b1, "w2": self.w2, "b2": self.b2}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", default="outputs/tables/oda_policy_dataset_samples.csv")
    parser.add_argument("--outputs-dir", default="outputs")
    parser.add_argument("--future-risk-horizon-steps", type=int, default=8)
    parser.add_argument("--danger-clearance", type=float, default=0.50)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=8e-4)
    parser.add_argument("--seed", type=int, default=7)
    return parser.parse_args()


def build_future_risk_rows(rows: list[dict[str, str]], horizon_steps: int, danger_clearance: float) -> list[dict[str, object]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[row["sequence"]].append(row)
    out: list[dict[str, object]] = []
    for _, items in grouped.items():
        items = sorted(items, key=lambda r: int(r["step"]))
        clearance = np.asarray([float(r["clearance_m"]) for r in items], dtype=float)
        for idx, row in enumerate(items):
            future = int(np.min(clearance[idx : min(len(items), idx + horizon_steps + 1)]) <= danger_clearance)
            out.append({**row, "future_risk": future})
    return out


def matrix(rows: list[dict[str, object]]) -> tuple[np.ndarray, np.ndarray]:
    return (
        np.asarray([[float(row[col]) for col in FEATURE_COLUMNS] for row in rows], dtype=np.float32),
        np.asarray([int(row["future_risk"]) for row in rows], dtype=np.float32),
    )


def standardize(x_train: np.ndarray, *others: np.ndarray) -> tuple[np.ndarray, list[np.ndarray], np.ndarray, np.ndarray]:
    mean = x_train.mean(axis=0)
    std = x_train.std(axis=0)
    std[std < 1e-6] = 1.0
    return (x_train - mean) / std, [(x - mean) / std for x in others], mean, std


def train_mlp(
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_val: np.ndarray,
    y_val: np.ndarray,
    epochs: int,
    batch_size: int,
    lr: float,
    seed: int,
) -> BinaryMLP:
    rng = np.random.default_rng(seed)
    model = BinaryMLP(x_train.shape[1], seed=seed)
    positives = float(np.sum(y_train))
    negatives = float(len(y_train) - positives)
    pos_weight = negatives / max(positives, 1.0)
    best_auc = -1.0
    best_state = model.state()
    for _ in range(epochs):
        model.train_epoch(x_train, y_train, lr=lr, batch_size=batch_size, pos_weight=pos_weight, rng=rng)
        eval_x = x_val if has_both_classes(y_val) else x_train
        eval_y = y_val if has_both_classes(y_val) else y_train
        score = model.predict_proba(eval_x) if len(eval_x) else np.zeros(0)
        auc = pr_auc_score(eval_y.astype(int), score) if len(score) else 0.0
        if auc > best_auc:
            best_auc = auc
            best_state = {k: v.copy() for k, v in model.state().items()}
    restored = BinaryMLP(x_train.shape[1], seed=seed)
    for key, value in best_state.items():
        setattr(restored, key, value)
    return restored


def tune_threshold(y_val: np.ndarray, score: np.ndarray) -> float:
    best_t = 0.5
    best = -1.0
    for threshold in np.linspace(0.05, 0.95, 19):
        metrics = binary_metrics(y_val.astype(int), score, threshold=float(threshold))
        value = metrics["risk_recall"] + 0.25 * metrics["balanced_accuracy"]
        if value > best:
            best = value
            best_t = float(threshold)
    return best_t


def has_both_classes(y: np.ndarray) -> bool:
    return len(y) > 0 and np.any(y > 0.5) and np.any(y <= 0.5)


def ttc_distance_score(rows: list[dict[str, object]]) -> np.ndarray:
    clearance = np.asarray([float(r["clearance_m"]) for r in rows], dtype=float)
    speed = np.linalg.norm(np.asarray([[float(r["vel_x"]), float(r["vel_z"])] for r in rows], dtype=float), axis=1)
    ttc = clearance / np.maximum(speed, 0.05)
    raw = -ttc
    lo, hi = float(np.percentile(raw, 1)), float(np.percentile(raw, 99))
    return np.clip((raw - lo) / max(hi - lo, 1e-6), 0.0, 1.0)


def plot_pr_curve(path: Path, y_true: np.ndarray, model_scores: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(6.5, 4.5), constrained_layout=True)
    for name, score in model_scores.items():
        order = np.argsort(-score)
        y_sorted = y_true.astype(int)[order]
        tp = np.cumsum(y_sorted)
        fp = np.cumsum(1 - y_sorted)
        precision = tp / np.maximum(tp + fp, 1)
        recall = tp / max(int(y_true.sum()), 1)
        ax.plot(recall, precision, label=f"{name} PR-AUC={pr_auc_score(y_true.astype(int), score):.3f}", linewidth=2)
    ax.set_xlabel("Risk recall")
    ax.set_ylabel("Precision")
    ax.set_title("ODA-Risk future-risk prediction")
    ax.grid(True, alpha=0.25)
    ax.legend()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    outputs = Path(args.outputs_dir)
    tables = outputs / "tables"
    figures = outputs / "figures"
    (outputs / "models").mkdir(parents=True, exist_ok=True)
    rows = build_future_risk_rows(read_csv(args.samples), args.future_risk_horizon_steps, args.danger_clearance)
    write_csv(tables / "oda_risk_labeled_samples.csv", rows)
    train_rows = [r for r in rows if r["split"] == "train"]
    val_rows = [r for r in rows if r["split"] == "val"]
    test_rows = [r for r in rows if r["split"] == "test"]
    x_train, y_train = matrix(train_rows)
    x_val, y_val = matrix(val_rows)
    x_test, y_test = matrix(test_rows)
    x_train_n, [x_val_n, x_test_n], mean, std = standardize(x_train, x_val, x_test)

    ttc_val = ttc_distance_score(val_rows)
    ttc_test = ttc_distance_score(test_rows)
    threshold_y = y_val if has_both_classes(y_val) else y_train
    ttc_threshold_scores = ttc_val if has_both_classes(y_val) else ttc_distance_score(train_rows)
    ttc_threshold = tune_threshold(threshold_y, ttc_threshold_scores) if len(threshold_y) else 0.5

    model = train_mlp(x_train_n, y_train, x_val_n, y_val, args.epochs, args.batch_size, args.lr, args.seed)
    mlp_val = model.predict_proba(x_val_n)
    mlp_test = model.predict_proba(x_test_n)
    mlp_threshold_scores = mlp_val if has_both_classes(y_val) else model.predict_proba(x_train_n)
    mlp_threshold = tune_threshold(threshold_y, mlp_threshold_scores) if len(threshold_y) else 0.5

    result_rows = []
    for name, score, threshold in [
        ("ttc_distance_threshold", ttc_test, ttc_threshold),
        ("small_mlp_risk_predictor", mlp_test, mlp_threshold),
    ]:
        metrics = binary_metrics(y_test.astype(int), score, threshold=threshold)
        result_rows.append(
            {
                "method": name,
                "test_samples": len(y_test),
                "test_positive_rate": round(float(np.mean(y_test)), 4) if len(y_test) else 0.0,
                **{k: round(float(v), 6) for k, v in metrics.items()},
            }
        )

    write_csv(tables / "oda_risk_results.csv", result_rows)
    plot_pr_curve(figures / "risk_predictor_pr_curve.png", y_test, {"TTC/distance": ttc_test, "Small MLP": mlp_test})
    np.savez_compressed(outputs / "models" / "small_mlp_risk_predictor.npz", **model.state(), mean=mean, std=std)
    print(f"Wrote {tables / 'oda_risk_results.csv'}")
    print(f"Wrote {figures / 'risk_predictor_pr_curve.png'}")


if __name__ == "__main__":
    main()
