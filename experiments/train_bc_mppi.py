#!/usr/bin/env python3
"""Train Plain BC-MPPI and Filtered BC-MPPI lightweight numpy policies."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from src.oda_bench_downstream import (
    FEATURE_COLUMNS,
    aggregate_method_rows,
    evaluate_rollout,
    load_policy_dataset,
    load_trial_specs,
    rollout_policy,
    standardize_train_val_test,
    write_csv,
)


class NumpyBCPolicy:
    def __init__(
        self,
        input_dim: int,
        hidden_dim: int = 64,
        seed: int = 7,
        params: dict[str, np.ndarray] | None = None,
    ):
        if params is not None:
            self.w1 = params["w1"].astype(np.float32)
            self.b1 = params["b1"].astype(np.float32)
            self.w2 = params["w2"].astype(np.float32)
            self.b2 = params["b2"].astype(np.float32)
            self.w3 = params["w3"].astype(np.float32)
            self.b3 = params["b3"].astype(np.float32)
            return
        rng = np.random.default_rng(seed)
        self.w1 = (rng.normal(0, 1 / np.sqrt(input_dim), size=(input_dim, hidden_dim))).astype(np.float32)
        self.b1 = np.zeros(hidden_dim, dtype=np.float32)
        self.w2 = (rng.normal(0, 1 / np.sqrt(hidden_dim), size=(hidden_dim, hidden_dim))).astype(np.float32)
        self.b2 = np.zeros(hidden_dim, dtype=np.float32)
        self.w3 = (rng.normal(0, 1 / np.sqrt(hidden_dim), size=(hidden_dim, 2))).astype(np.float32)
        self.b3 = np.zeros(2, dtype=np.float32)

    def _forward(self, x: np.ndarray) -> tuple[np.ndarray, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
        z1 = x @ self.w1 + self.b1
        h1 = np.maximum(z1, 0.0)
        z2 = h1 @ self.w2 + self.b2
        h2 = np.maximum(z2, 0.0)
        out = h2 @ self.w3 + self.b3
        return out, (z1, h1, z2, h2)

    def predict(self, x: np.ndarray) -> np.ndarray:
        return self._forward(x.astype(np.float32))[0]

    def train_epoch(self, x: np.ndarray, y: np.ndarray, lr: float, batch_size: int, rng: np.random.Generator) -> None:
        order = rng.permutation(len(x))
        for start in range(0, len(order), batch_size):
            idx = order[start : start + batch_size]
            xb = x[idx].astype(np.float32)
            yb = y[idx].astype(np.float32)
            pred, (z1, h1, z2, h2) = self._forward(xb)
            grad = (2.0 / max(1, len(xb))) * (pred - yb)
            gw3 = h2.T @ grad
            gb3 = grad.sum(axis=0)
            gh2 = grad @ self.w3.T
            gz2 = gh2 * (z2 > 0)
            gw2 = h1.T @ gz2
            gb2 = gz2.sum(axis=0)
            gh1 = gz2 @ self.w2.T
            gz1 = gh1 * (z1 > 0)
            gw1 = xb.T @ gz1
            gb1 = gz1.sum(axis=0)
            for name, grad_arr in [
                ("w1", gw1),
                ("b1", gb1),
                ("w2", gw2),
                ("b2", gb2),
                ("w3", gw3),
                ("b3", gb3),
            ]:
                value = getattr(self, name)
                setattr(self, name, (value - lr * grad_arr).astype(np.float32))

    def state(self) -> dict[str, np.ndarray]:
        return {"w1": self.w1, "b1": self.b1, "w2": self.w2, "b2": self.b2, "w3": self.w3, "b3": self.b3}

    @staticmethod
    def load(path: str | Path) -> tuple["NumpyBCPolicy", np.ndarray, np.ndarray, float]:
        payload = np.load(path)
        params = {key: payload[key] for key in ["w1", "b1", "w2", "b2", "w3", "b3"]}
        model = NumpyBCPolicy(input_dim=params["w1"].shape[0], params=params)
        return model, payload["mean"].astype(np.float32), payload["std"].astype(np.float32), float(payload["max_step_m"])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default="outputs/datasets/oda_mppi_policy_dataset.npz")
    parser.add_argument("--trial-specs", default="outputs/datasets/oda_policy_trial_specs.csv")
    parser.add_argument("--outputs-dir", default="outputs")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--filtered-clearance-quantile", type=float, default=0.60)
    parser.add_argument("--max-rollout-steps", type=int, default=100)
    return parser.parse_args()


def mse(model: NumpyBCPolicy, x: np.ndarray, y: np.ndarray) -> float:
    if len(x) == 0:
        return 0.0
    pred = model.predict(x)
    return float(np.mean((pred - y) ** 2))


def train_model(
    name: str,
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_val: np.ndarray,
    y_val: np.ndarray,
    epochs: int,
    batch_size: int,
    lr: float,
    seed: int,
) -> tuple[NumpyBCPolicy, dict[str, object]]:
    rng = np.random.default_rng(seed)
    model = NumpyBCPolicy(input_dim=x_train.shape[1], seed=seed)
    best_state = model.state()
    best_val = float("inf")
    for _ in range(epochs):
        model.train_epoch(x_train, y_train, lr=lr, batch_size=batch_size, rng=rng)
        val = mse(model, x_val, y_val)
        if val < best_val:
            best_val = val
            best_state = {k: v.copy() for k, v in model.state().items()}
    model = NumpyBCPolicy(input_dim=x_train.shape[1], params=best_state)
    return model, {"model": name, "train_mse": round(mse(model, x_train, y_train), 8), "val_mse": round(mse(model, x_val, y_val), 8)}


def batch_mse_and_latency(model: NumpyBCPolicy, x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    started = perf_counter()
    pred = model.predict(x)
    latency_ms = (perf_counter() - started) * 1000.0 / max(1, len(x))
    return (float(np.mean((pred - y) ** 2)) if len(x) else 0.0), latency_ms


def plot_bc_results(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    methods = [str(r["method"]) for r in rows]
    collision = [float(r["collision_rate"]) for r in rows]
    violation = [float(r["safety_violation_rate"]) for r in rows]
    clearance = [float(r["mean_min_clearance_m"]) for r in rows]
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.4), constrained_layout=True)
    colors = ["#2563eb", "#ee0033"][: len(methods)]
    for ax, values, title in [
        (axes[0], collision, "Collision rate"),
        (axes[1], violation, "Violation rate"),
        (axes[2], clearance, "Mean min clearance [m]"),
    ]:
        bars = ax.bar(methods, values, color=colors)
        ax.set_title(title)
        ymax = max(0.05, max(values) * 1.35)
        ax.set_ylim(0, ymax)
        ax.grid(axis="y", alpha=0.25)
        ax.tick_params(axis="x", rotation=15)
        for bar, value in zip(bars, values):
            label = f"{value:.2f}" if "rate" in title.lower() else f"{value:.2f} m"
            y = value + ymax * 0.035
            ax.text(bar.get_x() + bar.get_width() / 2, y, label, ha="center", va="bottom", fontsize=8)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    outputs = Path(args.outputs_dir)
    tables = outputs / "tables"
    figures = outputs / "figures"
    models_dir = outputs / "models"
    models_dir.mkdir(parents=True, exist_ok=True)

    arrays = load_policy_dataset(args.dataset)
    normalized, mean, std = standardize_train_val_test(arrays)
    clearance_idx = FEATURE_COLUMNS.index("clearance_m")
    raw_clearance = arrays["x_train"][:, clearance_idx]
    threshold = float(np.quantile(raw_clearance, args.filtered_clearance_quantile)) if len(raw_clearance) else 0.0
    filtered_mask = raw_clearance >= threshold
    if int(filtered_mask.sum()) < 100:
        filtered_mask = raw_clearance >= float(np.quantile(raw_clearance, 0.40))

    models: dict[str, NumpyBCPolicy] = {}
    fit_rows: list[dict[str, object]] = []
    for name, x_train, y_train, seed in [
        ("plain_bc_mppi", normalized["x_train"], normalized["y_train"], args.seed),
        ("filtered_bc_mppi", normalized["x_train"][filtered_mask], normalized["y_train"][filtered_mask], args.seed + 1),
    ]:
        model, row = train_model(
            name,
            x_train,
            y_train,
            normalized["x_val"],
            normalized["y_val"],
            args.epochs,
            args.batch_size,
            args.lr,
            seed,
        )
        if name == "filtered_bc_mppi":
            row["filtered_clearance_threshold_m"] = round(threshold, 4)
            row["filtered_train_samples"] = int(filtered_mask.sum())
        models[name] = model
        fit_rows.append(row)

    max_step_m = float(np.quantile(np.linalg.norm(arrays["y_train"], axis=1), 0.98)) if len(arrays["y_train"]) else 0.20
    max_step_m = max(0.05, min(0.35, max_step_m))
    specs = [spec for spec in load_trial_specs(args.trial_specs) if spec.split == "test"]
    detail_rows: list[dict[str, object]] = []
    test_rows: list[dict[str, object]] = []
    for name, model in models.items():
        test_mse, latency_ms = batch_mse_and_latency(model, normalized["x_test"], normalized["y_test"])
        for spec in specs:
            path, compute_ms = rollout_policy(
                spec,
                model,
                mean,
                std,
                max_steps=args.max_rollout_steps,
                max_step_m=max_step_m,
            )
            row = evaluate_rollout(name, spec, path, compute_time_ms=compute_ms)
            row["test_action_mse"] = round(test_mse, 8)
            row["inference_latency_ms_per_sample"] = round(latency_ms, 6)
            detail_rows.append(row)
        test_rows.append(
            {
                "method": name,
                "test_action_mse": round(test_mse, 8),
                "inference_latency_ms_per_sample": round(latency_ms, 6),
                "max_step_m": round(max_step_m, 4),
            }
        )
        np.savez_compressed(models_dir / f"{name}.npz", **model.state(), mean=mean, std=std, max_step_m=max_step_m)

    summary = aggregate_method_rows(detail_rows)
    summary_by_method = {row["method"]: row for row in summary}
    for row in test_rows:
        row.update(summary_by_method.get(row["method"], {}))

    write_csv(tables / "oda_bc_fit_metrics.csv", fit_rows)
    write_csv(tables / "oda_bc_rollout_detail.csv", detail_rows)
    write_csv(tables / "oda_bc_results.csv", test_rows)
    plot_bc_results(figures / "bc_plain_vs_filtered_safety.png", summary)
    print(f"Wrote {tables / 'oda_bc_results.csv'}")
    print(f"Wrote {figures / 'bc_plain_vs_filtered_safety.png'}")


if __name__ == "__main__":
    main()
