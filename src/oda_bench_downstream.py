"""Shared helpers for lightweight ODA-Bench downstream experiments.

The downstream tracks intentionally use low-dimensional geometry features so
they can run on a MacBook-class machine and remain tied to the existing ODA
clearance/collision evaluator.
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import numpy as np

from src.metrics import compute_trial_metrics, path_length, pairwise_ground_distances
from src.oda_io import dataset_root, load_optitrack, obstacle_array, read_trial_overview


FEATURE_COLUMNS = [
    "rel_goal_x",
    "rel_goal_z",
    "nearest_obs_x",
    "nearest_obs_z",
    "vel_x",
    "vel_z",
    "clearance_m",
    "obstacle_count",
]
ACTION_COLUMNS = ["action_dx", "action_dz"]


@dataclass(frozen=True)
class TrialSpec:
    sequence: str
    split: str
    start: tuple[float, float]
    goal: tuple[float, float]
    obstacles: tuple[tuple[float, float], ...]
    duration_s: float
    obstacle_radius_m: float
    safety_distance_m: float

    def to_row(self) -> dict[str, object]:
        return {
            "sequence": self.sequence,
            "split": self.split,
            "start_x": self.start[0],
            "start_z": self.start[1],
            "goal_x": self.goal[0],
            "goal_z": self.goal[1],
            "obstacles_json": json.dumps(self.obstacles),
            "duration_s": self.duration_s,
            "obstacle_radius_m": self.obstacle_radius_m,
            "safety_distance_m": self.safety_distance_m,
        }

    @staticmethod
    def from_row(row: dict[str, str]) -> "TrialSpec":
        obstacles = tuple(tuple(float(v) for v in item) for item in json.loads(row["obstacles_json"]))
        return TrialSpec(
            sequence=str(row["sequence"]),
            split=str(row["split"]),
            start=(float(row["start_x"]), float(row["start_z"])),
            goal=(float(row["goal_x"]), float(row["goal_z"])),
            obstacles=obstacles,
            duration_s=float(row["duration_s"]),
            obstacle_radius_m=float(row["obstacle_radius_m"]),
            safety_distance_m=float(row["safety_distance_m"]),
        )


def write_csv(path: str | Path, rows: list[dict[str, object]]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fieldnames: list[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open(newline="") as f:
        return list(csv.DictReader(f))


def read_trial_ids(path: str | Path, ready_only: bool = True, limit: int | None = None) -> list[str]:
    rows = read_csv(path)
    trial_ids: list[str] = []
    for row in rows:
        if ready_only and row.get("ready", "1") != "1":
            continue
        trial_ids.append(str(row["sequence"]))
        if limit is not None and len(trial_ids) >= limit:
            break
    return trial_ids


def split_trial_ids(
    trial_ids: list[str],
    seed: int = 7,
    train_fraction: float = 0.70,
    val_fraction: float = 0.15,
) -> dict[str, list[str]]:
    ids = np.asarray([str(item) for item in trial_ids], dtype=object)
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(ids))
    ids = ids[order]
    n = len(ids)
    train_end = max(1, int(round(n * train_fraction)))
    val_end = min(n - 1, train_end + max(1, int(round(n * val_fraction)))) if n >= 3 else train_end
    return {
        "train": ids[:train_end].tolist(),
        "val": ids[train_end:val_end].tolist(),
        "test": ids[val_end:].tolist(),
    }


def load_trial_spec(
    dataset_dir: str | Path,
    sequence: str,
    split: str,
    obstacle_radius_m: float = 0.20,
    safety_distance_m: float = 0.50,
) -> TrialSpec:
    root = dataset_root(dataset_dir)
    trials = read_trial_overview(root)
    trial = trials[str(sequence)]
    opt = load_optitrack(root, sequence)
    xy = np.column_stack([opt["ground_x_m"], opt["ground_y_m"]])
    obstacles = obstacle_array(trial.obstacles)
    duration = float(opt["time_s"][-1] - opt["time_s"][0])
    return TrialSpec(
        sequence=str(sequence),
        split=split,
        start=tuple(float(v) for v in xy[0]),
        goal=tuple(float(v) for v in xy[-1]),
        obstacles=tuple(tuple(float(v) for v in row) for row in obstacles),
        duration_s=duration,
        obstacle_radius_m=obstacle_radius_m,
        safety_distance_m=safety_distance_m,
    )


def clearance_for_point(point_xy: np.ndarray, obstacles_xy: np.ndarray, obstacle_radius_m: float) -> float:
    if len(obstacles_xy) == 0:
        return float("inf")
    distances = np.linalg.norm(obstacles_xy - point_xy[None, :], axis=1)
    return float(np.min(distances) - obstacle_radius_m)


def nearest_obstacle_delta(point_xy: np.ndarray, obstacles_xy: np.ndarray) -> np.ndarray:
    if len(obstacles_xy) == 0:
        return np.zeros(2, dtype=float)
    idx = int(np.argmin(np.linalg.norm(obstacles_xy - point_xy[None, :], axis=1)))
    return obstacles_xy[idx] - point_xy


def make_observation(
    point_xy: np.ndarray,
    goal_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    velocity_xy: np.ndarray,
    obstacle_radius_m: float = 0.20,
) -> np.ndarray:
    clearance = clearance_for_point(point_xy, obstacles_xy, obstacle_radius_m)
    nearest = nearest_obstacle_delta(point_xy, obstacles_xy)
    return np.asarray(
        [
            goal_xy[0] - point_xy[0],
            goal_xy[1] - point_xy[1],
            nearest[0],
            nearest[1],
            velocity_xy[0],
            velocity_xy[1],
            clearance,
            float(len(obstacles_xy)),
        ],
        dtype=np.float32,
    )


def observations_actions_from_path(
    sequence: str,
    split: str,
    trajectory_xy: np.ndarray,
    obstacles_xy: np.ndarray,
    duration_s: float,
    obstacle_radius_m: float = 0.20,
    safety_distance_m: float = 0.50,
) -> tuple[list[dict[str, object]], np.ndarray, np.ndarray]:
    goal = trajectory_xy[-1]
    dt = duration_s / max(1, len(trajectory_xy) - 1)
    rows: list[dict[str, object]] = []
    obs_rows: list[np.ndarray] = []
    action_rows: list[np.ndarray] = []
    prev_action = np.zeros(2, dtype=float)
    for idx in range(len(trajectory_xy) - 1):
        point = trajectory_xy[idx]
        action = trajectory_xy[idx + 1] - point
        velocity = prev_action / max(dt, 1e-6)
        obs = make_observation(point, goal, obstacles_xy, velocity, obstacle_radius_m)
        clearance = float(obs[6])
        row = {
            "sequence": sequence,
            "split": split,
            "step": idx,
            "time_s": round(idx * dt, 4),
            "x": point[0],
            "z": point[1],
            **{name: float(value) for name, value in zip(FEATURE_COLUMNS, obs)},
            "action_dx": action[0],
            "action_dz": action[1],
            "safe_sample": int(clearance >= safety_distance_m),
        }
        rows.append(row)
        obs_rows.append(obs)
        action_rows.append(action.astype(np.float32))
        prev_action = action
    return rows, np.asarray(obs_rows, dtype=np.float32), np.asarray(action_rows, dtype=np.float32)


def load_policy_dataset(npz_path: str | Path) -> dict[str, np.ndarray]:
    return dict(np.load(npz_path, allow_pickle=False))


def load_trial_specs(path: str | Path) -> list[TrialSpec]:
    return [TrialSpec.from_row(row) for row in read_csv(path)]


def save_trial_specs(path: str | Path, specs: list[TrialSpec]) -> None:
    write_csv(path, [spec.to_row() for spec in specs])


def standardize_train_val_test(
    arrays: dict[str, np.ndarray],
) -> tuple[dict[str, np.ndarray], np.ndarray, np.ndarray]:
    x_train = arrays["x_train"].astype(np.float32)
    mean = x_train.mean(axis=0)
    std = x_train.std(axis=0)
    std[std < 1e-6] = 1.0
    normalized: dict[str, np.ndarray] = {}
    for split in ["train", "val", "test"]:
        normalized[f"x_{split}"] = ((arrays[f"x_{split}"].astype(np.float32) - mean) / std).astype(np.float32)
        normalized[f"y_{split}"] = arrays[f"y_{split}"].astype(np.float32)
    return normalized, mean.astype(np.float32), std.astype(np.float32)


def policy_action(
    model,
    obs: np.ndarray,
    mean: np.ndarray,
    std: np.ndarray,
    max_step_m: float,
) -> np.ndarray:
    x = ((obs.astype(np.float32) - mean) / std).reshape(1, -1)
    if hasattr(model, "predict"):
        action = np.asarray(model.predict(x)[0], dtype=float)
    else:
        action = np.asarray(model(x)[0], dtype=float)
    norm = float(np.linalg.norm(action))
    if norm > max_step_m:
        action = action / norm * max_step_m
    return action


def rollout_policy(
    spec: TrialSpec,
    model,
    mean: np.ndarray,
    std: np.ndarray,
    max_steps: int = 100,
    goal_tolerance_m: float = 0.18,
    max_step_m: float = 0.20,
) -> tuple[np.ndarray, float]:
    started = perf_counter()
    position = np.asarray(spec.start, dtype=float)
    goal = np.asarray(spec.goal, dtype=float)
    obstacles = np.asarray(spec.obstacles, dtype=float)
    prev_action = np.zeros(2, dtype=float)
    points = [position.copy()]
    for _ in range(max_steps):
        if float(np.linalg.norm(goal - position)) <= goal_tolerance_m:
            break
        obs = make_observation(position, goal, obstacles, prev_action, spec.obstacle_radius_m)
        action = policy_action(model, obs, mean, std, max_step_m=max_step_m)
        if not np.all(np.isfinite(action)) or float(np.linalg.norm(action)) < 1e-6:
            direction = goal - position
            norm = float(np.linalg.norm(direction))
            action = direction / max(norm, 1e-6) * min(max_step_m, norm)
        next_position = position + action
        points.append(next_position.copy())
        prev_action = action
        position = next_position
    if float(np.linalg.norm(goal - position)) > goal_tolerance_m:
        points.append(goal.copy())
    elapsed_ms = (perf_counter() - started) * 1000.0
    return np.asarray(points, dtype=float), elapsed_ms


def evaluate_rollout(
    method: str,
    spec: TrialSpec,
    trajectory_xy: np.ndarray,
    compute_time_ms: float = 0.0,
) -> dict[str, object]:
    time_s = np.linspace(0.0, spec.duration_s, len(trajectory_xy))
    obstacles = np.asarray(spec.obstacles, dtype=float)
    metrics = compute_trial_metrics(
        sequence=spec.sequence,
        time_s=time_s,
        trajectory_xy=trajectory_xy,
        obstacles_xy=obstacles,
        obstacle_radius_m=spec.obstacle_radius_m,
        safety_distance_m=spec.safety_distance_m,
    )
    row = metrics.as_row()
    row.update(
        {
            "method": method,
            "split": spec.split,
            "planner_compute_time_ms": round(compute_time_ms, 4),
            "waypoint_count": len(trajectory_xy),
            "smoothness_heading_change": round(smoothness_score(trajectory_xy), 6),
            "goal_error_m": round(float(np.linalg.norm(trajectory_xy[-1] - np.asarray(spec.goal))), 4),
        }
    )
    return row


def smoothness_score(trajectory_xy: np.ndarray) -> float:
    if len(trajectory_xy) < 3:
        return 0.0
    diffs = np.diff(trajectory_xy, axis=0)
    headings = np.unwrap(np.arctan2(diffs[:, 1], diffs[:, 0]))
    return float(np.mean(np.diff(headings) ** 2))


def aggregate_method_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row["method"]), []).append(row)
    out: list[dict[str, object]] = []
    for method, items in sorted(grouped.items()):
        n = len(items)
        out.append(
            {
                "method": method,
                "cases": n,
                "collision_rate": round(float(np.mean([int(r["collision"]) for r in items])), 4),
                "safety_violation_rate": round(float(np.mean([int(r["safety_violation"]) for r in items])), 4),
                "mean_min_clearance_m": round(float(np.mean([float(r["min_boundary_clearance_m"]) for r in items])), 4),
                "mean_path_length_m": round(float(np.mean([float(r["path_length_m"]) for r in items])), 4),
                "mean_smoothness": round(float(np.mean([float(r["smoothness_heading_change"]) for r in items])), 6),
                "mean_compute_time_ms": round(float(np.mean([float(r["planner_compute_time_ms"]) for r in items])), 4),
                "success_rate": round(float(np.mean([float(r.get("success", int(not int(r["collision"])))) for r in items])), 4),
            }
        )
    return out


def pr_auc_score(y_true: np.ndarray, score: np.ndarray) -> float:
    y = y_true.astype(int)
    if int(y.sum()) == 0:
        return 0.0
    order = np.argsort(-score)
    y_sorted = y[order]
    tp = np.cumsum(y_sorted)
    fp = np.cumsum(1 - y_sorted)
    precision = tp / np.maximum(tp + fp, 1)
    recall = tp / max(int(y.sum()), 1)
    precision = np.concatenate([[1.0], precision])
    recall = np.concatenate([[0.0], recall])
    return float(np.trapezoid(precision, recall))


def expected_calibration_error(y_true: np.ndarray, score: np.ndarray, bins: int = 10) -> float:
    y = y_true.astype(float)
    edges = np.linspace(0.0, 1.0, bins + 1)
    ece = 0.0
    for lo, hi in zip(edges[:-1], edges[1:]):
        mask = (score >= lo) & (score < hi if hi < 1.0 else score <= hi)
        if not np.any(mask):
            continue
        conf = float(np.mean(score[mask]))
        acc = float(np.mean(y[mask]))
        ece += float(np.mean(mask)) * abs(conf - acc)
    return ece


def binary_metrics(y_true: np.ndarray, score: np.ndarray, threshold: float = 0.5) -> dict[str, float]:
    y = y_true.astype(int)
    pred = (score >= threshold).astype(int)
    tp = int(np.sum((pred == 1) & (y == 1)))
    tn = int(np.sum((pred == 0) & (y == 0)))
    fp = int(np.sum((pred == 1) & (y == 0)))
    fn = int(np.sum((pred == 0) & (y == 1)))
    pos_recall = tp / max(tp + fn, 1)
    neg_recall = tn / max(tn + fp, 1)
    precision = tp / max(tp + fp, 1)
    return {
        "threshold": float(threshold),
        "pr_auc": pr_auc_score(y, score),
        "risk_recall": pos_recall,
        "false_negative_rate": fn / max(tp + fn, 1),
        "balanced_accuracy": 0.5 * (pos_recall + neg_recall),
        "precision": precision,
        "ece": expected_calibration_error(y, score),
        "tp": float(tp),
        "tn": float(tn),
        "fp": float(fp),
        "fn": float(fn),
    }
