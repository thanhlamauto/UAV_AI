#!/usr/bin/env python3
"""Run ODA-like obstacle-course validation for planners and BC policies.

The default backend is ``auto``: use gym-pybullet-drones when it is importable,
otherwise fall back to the deterministic kinematic evaluator.  The PyBullet
backend tracks each planned path with CtrlAviary + DSLPIDControl and inserts
course obstacles as simple cylinder bodies.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from time import perf_counter

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from src.oda_bench_downstream import TrialSpec, aggregate_method_rows, evaluate_rollout, rollout_policy, write_csv
from src.planners.mppi import MPPIConfig, mppi_path
from src.planners.rrt_star import RRTStarConfig, rrt_star_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outputs-dir", default="outputs")
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--speeds", nargs="*", type=float, default=[1.0, 2.0, 3.0, 4.0])
    parser.add_argument("--obstacle-radius", type=float, default=0.20)
    parser.add_argument("--safety-distance", type=float, default=0.50)
    parser.add_argument(
        "--reaction-delay-s",
        type=float,
        default=0.15,
        help="Backward-compatible single reaction delay in seconds.",
    )
    parser.add_argument(
        "--reaction-delays-s",
        nargs="*",
        type=float,
        default=None,
        help="Optional delay sweep in seconds, e.g. 0 0.1 0.2 0.3. Overrides --reaction-delay-s.",
    )
    parser.add_argument("--plain-model", default="outputs/models/plain_bc_mppi.npz")
    parser.add_argument("--filtered-model", default="outputs/models/filtered_bc_mppi.npz")
    parser.add_argument("--backend", choices=["auto", "kinematic", "gym-pybullet-drones"], default="auto")
    parser.add_argument("--pybullet-gui", action="store_true")
    parser.add_argument("--pybullet-altitude-m", type=float, default=1.0)
    parser.add_argument("--pybullet-control-freq-hz", type=int, default=48)
    parser.add_argument("--pybullet-sim-freq-hz", type=int, default=240)
    parser.add_argument("--pybullet-time-scale", type=float, default=1.0)
    return parser.parse_args()


def gym_pybullet_status() -> tuple[bool, str]:
    try:
        import gym_pybullet_drones  # noqa: F401

        return True, "available"
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"


def make_course(seed: int, speed_mps: float, obstacle_radius: float, safety_distance: float) -> TrialSpec:
    rng = np.random.default_rng(seed)
    start = np.asarray([0.0, 0.0], dtype=float)
    goal = np.asarray([5.0 + 0.25 * rng.normal(), 0.25 * rng.normal()], dtype=float)
    obstacle_count = 1 if seed % 2 == 0 else 2
    obstacles = []
    for idx in range(obstacle_count):
        x = 1.8 + idx * 1.35 + rng.normal(0.0, 0.15)
        z = rng.choice([-1.0, 1.0]) * rng.uniform(0.15, 0.75)
        obstacles.append((float(x), float(z)))
    duration = float(np.linalg.norm(goal - start) / max(speed_mps, 1e-6))
    return TrialSpec(
        sequence=f"sim_{seed}_{speed_mps:g}",
        split="sim",
        start=tuple(start),
        goal=tuple(goal),
        obstacles=tuple(obstacles),
        duration_s=duration,
        obstacle_radius_m=obstacle_radius,
        safety_distance_m=safety_distance,
    )


def load_policy(path: str | Path):
    from experiments.train_bc_mppi import NumpyBCPolicy

    return NumpyBCPolicy.load(path)


def latency_adjusted(row: dict[str, object], speed_mps: float, reaction_delay_s: float) -> dict[str, object]:
    delay_distance = speed_mps * reaction_delay_s
    clearance = float(row["min_boundary_clearance_m"])
    latency_violation = int(clearance < delay_distance)
    success = int(int(row["collision"]) == 0 and latency_violation == 0 and float(row.get("goal_error_m", 0.0)) < 0.35)
    row.update(
        {
            "speed_mps": speed_mps,
            "reaction_delay_s": reaction_delay_s,
            "delay_distance_m": round(delay_distance, 4),
            "latency_violation": latency_violation,
            "success": success,
        }
    )
    return row


def plan_rrt_star(spec: TrialSpec, seed: int) -> tuple[np.ndarray, float]:
    started = perf_counter()
    path = rrt_star_path(
        start=np.asarray(spec.start),
        goal=np.asarray(spec.goal),
        obstacles_xy=np.asarray(spec.obstacles),
        config=RRTStarConfig(
            max_iterations=900,
            step_size_m=0.35,
            neighbor_radius_m=0.75,
            obstacle_radius_m=spec.obstacle_radius_m,
            safety_distance_m=spec.safety_distance_m,
            seed=seed,
        ),
        num_points=90,
    )
    return path.trajectory_xy, (perf_counter() - started) * 1000.0


def plan_mppi(spec: TrialSpec, seed: int) -> tuple[np.ndarray, float]:
    started = perf_counter()
    path = mppi_path(
        start=np.asarray(spec.start),
        goal=np.asarray(spec.goal),
        obstacles_xy=np.asarray(spec.obstacles),
        config=MPPIConfig(
            num_rollouts=192,
            max_iterations=5,
            horizon_steps=50,
            obstacle_radius_m=spec.obstacle_radius_m,
            safety_distance_m=spec.safety_distance_m,
            seed=seed,
        ),
        num_points=90,
    )
    return path.trajectory_xy, (perf_counter() - started) * 1000.0


def resolve_backend(requested: str) -> tuple[str, str]:
    available, reason = gym_pybullet_status()
    if requested == "kinematic":
        return "kinematic_fallback", "forced_kinematic"
    if requested == "gym-pybullet-drones":
        if not available:
            raise RuntimeError(f"gym-pybullet-drones requested but unavailable: {reason}")
        return "gym-pybullet-drones", reason
    if available:
        return "gym-pybullet-drones", reason
    return "kinematic_fallback", reason


def simulate_gym_pybullet_path(
    spec: TrialSpec,
    trajectory_xy: np.ndarray,
    gui: bool,
    altitude_m: float,
    control_freq_hz: int,
    sim_freq_hz: int,
    time_scale: float,
) -> tuple[np.ndarray, dict[str, object]]:
    from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
    from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
    from gym_pybullet_drones.utils.enums import DroneModel, Physics
    import pybullet as p

    start = np.asarray([trajectory_xy[0, 0], trajectory_xy[0, 1], altitude_m], dtype=float)
    env = CtrlAviary(
        drone_model=DroneModel.CF2X,
        num_drones=1,
        initial_xyzs=start.reshape(1, 3),
        initial_rpys=np.zeros((1, 3)),
        physics=Physics.PYB,
        pyb_freq=sim_freq_hz,
        ctrl_freq=control_freq_hz,
        gui=gui,
        record=False,
        obstacles=False,
        user_debug_gui=False,
    )
    client = env.getPyBulletClient()
    try:
        for obs_xy in spec.obstacles:
            collision = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=spec.obstacle_radius_m,
                height=max(2.0, altitude_m * 2.0),
                physicsClientId=client,
            )
            visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=spec.obstacle_radius_m,
                length=max(2.0, altitude_m * 2.0),
                rgbaColor=[0.9, 0.1, 0.1, 0.6],
                physicsClientId=client,
            )
            p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=[float(obs_xy[0]), float(obs_xy[1]), max(2.0, altitude_m * 2.0) / 2.0],
                physicsClientId=client,
            )

        ctrl = DSLPIDControl(drone_model=DroneModel.CF2X)
        action = np.zeros((1, 4), dtype=float)
        duration_s = max(spec.duration_s * time_scale, 0.5)
        steps = max(2, int(round(duration_s * control_freq_hz)))
        target_index = np.linspace(0, len(trajectory_xy) - 1, steps).astype(int)
        observed = []
        collision_hit = 0
        for idx in target_index:
            obs, _, _, _, _ = env.step(action)
            state = obs[0]
            observed.append([float(state[0]), float(state[1])])
            target = np.asarray([trajectory_xy[idx, 0], trajectory_xy[idx, 1], altitude_m], dtype=float)
            action[0, :], _, _ = ctrl.computeControlFromState(
                control_timestep=env.CTRL_TIMESTEP,
                state=state,
                target_pos=target,
                target_rpy=np.zeros(3),
            )
            drone_id = getattr(env, "DRONE_IDS", [None])[0]
            if drone_id is not None and len(p.getContactPoints(bodyA=drone_id, physicsClientId=client)) > 0:
                collision_hit = 1
        if not observed:
            observed = trajectory_xy.tolist()
        return np.asarray(observed, dtype=float), {"pybullet_contact": collision_hit}
    finally:
        env.close()


def evaluate_path_with_backend(
    method: str,
    spec: TrialSpec,
    trajectory: np.ndarray,
    compute_ms: float,
    backend: str,
    args: argparse.Namespace,
) -> dict[str, object]:
    if backend == "gym-pybullet-drones":
        simulated, extra = simulate_gym_pybullet_path(
            spec,
            trajectory,
            gui=args.pybullet_gui,
            altitude_m=args.pybullet_altitude_m,
            control_freq_hz=args.pybullet_control_freq_hz,
            sim_freq_hz=args.pybullet_sim_freq_hz,
            time_scale=args.pybullet_time_scale,
        )
        row = evaluate_rollout(method, spec, simulated, compute_time_ms=compute_ms)
        if int(extra.get("pybullet_contact", 0)):
            row["collision"] = 1
            row["pybullet_contact"] = 1
        else:
            row["pybullet_contact"] = 0
        return row
    return evaluate_rollout(method, spec, trajectory, compute_time_ms=compute_ms)


def main() -> None:
    args = parse_args()
    outputs = Path(args.outputs_dir)
    tables = outputs / "tables"
    backend, backend_reason = resolve_backend(args.backend)
    reaction_delays = args.reaction_delays_s if args.reaction_delays_s is not None else [args.reaction_delay_s]
    plain_model, plain_mean, plain_std, plain_step = load_policy(args.plain_model)
    filtered_model, filtered_mean, filtered_std, filtered_step = load_policy(args.filtered_model)

    rows: list[dict[str, object]] = []
    for seed in range(args.seeds):
        for speed in args.speeds:
            spec = make_course(seed, speed, args.obstacle_radius, args.safety_distance)
            for method in ["rrt_star", "mppi"]:
                try:
                    if method == "rrt_star":
                        trajectory, compute_ms = plan_rrt_star(spec, seed=1000 + seed)
                    else:
                        trajectory, compute_ms = plan_mppi(spec, seed=2000 + seed)
                    row = evaluate_path_with_backend(method, spec, trajectory, compute_ms, backend, args)
                    row["sim_backend"] = backend
                    row["sim_backend_reason"] = backend_reason
                    for reaction_delay_s in reaction_delays:
                        rows.append(latency_adjusted(dict(row), speed, reaction_delay_s))
                except Exception as exc:
                    for reaction_delay_s in reaction_delays:
                        rows.append(
                            {
                                "method": method,
                                "sequence": spec.sequence,
                                "speed_mps": speed,
                                "reaction_delay_s": reaction_delay_s,
                                "delay_distance_m": round(speed * reaction_delay_s, 4),
                                "sim_backend": backend,
                                "sim_backend_reason": backend_reason,
                                "collision": 1,
                                "safety_violation": 1,
                                "latency_violation": 1,
                                "success": 0,
                                "reason": str(exc),
                            }
                        )

            for method, model, mean, std, max_step in [
                ("plain_bc_mppi", plain_model, plain_mean, plain_std, plain_step),
                ("filtered_bc_mppi", filtered_model, filtered_mean, filtered_std, filtered_step),
            ]:
                trajectory, compute_ms = rollout_policy(
                    spec,
                    model,
                    mean,
                    std,
                    max_steps=120,
                    max_step_m=max_step,
                )
                row = evaluate_path_with_backend(method, spec, trajectory, compute_ms, backend, args)
                row["sim_backend"] = backend
                row["sim_backend_reason"] = backend_reason
                for reaction_delay_s in reaction_delays:
                    rows.append(latency_adjusted(dict(row), speed, reaction_delay_s))

    summary = aggregate_method_rows(rows)
    for row in summary:
        method_rows = [r for r in rows if r["method"] == row["method"]]
        row["latency_violation_rate"] = round(float(np.mean([int(r["latency_violation"]) for r in method_rows])), 4)
        row["sim_backend"] = backend
        row["sim_backend_reason"] = backend_reason
        row["speed_mps"] = "all"
        row["reaction_delay_s"] = "all"
    sweep_summary: list[dict[str, object]] = []
    for method in sorted({str(r["method"]) for r in rows}):
        for speed in sorted({float(r["speed_mps"]) for r in rows if str(r["method"]) == method}):
            for reaction_delay_s in sorted(
                {float(r["reaction_delay_s"]) for r in rows if str(r["method"]) == method and float(r["speed_mps"]) == speed}
            ):
                group = [
                    r
                    for r in rows
                    if str(r["method"]) == method
                    and float(r["speed_mps"]) == speed
                    and float(r["reaction_delay_s"]) == reaction_delay_s
                ]
                if not group:
                    continue
                n = len(group)
                sweep_summary.append(
                    {
                        "method": method,
                        "speed_mps": speed,
                        "reaction_delay_s": reaction_delay_s,
                        "cases": n,
                        "success_rate": round(float(np.mean([int(r["success"]) for r in group])), 4),
                        "collision_rate": round(float(np.mean([int(r["collision"]) for r in group])), 4),
                        "safety_violation_rate": round(float(np.mean([int(r["safety_violation"]) for r in group])), 4),
                        "latency_violation_rate": round(float(np.mean([int(r["latency_violation"]) for r in group])), 4),
                        "mean_min_clearance_m": round(
                            float(np.mean([float(r.get("min_boundary_clearance_m", 0.0)) for r in group])), 4
                        ),
                        "mean_compute_time_ms": round(
                            float(np.mean([float(r.get("planner_compute_time_ms", 0.0)) for r in group])), 4
                        ),
                        "sim_backend": backend,
                    }
                )
    write_csv(tables / "pybullet_validation_results.csv", summary)
    write_csv(tables / "pybullet_validation_sweep_summary.csv", sweep_summary)
    write_csv(tables / "pybullet_validation_detail.csv", rows)
    print(f"Wrote {tables / 'pybullet_validation_results.csv'}")
    print(f"Wrote {tables / 'pybullet_validation_sweep_summary.csv'}")


if __name__ == "__main__":
    main()
