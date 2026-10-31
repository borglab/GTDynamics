# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
"""Compare contact and IK initialization by calling the G1 kinematics example.

The example owns graph construction, initialization, CM-Opt, and evaluation.
This driver owns repeated runs, wall-clock measurements, and timing reports.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from statistics import median
from time import perf_counter
from typing import Any

import gtdynamics as gtd
import numpy as np

import kinematic_walk as walk_example
from mujoco_io import KinematicRecording


def compare(
    walk: KinematicRecording, repeats: int = 1, max_iterations: int = 50
) -> dict[str, Any]:
    """Alternate run order; time preparation once, then init and CM-Opt separately."""
    start = perf_counter()
    costs, constraints, keys = walk_example.build_problem(walk)
    graph_seconds = perf_counter() - start
    results: dict[str, list[dict[str, Any]]] = {"contact": [], "ik": []}
    for repeat in range(repeats):
        order = ("contact", "ik") if repeat % 2 == 0 else ("ik", "contact")
        for method in order:
            start = perf_counter()
            seed, initialization = walk_example.initialize(walk, method, keys)
            init_seconds = perf_counter() - start
            initial_metrics = walk_example.evaluate(walk, costs, constraints, seed)
            start = perf_counter()
            solution, solver = walk_example.solve_cm(
                costs, constraints, seed, walk.root.id(), max_iterations
            )
            solve_seconds = perf_counter() - start
            metrics = walk_example.evaluate(walk, costs, constraints, solution)
            if not all(np.isfinite(v) for v in metrics.values()):
                raise RuntimeError(f"{method} returned nonfinite metrics")
            run = {
                "initialization_seconds": init_seconds,
                "solve_seconds": solve_seconds,
                "total_seconds": init_seconds + solve_seconds,
                "initialization": initialization,
                "solver": solver,
                "initial": initial_metrics,
                "final": metrics,
                "equality_feasible": metrics["kinematic_equality_l2"] < 1e-5,
                "iteration_limit_reached": solver["iterations"] >= max_iterations,
            }
            results[method].append(run)
            print(
                f"{method:7s} run {repeat + 1}: init {init_seconds:.3f}s, CM-Opt {solve_seconds:.3f}s, "
                f'cost {metrics["cost"]:.5g}, equality L2 {metrics["kinematic_equality_l2"]:.3g}',
                flush=True,
            )
    return {
        "samples": walk.count,
        "duration_seconds": (walk.count - 1) * walk.dt,
        "phases": walk.trajectory.numPhases(),
        "equality_constraints": constraints.size(),
        "inequality_constraints": 0,
        "cost_factors": costs.size(),
        "graph_build_seconds": graph_seconds,
        "max_iterations": max_iterations,
        "input_sha256": walk.provenance,
        "timing_scope": "initialization includes native adapter/FK and Python assembly; solve includes CM-Opt transformation, feasibility retraction, LM, and recovery; IO, shared graph build, and evaluation excluded",
        "runs": results,
        "medians": {
            method: {
                name: median(r[name] for r in runs)
                for name in ("initialization_seconds", "solve_seconds", "total_seconds")
            }
            for method, runs in results.items()
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path(__file__).parent / "results")
    parser.add_argument(
        "--contacts",
        type=Path,
        default=walk_example.REPO / "apps/contact-initialization/public/g1-walk.json",
    )
    parser.add_argument(
        "--repeats", type=int, default=3, help="Number of paired contact/IK repetitions"
    )
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument(
        "--max-frames",
        type=int,
        help="Diagnostic prefix only; default is the ENTIRE snippet",
    )
    parser.add_argument(
        "--output", type=Path, help="Optional timing and validation JSON"
    )
    args = parser.parse_args()
    if (
        args.repeats < 1
        or args.max_iterations < 1
        or (args.max_frames is not None and args.max_frames < 2)
    ):
        parser.error("Counts must be positive and --max-frames must be at least two")
    walk = walk_example.load_walk(args.data, args.contacts, args.max_frames)
    print(
        f"One trajectory: {walk.count} samples, {walk.trajectory.numPhases()} contact-aligned phases",
        flush=True,
    )
    result = compare(walk, args.repeats, args.max_iterations)
    result["recorded_utc"] = datetime.now(timezone.utc).isoformat()
    result["platform"] = platform.platform()
    result["revision"] = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=walk_example.REPO, text=True
    ).strip()
    result["source_sha256"] = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (
            Path(__file__),
            Path(walk_example.__file__),
            Path(__file__).with_name("mujoco_io.py"),
            walk_example.REPO / "gtdynamics.i",
            walk_example.REPO / "gtdynamics/kinematics/KinematicsTrajectory.cpp",
            walk_example.REPO / "gtdynamics/factors/ContactEqualityFactor.h",
            walk_example.REPO / "gtdynamics/utils/ContactTrajectoryInitializer.cpp",
        )
    }
    result["module_sha256"] = hashlib.sha256(
        Path(gtd.gtdynamics.__file__).read_bytes()
    ).hexdigest()
    print(json.dumps(result["medians"], indent=2))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    if any(
        not run["equality_feasible"] for runs in result["runs"].values() for run in runs
    ):
        raise SystemExit(
            "CM-Opt returned an infeasible kinematic trajectory; see diagnostics"
        )


if __name__ == "__main__":
    main()
