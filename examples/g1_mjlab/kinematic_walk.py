# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinematic_walk.py
# @brief Solve G1 kinematics on selected supported slices.
# @author GTDynamics contributors
"""Fit a complete G1 walk with CM-Opt and contact or IK initialization.

Read load_walk -> build_problem -> initialize -> solve_cm -> evaluate.
Phase inference, initialization, and smoothing live in their named modules.
Run one fit with --initializer contact (default) or --initializer ik.
The separate compare_kinematic_walk.py timing driver calls this example.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

# Direct imports also expose the example's steps to comparison/export drivers.
from contact_phases import infer_contact_phases
from kinematic_initialization import (
    initialize,
    initialize_contact_slices,
    seed_contact_landmarks,
)
from kinematic_smoothing import (
    CONTACT_ROTATION_SIGMA_DEGREES,
    build_problem,
    evaluate,
    solve_cm,
)
from mujoco_io import KinematicRecording, load_kinematic_recording

REPO = Path(__file__).resolve().parents[2]


def load_walk(
    data: Path, contacts_path: Path, max_frames: int | None = None
) -> KinematicRecording:
    """Load geometry, measurements, and the trajectory of contact phases."""
    return load_kinematic_recording(data, contacts_path, max_frames)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upright-prior", action="store_true")
    parser.add_argument("--height-prior", action="store_true")
    parser.add_argument("--data", type=Path, default=Path(__file__).parent / "results")
    parser.add_argument(
        "--contacts",
        type=Path,
        default=REPO / "apps/contact-initialization/public/g1-walk.json",
    )
    parser.add_argument(
        "--contact-slices",
        action="store_true",
        default=True,
        help="Infer contact phases, initialize and solve only selected supported slices",
    )
    parser.add_argument("--initializer", choices=("contact", "ik"), default="contact")
    parser.add_argument("--max-slice-interval-ms", type=float, default=200.0)
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument(
        "--contact-translation-sigma",
        type=float,
        default=0.01,
        help="Contact-to-landmark translation sigma in metres per observation",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        help="Diagnostic prefix only; default is the ENTIRE snippet",
    )
    parser.add_argument(
        "--output", type=Path, help="Optional solution diagnostics JSON"
    )
    args = parser.parse_args()
    if args.max_iterations < 1 or (args.max_frames is not None and args.max_frames < 2):
        parser.error("--max-iterations must be positive and --max-frames at least two")

    walk = load_walk(args.data, args.contacts, args.max_frames)
    selected_seed = None
    phase_info = None
    if args.contact_slices:
        if np.isclose(walk.dt, 0.02):
            walk, phase_info = infer_contact_phases(walk, args.data)
        walk, selected_seed, initialization = initialize_contact_slices(
            walk,
            rotation_sigma_degrees=CONTACT_ROTATION_SIGMA_DEGREES,
            max_slice_interval_ms=args.max_slice_interval_ms,
        )
    costs, constraints, keys = build_problem(
        walk,
        args.contact_translation_sigma,
        upright_prior=args.upright_prior,
        height_prior=args.height_prior,
    )
    if selected_seed is None or args.initializer == "ik":
        seed, initialization = initialize(walk, args.initializer, keys)
    else:
        seed = seed_contact_landmarks(walk, selected_seed).extract(keys)
    initial_metrics = evaluate(walk, costs, constraints, seed)
    solution, solver = solve_cm(
        costs, constraints, seed, walk.root.id(), args.max_iterations
    )
    metrics = evaluate(walk, costs, constraints, solution)
    if not all(np.isfinite(value) for value in metrics.values()):
        raise RuntimeError("CM-Opt returned nonfinite metrics")
    result = {
        "samples": walk.count,
        "phases": walk.trajectory.numPhases(),
        "initializer": args.initializer,
        "contact_translation_sigma_m": args.contact_translation_sigma,
        "initialization": initialization,
        "contact_phase_inference": phase_info,
        "source_indices": (
            walk.source_indices.tolist() if walk.source_indices is not None else None
        ),
        "timestamps": walk.times.tolist(),
        "solver": solver,
        "initial": initial_metrics,
        "final": metrics,
        "equality_feasible": metrics["kinematic_equality_l2"] < 1e-5,
        "iteration_limit_reached": solver["iterations"] >= args.max_iterations,
    }
    print(json.dumps(result, indent=2, allow_nan=False))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    if not result["equality_feasible"]:
        raise SystemExit("CM-Opt returned an infeasible kinematic trajectory")


if __name__ == "__main__":
    main()
