#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file dynamics_walk.py
# @brief Estimate G1 walking accelerations, sole wrenches, and joint torques.
# @author GTDynamics contributors

"""Estimate G1 walking accelerations, contact wrenches, and actuator torques.

Read solve_walk(): fit GP velocities -> prepare fixed kinematics -> define sole
contacts -> solve with CMC-Opt. Poses and velocities stay fixed. Accelerations
have soft priors from local GP velocity differences and remain free to satisfy
the exact articulated dynamics and sole-wrench inequalities.

Run with --start 78 --end 98 --output /tmp/g1-dynamics. The optional output
directory receives dynamics.npz and summary.json. The browser exporter calls
these same functions and handles its own display geometry and replay format.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import time
from typing import Any

import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from fixed_kinematics import FixedKinematics, prepare_fixed_kinematics
from interpolate_walk import InterpolatedWalk, fit_walk
from sole_support import support_rectangle

REPO = Path(__file__).resolve().parents[2]
GP_SETTINGS = {
    "joint_process_sigma_degrees": 100.0,
    "joint_measurement_sigma_degrees": 0.5,
    "base_angular_process_sigma_degrees": 90.0,
    "base_linear_process_sigma": 3.0,
    "base_velocity_sigma": 0.5,
}


@dataclass
class DynamicsSolution:
    """Native solution and its fixed inputs; times are original source knots.

    result.values holds estimated link twist derivatives, joint accelerations,
    wrenches, and torques. The root/sole frames and units follow the native
    solveFixedKinematicsDynamics API. elapsed_seconds covers that call, including
    graph construction. GP construction/elimination have separate native
    timings; input preparation is excluded from elapsed_seconds.
    """

    walk: InterpolatedWalk
    fixed: FixedKinematics
    options: gtd.fixed_dynamics.Options
    result: gtd.fixed_dynamics.Result
    support_rectangles: list[tuple[float, float]]
    elapsed_seconds: float


def solve_walk(
    kinematics_public: Path,
    dense_data: Path,
    *,
    start: float = 78.0,
    end: float = 98.0,
    friction: float = 0.8,
    acceleration_step_seconds: float = 0.02,
    options: gtd.fixed_dynamics.Options | None = None,
) -> DynamicsSolution:
    """Solve dynamics at retained knots in a window of the best kinematics fit.

    The complete GP fit provides velocity context outside the selected window.
    Native acceleration-prior scales use degrees/s² for rotation and m/s² for
    translation. Each sole carries one local [moment; force] wrench in Nm and N.
    Stance-foot velocity/acceleration stationarity constraints remain absent.
    """
    # Fit all source knots before selecting the requested time window.
    walk: InterpolatedWalk = fit_walk(kinematics_public, dense_data, **GP_SETTINGS)
    fixed: FixedKinematics = prepare_fixed_kinematics(
        walk, start, end, acceleration_step_seconds
    )

    # Derive support patches from the same canonical sole frames used by FK.
    rectangles = [support_rectangle(walk.model, side) for side in ("left", "right")]
    contacts: list[gtd.fixed_dynamics.Contact] = [
        gtd.fixed_dynamics.Contact(fixed.contacts[side], *rectangle, friction)
        for side, rectangle in zip(("left", "right"), rectangles)
    ]
    if options is None:
        options = gtd.fixed_dynamics.Options()

    # The native graph fixes q/v, estimates accelerations, and solves all knots
    # jointly with the affine ReducedALRetractor and exact wrench inequalities.
    started = time.perf_counter()
    result: gtd.fixed_dynamics.Result = gtd.solveFixedKinematicsDynamics(
        fixed.robot,
        fixed.trajectory,
        fixed.slices,
        fixed.times.tolist(),
        fixed.values,
        fixed.acceleration_priors,
        contacts,
        fixed.armature,
        options,
    )
    return DynamicsSolution(
        walk, fixed, options, result, rectangles, time.perf_counter() - started
    )


def solution_arrays(solution: DynamicsSolution) -> dict[str, NDArray[Any]]:
    """Extract portable arrays; joint columns follow joint_ids and joint_names.

    times are seconds and source_indices identify original 50 Hz slices.
    wrenches are (knots, left/right, 6), sole-local [moment; force], Nm and N.
    Torques are Nm; joint accelerations are rad/s². Root twist derivatives are
    angular-first in moving COM axes: physical linear acceleration additionally
    includes omega_body cross v_body. Prior arrays use these same conventions.
    """
    fixed, result = solution.fixed, solution.result
    count = len(fixed.times)
    root = fixed.robot.link("pelvis").id()
    names = {joint.id(): joint.name() for joint in fixed.robot.joints()}
    arrays = {
        "times": fixed.times,
        "source_indices": np.array([slice_.k for slice_ in fixed.slices]),
        "contact": np.asarray(fixed.active),
        "foot_names": np.array(["left", "right"]),
        "joint_ids": np.array(result.jointIds),
        "joint_names": np.array([names[joint] for joint in result.jointIds]),
        "wrenches": np.array(
            [result.values.atVector(key) for key in result.contactKeys]
        ).reshape(count, 2, 6),
        "joint_torques": np.array(
            [result.values.atDouble(key) for key in result.torqueKeys]
        ).reshape(count, -1),
    }
    for label, values in (
        ("accelerations", result.values),
        ("acceleration_priors", fixed.acceleration_priors),
    ):
        arrays[f"base_twist_{label}"] = np.array(
            [gtd.TwistAccel(values, root, slice_.k) for slice_ in fixed.slices]
        )
        arrays[f"joint_{label}"] = np.array(
            [
                [gtd.JointAccel(values, joint, slice_.k) for joint in result.jointIds]
                for slice_ in fixed.slices
            ]
        )
    return arrays


def summarize(solution: DynamicsSolution) -> dict[str, Any]:
    """Report convergence and timing scopes without browser-specific metadata."""
    result = solution.result
    return {
        "variant": solution.walk.variant,
        "samples": len(solution.fixed.times),
        "first_time_seconds": float(solution.fixed.times[0]),
        "last_time_seconds": float(solution.fixed.times[-1]),
        "iterations": result.iterations,
        "dynamics_seconds": solution.elapsed_seconds,
        "gp_construction_seconds": solution.walk.interpolation.graphConstructionSeconds(),
        "gp_solve_seconds": solution.walk.interpolation.solveSeconds(),
        "objective": result.objectiveFinal,
        "equality_l2": result.equalityL2,
        "inequality_l2": result.inequalityL2,
        "max_force_residual_n": result.maxForceResidual,
        "max_moment_residual_nm": result.maxMomentResidual,
        "feasible": bool(result.feasible),
        "soft_base_balance": bool(result.softDynamics),
        "affine_equalities": bool(result.affineEqualities),
        "equality_only": bool(result.equalityOnly),
        "conditioned_variable_dimension": result.conditionedVariableDimension,
        "conditioning_seconds": result.conditioningSeconds,
        "original_inequality_dimension": result.originalInequalityDimension,
        "original_inequality_l2": result.originalInequalityL2,
        "assembly_seconds": result.assemblySeconds,
        "anchor_seconds": result.initializationSeconds,
        "manifold_seconds": result.manifoldSeconds,
        "optimization_seconds": result.optimizationSeconds,
        "optimizer_linearization_seconds": result.optimizerLinearizationSeconds,
        "optimizer_linear_solve_seconds": result.optimizerLinearSolveSeconds,
        "retraction_seconds": result.retractionSeconds,
        "equality_preparations": result.equalityPreparations,
        "affine_basis_constructions": result.affineBasisConstructions,
        "gaussian_solves": result.gaussianSolves,
        "qp_solves": result.qpSolves,
        "projected_gradient_norm": result.projectedGradientNorm,
        "stationary": bool(result.stationary),
        "support_rectangles_m": solution.support_rectangles,
    }


def argument_parser() -> argparse.ArgumentParser:
    """Shared input and solver arguments for the example and replay exporter."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--kinematics", type=Path, default=REPO / "apps/kinematics-inspector/public"
    )
    parser.add_argument(
        "--dense-data",
        type=Path,
        default=REPO / "apps/kinematics-inspector/.cache/openio-contact-source",
    )
    parser.add_argument("--start", type=float, default=78.0)
    parser.add_argument("--end", type=float, default=98.0)
    parser.add_argument("--friction", type=float, default=0.8)
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument("--equality-only", action="store_true",
                        help="Explicit benchmark relaxation: omit wrench/torque bounds")
    parser.add_argument("--linear-solver", choices=("automatic", "gaussian", "gtsam-qp", "proxqp"),
                        default="automatic")
    parser.add_argument("--retraction-threads", type=int, default=1)
    parser.add_argument("--acceleration-step-ms", type=float, default=20.0)
    parser.add_argument(
        "--base-angular-acceleration-sigma-degrees", type=float, default=90.0
    )
    parser.add_argument("--base-linear-acceleration-sigma", type=float, default=3.0)
    parser.add_argument("--joint-acceleration-sigma-degrees", type=float, default=100.0)
    parser.add_argument(
        "--base-force-sigma", type=float, default=5.0, help="N, soft mode"
    )
    parser.add_argument(
        "--base-moment-sigma", type=float, default=1.0, help="Nm, soft mode"
    )
    parser.add_argument(
        "--soft-base-balance",
        action="store_true",
        help="Publish a residual base wrench",
    )
    return parser


def options_from_arguments(args: argparse.Namespace) -> gtd.fixed_dynamics.Options:
    """Configure native options directly; rotational scales stay in degrees."""
    options = gtd.fixed_dynamics.Options()
    options.maxIterations = args.max_iterations
    options.equalityOnly = args.equality_only
    options.retractionThreads = args.retraction_threads
    options.linearSolver = {
        "automatic": gtd.fixed_dynamics.LinearSolver.Automatic,
        "gaussian": gtd.fixed_dynamics.LinearSolver.Gaussian,
        "gtsam-qp": gtd.fixed_dynamics.LinearSolver.GtsamQP,
        "proxqp": gtd.fixed_dynamics.LinearSolver.ProxQP,
    }[args.linear_solver]
    options.baseAngularAccelerationSigmaDegrees = (
        args.base_angular_acceleration_sigma_degrees
    )
    options.baseLinearAccelerationSigma = args.base_linear_acceleration_sigma
    options.jointAccelerationSigmaDegrees = args.joint_acceleration_sigma_degrees
    options.softDynamics = args.soft_base_balance
    options.balanceForceSigma = args.base_force_sigma
    options.balanceMomentSigma = args.base_moment_sigma
    return options


def main() -> None:
    """Run one dynamics solve and optionally save its arrays and diagnostics."""
    parser = argument_parser()
    parser.add_argument(
        "--output", type=Path, help="Directory for dynamics.npz and summary.json"
    )
    args = parser.parse_args()
    solution = solve_walk(
        args.kinematics,
        args.dense_data,
        start=args.start,
        end=args.end,
        friction=args.friction,
        acceleration_step_seconds=args.acceleration_step_ms / 1000,
        options=options_from_arguments(args),
    )
    summary = json.dumps(summarize(solution), indent=2, allow_nan=False) + "\n"
    print(summary, end="")
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.output / "dynamics.npz", **solution_arrays(solution))
        (args.output / "summary.json").write_text(summary)
    if not solution.result.feasible:
        raise SystemExit("CMC-Opt returned dynamics constraint violations")


if __name__ == "__main__":
    main()
