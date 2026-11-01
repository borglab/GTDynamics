# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
"""Fit the complete G1 walk with CM-Opt and a contact or IK initializer.

Read load_walk -> build_problem -> initialize -> solve_cm -> evaluate.
The graph is assembled here using the native Slice/Phase/Trajectory API.
Run one fit with --initializer contact (default) or --initializer ik.
See compare_kinematic_walk.py for the timing comparison that calls this example.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Iterator, Sequence
import json
from pathlib import Path

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from mujoco_io import KinematicRecording, load_kinematic_recording

REPO = Path(__file__).resolve().parents[2]


def load_walk(
    data: Path, contacts_path: Path, max_frames: int | None = None
) -> KinematicRecording:
    """Load geometry, measurements, and the trajectory of contact phases."""
    return load_kinematic_recording(data, contacts_path, max_frames)


def contact_slices(
    trajectory: gtd.Trajectory,
) -> Iterator[tuple[int, gtd.PointOnLinks, list[gtd.PointOnLink]]]:
    """Visit disjoint phase slices, retaining ground contacts across boundaries."""
    previous: Sequence[gtd.PointOnLink] = []
    previous_k: int | None = None
    phases: Sequence[gtd.Phase] = trajectory.phases()
    for phase in phases:
        spec: gtd.ConstraintSpec = phase.constraintSpec()
        points: gtd.PointOnLinks = spec.groundContactPoints()
        for k in range(phase.k_start, phase.k_end + 1):
            continuing = [
                p
                for p in points
                if previous_k == k - 1 and any(p.equals(q, 0.0) for q in previous)
            ]
            yield k, points, continuing
            previous, previous_k = points, k


def build_problem(
    walk: KinematicRecording,
    contact_slip_sigma: float = 0.005,
) -> tuple[gtsam.NonlinearFactorGraph, gtd.NonlinearEqualityConstraints, list[int]]:
    """Build one full trajectory problem for either initializer.

    Hard equalities: each link/joint pose relation, via Kinematics.constraints.
    Costs: measured q, level stance soles, soft contact slip, weak root
    smoothness, and a common first-pose gauge. Contact observations are soft
    because stance feet may slip. contact_slip_sigma is the per-axis standard
    deviation of point displacement in metres per adjacent slice interval.
    It is not a velocity sigma and is not automatically scaled with sample rate.
    There are no inequalities, dynamics variables, or per-frame final solves.
    """
    if not np.isfinite(contact_slip_sigma) or contact_slip_sigma <= 0:
        raise ValueError("contact_slip_sigma must be finite and positive")
    parameters = gtd.KinematicsParameters()
    parameters.p_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 1.0)
    parameters.prior_q_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.03)
    parameters.cp_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.01)
    kinematics = gtd.Kinematics(parameters)
    constraints: gtd.NonlinearEqualityConstraints = kinematics.constraints(
        walk.trajectory, walk.robot
    )
    costs: gtsam.NonlinearFactorGraph = kinematics.jointAngleObjectives(
        walk.trajectory, walk.robot, walk.measured
    )
    for phase in walk.trajectory.phases():
        height_costs: gtsam.NonlinearFactorGraph = kinematics.contactHeightObjectives(
            phase, np.array([0.0, 0.0, -9.81])
        )
        costs.push_back(height_costs)
    # Penalize displacement of each continuing ground contact point, including
    # across phase boundaries where the other foot changes contact.
    costs.push_back(
        kinematics.contactEqualityFactors(
            walk.trajectory, gtsam.noiseModel.Isotropic.Sigma(3, contact_slip_sigma)
        )
    )
    for k in range(walk.count - 1):
        costs.add(
            gtsam.BetweenFactorPose3(
                gtd.PoseKey(walk.root.id(), k),
                gtd.PoseKey(walk.root.id(), k + 1),
                gtsam.Pose3(),
                gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3] * 3 + [0.1] * 3)),
            )
        )
    first = gtd.Slice(0)
    costs.add(
        gtsam.PriorFactorPose3(
            gtd.PoseKey(walk.root.id(), first.k),
            walk.anchor,
            gtsam.noiseModel.Isotropic.Sigma(6, 1e-5),
        )
    )
    keys = sorted(set(costs.keyVector()) | set(constraints.keyVector()))
    return costs, constraints, keys


def nominal_root(walk: KinematicRecording, k: int) -> gtsam.Pose3:
    """A commanded straight-line path for IK, not the recorded root trajectory."""
    matrix: NDArray[np.float64] = walk.anchor.matrix()
    matrix[0, 3] += walk.speed * k * walk.dt
    return gtsam.Pose3(matrix)


def initialize(
    walk: KinematicRecording,
    method: str,
    keys: Sequence[int],
    contact_diagnostics: gtd.contact_init.Result | None = None,
    ik_iteration: Callable[[int, int, float, gtsam.Values], None] | None = None,
    contact_options: gtd.contact_init.Options | None = None,
) -> tuple[gtsam.Values, dict[str, int | float | bool]]:
    """Call the native initializers; optional observers support offline inspection.

    Contact diagnostics retain the sparse solve history when supplied. The IK
    observer receives each slice's initial state and completed LM iterations,
    with global trajectory keys. Ordinary timing runs leave observers disabled.
    """
    initializer = gtd.Initializer()
    if method == "contact":
        options = (
            contact_options
            if contact_options is not None
            else gtd.contact_init.Options()
        )
        options.recordHistory = contact_diagnostics is not None
        diagnostics = (
            contact_diagnostics
            if contact_diagnostics is not None
            else gtd.contact_init.Result()
        )
        values: gtsam.Values = initializer.InitializeSolutionContacts(
            walk.robot,
            walk.root.name(),
            walk.anchor,
            walk.measured,
            walk.trajectory,
            walk.dt,
            options,
            diagnostics,
        )
        info = {
            "sparse_frames": len(diagnostics.poses),
            "iterations": diagnostics.iterations,
            "contact_rms_m": diagnostics.residualRms,
            "converged": diagnostics.converged,
        }
        return values.extract(keys), info
    if method != "ik":
        raise ValueError(f"Unknown initialization: {method}")
    values = gtsam.Values()
    for phase in walk.trajectory.phases():
        spec: gtd.ConstraintSpec = phase.constraintSpec()
        # The legacy IK initializer uses local indices starting at zero and a
        # fixed contact set, so call it once per phase, then map to global keys.
        steps = max(1, phase.k_end - phase.k_start)

        def report_ik(
            sample: int, iteration: int, error: float, state: gtsam.Values
        ) -> None:
            if ik_iteration is None or phase.k_start + sample > phase.k_end:
                return
            # The observer receives global trajectory keys, like the final seed.
            global_state = gtsam.Values()
            for link in walk.robot.links():
                global_state.insert(
                    gtd.PoseKey(link.id(), phase.k_start + sample),
                    state.atPose3(gtd.PoseKey(link.id(), sample)),
                )
            for joint in walk.robot.joints():
                global_state.insert(
                    gtd.JointAngleKey(joint.id(), phase.k_start + sample),
                    state.atDouble(gtd.JointAngleKey(joint.id(), sample)),
                )
            ik_iteration(phase.k_start + sample, iteration, error, global_state)

        local: gtsam.Values = initializer.InitializeSolutionInverseKinematics(
            walk.robot,
            walk.root.name(),
            nominal_root(walk, phase.k_start),
            [nominal_root(walk, phase.k_start + steps)],
            [steps * walk.dt],
            walk.dt,
            0.0,
            spec.contactPoints(),
            report_ik if ik_iteration is not None else None,
        )
        for k in range(phase.k_start, phase.k_end + 1):
            for link in walk.robot.links():
                values.insert(
                    gtd.PoseKey(link.id(), k),
                    local.atPose3(gtd.PoseKey(link.id(), k - phase.k_start)),
                )
            for joint in walk.robot.joints():
                values.insert(
                    gtd.JointAngleKey(joint.id(), k),
                    local.atDouble(gtd.JointAngleKey(joint.id(), k - phase.k_start)),
                )
    return values.extract(keys), {"phase_calls": walk.trajectory.numPhases()}


def evaluate(
    walk: KinematicRecording,
    costs: gtsam.NonlinearFactorGraph,
    constraints: gtd.NonlinearEqualityConstraints,
    values: gtsam.Values,
) -> dict[str, float]:
    """Report physical residuals separately from solver stopping criteria."""
    heights, slips, joint_errors, reference_errors = [], [], [], []
    for k, points, continuing in contact_slices(walk.trajectory):
        for point in points:
            position: NDArray[np.float64] = point.predict(values, k)
            heights.append(position[2])
        for point in continuing:
            current: NDArray[np.float64] = point.predict(values, k)
            previous: NDArray[np.float64] = point.predict(values, k - 1)
            slips.append(np.linalg.norm(current - previous))
    for joint in walk.robot.joints():
        for k in range(walk.count):
            key = gtd.JointAngleKey(joint.id(), k)
            joint_errors.append(values.atDouble(key) - walk.measured.atDouble(key))
    for k, reference in enumerate(walk.reference_root_positions):
        reference_errors.append(
            np.linalg.norm(
                values.atPose3(gtd.PoseKey(walk.root.id(), k)).translation() - reference
            )
        )

    def rms(samples: Sequence[float]) -> float:
        return float(np.sqrt(np.mean(np.square(samples))))

    return {
        "cost": costs.error(values),
        "kinematic_equality_l2": float(constraints.violationNorm(values)),
        "joint_measurement_rms_rad": rms(joint_errors),
        "stance_height_rms_m": rms(heights),
        "stance_step_slip_rms_m": rms(slips),
        "reference_root_translation_rms_m": rms(reference_errors),
    }


def solve_cm(
    costs: gtsam.NonlinearFactorGraph,
    constraints: gtd.NonlinearEqualityConstraints,
    initial: gtsam.Values,
    root_id: int,
    max_iterations: int = 50,
    on_iteration: Callable[[int, gtsam.Values], None] | None = None,
) -> tuple[gtsam.Values, dict[str, int]]:
    """Solve the entire trajectory; optionally observe feasible start and iterates.

    Observer Values contain recovered full robot states, not reduced coordinates.
    History capture adds overhead, so the timing driver leaves it disabled.
    """

    def basis_keys(keys: Sequence[int]) -> list[int]:
        return [
            key
            for key in keys
            if gtd.DynamicsSymbol(key).label() == "q"
            or key == gtd.PoseKey(root_id, gtd.DynamicsSymbol(key).time())
        ]

    parameters = gtd.ManifoldOptimizerParameters()
    parameters.constraintManifoldParams.basisCreator = gtd.EliminationBasisCreator(
        basis_keys
    )
    retraction = gtd.RetractionParams()
    retraction.lmParams.setMaxIterations(30)
    retraction.lmParams.setAbsoluteErrorTol(1e-18)
    retraction.lmParams.setRelativeErrorTol(1e-10)
    retraction.feasibleThreshold = 1e-20
    parameters.constraintManifoldParams.retractorCreator = gtd.BasisRetractorCreator(
        basis_keys, retraction
    )
    lm = gtsam.LevenbergMarquardtParams()
    lm.setMaxIterations(max_iterations)
    lm.setRelativeErrorTol(1e-7)
    lm.setAbsoluteErrorTol(1e-8)

    # Install the observer before constructing CM-Opt, which copies LM settings.
    observer: (
        tuple[
            gtd.NonlinearManifoldOptimizer,
            gtd.ManifoldOptimizationProblem,
            gtsam.NonlinearOptimizer,
        ]
        | None
    ) = None

    def report_iteration(iteration: int, before: float, after: float) -> None:
        if on_iteration is not None and observer is not None:
            owner, manifold, solver = observer
            on_iteration(iteration, owner.baseValues(manifold, solver.values()))

    if on_iteration is not None:
        lm.iterationHook = report_iteration
    cm = gtd.NonlinearManifoldOptimizer(parameters, lm)
    problem: gtd.ManifoldOptimizationProblem = cm.initializeManifoldOptimizationProblem(
        costs, constraints, initial
    )
    optimizer: gtsam.NonlinearOptimizer = cm.constructNonlinearOptimizer(problem)
    try:
        if on_iteration is not None:
            observer = (cm, problem, optimizer)
            on_iteration(0, cm.baseValues(problem, optimizer.values()))
        manifold_values: gtsam.Values = optimizer.optimize()
        solution: gtsam.Values = cm.baseValues(problem, manifold_values)
    finally:
        # The native optimizer owns the callback. Release its references back to
        # the optimizer/problem even if a recording callback raises an exception.
        observer = None
    return solution, {
        "iterations": optimizer.iterations(),
        "components": len(problem.components),
        "ambient_dimension": initial.dim(),
        "reduced_dimension": problem.values.dim(),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path(__file__).parent / "results")
    parser.add_argument(
        "--contacts",
        type=Path,
        default=REPO / "apps/contact-initialization/public/g1-walk.json",
    )
    parser.add_argument("--initializer", choices=("contact", "ik"), default="contact")
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument(
        "--contact-slip-sigma",
        type=float,
        default=0.005,
        help="Contact displacement sigma in metres per slice interval",
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
    costs, constraints, keys = build_problem(walk, args.contact_slip_sigma)
    seed, initialization = initialize(walk, args.initializer, keys)
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
        "contact_slip_sigma_m": args.contact_slip_sigma,
        "initialization": initialization,
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
