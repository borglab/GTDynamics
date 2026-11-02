# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
"""Build, solve, and evaluate the constrained G1 trajectory factor graph."""

from __future__ import annotations

from collections.abc import Callable, Sequence

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from contact_phases import contact_slices, stance_observations
from mujoco_io import KinematicRecording

CONTACT_ROTATION_SIGMA_DEGREES = 90.0


def build_problem(
    walk: KinematicRecording,
    contact_translation_sigma: float = 0.01,
    objective_groups: dict[str, gtsam.NonlinearFactorGraph] | None = None,
    *,
    upright_prior: bool = False,
    height_prior: bool = False,
    rotation_sigma_degrees: float = CONTACT_ROTATION_SIGMA_DEGREES,
) -> tuple[gtsam.NonlinearFactorGraph, gtd.NonlinearEqualityConstraints, list[int]]:
    """Fit the trajectory with shared stance landmarks, as in initialization.

    ContactPoseFactor measures Log((world_T_footCOM * COM_T_sole)^-1 *
    world_T_stance). Unlike frozen root-relative FK in initialization, the foot
    COM pose is optimized here through the hard link/joint kinematic equalities.
    Each observation has independent rotation (degrees) and translation (metres)
    scales. This is an observation-to-landmark cost, not interval displacement.

    Optional priors reuse the initializer's native factors: root upright at 15
    degrees, and landmark world Z=0 at 10 cm, once per uninterrupted stance.
    Height does not constrain sole orientation. No weak motion prior is added;
    the caller can supply IMU factors. Joint measurements and the root gauge
    remain present in every configuration. Initialize the extra Pose3 variables
    with seed_contact_landmarks before solving.
    """
    if not np.isfinite(contact_translation_sigma) or contact_translation_sigma <= 0:
        raise ValueError("contact_translation_sigma must be finite and positive")
    if not np.isfinite(rotation_sigma_degrees) or rotation_sigma_degrees <= 0:
        raise ValueError("rotation_sigma_degrees must be finite and positive")
    parameters = gtd.KinematicsParameters()
    parameters.p_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 1.0)
    parameters.prior_q_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.03)
    kinematics = gtd.Kinematics(parameters)
    constraints: gtd.NonlinearEqualityConstraints = kinematics.constraints(
        walk.trajectory, walk.robot
    )
    groups = {
        "joint": kinematics.jointAngleObjectives(
            walk.trajectory, walk.robot, walk.measured
        ),
        "contact": gtsam.NonlinearFactorGraph(),
        "anchor": gtsam.NonlinearFactorGraph(),
    }
    contact_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array(
            [np.deg2rad(rotation_sigma_degrees)] * 3 + [contact_translation_sigma] * 3
        )
    )
    landmarks = set()
    for k, contact, key in stance_observations(walk):
        groups["contact"].add(
            gtd.ContactPoseFactor(
                gtd.PoseKey(contact.link.id(), k),
                key,
                contact_noise,
                contact.comTcontact,
            )
        )
        landmarks.add(key)
    if height_prior:
        height = gtsam.NonlinearFactorGraph()
        for key in sorted(landmarks):
            height.add(
                gtd.ContactHeightFactor(
                    key,
                    gtsam.noiseModel.Isotropic.Sigma(1, 0.1),
                    np.zeros(3),
                    np.array([0.0, 0.0, -1.0]),
                    0.0,
                )
            )
        groups["height"] = height
    if upright_prior:
        upright = gtsam.NonlinearFactorGraph()
        upright_sigma_degrees = 15.0
        for k in range(1, walk.count):
            upright.add(
                gtd.contact_init.UprightFactor(
                    gtd.PoseKey(walk.root.id(), k), np.deg2rad(upright_sigma_degrees)
                )
            )
        groups["upright"] = upright
    groups["anchor"].add(
        gtsam.PriorFactorPose3(
            gtd.PoseKey(walk.root.id(), 0),
            walk.anchor,
            gtsam.noiseModel.Diagonal.Sigmas(
                np.array([np.deg2rad(0.000573)] * 3 + [1e-5] * 3)
            ),
        )
    )
    costs = gtsam.NonlinearFactorGraph()
    for group in groups.values():
        costs.push_back(group)
    if objective_groups is not None:
        objective_groups.update(groups)
    keys = sorted(set(costs.keyVector()) | set(constraints.keyVector()))
    return costs, constraints, keys


def evaluate(
    walk: KinematicRecording,
    costs: gtsam.NonlinearFactorGraph,
    constraints: gtd.NonlinearEqualityConstraints,
    values: gtsam.Values,
) -> dict[str, float]:
    """Report physical residuals separately from solver stopping criteria."""
    heights, slips, joint_errors, reference_errors = [], [], [], []
    for k, points, continuing in contact_slices(walk.trajectory, walk.contact_breaks):
        for point in points:
            position: NDArray[np.float64] = point.predict(values, k).translation()
            heights.append(position[2])
        for point in continuing:
            current: NDArray[np.float64] = point.predict(values, k).translation()
            previous: NDArray[np.float64] = point.predict(values, k - 1).translation()
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

    last_reported_iteration = 0

    def report_iteration(iteration: int, before: float, after: float) -> None:
        nonlocal last_reported_iteration
        # LM also invokes its hook when a terminal rejected step leaves the
        # iteration counter and Values unchanged. Record completed iterations.
        if iteration <= last_reported_iteration:
            return
        last_reported_iteration = iteration
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
