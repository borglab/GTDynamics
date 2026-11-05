# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinodynamic_initialization.py
# @brief Construct FK or contact-pose starting configurations for kinodynamics.
# @author GTDynamics contributors
"""Native kinematic and dynamics strategies for the coupled G1 solve."""

from __future__ import annotations

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from mujoco_io import KinematicRecording


def initialize_fk(walk: KinematicRecording) -> gtsam.Values:
    """Use the native kinematic-only FK strategy; motion starts at zero."""
    return gtd.ForwardKinematicsInitializer(robot=walk.robot).run(
        rootLinkName=walk.root.name(),
        initialPose=walk.anchor,
        measuredJoints=walk.measured,
        slices=[gtd.Slice(k) for k in range(walk.count)],
    )


def contact_options() -> gtd.pose_contact_init.Options:
    """Use the established contact-pose noise and upright/motion/height priors."""
    settings = gtd.pose_contact_init.Options()
    settings.recordHistory = False
    settings.initializeUpright = True
    contact_rotation_sigma_degrees = 90.0
    upright_sigma_degrees = 15.0
    motion_rotation_sigma_degrees = 180.0
    settings.rotationSigma = float(np.deg2rad(contact_rotation_sigma_degrees))
    settings.addUprightFactors = True
    settings.uprightSigma = float(np.deg2rad(upright_sigma_degrees))
    settings.addBetweenFactors = True
    settings.betweenRotationSigma = float(np.deg2rad(motion_rotation_sigma_degrees))
    settings.betweenTranslationSigma = 10.0
    settings.addGroundFactors = True
    settings.addNormalFactors = False
    settings.groundHeight = 0.0
    settings.groundSigma = 0.1
    return settings


def initialize_walk(
    walk: KinematicRecording,
    selected_slices: list[int],
    strategy: str,
    contacts: list[gtd.fixed_dynamics.Contact],
    armature: NDArray[np.float64],
) -> tuple[gtsam.Values, dict[str, object]]:
    """Run native immutable initialization and return explicit stage diagnostics.

    The contact mode uses the original contact-pose optimizer and zero
    motion/forces. The default contact-gp-dynamics fits GP velocities and then
    constructs the affine equality-feasible dynamics seed. Positive normal
    support loads and exact mechanics determine accelerations, wrenches and
    torques in one Gaussian solve. No dynamics manifold or iterative optimizer
    is constructed. The later coupled solve optimizes effort/rate costs. The GP
    has no external world-velocity observations. FK remains the zero-motion
    alternative, with gravity support supplied by coupled native assembly.
    """
    if strategy == "fk":
        return initialize_fk(walk), {"configurationInitializationIterations": 0}
    if strategy == "contact":
        diagnostics = gtd.pose_contact_init.Result()
        values: gtsam.Values = gtd.PoseContactInitializer(
            robot=walk.robot, options=contact_options(), contactBuffer=0
        ).run(
            rootLinkName=walk.root.name(),
            initialPose=walk.anchor,
            measuredJoints=walk.measured,
            trajectory=walk.trajectory,
            dt=walk.dt,
            diagnostics=diagnostics,
            selectedSlices=[gtd.Slice(k) for k in selected_slices],
        )
        return values, {"configurationInitializationIterations": diagnostics.iterations}
    if strategy != "contact-gp-dynamics":
        raise ValueError(
            "initialization must be 'fk', 'contact', or 'contact-gp-dynamics'"
        )
    options = gtd.KinodynamicInitializationOptions()
    kinematics = gtd.ContactInterpolationOptions()
    kinematics.contact = contact_options()
    options.kinematics = kinematics
    result: gtd.KinodynamicInitializationResult = gtd.KinodynamicInitializer(
        robot=walk.robot, options=options
    ).run(
        rootLinkName=walk.root.name(),
        initialPose=walk.anchor,
        measuredJoints=walk.measured,
        trajectory=walk.trajectory,
        slices=[gtd.Slice(k) for k in range(walk.count)],
        times=walk.times.tolist(),
        dt=walk.dt,
        contacts=contacts,
        armature=armature,
        selectedSlices=[gtd.Slice(k) for k in selected_slices],
    )
    return result.values, {
        "configurationInitializationIterations": result.kinematics.contact.iterations,
        "contactInitializationSeconds": result.kinematics.contactSeconds,
        "gpInitializationSeconds": result.kinematics.interpolationSeconds,
        "dynamicsInitializationSeconds": result.dynamicsSeconds,
        "dynamicsInitializationIterations": result.dynamics.iterations,
        "dynamicsInitializationAssemblySeconds": result.dynamics.assemblySeconds,
        "dynamicsInitializationAnchorSeconds": result.dynamics.initializationSeconds,
        "dynamicsInitializationManifoldSeconds": result.dynamics.manifoldSeconds,
        "dynamicsInitializationOptimizationSeconds": result.dynamics.optimizationSeconds,
        "dynamicsInitializationObjectiveInitial": result.dynamics.objectiveInitial,
        "dynamicsInitializationObjectiveFinal": result.dynamics.objectiveFinal,
        "dynamicsInitializationEqualityL2": result.dynamics.equalityL2,
        "dynamicsInitializationInequalityL2": result.dynamics.inequalityL2,
        "dynamicsInitializationFeasible": result.dynamics.feasible,
        "nativePipelineSeconds": result.totalSeconds,
    }
