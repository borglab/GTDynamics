# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinodynamic_problem.py
# @brief Assemble observations and model parameters for the coupled G1 solve.
# @author GTDynamics contributors
"""Python observations and model inputs for the native G1 kinodynamic problem.

The native solver adds dynamics, collocation, wrench constraints, and motion
variables. These helpers supply measured joints, sole-pose observations, IMU
factors, and the canonical model's contact patches and rotor inertias.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from contact_phases import stance_observations
from g1_model import load_g1_model
from imu_aiding import BIAS_KEY, ImuAiding, build_imu_aiding
from kinematic_initialization import seed_contact_landmarks
from mujoco_io import KinematicRecording, contact_frames
from sole_support import support_rectangle


def encoder_factors(walk: KinematicRecording) -> gtsam.NonlinearFactorGraph:
    """Constrain each joint angle to its encoder reading with 0.5 degree sigma."""
    encoders = gtsam.NonlinearFactorGraph()
    encoder_sigma_degrees = 0.5
    encoder_noise = gtsam.noiseModel.Isotropic.Sigma(
        1, np.deg2rad(encoder_sigma_degrees)
    )
    for k in range(walk.count):
        for joint in walk.robot.joints():
            key = gtd.JointAngleKey(joint.id(), k)
            encoders.add(
                gtsam.PriorFactorDouble(key, walk.measured.atDouble(key), encoder_noise)
            )
    return encoders


def contact_pose_factors(walk: KinematicRecording) -> gtsam.NonlinearFactorGraph:
    """Observe each stance landmark from link COM poses with 90 deg/1 cm sigmas.

    The fixed COM-to-sole transform maps each link pose to its contact frame.
    One world-frame landmark is shared over an uninterrupted stance, including
    phase boundaries, and is replaced after lift-off and recontact.
    """
    contacts = gtsam.NonlinearFactorGraph()
    contact_rotation_sigma_degrees = 90.0
    contact_noise = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([np.deg2rad(contact_rotation_sigma_degrees)] * 3 + [0.01] * 3)
    )
    for k, contact, key in stance_observations(walk):
        contacts.add(
            gtd.ContactPoseFactor(
                gtd.PoseKey(contact.link.id(), k),
                key,
                contact_noise,
                contact.comTcontact,
            )
        )
    return contacts


def prepare_measurements(
    walk: KinematicRecording, directory: Path, initial: gtsam.Values
) -> tuple[gtsam.Values, dict[str, gtsam.NonlinearFactorGraph], dict[str, Any]]:
    """Return the augmented seed, named observation graphs, and IMU provenance.

    Seed each stance landmark from its first initialized sole pose. The shared
    IMU bias starts at zero; its calibrated prior remains a measurement. The
    returned Values is a copy, so the configuration seed is left unchanged.
    """
    initial = seed_contact_landmarks(walk, initial)
    encoders = encoder_factors(walk)
    contacts = contact_pose_factors(walk)
    imu: ImuAiding = build_imu_aiding(walk, directory)
    initial.insert(BIAS_KEY, gtsam.imuBias.ConstantBias())
    return (
        initial,
        {
            "Encoders": encoders,
            "Contact poses": contacts,
            "Gal3 IMU and bias": imu.factors,
        },
        imu.metadata,
    )


def dynamics_model(
    walk: KinematicRecording,
) -> tuple[list[gtd.fixed_dynamics.Contact], NDArray[np.float64]]:
    """Return left/right sole patches (metres, friction 0.8) and joint rotor inertias.

    Patches use the canonical COM-to-sole frames. Armature entries are in kg m²,
    indexed by MuJoCo joint ID, matching the imported GTDynamics joint IDs.
    """
    model = load_g1_model()
    frames: dict[str, gtd.PoseOnLink] = contact_frames(model, walk.robot)
    patches: list[gtd.fixed_dynamics.Contact] = [
        gtd.fixed_dynamics.Contact(frames[side], *support_rectangle(model, side), 0.8)
        for side in ("left", "right")
    ]
    armature = np.zeros(model.njnt)
    for actuator in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator, 0])
        armature[joint_id] = model.dof_armature[model.jnt_dofadr[joint_id]]
    return patches, armature


def solve_coupled_problem(
    walk: KinematicRecording,
    initial: gtsam.Values,
    measurements: dict[str, gtsam.NonlinearFactorGraph],
    patches: list[gtd.fixed_dynamics.Contact],
    armature: NDArray[np.float64],
    options: dict[str, Any],
) -> dict[str, Any]:
    """Run native CMC-Opt on the retained support slices with their actual times.

    Native assembly adds motion and force variables, mechanical constraints,
    and collocation factors. It restores feasibility before joint optimization;
    its result reports assembly, restoration, and optimization times separately.
    Measurement insertion order determines the reported error-group order.
    """
    from _g1_kinodynamics import solve

    return solve(
        walk.robot,
        walk.trajectory,
        [gtd.Slice(k) for k in range(walk.count)],
        walk.times.tolist(),
        initial,
        list(measurements.values()),
        list(measurements),
        patches,
        armature,
        options,
    )
