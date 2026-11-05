# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinodynamic_initialization.py
# @brief Construct FK or contact-pose starting configurations for kinodynamics.
# @author GTDynamics contributors
"""Configuration seeds; native assembly supplies zero motion and support forces."""

from __future__ import annotations

import gtsam
import gtdynamics as gtd
import numpy as np

from mujoco_io import KinematicRecording


def initialize_fk(walk: KinematicRecording) -> gtsam.Values:
    """Run native FK with a constant root COM pose and measured joint angles.

    The caller supplies the upright root anchor, including the fixed body-to-COM
    transform. Each slice has zero root twist and joint rates. No pose fitting,
    temporal interpolation, or inverse kinematics is performed.
    """
    result = gtsam.Values()
    for k in range(walk.count):
        known = gtsam.Values()
        known.insert(gtd.PoseKey(walk.root.id(), k), walk.anchor)
        for joint in walk.robot.joints():
            key = gtd.JointAngleKey(joint.id(), k)
            known.insert(key, walk.measured.atDouble(key))
            known.insert(gtd.JointVelKey(joint.id(), k), 0.0)
        fk: gtsam.Values = walk.robot.forwardKinematics(known, k, walk.root.name())
        result.insert(fk)
    return result


def initialize_contact(
    walk: KinematicRecording, selected_slices: list[int]
) -> tuple[gtsam.Values, int]:
    """Retain the original contact-pose optimizer and its upright/motion/height priors."""
    settings: gtd.pose_contact_init.Options = gtd.pose_contact_init.Options()
    settings.selectedSlices = [gtd.Slice(k) for k in selected_slices]
    settings.interpolateOutput = True
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
    diagnostics: gtd.pose_contact_init.Result = gtd.pose_contact_init.Result()
    initial: gtsam.Values = gtd.Initializer().InitializeSolutionContactPoses(
        walk.robot,
        walk.root.name(),
        walk.anchor,
        walk.measured,
        walk.trajectory,
        walk.dt,
        settings,
        diagnostics,
        0,
    )
    return initial, diagnostics.iterations


def initialize_configurations(
    walk: KinematicRecording, selected_slices: list[int], strategy: str
) -> tuple[gtsam.Values, int]:
    """Return configuration values and optimizer iterations for FK or contact init."""
    if strategy == "fk":
        return initialize_fk(walk), 0
    if strategy == "contact":
        return initialize_contact(walk, selected_slices)
    raise ValueError("initialization must be 'fk' or 'contact'")
