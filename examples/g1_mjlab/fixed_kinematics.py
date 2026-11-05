# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file fixed_kinematics.py
# @brief Prepare fixed GP states and local acceleration prior means.
# @author GTDynamics contributors

"""Prepare fixed GP configurations/velocities and soft acceleration observations.

The interpolation is fitted once over the complete saved trajectory with dense
encoders. Dynamics estimates accelerations at the original retained knots.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from interpolate_walk import InterpolatedWalk
from mujoco_io import contact_frames, import_robot, model_bodies


@dataclass
class FixedKinematics:
    """Fixed COM poses/body twists and separate root/joint acceleration means."""

    robot: gtd.Robot
    values: gtsam.Values
    acceleration_priors: gtsam.Values
    contacts: dict[str, gtd.PoseOnLink]
    trajectory: gtd.Trajectory
    slices: list[gtd.Slice]
    times: NDArray[np.float64]
    indices: NDArray[np.int64]
    active: list[list[bool]]
    armature: NDArray[np.float64]
    effort_limits: NDArray[np.float64]
    diagnostics: dict[str, Any]


def acceleration_prior_values(
    interpolation: gtd.FloatingBaseInterpolation,
    robot: gtd.Robot,
    time: float,
    k: int,
    step_seconds: float,
) -> gtsam.Values:
    """Differentiate GP velocities with [-1, 0, 1] / (2h) at a knot.

    Queries use seconds and stay inside the complete fit. At its outer endpoints
    the stencil becomes one-sided and uses the actual elapsed time. Body twist
    components are differentiated in their moving COM axes. Thus the linear
    result is vdot_body; physical acceleration is vdot_body + omega cross v.
    Joint means follow interpolation.jointIds(), in rad/s² for revolute joints.
    """
    return gtd.gpAccelerationPriors(
        interpolation, robot.link("pelvis").id(), time, k, step_seconds
    )


def prepare_fixed_kinematics(
    walk: InterpolatedWalk,
    start: float,
    end: float,
    acceleration_step_seconds: float = 0.02,
) -> FixedKinematics:
    """Select original GP knots and preserve the source contact phase sequence.

    Only poses, joint angles, body twists, and joint rates enter fixed Values.
    Acceleration observations are separate soft means for the native CMC solve.
    Source slice indices preserve lift-off/recontact even across unsampled gaps.
    """
    if not np.isfinite(start) or not np.isfinite(end) or start >= end:
        raise ValueError("The development window must be finite and increasing")
    indices = np.flatnonzero((walk.knot_times >= start) & (walk.knot_times <= end))
    if len(indices) < 2:
        raise ValueError("The requested clip contains fewer than two GP knots")
    model, record = walk.model, walk.record
    robot = import_robot(model_bodies(model))
    contacts = contact_frames(model, robot)
    source_indices = np.asarray(record["sampling"]["sourceIndices"], dtype=int)
    source_phases = record["contactPhaseInference"]["phases"]
    phases = [
        gtd.Phase(
            phase["start"],
            phase["end"],
            gtd.PoseContactConstraintSpec(
                [contacts[side] for side in phase["contacts"]]
            ),
        )
        for phase in source_phases
    ]
    values, priors = gtsam.Values(), gtsam.Values()
    slices, active = [], []
    for source in indices:
        k, time = int(source_indices[source]), float(walk.knot_times[source])
        slices.append(gtd.Slice(k))
        values.insert(walk.interpolation.values(time, k))
        priors.insert(
            acceleration_prior_values(
                walk.interpolation,
                robot,
                time,
                k,
                acceleration_step_seconds,
            )
        )
        phase = next(p for p in source_phases if p["start"] <= k <= p["end"])
        active.append([side in phase["contacts"] for side in ("left", "right")])
    armature, effort = np.zeros(model.njnt), np.zeros(model.njnt)
    for actuator in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator, 0])
        armature[joint_id] = model.dof_armature[model.jnt_dofadr[joint_id]]
        effort[joint_id] = np.max(np.abs(model.actuator_forcerange[actuator]))
    return FixedKinematics(
        robot,
        values,
        priors,
        contacts,
        gtd.Trajectory(gtd.WalkCycle(phases), 1),
        slices,
        walk.knot_times[indices],
        indices,
        active,
        armature,
        effort,
        {
            "derivativeContextSamples": len(walk.knot_times),
            "denseEncoderSamples": len(walk.dense_times),
            "accelerationDifferenceStepSeconds": acceleration_step_seconds,
            "accelerationDifferenceMask": [-1, 0, 1],
            "accelerationDifferenceNormalization": "actual stencil elapsed seconds",
            "gpGraphConstructionSeconds": walk.interpolation.graphConstructionSeconds(),
            "gpSolveSeconds": walk.interpolation.solveSeconds(),
            "gpJointMeasurementRmsRad": walk.interpolation.jointMeasurementRms(),
            "gpProcessError": walk.interpolation.processError(),
        },
    )
