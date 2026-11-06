# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinematic_initialization.py
# @brief Initialize G1 configurations using the shared contact sampling policy.
# @author GTDynamics contributors
"""Initialize G1 kinematics and retain the native contact-selected slices."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import replace

import gtsam
import gtdynamics as gtd
import numpy as np
from numpy.typing import NDArray

from contact_phases import contact_slices, initialization_slices, stance_observations
from mujoco_io import KinematicRecording


def align_contact_ground(
    walk: KinematicRecording, anchor_slice: int = 0
) -> tuple[KinematicRecording, NDArray[np.float64]]:
    """Translate the world so active soles at the fixed root gauge average z=0.

    walk.anchor must refer to anchor_slice. Only that supplied gauge and measured
    joint FK determine the shift; later reference positions are translated for
    evaluation, never used to estimate the ground. The same shift must be applied
    to the caller's reference replay. Joint values, rotations and IMU samples do
    not change. The example's smoothing height objectives then share this frame.
    """
    contacts = {
        c.link.id(): c
        for phase in walk.trajectory.phases()
        if phase.k_start <= anchor_slice <= phase.k_end
        for c in phase.constraintSpec().contactPoses()
        if c.isGround
    }
    if not contacts:
        raise ValueError("Ground alignment requires contact at the fixed root slice")
    initial = gtsam.Values()
    initial.insert(gtd.PoseKey(walk.root.id(), anchor_slice), walk.anchor)
    for joint in walk.robot.joints():
        key = gtd.JointAngleKey(joint.id(), anchor_slice)
        initial.insert(key, walk.measured.atDouble(key))
        initial.insert(gtd.JointVelKey(joint.id(), anchor_slice), 0.0)
    fk: gtsam.Values = walk.robot.forwardKinematics(
        initial, anchor_slice, walk.root.name()
    )
    height = float(
        np.mean([c.predict(fk, anchor_slice).z() for c in contacts.values()])
    )
    translation = np.array([0.0, 0.0, -height])
    return (
        replace(
            walk,
            anchor=gtsam.Pose3(
                walk.anchor.rotation(), walk.anchor.translation() + translation
            ),
            reference_root_positions=walk.reference_root_positions + translation,
        ),
        translation,
    )


def contact_initialization_info(
    options: gtd.pose_contact_init.Options, diagnostics: gtd.pose_contact_init.Result
) -> dict[str, int | float | bool]:
    """Native stopping status and the actual prior settings used by the solve."""
    info: dict[str, int | float | bool] = {
        "sparse_frames": len(diagnostics.poses),
        "iterations": diagnostics.iterations,
        "contact_rms_m": diagnostics.residualRms,
        "rotation_rms_degrees": float(np.rad2deg(diagnostics.rotationRms)),
        "rotation_sigma_degrees": float(np.rad2deg(options.rotationSigma)),
        "translation_sigma_m": options.translationSigma,
        "initialize_upright": options.initializeUpright,
        "upright_prior": options.addUprightFactors,
        "height_prior": options.addGroundFactors,
        "normal_prior": options.addGroundFactors and options.addNormalFactors,
        "motion_prior": options.addBetweenFactors,
        "upright_sigma_degrees": float(np.rad2deg(options.uprightSigma)),
        "height_sigma_m": options.groundSigma,
        "motion_rotation_sigma_degrees": float(
            np.rad2deg(options.betweenRotationSigma)
        ),
        "motion_translation_sigma_m": options.betweenTranslationSigma,
        "converged": diagnostics.converged,
        "stalled": diagnostics.stalled,
    }
    if options.groundHeight is not None:
        info["ground_height_m"] = options.groundHeight
    return info


def initialize_contact_slices(
    source: KinematicRecording,
    options: gtd.pose_contact_init.Options | None = None,
    contact_buffer: int = 1,
    long_phase_seconds: float = 0.2,
    rotation_sigma_degrees: float | None = None,
    max_slice_interval_ms: float = 200.0,
    contact_diagnostics: gtd.pose_contact_init.Result | None = None,
) -> tuple[KinematicRecording, gtsam.Values, dict[str, int | float | bool]]:
    """Initialize on buffered contacts and retain exactly those slices for fitting.

    Use the initialization app's shared native sampler in every supported phase:
    a central slice for short phases and evenly spaced buffered slices for long
    phases. The interval bound is max_slice_interval_ms within each phase.
    No interpolated Values are created.
    rotation_sigma_degrees overrides the contact observation's per-axis rotation
    standard deviation; conversion to the native API's radians happens here.

    The native result supplies the source indices; no second sampling schedule
    is constructed. Solver keys are renumbered 0..N-1, while times and source
    indices retain their original meaning. Source contact specifications are
    preserved for smoothing (the initializer filters its own observations).
    The first retained slice uses the supplied initial gauge, as documented by
    InitializeSolutionContactPoses; no later reference pose enters initialization.
    """
    if source.timestamps is not None or source.source_indices is not None:
        raise ValueError("Contact selection requires the original uniform recording")
    settings = options if options is not None else gtd.pose_contact_init.Options()
    if rotation_sigma_degrees is not None:
        if not np.isfinite(rotation_sigma_degrees) or rotation_sigma_degrees <= 0:
            raise ValueError("rotation_sigma_degrees must be finite and positive")
        settings.rotationSigma = float(np.deg2rad(rotation_sigma_degrees))
    settings.interpolateOutput = False
    selected_slices = initialization_slices(
        source.trajectory,
        source.dt,
        max_slice_interval_ms,
        contact_buffer,
        long_phase_seconds,
    )
    if not selected_slices:
        raise ValueError("No supported phases available for initialization")
    diagnostics = (
        contact_diagnostics
        if contact_diagnostics is not None
        else gtd.pose_contact_init.Result()
    )
    values: gtsam.Values = gtd.PoseContactInitializer(
        robot=source.robot, options=settings, contactBuffer=0
    ).run(
        rootLinkName=source.root.name(),
        initialPose=source.anchor,
        measuredJoints=source.measured,
        trajectory=source.trajectory,
        dt=source.dt,
        diagnostics=diagnostics,
        selectedSlices=selected_slices,
    )
    indices = np.array([s.k for s in diagnostics.slices()], dtype=np.int64)
    if len(indices) < 2:
        raise ValueError("Estimation requires at least two retained contact slices")
    measured = gtsam.Values()
    seed = gtsam.Values()
    for k, original in enumerate(indices):
        for joint in source.robot.joints():
            key = gtd.JointAngleKey(joint.id(), k)
            angle = source.measured.atDouble(
                gtd.JointAngleKey(joint.id(), int(original))
            )
            measured.insert(key, angle)
            seed.insert(key, angle)
        for link in source.robot.links():
            key = gtd.PoseKey(link.id(), int(original))
            if values.exists(key):
                seed.insert(gtd.PoseKey(link.id(), k), values.atPose3(key))
    phases: list[gtd.Phase] = []
    for phase in source.trajectory.phases():
        first, stop = np.searchsorted(indices, [phase.k_start, phase.k_end + 1])
        if first < stop:
            phases.append(gtd.Phase(int(first), int(stop - 1), phase.constraintSpec()))
    # Record discontinuities hidden by sparse sampling. A foot may lift and
    # touch down between selected slices; do not constrain those as one stance.
    source_contacts = list(contact_slices(source.trajectory))
    breaks: set[tuple[int, int]] = set()
    for k, (left, right) in enumerate(zip(indices, indices[1:]), start=1):
        for point in source_contacts[right][1]:
            if any(point.equals(p, 0.0) for p in source_contacts[left][1]):
                if any(
                    not any(point.equals(p, 0.0) for p in source_contacts[j][1])
                    for j in range(left + 1, right)
                ):
                    breaks.add((k, point.link.id()))
    walk = replace(
        source,
        trajectory=gtd.Trajectory(gtd.WalkCycle(phases), 1),
        measured=measured,
        count=len(indices),
        timestamps=source.times[indices],
        source_indices=indices,
        contact_breaks=frozenset(breaks),
        reference_root_positions=source.reference_root_positions[indices],
    )
    return (
        walk,
        seed,
        {
            **contact_initialization_info(settings, diagnostics),
            "source_frames": source.count,
            "contact_buffer": contact_buffer,
            "supported_phases": True,
            "long_phase_seconds": min(
                long_phase_seconds, max_slice_interval_ms / 1000.0
            ),
            "max_slice_interval_ms": max_slice_interval_ms,
        },
    )


def seed_contact_landmarks(
    walk: KinematicRecording, values: gtsam.Values
) -> gtsam.Values:
    """Add each stance's world sole pose from its first estimated observation.

    This provides starting values only; landmarks remain free Pose3 variables
    in CM-Opt. Both IK and contact starts use this rule, without reference poses.
    Existing landmark values are preserved.
    """
    result = gtsam.Values(values)
    for k, contact, key in stance_observations(walk):
        if not result.exists(key):
            result.insert(key, contact.predict(values, k))
    return result


def nominal_root(walk: KinematicRecording, k: int) -> gtsam.Pose3:
    """A commanded straight-line path for IK, not the recorded root trajectory."""
    matrix: NDArray[np.float64] = walk.anchor.matrix()
    matrix[0, 3] += walk.speed * (walk.times[k] - walk.times[0])
    return gtsam.Pose3(matrix)


def initialize(
    walk: KinematicRecording,
    method: str,
    keys: Sequence[int],
    contact_diagnostics: gtd.pose_contact_init.Result | None = None,
    ik_iteration: Callable[[int, int, float, gtsam.Values], None] | None = None,
    contact_options: gtd.pose_contact_init.Options | None = None,
) -> tuple[gtsam.Values, dict[str, int | float | bool]]:
    """Call the native initializers; optional observers support offline inspection.

    Contact diagnostics retain the sparse solve history when supplied. The IK
    observer receives each slice's initial state and completed LM iterations,
    with global trajectory keys. Ordinary timing runs leave observers disabled.
    """
    if walk.timestamps is not None and method == "contact":
        raise ValueError(
            "Use the seed returned by initialize_contact_slices for irregular slices"
        )

    if method == "contact":
        options = (
            contact_options
            if contact_options is not None
            else gtd.pose_contact_init.Options()
        )
        options.recordHistory = contact_diagnostics is not None
        diagnostics = (
            contact_diagnostics
            if contact_diagnostics is not None
            else gtd.pose_contact_init.Result()
        )
        values: gtsam.Values = gtd.PoseContactInitializer(
            robot=walk.robot, options=options
        ).run(
            rootLinkName=walk.root.name(),
            initialPose=walk.anchor,
            measuredJoints=walk.measured,
            trajectory=walk.trajectory,
            dt=walk.dt,
            diagnostics=diagnostics,
        )
        info = contact_initialization_info(options, diagnostics)
        return seed_contact_landmarks(walk, values).extract(keys), info
    if method != "ik":
        raise ValueError(f"Unknown initialization: {method}")
    root_guesses: list[gtsam.Pose3] = [nominal_root(walk, k) for k in range(walk.count)]
    values: gtsam.Values = gtd.PoseIKInitializer(
        robot=walk.robot, maxIterations=10
    ).run(
        rootLinkName=walk.root.name(),
        rootGuesses=root_guesses,
        measuredJoints=walk.measured,
        trajectory=walk.trajectory,
        iterationCallback=ik_iteration,
    )
    return seed_contact_landmarks(walk, values).extract(keys), {
        "slice_calls": walk.count
    }
