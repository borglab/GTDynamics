#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
## @file kinodynamic_preparation.py
# @brief Prepare measured joints, support phases, and the initial G1 root gauge.
# @author GTDynamics contributors
"""Select sparse support slices from the 50 Hz G1 clip for kinodynamics.

LiDAR poses supply the horizontal/yaw gauge and evaluation overlay; the contact
option also uses the initial tilt. FK starts with an upright pelvis body.
Privileged LiDAR motion supplied the saved support labels. Joint indices are
mapped by name through the canonical compiled model.
"""

from __future__ import annotations

import gzip
import json
from pathlib import Path
from typing import Any

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np
from numpy.typing import NDArray

from contact_phases import initialization_slices
from g1_model import load_g1_model
from kinematic_initialization import align_contact_ground
from mujoco_io import (
    KinematicRecording,
    contact_frames,
    import_robot,
    load_recording,
    model_bodies,
    pose,
)

REPO = Path(__file__).resolve().parents[2]


def load_support_clip(
    labels_path: Path, start: float, end: float
) -> tuple[NDArray[np.float64], NDArray[np.int64], NDArray[np.int64]]:
    """Crop labels by dataset seconds, returning times, source indices, and states."""
    labels = json.loads(gzip.decompress(labels_path.read_bytes()))
    all_times = np.asarray(labels["times"])
    selected = (all_times >= start - 1e-8) & (all_times <= end + 1e-8)
    times = all_times[selected]
    if len(times) < 2 or not np.allclose(np.diff(times), 0.02, atol=1e-8):
        raise ValueError("Requested clip requires consecutive 50 Hz label samples")
    indices = np.rint(times * 50).astype(np.int64)
    states = np.asarray(labels["supportGraph"]["states"])[selected]
    return times, indices, states


def measured_joint_angles(
    model: mujoco.MjModel,
    robot: gtd.Robot,
    qpos: NDArray[np.float64],
    indices: NDArray[np.int64],
) -> gtsam.Values:
    """Map source joint angles to slice keys in radians relative to model zero."""
    measured = gtsam.Values()
    for joint in robot.joints():
        address = model.jnt_qposadr[model.joint("robot/" + joint.name()).id]
        for k, source in enumerate(indices):
            measured.insert(
                gtd.JointAngleKey(joint.id(), k),
                float(qpos[source, address] - model.qpos0[address]),
            )
    return measured


def support_trajectory(
    states: NDArray[np.int64], frames: dict[str, gtd.PoseOnLink]
) -> gtd.Trajectory:
    """Group labels (0 left, 1 right, 2 double stance) into contiguous phases."""
    phases: list[gtd.Phase] = []
    first = 0
    for stop in range(1, len(states) + 1):
        if stop < len(states) and states[stop] == states[first]:
            continue
        active = [
            side
            for side, valid in (
                ("left", states[first] in (0, 2)),
                ("right", states[first] in (1, 2)),
            )
            if valid
        ]
        phases.append(
            gtd.Phase(
                first,
                stop - 1,
                gtd.PoseContactConstraintSpec([frames[s] for s in active]),
            )
        )
        first = stop
    return gtd.Trajectory(gtd.WalkCycle(phases), 1)


def retained_initialization_slices(
    trajectory: gtd.Trajectory, initialization: str
) -> list[gtd.Slice]:
    """Select buffered support slices, allowing slice zero as an FK-only fallback."""
    retained: list[gtd.Slice] = initialization_slices(trajectory, 0.02)
    if not retained:
        if initialization != "fk":
            raise ValueError(
                "Contact initialization requires at least one supported phase"
            )
        retained = [gtd.Slice(0)]
    return retained


def initial_root_pose(
    reference: gtsam.Pose3,
    body_to_com: gtsam.Pose3,
    nominal_height: float,
    initialization: str,
) -> gtsam.Pose3:
    """Return the COM-to-world gauge pose, making the pelvis upright for FK."""
    anchor = reference
    if initialization == "fk":
        # Keep the same horizontal/yaw gauge, but start with an upright pelvis
        # body at the model's nominal height. The one-frame ground alignment
        # in prepare_walk places stance soles at z=0 for all FK slices.
        world_body = anchor * body_to_com.inverse()
        position = world_body.translation().copy()
        position[2] = nominal_height
        anchor = gtsam.Pose3(gtsam.Rot3.Yaw(world_body.rotation().yaw()), position)
        anchor = anchor * body_to_com
    return anchor


def prepare_walk(
    directory: Path,
    labels_path: Path,
    start: float,
    end: float,
    initialization: str = "contact-gp-dynamics",
) -> tuple[KinematicRecording, dict[str, Any]]:
    """Retain support knots, original times and stance labels; align the gauge.

    Every supported phase contributes slices, including single support during
    swing. Selection happens before constructing any optimization variables.
    Because every phase is represented, downsampling cannot merge a lift-off
    and recontact into one stance. Solver indices are renumbered consecutively;
    source_indices and timestamps keep their original recording coordinates.
    """
    if initialization not in ("fk", "contact", "contact-gp-dynamics"):
        raise ValueError(
            "initialization must be 'fk', 'contact', or 'contact-gp-dynamics'"
        )
    arrays, metadata = load_recording(directory)
    times, indices, states = load_support_clip(labels_path, start, end)
    model = load_g1_model()
    robot: gtd.Robot = import_robot(model_bodies(model))
    root: gtd.Link = robot.link("pelvis")
    frames: dict[str, gtd.PoseOnLink] = contact_frames(model, robot)
    trajectory = support_trajectory(states, frames)
    retained = retained_initialization_slices(trajectory, initialization)
    selected = np.array([s.k for s in retained], dtype=np.int64)
    if len(selected) < 2:
        raise ValueError("Kinodynamics requires at least two retained support slices")
    source_count = len(times)
    times, indices, states = times[selected], indices[selected], states[selected]
    measured = measured_joint_angles(model, robot, arrays["qpos"], indices)
    trajectory = support_trajectory(states, frames)
    body_to_com: gtsam.Pose3 = pose(
        model.body_ipos[root.id()], model.body_iquat[root.id()]
    )
    references = [pose(q[:3], q[3:7]) * body_to_com for q in arrays["qpos"][indices]]
    anchor = initial_root_pose(
        references[0], body_to_com, float(model.qpos0[2]), initialization
    )
    walk = KinematicRecording(
        robot,
        root,
        measured,
        anchor,
        trajectory,
        len(times),
        0.02,
        0.0,
        np.array([p.translation() for p in references]),
        {},
        times,
        indices,
    )
    walk, shift = align_contact_ground(walk, 0)
    return walk, {
        "source": metadata,
        "groundTranslation": shift.tolist(),
        "initializationSlices": list(range(len(selected))),
        "optimizationSourceSlices": selected.tolist(),
        "optimizationSampling": {
            "method": "contactInitializationSlices",
            "sourceSlices": source_count,
            "sourceSampleRateHz": 50,
            "selectedSlices": len(selected),
            "singleSupportSlices": int(np.count_nonzero(states != 2)),
            "doubleSupportSlices": int(np.count_nonzero(states == 2)),
            "maxSliceIntervalMs": 200,
            "contactBuffer": 1,
            "irregularTimestamps": True,
        },
        "referenceUsage": "Privileged support labels; initial root translation/yaw choose coordinates, tilt only initializes; later poses are evaluation data",
        "labels": (
            str(labels_path.relative_to(REPO))
            if labels_path.is_relative_to(REPO)
            else str(labels_path)
        ),
    }
