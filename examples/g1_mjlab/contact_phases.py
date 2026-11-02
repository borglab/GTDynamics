# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------
"""Infer contact phases and preserve stance identities on selected slices."""

from __future__ import annotations

from collections.abc import Iterator, Sequence
from dataclasses import replace
from pathlib import Path
from typing import Any

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np

from g1_model import load_g1_model, sole_frame
from mujoco_io import (
    KinematicRecording,
    contact_frames,
    load_recording,
    point_velocity_in_frame,
)


def infer_contact_phases(
    source: KinematicRecording,
    directory: Path,
) -> tuple[KinematicRecording, dict[str, Any]]:
    """Infer privileged L/R/double-stance phases on the original 50 Hz grid.

    World sole displacement uses the LiDAR reference plus measured-joint FK;
    forward body velocity uses measured dq. Only the inferred schedule, joint
    measurements, and initial gauge subsequently enter contact initialization.
    Slice 0 has no preceding displacement: its label uses slice 1's result.
    """
    if not np.isclose(source.dt, 0.02) or source.timestamps is not None:
        raise ValueError("Contact graph parameters require the uniform 50 Hz source")
    arrays, _ = load_recording(directory)
    model = load_g1_model()
    if "qvel" not in arrays or arrays["qvel"].shape != (len(arrays["qpos"]), model.nv):
        raise ValueError(
            "Recording must include measured joint qvel; rerun import_openio.py"
        )
    data = mujoco.MjData(model)
    pelvis = model.body("robot/pelvis").id
    bodies = [
        model.body(f"robot/{side}_ankle_roll_link").id for side in ("left", "right")
    ]
    offsets = [sole_frame(model, side)[:3, 3] for side in ("left", "right")]
    positions = np.empty((source.count, 2, 3))
    velocities = np.empty((source.count, 2))
    for k in range(source.count):
        data.qpos[:] = arrays["qpos"][k]
        data.qvel[:] = arrays["qvel"][k]
        data.qvel[:6] = 0
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        for foot, (body, offset) in enumerate(zip(bodies, offsets)):
            positions[k, foot] = (
                data.xpos[body] + data.xmat[body].reshape(3, 3) @ offset
            )
            velocities[k, foot] = point_velocity_in_frame(
                model, data, body, offset, pelvis
            )[0]
    displacement = np.linalg.norm(np.diff(positions, axis=0), axis=2)
    params = gtd.ContactSupportParams()
    graph: gtsam.DiscreteFactorGraph = gtd.contactSupportGraph(
        displacement[:, 0].tolist(), displacement[:, 1].tolist(), params
    )
    graph.push_back(
        gtd.contactVelocityFactors(
            velocities[1:, 0].tolist(), velocities[1:, 1].tolist()
        )
    )
    graph.push_back(gtd.contactWalkingConstraints(source.count - 1))
    states: gtsam.DiscreteValues = graph.optimize(gtsam.Ordering.OrderingType.NATURAL)
    configurations = [
        2 * states[2 * k] + states[2 * k + 1] for k in range(source.count - 1)
    ]
    configurations.insert(0, configurations[0])
    contacts = contact_frames(model, source.robot)
    phases: list[gtd.Phase] = []
    descriptions = []
    first = 0
    for stop in range(1, source.count + 1):
        if stop < source.count and configurations[stop] == configurations[first]:
            continue
        state = configurations[first]
        active = [side for bit, side in ((2, "left"), (1, "right")) if state & bit]
        spec = gtd.PoseContactConstraintSpec([contacts[side] for side in active])
        phases.append(gtd.Phase(first, stop - 1, spec))
        descriptions.append(dict(start=first, end=stop - 1, contacts=active))
        first = stop
    return replace(source, trajectory=gtd.Trajectory(gtd.WalkCycle(phases), 1)), {
        "method": "binary-foot max-product",
        "source": "SuperOdom sole displacements and measured joint velocities",
        "samplePeriodSeconds": source.dt,
        "sourceSlices": source.count,
        "velocityAiding": True,
        "enforceWalkingSequence": True,
        "singleSliceWeight": params.singleSliceWeight,
        "supportSigma": params.supportSigma,
        "swingSigma": params.swingSigma,
        "firstSliceLabel": "same as slice 1; no preceding displacement available",
        "phases": descriptions,
    }


def initialization_slices(
    trajectory: gtd.Trajectory,
    dt: float,
    max_slice_interval_ms: float = 200.0,
    contact_buffer: int = 1,
    long_phase_seconds: float = 0.2,
) -> list[gtd.Slice]:
    """Retain double-stance slices, filling long buffered phases evenly.

    Reuse native short-phase/endpoint selection. Within each long phase, choose
    the fewest intervals whose lengths are at most max_slice_interval_ms, then
    distribute their endpoints over actual source indices. Adjacent intervals
    differ by at most one source sample. The maximum applies within retained
    double-stance phases; skipped single-stance phases can create larger gaps.
    """
    interval_seconds = max_slice_interval_ms / 1000.0
    if not np.isfinite(interval_seconds) or interval_seconds < dt:
        raise ValueError("max_slice_interval_ms must be finite and at least dt * 1000")
    endpoints = gtd.doubleSupportSlices(
        trajectory, dt, min(long_phase_seconds, interval_seconds), contact_buffer
    )
    indices = np.array([s.k for s in endpoints], dtype=np.int64)
    max_steps = int(np.floor(interval_seconds / dt + 1e-12))
    selected: set[int] = set()
    for phase in trajectory.phases():
        start, stop = np.searchsorted(indices, [phase.k_start, phase.k_end + 1])
        if start == stop:
            continue
        first, last = int(indices[start]), int(indices[stop - 1])
        intervals = max(1, (last - first + max_steps - 1) // max_steps)
        selected.update(
            np.rint(np.linspace(first, last, intervals + 1)).astype(int).tolist()
        )
    return [gtd.Slice(k) for k in sorted(selected)]


def contact_slices(
    trajectory: gtd.Trajectory,
    contact_breaks: frozenset[tuple[int, int]] = frozenset(),
) -> Iterator[tuple[int, list[gtd.PoseOnLink], list[gtd.PoseOnLink]]]:
    """Visit disjoint phase slices, retaining ground contacts across boundaries."""
    previous: Sequence[gtd.PoseOnLink] = []
    previous_k: int | None = None
    phases: Sequence[gtd.Phase] = trajectory.phases()
    for phase in phases:
        spec: gtd.ConstraintSpec = phase.constraintSpec()
        points: list[gtd.PoseOnLink] = [c for c in spec.contactPoses() if c.isGround]
        for k in range(phase.k_start, phase.k_end + 1):
            continuing = [
                p
                for p in points
                if (k, p.link.id()) not in contact_breaks
                and previous_k == k - 1
                and any(p.equals(q, 0.0) for q in previous)
            ]
            yield k, points, continuing
            previous, previous_k = points, k


def stance_observations(
    walk: KinematicRecording,
) -> Iterator[tuple[int, gtd.PoseOnLink, int]]:
    """Yield (slice, sole frame, world Pose3 landmark key) for every stance.

    One landmark survives adjacent phase boundaries while this foot stays down.
    Hidden lift-offs in a sparsely sampled trajectory start new landmarks too.
    Symbol C is reserved for these free contact poses, separate from robot keys.
    """
    active: list[tuple[gtd.PoseOnLink, int]] = []
    stance = 0
    for k, contacts, continuing in contact_slices(walk.trajectory, walk.contact_breaks):
        current = []
        for contact in contacts:
            key = next(
                (
                    key
                    for previous, key in active
                    if contact.equals(previous, 0.0)
                    and any(contact.equals(c, 0.0) for c in continuing)
                ),
                None,
            )
            if key is None:
                key = gtsam.symbol("C", stance)
                stance += 1
            current.append((contact, key))
            yield k, contact, key
        active = current
