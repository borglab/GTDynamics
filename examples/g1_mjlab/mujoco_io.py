# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file mujoco_io.py
# @brief Shared MuJoCo recording loader and static robot conversion.
# @author Frank Dellaert

"""Shared MuJoCo recording and static-model import for the G1 examples.

GTSAM/GTDynamics imports are local to the kinematic conversion functions, so
forward-dynamics export also works in the original MJLab recording environment.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray

if TYPE_CHECKING:
    import gtsam
    import gtdynamics as gtd


def load_model(directory: Path, cone: str | None = None) -> mujoco.MjModel:
    """Load the recorded binary with the common high-accuracy oracle settings."""
    model = mujoco.MjModel.from_binary_path(str(directory / "model.mjb"))
    model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    model.opt.iterations = 100
    model.opt.tolerance = 1e-12
    if cone is not None:
        model.opt.cone = getattr(mujoco.mjtCone, "mjCONE_" + cone.upper())
    return model


def load_recording(
    directory: Path,
) -> tuple[dict[str, NDArray[Any]], dict[str, Any]]:
    """Read arrays and recording metadata, closing the NPZ archive immediately."""
    with np.load(directory / "walk.npz") as archive:
        recording = {name: archive[name] for name in archive.files}
    return recording, json.loads((directory / "walk.json").read_text())


def model_bodies(model: mujoco.MjModel) -> list[dict[str, Any]]:
    """Export static body/joint geometry shared by both GTDynamics import paths."""
    bodies = []
    for body in range(1, model.nbody):
        if model.body_mass[body] == 0:
            continue
        if model.body_jntnum[body] != 1:
            raise ValueError("This G1 importer requires one joint per moving body")
        joint = int(model.body_jntadr[body])
        kind = int(model.jnt_type[joint])
        if kind not in (
            int(mujoco.mjtJoint.mjJNT_FREE),
            int(mujoco.mjtJoint.mjJNT_HINGE),
        ):
            raise ValueError("Only the G1 floating root and hinge joints are supported")
        bodies.append(
            {
                "id": body,
                "name": model.body(body).name,
                "parent": int(model.body_parentid[body]),
                "position": model.body_pos[body].tolist(),
                "quaternion": model.body_quat[body].tolist(),
                "com_position": model.body_ipos[body].tolist(),
                "com_quaternion": model.body_iquat[body].tolist(),
                "mass": float(model.body_mass[body]),
                "inertia": model.body_inertia[body].tolist(),
                "joint": {
                    "id": joint,
                    "name": model.joint(joint).name,
                    "free": kind == int(mujoco.mjtJoint.mjJNT_FREE),
                    "position": model.jnt_pos[joint].tolist(),
                    "axis": model.jnt_axis[joint].tolist(),
                    "qpos_address": int(model.jnt_qposadr[joint]),
                    "dof_address": int(model.jnt_dofadr[joint]),
                    "reference": float(model.qpos0[model.jnt_qposadr[joint]]),
                },
            }
        )
    return bodies


def pose(position: ArrayLike, quaternion: ArrayLike) -> gtsam.Pose3:
    """MuJoCo wxyz quaternion and position as a GTSAM pose (float64 normalized)."""
    import gtsam

    q = np.asarray(quaternion, dtype=float)
    q = q / np.linalg.norm(q)
    return gtsam.Pose3(gtsam.Rot3.Quaternion(*q), np.asarray(position, dtype=float))


def import_robot(bodies: Sequence[dict[str, Any]]) -> gtd.Robot:
    """Build a COM-frame robot using the existing wrapped Link/Joint constructors."""
    import gtsam
    import gtdynamics as gtd

    rest, links, link_map, joint_map = {0: gtsam.Pose3()}, {}, {}, {}
    for body in bodies:
        identifier, parent = body["id"], body["parent"]
        rest[identifier] = rest[parent] * pose(body["position"], body["quaternion"])
        link = gtd.Link(
            identifier,
            body["name"].split("/")[-1],
            body["mass"],
            np.diag(body["inertia"]),
            rest[identifier] * pose(body["com_position"], body["com_quaternion"]),
            rest[identifier],
        )
        links[identifier] = link
        link_map[link.name()] = link
        j = body["joint"]
        if not j["free"]:
            joint = gtd.RevoluteJoint(
                j["id"],
                j["name"].split("/")[-1],
                rest[identifier] * gtsam.Pose3(gtsam.Rot3(), np.asarray(j["position"])),
                links[parent],
                link,
                np.asarray(j["axis"]),
            )
            links[parent].addJoint(joint)
            link.addJoint(joint)
            joint_map[joint.name()] = joint
    return gtd.Robot(link_map, joint_map)


def contact_phases(
    labels: Sequence[dict[str, Any]], count: int
) -> list[tuple[int, int, tuple[str, ...]]]:
    """Convert recorded stance labels to disjoint phases of active feet.

    Labels are an input-file format only. Their touchdown IDs are unnecessary:
    contact continuity follows from consecutive phase specifications.
    """
    active = [
        tuple(
            sorted(
                {
                    label["foot"]
                    for label in labels
                    if label["start"] <= k <= label["end"]
                }
            )
        )
        for k in range(count)
    ]
    boundaries = (
        [0] + [k for k in range(1, count) if active[k] != active[k - 1]] + [count]
    )
    return [(a, b - 1, active[a]) for a, b in zip(boundaries, boundaries[1:])]


@dataclass
class KinematicRecording:
    robot: gtd.Robot
    root: gtd.Link
    measured: gtsam.Values
    anchor: gtsam.Pose3
    trajectory: gtd.Trajectory
    count: int
    dt: float
    speed: float
    reference_root_positions: NDArray[np.float64]
    provenance: dict[str, str]


def load_kinematic_recording(
    directory: Path, contacts_path: Path, max_frames: int | None = None
) -> KinematicRecording:
    """Load geometry, encoders, and labeled stance; reserve root poses for evaluation.

    Only the first root pose is an input to estimation. Later recorded root poses
    are used by the import oracle and returned separately for residual reporting.
    """
    import gtsam
    import gtdynamics as gtd

    model = load_model(directory)
    recording, metadata = load_recording(directory)
    labels = json.loads(contacts_path.read_text())
    qpos = recording["qpos"]
    if len(qpos) != len(labels["samples"]):
        raise ValueError("Contact labels and recording must have the same sample count")
    joint_ids = {model.joint(j).name.split("/")[-1]: j for j in range(model.njnt)}
    body_ids = {model.body(b).name.split("/")[-1]: b for b in range(model.nbody)}
    addresses = [model.jnt_qposadr[joint_ids[n]] for n in labels["model"]["jointNames"]]
    if not np.allclose(
        qpos[:, addresses], [s["jointAngles"] for s in labels["samples"]], atol=2e-6
    ):
        raise ValueError("Contact labels belong to a different joint recording")
    count = len(qpos) if max_frames is None else min(max_frames, len(qpos))
    if count < 2:
        raise ValueError("Use at least two frames")
    robot = import_robot(model_bodies(model))
    root = robot.link("pelvis")
    measured = gtsam.Values()
    for joint in robot.joints():
        address = model.jnt_qposadr[joint_ids[joint.name()]]
        for k in range(count):
            measured.insert(
                gtd.JointAngleKey(joint.id(), k),
                float(qpos[k, address] - model.qpos0[address]),
            )
    root_id = body_ids["pelvis"]
    body_to_com = pose(model.body_ipos[root_id], model.body_iquat[root_id])
    reference = [pose(q[:3], q[3:7]) * body_to_com for q in qpos[:count]]
    points = {}
    for foot in ("left", "right"):
        link = robot.link(f"{foot}_ankle_roll_link")
        body = body_ids[link.name()]
        com = pose(model.body_ipos[body], model.body_iquat[body])
        points[foot] = gtd.PointOnLinks(
            [
                gtd.PointOnLink(link, com.transformTo(np.asarray(p)))
                for p in labels["model"]["soleCorners"]
            ]
        )
    # Convert the recording's labels once. Phases are the only schedule kept
    # by the example; touchdown IDs are not inputs to the initializer.
    phases = []
    for first, last, active in contact_phases(labels["contacts"], count):
        support = gtd.PointOnLinks([p for foot in active for p in points[foot]])
        spec = gtd.FootContactConstraintSpec(support)
        phases.append(gtd.Phase(first, last, spec))
    trajectory = gtd.Trajectory(gtd.WalkCycle(phases), 1)
    provenance = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [
            directory / "model.mjb",
            directory / "walk.npz",
            directory / "walk.json",
            contacts_path,
        ]
    }
    validate_model_import(model, qpos[:count], robot, root, measured, reference)
    return KinematicRecording(
        robot,
        root,
        measured,
        reference[0],
        trajectory,
        count,
        metadata["policy_dt"],
        metadata["command_speed_m_s"],
        np.array([p.translation() for p in reference]),
        provenance,
    )


def validate_model_import(
    model: mujoco.MjModel,
    qpos: NDArray[np.floating[Any]],
    robot: gtd.Robot,
    root: gtd.Link,
    measured: gtsam.Values,
    reference: Sequence[gtsam.Pose3],
) -> float:
    """Check every link COM against MuJoCo at three samples before timing."""
    import gtdynamics as gtd

    data = mujoco.MjData(model)
    maximum = 0.0
    for k in sorted({0, len(qpos) // 2, len(qpos) - 1}):
        data.qpos[:] = qpos[k]
        # GPU recordings contain float32 quaternions; normalize in float64.
        data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
        mujoco.mj_forward(model, data)
        known = measured.extract([gtd.JointAngleKey(j.id(), k) for j in robot.joints()])
        known.insert(gtd.PoseKey(root.id(), k), reference[k])
        known.insert(gtd.TwistKey(root.id(), k), np.zeros(6))
        fk = robot.forwardKinematics(known, k, root.name())
        for link in robot.links():
            expected = np.eye(4)
            expected[:3, :3] = data.ximat[link.id()].reshape(3, 3)
            expected[:3, 3] = data.xipos[link.id()]
            maximum = max(
                maximum,
                float(
                    np.max(
                        np.abs(
                            fk.atPose3(gtd.PoseKey(link.id(), k)).matrix() - expected
                        )
                    )
                ),
            )
    if maximum > 1e-8:
        raise ValueError(f"Imported COM-frame FK disagrees with MuJoCo: {maximum:g}")
    return maximum
