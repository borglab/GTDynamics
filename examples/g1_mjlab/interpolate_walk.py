"""Fit floating-base GPs to a saved kinematics solution and dense encoders.

The reusable estimator is C++; this module only maps the canonical G1 model,
recording joint names, and MuJoCo body frames to its public API.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np
from numpy.typing import NDArray

from g1_model import load_g1_model
from mujoco_io import contact_frames, import_robot, load_recording, model_bodies, pose


@dataclass
class InterpolatedWalk:
    """An offline GP fit with its exact source and named model mappings."""

    gp: gtd.FloatingBaseInterpolator
    model: mujoco.MjModel
    record: dict[str, Any]
    variant: str
    knot_times: NDArray[np.float64]
    dense_times: NDArray[np.float64]
    dense_angles: NDArray[np.float64]
    dense_indices: NDArray[np.int64]
    source_angles: NDArray[np.float64]
    contacts: dict[str, gtd.PoseOnLink]
    input_checksums: dict[str, str]


def checksum(path: Path) -> str:
    """SHA256 of a source artifact, before decoding."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fit_walk(
    kinematics_public: Path,
    dense_data: Path,
    joint_process_sigma_degrees: float = 100.0,
    joint_measurement_sigma_degrees: float = 0.5,
    base_angular_process_sigma_degrees: float = 90.0,
    base_linear_process_sigma: float = 3.0,
    base_velocity_sigma: float = 0.5,
) -> InterpolatedWalk:
    """Fit the comparison winner, retaining every source configuration exactly.

    Process standard deviations have units deg/s^(3/2) or m/s^(3/2).
    Encoder sigma is degrees; the base COM world-velocity sigma is m/s.
    Dense source root poses are never used as estimation measurements.
    """
    scales = (
        joint_process_sigma_degrees,
        joint_measurement_sigma_degrees,
        base_angular_process_sigma_degrees,
        base_linear_process_sigma,
        base_velocity_sigma,
    )
    if not all(np.isfinite(scale) and scale > 0 for scale in scales):
        raise ValueError("GP process and measurement scales must be positive")
    pack_directory = kinematics_public.parent / "scripts"
    sys.path.insert(0, str(pack_directory))
    from pack_recordings import quantized_chunks, read_metadata

    comparison = json.loads((kinematics_public / "comparison.json").read_text())
    variant = comparison["bestVariant"]
    directory = kinematics_public / "variants" / variant
    record = read_metadata(directory)
    binary = directory / "final.bin.gz"
    chunks = quantized_chunks(binary.read_bytes(), record["storage"])
    state = record["runs"]["contact"]["final"]
    count = record["samples"]
    body_poses = chunks[state["poseOffset"]].reshape(count, record["bodies"], 7) / 1e6
    saved_angles = chunks[state["jointOffset"]].reshape(count, -1) / 1e6
    world_velocities = chunks[state["velocityOffset"]].reshape(count, 3) / 1e6
    knots = np.asarray(record["timestamps"], dtype=float)
    arrays, metadata = load_recording(dense_data)
    for name in ("walk.npz", "walk.json"):
        expected = record["provenance"]["inputs"][name]
        if checksum(dense_data / name) != expected:
            raise ValueError(f"Dense source differs from the kinematics input: {name}")
    model = load_g1_model()
    names = [model.body(i).name for i in range(model.nbody)]
    if record["bodyNames"] != names:
        raise ValueError("Kinematics body names differ from the canonical model")
    robot = import_robot(model_bodies(model))
    joints = robot.orderedJoints()
    joint_names = [joint.name() for joint in joints]
    columns = [record["jointNames"].index(name) for name in joint_names]
    addresses = [model.joint(f"robot/{name}").qposadr[0] for name in joint_names]
    all_times = np.arange(len(arrays["qpos"])) * metadata["policy_dt"]
    source_indices = np.asarray(record["sampling"]["sourceIndices"], dtype=int)
    if not np.allclose(knots, all_times[source_indices], atol=1e-9, rtol=0):
        raise ValueError("Kinematics timestamps do not match the dense source")
    dense_indices = np.flatnonzero((all_times >= knots[0]) & (all_times <= knots[-1]))
    dense_angles = arrays["qpos"][dense_indices][:, addresses]
    gp = gtd.FloatingBaseInterpolator(
        robot,
        "pelvis",
        np.r_[
            np.full(3, np.deg2rad(base_angular_process_sigma_degrees) ** 2),
            np.full(3, base_linear_process_sigma**2),
        ],
        np.full(len(joints), np.deg2rad(joint_process_sigma_degrees) ** 2),
        np.full(len(joints), np.deg2rad(joint_measurement_sigma_degrees)),
    )
    root = model.body("robot/pelvis").id
    body_to_com = pose(model.body_ipos[root], model.body_iquat[root])
    for index, time in enumerate(knots):
        row = body_poses[index, root]
        quaternion = row[3:] / np.linalg.norm(row[3:])
        world_body = gtsam.Pose3(
            gtsam.Rot3.Quaternion(*quaternion[[3, 0, 1, 2]]), row[:3]
        )
        gp.addKnot(float(time), world_body * body_to_com, saved_angles[index, columns])
        gp.addBaseVelocityMeasurement(
            float(time), world_velocities[index], base_velocity_sigma
        )
    for time, angles in zip(all_times[dense_indices], dense_angles):
        gp.addJointMeasurement(float(time), angles)
    gp.solve()
    return InterpolatedWalk(
        gp,
        model,
        record,
        variant,
        knots,
        all_times[dense_indices],
        dense_angles,
        dense_indices,
        saved_angles[:, columns],
        contact_frames(model, robot),
        {
            "kinematicsFinalMetadata": checksum(directory / "final.json.gz"),
            "kinematicsFinalStates": checksum(binary),
            "comparison": checksum(kinematics_public / "comparison.json"),
            "denseJointSource": checksum(dense_data / "walk.npz"),
        },
    )


def sample_walk(walk: InterpolatedWalk) -> dict[str, NDArray[np.float64]]:
    """Evaluate dense FK and sole world twists; browser output is body xyz/xyzw.

    Joint angles/rates follow gp.jointNames(). No accelerations are estimated.
    Sole twists are angular-first and rotated into WORLD axes at the sole origin.
    """
    model = walk.model
    com_to_body = {
        body: pose(model.body_ipos[body], model.body_iquat[body]).inverse()
        for body in range(1, model.nbody)
        if model.body(body).name.startswith("robot/")
    }
    samples: dict[str, list[Any]] = {
        name: []
        for name in (
            "bodyPoses",
            "jointAngles",
            "jointVelocities",
            "baseTwists",
            "footTwists",
            "contactPoses",
        )
    }
    for time in walk.dense_times:
        state: gtd.FloatingBaseState = walk.gp.interpolate(float(time))
        values: gtsam.Values = walk.gp.values(float(time))
        bodies = np.zeros((model.nbody, 7))
        bodies[:, 6] = 1
        for body, transform in com_to_body.items():
            world_body: gtsam.Pose3 = values.atPose3(gtd.PoseKey(body, 0)) * transform
            bodies[body] = np.r_[
                world_body.translation(), world_body.rotation().toQuaternion().coeffs()
            ]
        foot_twists, contact_poses = [], []
        for contact in walk.contacts.values():
            world_contact: gtsam.Pose3 = contact.predict(values, 0)
            local_twist = contact.comTcontact.inverse().AdjointMap() @ gtd.Twist(
                values, contact.link.id(), 0
            )
            rotation = world_contact.rotation().matrix()
            foot_twists.append(
                np.r_[rotation @ local_twist[:3], rotation @ local_twist[3:]]
            )
            contact_poses.append(
                np.r_[
                    world_contact.translation(),
                    world_contact.rotation().toQuaternion().coeffs(),
                ]
            )
        for name, value in zip(
            samples,
            (
                bodies,
                state.jointAngles,
                state.jointVelocities,
                state.baseTwist,
                foot_twists,
                contact_poses,
            ),
        ):
            samples[name].append(value)
    return {name: np.asarray(rows) for name, rows in samples.items()}


def validate_model_frames(
    walk: InterpolatedWalk, samples: dict[str, NDArray[np.float64]]
) -> dict[str, float]:
    """Check native body poses and sole world twists against compiled MuJoCo FK.

    Twelve dense samples include both endpoints. Free-joint angular velocity
    uses pelvis body axes, whereas the estimator twist uses inertial COM axes.
    The world linear velocity also changes origin from COM to body origin.
    This validates frames and FK, not contact stationarity or physical dynamics.
    """
    model, data = walk.model, mujoco.MjData(walk.model)
    root = model.body("robot/pelvis").id
    root_joint = int(model.body_jntadr[root])
    root_qpos, root_qvel = model.jnt_qposadr[root_joint], model.jnt_dofadr[root_joint]
    joint_qpos = [
        model.joint(f"robot/{name}").qposadr[0] for name in walk.gp.jointNames()
    ]
    joint_qvel = [
        model.joint(f"robot/{name}").dofadr[0] for name in walk.gp.jointNames()
    ]
    body_to_com = pose(model.body_ipos[root], model.body_iquat[root])
    pose_error, twist_error = 0.0, 0.0
    for frame in np.unique(np.linspace(0, len(walk.dense_times) - 1, 12).astype(int)):
        row = samples["bodyPoses"][frame, root]
        world_body = gtsam.Pose3(gtsam.Rot3.Quaternion(*row[[6, 3, 4, 5]]), row[:3])
        world_com = world_body * body_to_com
        twist = samples["baseTwists"][frame]
        angular_world = world_com.rotation().rotate(twist[:3])
        linear_world = world_com.rotation().rotate(twist[3:]) - np.cross(
            angular_world, world_com.translation() - world_body.translation()
        )
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0
        data.qpos[root_qpos : root_qpos + 7] = np.r_[row[:3], row[[6, 3, 4, 5]]]
        data.qpos[joint_qpos] = samples["jointAngles"][frame]
        data.qvel[root_qvel : root_qvel + 6] = np.r_[
            linear_world, world_body.rotation().unrotate(angular_world)
        ]
        data.qvel[joint_qvel] = samples["jointVelocities"][frame]
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        for body in range(1, model.nbody):
            if not model.body(body).name.startswith("robot/"):
                continue
            expected = samples["bodyPoses"][frame, body]
            rotation = gtsam.Rot3.Quaternion(*expected[[6, 3, 4, 5]]).matrix()
            pose_error = max(
                pose_error,
                float(np.max(np.abs(data.xpos[body] - expected[:3]))),
                float(np.max(np.abs(data.xmat[body].reshape(3, 3) - rotation))),
            )
        for foot, side in enumerate(("left", "right")):
            body = model.body(f"robot/{side}_ankle_roll_link").id
            point = samples["contactPoses"][frame, foot, :3].copy()
            translation, angular = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, translation, angular, point, body)
            expected_twist = np.r_[angular @ data.qvel, translation @ data.qvel]
            twist_error = max(
                twist_error,
                float(
                    np.max(np.abs(expected_twist - samples["footTwists"][frame, foot]))
                ),
            )
    if pose_error > 1e-8 or twist_error > 1e-8:
        raise ValueError(
            f"Native/MuJoCo frame mismatch: pose {pose_error:g}, twist {twist_error:g}"
        )
    return {"maxBodyPoseError": pose_error, "maxSoleTwistError": twist_error}
