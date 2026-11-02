#!/usr/bin/env python3
"""Check the published acceleration/wrench solution against MuJoCo dynamics.

Independent resultant-wrench and four-corner LPs check contact-cone feasibility.
The native CMC solve supplies every published force, acceleration, and torque.
"""

from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path

import gtsam
import gtdynamics as gtd
import mujoco
import numpy as np
from scipy.optimize import linprog

from export_recording import REPO
from g1_model import load_g1_model
from mujoco_io import import_robot, model_bodies, pose


def validate(directory: Path) -> dict:
    """Compare saved optimized accelerations and torques with an external oracle.

    MuJoCo contact constraints and passive joint damping/friction are disabled:
    the native balance equations model inertia, gravity, Coriolis, and armature.
    Joint torque is MuJoCo inverse generalized force minus sole-wrench loading.
    """
    record = json.loads(gzip.decompress((directory / "recording.json.gz").read_bytes()))
    model = load_g1_model()
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
    model.dof_damping[:] = 0
    model.dof_frictionloss[:] = 0
    robot = import_robot(model_bodies(model))
    root = robot.link("pelvis")
    root_body = model.body("robot/pelvis").id
    body_to_com = pose(model.body_ipos[root_body], model.body_iquat[root_body])
    joints = robot.orderedJoints()
    columns = [record["jointNames"].index(joint.name()) for joint in joints]
    joint_dofs = [model.joint("robot/" + joint.name()).dofadr[0] for joint in joints]
    settings = record["settings"]
    friction, rectangles = settings["friction"], settings["supportRectangles"]
    cones, rays = [], []
    for x, y in rectangles:
        cone = gtd.RectangularContactWrench(x, y, friction).matrix()
        generators = []
        for sx, sy, fx, fy in np.ndindex(2, 2, 2, 2):
            force = np.array([2 * fx - 1, 2 * fy - 1, np.sqrt(2) / friction])
            generators.append(
                np.r_[
                    np.cross([(2 * sx - 1) * x, (2 * sy - 1) * y, 0], force),
                    force,
                ]
            )
        generators = np.asarray(generators).T
        if np.max(cone @ generators) > 1e-10:
            raise AssertionError("A corner-force ray violates the surface wrench cone")
        cones.append(cone)
        rays.append(generators)
    inequalities = np.zeros((34, 12))
    inequalities[:17, :6], inequalities[17:, 6:] = cones
    data = mujoco.MjData(model)
    errors, torque_errors, infeasible = [], [], []
    balance_errors, stance_speeds = [], []
    for k, time in enumerate(record["times"]):
        row = np.asarray(record["bodyPoses"][k][root_body])
        world_body = gtsam.Pose3(gtsam.Rot3.Quaternion(*row[[6, 3, 4, 5]]), row[:3])
        root_pose = world_body * body_to_com
        known = gtsam.Values()
        gtd.InsertPose(known, root.id(), 0, root_pose)
        gtd.InsertTwist(known, root.id(), 0, np.asarray(record["baseTwists"][k]))
        for joint, column in zip(joints, columns):
            gtd.InsertJointAngle(known, joint.id(), 0, record["jointAngles"][k][column])
            gtd.InsertJointVel(
                known, joint.id(), 0, record["jointVelocities"][k][column]
            )
        values: gtsam.Values = robot.forwardKinematics(known, 0, root.name())
        required = np.zeros(6)
        for link in robot.links():
            body = model.body("robot/" + link.name()).id
            world_com = gtd.Pose(values, link.id())
            twist = gtd.Twist(values, link.id())
            derivative = np.asarray(record["linkTwistAccelerations"][k][body])
            inertia = np.diag(
                np.r_[model.body_inertia[body], np.repeat(model.body_mass[body], 3)]
            )
            gravity = np.r_[
                np.zeros(3),
                model.body_mass[body]
                * world_com.rotation().unrotate(model.opt.gravity),
            ]
            wrench = (
                inertia @ derivative
                - gtsam.Pose3.adjointMap(twist).T @ inertia @ twist
                - gravity
            )
            required += (
                gtd.contactWrenchTransform(root_pose.between(world_com)) @ wrench
            )
        body_twist = body_to_com.AdjointMap() @ np.asarray(record["baseTwists"][k])
        body_derivative = body_to_com.AdjointMap() @ np.asarray(
            record["baseTwistAccelerations"][k]
        )
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0
        data.qacc[:] = 0
        data.qpos[:3], data.qpos[3:7] = row[:3], row[[6, 3, 4, 5]]
        data.qvel[:3] = world_body.rotation().rotate(body_twist[3:])
        data.qvel[3:6] = body_twist[:3]
        data.qacc[:3] = world_body.rotation().rotate(
            body_derivative[3:] + np.cross(body_twist[:3], body_twist[3:])
        )
        data.qacc[3:6] = body_derivative[:3]
        for joint, column, dof in zip(joints, columns, joint_dofs):
            address = model.joint("robot/" + joint.name()).qposadr[0]
            data.qpos[address] += record["jointAngles"][k][column]
            data.qvel[dof] = record["jointVelocities"][k][column]
            data.qacc[dof] = record["jointAccelerations"][k][column]
        mujoco.mj_inverse(model, data)
        force_body = world_body.rotation().unrotate(data.qfrc_inverse[:3])
        moment_body = data.qfrc_inverse[3:6] - np.cross(
            model.body_ipos[root_body], force_body
        )
        oracle = np.r_[
            body_to_com.rotation().unrotate(moment_body),
            body_to_com.rotation().unrotate(force_body),
        ]
        errors.append(oracle - required)
        transforms, loading = [], np.zeros(model.nv)
        for foot, side in enumerate(("left", "right")):
            contact = np.asarray(record["contactPoses"][k][foot])
            world_contact = gtsam.Pose3(
                gtsam.Rot3.Quaternion(*contact[[6, 3, 4, 5]]), contact[:3]
            )
            transforms.append(
                gtd.contactWrenchTransform(root_pose.between(world_contact))
            )
            translation, angular = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            body = model.body(f"robot/{side}_ankle_roll_link").id
            mujoco.mj_jac(model, data, translation, angular, contact[:3].copy(), body)
            loading += translation.T @ np.asarray(
                record["worldForces"][k][foot]
            ) + angular.T @ np.asarray(record["worldMoments"][k][foot])
            if record["contact"][k][foot]:
                stance_speeds.append(float(np.linalg.norm(translation @ data.qvel)))
        torque_errors.append(
            data.qfrc_inverse[joint_dofs]
            - loading[joint_dofs]
            - np.asarray(record["jointTorques"][k])[columns]
        )
        supplied = np.hstack(transforms) @ np.asarray(record["wrenches"][k]).reshape(-1)
        balance_errors.append(required - supplied)
        bounds = [
            (None, None) if active else (0, 0)
            for active in record["contact"][k]
            for _ in range(6)
        ]
        resultant = linprog(
            np.zeros(12),
            A_ub=inequalities,
            b_ub=np.zeros(34),
            A_eq=np.hstack(transforms),
            b_eq=required,
            bounds=bounds,
            method="highs",
        )
        corner = linprog(
            np.zeros(32),
            A_eq=np.hstack(
                [transform @ ray for transform, ray in zip(transforms, rays)]
            ),
            b_eq=required,
            bounds=[
                (0, None) if active else (0, 0)
                for active in record["contact"][k]
                for _ in range(16)
            ],
            method="highs",
        )
        if resultant.status not in (0, 2) or corner.status not in (0, 2):
            raise RuntimeError("An LP check failed without establishing feasibility")
        if resultant.success != corner.success:
            raise AssertionError(
                "Resultant-wrench and corner-force feasibility disagree"
            )
        if not resultant.success:
            infeasible.append({"slice": k, "timeSeconds": time})
    errors, balance_errors = np.asarray(errors), np.asarray(balance_errors)
    force_error = float(np.max(np.linalg.norm(errors[:, 3:], axis=1)))
    moment_error = float(np.max(np.linalg.norm(errors[:, :3], axis=1)))
    torque_error = float(np.max(np.abs(torque_errors)))
    if max(force_error, moment_error, torque_error) > 1e-6:
        raise AssertionError(
            f"MuJoCo disagreement: {force_error:g} N, {moment_error:g} Nm, joint {torque_error:g} Nm"
        )
    return {
        "samples": len(record["times"]),
        "maxMuJoCoForceDisagreementN": force_error,
        "maxMuJoCoMomentDisagreementNm": moment_error,
        "maxMuJoCoJointTorqueDisagreementNm": torque_error,
        "maxPublishedBalanceResidual": float(
            np.max(np.linalg.norm(balance_errors, axis=1))
        ),
        "stanceFootSpeedRmsMetersPerSecond": float(
            np.sqrt(np.mean(np.square(stance_speeds)))
        ),
        "exactFeasibleSlices": len(record["times"]) - len(infeasible),
        "exactInfeasibleSlices": infeasible,
        "cornerForceAndResultantConeAgreement": True,
        "lpUsedForPublishedForces": False,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, default=REPO / "apps/dynamics/public")
    args = parser.parse_args()
    print(json.dumps(validate(args.recording), indent=2))
