#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file validate_recording.py
# @brief Validate dynamics with MuJoCo and independent GTSAM feasibility LPs.
# @author GTDynamics contributors

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
from numpy.typing import NDArray

from export_recording import REPO
from g1_model import load_g1_model
from mujoco_io import import_robot, model_bodies, pose


def linear_feasible(
    equalities: NDArray[np.float64],
    target: NDArray[np.float64],
    inequalities: NDArray[np.float64],
) -> bool:
    """Check A x = b and C x <= 0 using a GTSAM phase-I LP.

    Keep balance exact and minimize s subject to C x <= s and s >= 0. A zero
    optimum establishes feasibility. Keeping the equalities exact avoids the
    dependent positive/negative equality rows of automatic phase-I creation.
    This check contributes no published forces. Solver failures propagate.
    """
    tolerance = 1e-6
    dimension = equalities.shape[1]
    if dimension == 0:
        return bool(np.max(np.abs(target)) <= tolerance)
    # Minimize ||x||^2 with exact balance to obtain the LP's initial point.
    seed_graph = gtsam.GaussianFactorGraph()
    seed_graph.add(
        gtsam.JacobianFactor(
            0,
            np.eye(dimension),
            np.zeros(dimension),
            gtsam.noiseModel.Unit.Create(dimension),
        )
    )
    seed_graph.add(
        gtsam.JacobianFactor(
            0, equalities, target, gtsam.noiseModel.Constrained.All(len(target))
        )
    )
    seed = seed_graph.optimize().at(0)
    if np.max(np.abs(equalities @ seed - target)) > tolerance:
        return False
    nonzero = inequalities[np.any(inequalities != 0, axis=1)]
    if not len(nonzero):
        return True
    problem = gtsam.LpProblem()
    cost = np.zeros((1, dimension + 1))
    cost[0, -1] = 1
    problem.addCost(
        gtsam.JacobianFactor(0, cost, np.zeros(1), gtsam.noiseModel.Unit.Create(1))
    )
    problem.addConstraint(
        gtsam.LinearConstraint.Equal(
            gtsam.JacobianFactor(
                0,
                np.column_stack([equalities, np.zeros(len(target))]),
                target,
                gtsam.noiseModel.Unit.Create(len(target)),
            )
        )
    )
    bounds = np.vstack([np.column_stack([nonzero, -np.ones(len(nonzero))]), -cost])
    problem.addConstraint(
        gtsam.LinearConstraint.LessEqual(
            gtsam.JacobianFactor(
                0,
                bounds,
                np.zeros(len(bounds)),
                gtsam.noiseModel.Unit.Create(len(bounds)),
            )
        )
    )
    initial = gtsam.Values()
    initial.insert(0, np.r_[seed, max(1.0, float(np.max(nonzero @ seed)) + 1.0)])
    optimum = problem.optimize(initial).atVector(0)
    solution, slack = optimum[:-1], optimum[-1]
    if (
        np.max(np.abs(equalities @ solution - target)) > tolerance
        or np.any(nonzero @ solution - slack > tolerance)
        or slack < -tolerance
    ):
        raise RuntimeError("The GTSAM LP result violates feasibility tolerances")
    return bool(slack <= tolerance)


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
        active_wrenches = np.repeat(record["contact"][k], 6)
        resultant = linear_feasible(
            np.hstack(transforms)[:, active_wrenches],
            required,
            inequalities[:, active_wrenches],
        )
        active_rays = np.repeat(record["contact"][k], 16)
        corner = linear_feasible(
            np.hstack([transform @ ray for transform, ray in zip(transforms, rays)])[
                :, active_rays
            ],
            required,
            -np.eye(np.count_nonzero(active_rays)),
        )
        if resultant != corner:
            raise AssertionError(
                "Resultant-wrench and corner-force feasibility disagree"
            )
        if not resultant:
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
