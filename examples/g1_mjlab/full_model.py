# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file full_model.py
# @brief Export model parameters/state and collision contacts, never dynamics/Jacobians.
# @author Frank Dellaert

"""Export model parameters/state and collision contacts, never dynamics/Jacobians.

The contact compliance R and reference acceleration are explicit inputs to the
matched contact law. GTDynamics constructs its own robot, forward kinematics,
acceleration constraints, and Newton--Euler equations from these parameters.
"""

import mujoco
import numpy as np


def snapshot(model, data, cone):
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
    contacts = []
    moving = {b["id"] for b in bodies}
    expected_type = getattr(mujoco.mjtConstraint, "mjCNSTR_CONTACT_" + cone.upper())
    if np.any(data.efc_type != int(expected_type)):
        raise ValueError("Unsupported joint-limit or non-contact constraints")
    for contact in data.contact:
        if contact.efc_address < 0:
            continue
        b1, b2 = (int(model.geom_bodyid[g]) for g in contact.geom)
        if contact.dim != 3 or (b1 in moving) == (b2 in moving):
            raise ValueError("Expected a 3D contact between the G1 and fixed terrain")
        contacts.append(
            {
                "address": int(contact.efc_address),
                "body": b2 if b2 in moving else b1,
                "sign": 1 if b2 in moving else -1,
                "position": contact.pos.tolist(),
                "frame": contact.frame.reshape(3, 3).tolist(),
                "friction": contact.friction[:2].tolist(),
            }
        )
    return {
        "schema": "gtdynamics-full-graph-v1",
        "bodies": bodies,
        "qpos": data.qpos.tolist(),
        "qvel": data.qvel.tolist(),
        "armature": model.dof_armature.tolist(),
        "gravity": model.opt.gravity.tolist(),
        "applied": (
            data.qfrc_actuator + data.qfrc_passive + data.qfrc_applied
        ).tolist(),
        "cone": cone,
        "contacts": contacts,
        "R": data.efc_R.tolist(),
        "aref": data.efc_aref.tolist(),
    }
