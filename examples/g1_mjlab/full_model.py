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

from mujoco_io import model_bodies


def snapshot(model, data, cone):
    bodies = model_bodies(model)
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
