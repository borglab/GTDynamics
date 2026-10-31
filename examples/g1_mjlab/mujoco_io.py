# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------

## @file mujoco_io.py
# @brief Shared MuJoCo recording loader and static robot conversion.
# @author Frank Dellaert

"""MuJoCo recording and static-model export for the G1 comparison."""

import json

import mujoco
import numpy as np


def load_model(directory, cone=None):
    """Load the recorded binary with the common high-accuracy oracle settings."""
    model = mujoco.MjModel.from_binary_path(str(directory / "model.mjb"))
    model.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    model.opt.iterations = 100
    model.opt.tolerance = 1e-12
    if cone is not None:
        model.opt.cone = getattr(mujoco.mjtCone, "mjCONE_" + cone.upper())
    return model


def load_recording(directory):
    """Read arrays and recording metadata, closing the NPZ archive immediately."""
    with np.load(directory / "walk.npz") as archive:
        recording = {name: archive[name] for name in archive.files}
    return recording, json.loads((directory / "walk.json").read_text())


def model_bodies(model):
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
