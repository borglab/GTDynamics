# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# ----------------------------------------------------------------------------
## @file g1_urdf.py
# @brief Export the canonical G1 as a fixed-pelvis IK model.
# @author GTDynamics contributors

"""Export the canonical G1 joint geometry and inertias for the IK explorer."""

from __future__ import annotations

from math import atan2, hypot
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from g1_model import sole_frame


def _vector(values) -> str:
    return " ".join(str(float(value)) for value in values)


def _origin(element: ET.Element, position, quaternion) -> None:
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, quaternion)
    rotation = rotation.reshape(3, 3)
    horizontal = hypot(rotation[0, 0], rotation[1, 0])
    pitch = atan2(-rotation[2, 0], horizontal)
    if horizontal > 1e-12:
        roll = atan2(rotation[2, 1], rotation[2, 2])
        yaw = atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = atan2(-rotation[0, 1], rotation[1, 1])
    ET.SubElement(
        element, "origin", xyz=_vector(position), rpy=_vector((roll, pitch, yaw))
    )


def _joint(
    robot: ET.Element, name: str, kind: str, parent: str, child: str
) -> ET.Element:
    joint = ET.SubElement(robot, "joint", name=name, type=kind)
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    return joint


def write_ik_urdf(model: mujoco.MjModel, destination: Path) -> None:
    """Write G1's 29 hinges and four tool frames in a pelvis-fixed world."""
    robot = ET.Element("robot", name="unitree_g1")
    robot.append(
        ET.Comment(
            "Generated from the pinned Menagerie/MJLab G1. The pelvis is fixed at the world origin."
        )
    )
    names = [
        model.body(body).name.removeprefix("robot/") for body in range(model.nbody)
    ]
    for body in range(1, model.nbody):
        link = ET.SubElement(robot, "link", name=names[body])
        inertial = ET.SubElement(link, "inertial")
        _origin(inertial, model.body_ipos[body], model.body_iquat[body])
        ET.SubElement(inertial, "mass", value=str(model.body_mass[body]))
        ixx, iyy, izz = model.body_inertia[body]
        ET.SubElement(
            inertial,
            "inertia",
            ixx=str(ixx),
            iyy=str(iyy),
            izz=str(izz),
            ixy="0",
            ixz="0",
            iyz="0",
        )

        parent = int(model.body_parentid[body])
        if parent == 0:  # The floating pelvis becomes the root link.
            continue
        joint_id = int(model.body_jntadr[body])
        if (
            model.body_jntnum[body] != 1
            or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE
        ):
            raise ValueError(f"Expected one hinge on {names[body]}")
        if (
            np.any(model.jnt_pos[joint_id])
            or model.qpos0[model.jnt_qposadr[joint_id]] != 0
        ):
            raise ValueError(
                f"Expected a zero-reference hinge at {names[body]}'s origin"
            )
        name = model.joint(joint_id).name.removeprefix("robot/")
        joint = _joint(robot, name, "revolute", names[parent], names[body])
        _origin(joint, model.body_pos[body], model.body_quat[body])
        ET.SubElement(joint, "axis", xyz=_vector(model.jnt_axis[joint_id]))
        lower, upper = model.jnt_range[joint_id]
        # The IK model uses position bounds. Effort and speed are unused.
        ET.SubElement(
            joint, "limit", lower=str(lower), upper=str(upper), effort="0", velocity="0"
        )

    for side in ("left", "right"):
        palm = model.site(f"robot/{side}_palm")
        tools = (
            (f"{side}_palm", names[int(palm.bodyid[0])], palm.pos, palm.quat),
            (
                f"{side}_sole",
                f"{side}_ankle_roll_link",
                sole_frame(model, side)[:3, 3],
                (1, 0, 0, 0),
            ),
        )
        for name, parent, position, quaternion in tools:
            ET.SubElement(robot, "link", name=name)
            _origin(
                _joint(robot, name + "_frame", "fixed", parent, name),
                position,
                np.asarray(quaternion, dtype=float),
            )

    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(robot)
    ET.ElementTree(robot).write(destination, encoding="utf-8", xml_declaration=True)
