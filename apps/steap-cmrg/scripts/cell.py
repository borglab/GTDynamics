#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file cell.py
# @brief BAR Lab CMRG cell model: joints, limits, tools and forward kinematics.
# @author GTDynamics contributors

"""Joint names, limits, tool geometry and kinematics for the two-arm CMRG cell.

The cell is two ABB IRB6620 arms, each on its own gantry bridge (three
prismatic axes), loaded from ``models/urdfs/bar_lab_tools.urdf``. Arm 1 carries
a timber gripper and arm 2 a timber nailer. GTDynamics poses links at their
centre of mass, so tool and sphere frames are converted through the link
centroid offset. Tool constants were measured on the tool meshes in BAR Lab.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import gtdynamics as gtd
import gtsam
import numpy as np

import urdf_display as ud

URDF_FILE = ud.URDF_ROOT / "bar_lab_tools.urdf"
TOOL_MESHES = {
    "gripper": ud.URDF_ROOT / "bar_tools" / "TimberGripper.stl",
    "nailer": ud.URDF_ROOT / "bar_tools" / "TimberNailer.stl",
}
ARM_TOOL = {1: "gripper", 2: "nailer"}
BASE_LINK = "columns"

# Measured tool geometry in the tool0 frame (m, deg), from BAR Lab tool meshes.
GRIPPER_AXIS_YAW_DEG = -52.711
GRIPPER_AXIS_XY = (0.00144, 0.00181)
GRIPPER_GRIP_PLANE_Z = 0.3525  # inner slot, board on edge
NAILER_TIP_POINT = (0.0, 0.0, 0.43104)
GRIPPER_SPHERES = {
    "centers": [
        (+0.307413, -0.511035, +0.115016),
        (+0.264630, -0.408397, +0.137066),
        (+0.249548, -0.243989, +0.224125),
        (-0.032896, -0.054041, +0.094642),
        (-0.067643, +0.128779, +0.089071),
        (-0.244722, +0.289513, +0.246426),
        (-0.329682, +0.474774, +0.109257),
    ],
    "radii": [0.075841, 0.178937, 0.235502, 0.218895, 0.171471, 0.224510, 0.105500],
}
NAILER_SPHERES = {
    "centers": [
        (+0.023889, +0.014001, +0.008496),
        (+0.029994, +0.020309, +0.187978),
        (+0.075521, +0.056831, +0.378564),
    ],
    "radii": [0.211336, 0.237019, 0.121754],
}
TOOLS = {
    "gripper": {
        "spheres": GRIPPER_SPHERES,
        "tcp_point": (*GRIPPER_AXIS_XY, GRIPPER_GRIP_PLANE_Z),
        "tcp_yaw_deg": GRIPPER_AXIS_YAW_DEG,
    },
    "nailer": {
        "spheres": NAILER_SPHERES,
        "tcp_point": NAILER_TIP_POINT,
        "tcp_yaw_deg": 0.0,
    },
}

# Dress-cable anchors (m) and exit normals in link frames, from the zero-config world points on the
# Grasshopper canvas (BAR-robot-cell robot_vizualize_V2_RH8_cables_code.ghx, origin/loop_collab).
CABLE_ANCHORS = {
    1: {"link_3": np.array([0.21667971, 0.04699000, 0.46007500]),
        "link_6": np.array([0.11704072, 0.33257824, 0.11511609])},
    2: {"link_3": np.array([0.21667971, 0.04699000, 0.46007498]),
        "link_6": np.array([0.11699171, 0.33239774, 0.11499894])},
}
CABLE_NORMALS = {"link_3": np.array([1.0, 0.0, 0.0]),
                 "link_6": np.array([-0.9659258262890683, 0.25881904510252074, 0.0])}

# Park positions (mm): the arms start 5 m apart on the rail, closer than the lab home positions.
HOME_EXTERNAL_MM = {1: (3000.0, 3650.0, 1150.0), 2: (8600.0, 3650.0, 1150.0)}


def arm_joint_names(arm: int) -> list[str]:
    """Return gantry axes then arm joints for an arm, in planner state order."""
    gantry = [f"bridge{arm}_joint_EA_X", f"robot{arm}_joint_EA_Y", f"robot{arm}_joint_EA_Z"]
    return gantry + [f"robot{arm}_joint_{i}" for i in range(1, 7)]


def _joint_limit_attribute(name: str, attribute: str) -> float:
    root = ET.parse(URDF_FILE).getroot()
    return float(root.find(f".//joint[@name='{name}']/limit").get(attribute))


def joint_limits(arm: int) -> dict[str, tuple[float, float]]:
    """Return ``{joint: (lower, upper)}`` from the URDF, in m and rad."""
    return {
        name: (_joint_limit_attribute(name, "lower"), _joint_limit_attribute(name, "upper"))
        for name in arm_joint_names(arm)
    }


def velocity_limits(arm: int) -> dict[str, float]:
    """Return ``{joint: max speed}`` from the URDF, in m/s and rad/s."""
    return {name: _joint_limit_attribute(name, "velocity") for name in arm_joint_names(arm)}


def pose_from_matrix(matrix: np.ndarray) -> gtsam.Pose3:
    """Return a Pose3 for a 4x4 homogeneous transform."""
    return gtsam.Pose3(np.asarray(matrix, dtype=float))


@lru_cache(maxsize=None)
def link_centroid_offset(link_name: str) -> gtsam.Pose3:
    """Return the link-frame to lumped-CoM translation as GTDynamics poses links.

    Descendants joined by fixed joints are lumped into the link, as in
    GTDynamics, so the inverse recovers the URDF link frame from a GTDynamics
    link pose.
    """
    root = ET.parse(URDF_FILE).getroot()
    model = ud.parse_urdf(URDF_FILE)
    fixed_children: dict[str, list[ud.Joint]] = {}
    for joint in model.joints:
        if joint.kind == "fixed":
            fixed_children.setdefault(joint.parent, []).append(joint)

    def inertial(name: str) -> tuple[float, np.ndarray]:
        link = root.find(f".//link[@name='{name}']")
        inert = link.find("inertial") if link is not None else None
        if inert is None:
            return 0.0, np.zeros(3)
        mass = float(inert.find("mass").get("value"))
        origin = inert.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()]) if origin is not None else np.zeros(3)
        return mass, xyz

    total_mass, weighted = 0.0, np.zeros(3)
    stack = [(link_name, np.eye(4))]
    while stack:
        name, frame = stack.pop()
        mass, com = inertial(name)
        total_mass += mass
        weighted += mass * (frame @ np.append(com, 1.0))[:3]
        for joint in fixed_children.get(name, []):
            stack.append((joint.child, frame @ joint.origin))
    if total_mass <= 0.0:
        return gtsam.Pose3()
    return gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(*(weighted / total_mass)))


@lru_cache(maxsize=None)
def link6_tool0_offset(arm: int) -> gtsam.Pose3:
    """Return the fixed link_6 to tool0 transform (attachment joint, then tool0)."""
    model = ud.parse_urdf(URDF_FILE)
    joints = {joint.name: joint for joint in model.joints}
    chain = [f"robot{arm}_joint_6_attachment", f"robot{arm}_joint_6-tool0"]
    matrix = np.eye(4)
    for name in chain:
        matrix = matrix @ joints[name].origin
    return pose_from_matrix(matrix)


def tool_tcp_pose(tool: str) -> gtsam.Pose3:
    """Return the tool working point as a pose in tool0 coordinates."""
    spec = TOOLS[tool]
    return gtsam.Pose3(
        gtsam.Rot3.Rz(np.deg2rad(spec["tcp_yaw_deg"])), gtsam.Point3(*spec["tcp_point"])
    )


def tool_transform(arm: int) -> gtsam.Pose3:
    """Return the transform from a link_6 CoM frame to the arm tool's working point."""
    com = link_centroid_offset(f"robot{arm}_link_6")
    return com.inverse().compose(link6_tool0_offset(arm)).compose(tool_tcp_pose(ARM_TOOL[arm]))


def tool_spheres_in_com(arm: int) -> tuple[list[np.ndarray], list[float]]:
    """Return a tool's collision spheres in its link_6 CoM frame (centres, radii)."""
    com = link_centroid_offset(f"robot{arm}_link_6")
    com_t_tool0 = com.inverse().compose(link6_tool0_offset(arm)).matrix()
    spec = TOOLS[ARM_TOOL[arm]]["spheres"]
    centers = [(com_t_tool0 @ np.append(np.asarray(c, dtype=float), 1.0))[:3] for c in spec["centers"]]
    return centers, list(spec["radii"])


def load_robot() -> gtd.Robot:
    """Load the cell with ``columns`` fixed to the world, as the paper planner does."""
    return gtd.CreateRobotFromFile(str(URDF_FILE)).fixLink(BASE_LINK)


@dataclass
class ArmSpec:
    """Per-arm planning context: tool link, tool transform, joints and limits."""

    arm: int
    link: gtd.Link
    tool_t: gtsam.Pose3
    q_names: list[str]
    limits: dict[str, tuple[float, float]]
    vel_limits: dict[str, float]


def make_arm_specs(robot: gtd.Robot) -> list[ArmSpec]:
    """Return the specs of arm 1 (gripper) and arm 2 (nailer)."""
    return [
        ArmSpec(
            arm,
            robot.link(f"robot{arm}_link_6"),
            tool_transform(arm),
            arm_joint_names(arm),
            joint_limits(arm),
            velocity_limits(arm),
        )
        for arm in (1, 2)
    ]


def forward_kinematics(robot: gtd.Robot, joint_values: dict[str, float]) -> gtsam.Values:
    """Return FK values for a joint-name to value map; unspecified joints are zero."""
    values = gtsam.Values()
    for joint in robot.joints():
        gtd.InsertJointAngle(values, joint.id(), 0, joint_values.get(joint.name(), 0.0))
    return robot.forwardKinematics(values, 0, BASE_LINK)


def tool_pose(robot: gtd.Robot, spec: ArmSpec, q: np.ndarray) -> gtsam.Pose3:
    """Return the world pose of an arm's tool working point at configuration q."""
    fk = forward_kinematics(robot, dict(zip(spec.q_names, q)))
    return gtd.Pose(fk, spec.link.id(), 0).compose(spec.tool_t)


def downward_goal(x: float, y: float, z: float, yaw: float = 0.0) -> gtsam.Pose3:
    """Return a tool goal pointing straight down with its x axis along ``yaw``."""
    axis = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    down = np.array([0.0, 0.0, -1.0])
    side = np.cross(down, axis)
    return gtsam.Pose3(gtsam.Rot3(np.column_stack([axis, side, down])), gtsam.Point3(x, y, z))


def approach_goal(x: float, y: float, z: float, approach, yaw: float = 0.0) -> gtsam.Pose3:
    """Return a tool goal whose z axis points along ``approach``."""
    a = np.asarray(approach, dtype=float)
    a = a / np.linalg.norm(a)
    for reference in (
        np.array([np.cos(yaw), np.sin(yaw), 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 0.0, 0.0]),
    ):
        x_axis = reference - np.dot(reference, a) * a
        if np.linalg.norm(x_axis) > 1e-6:
            break
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(a, x_axis)
    return gtsam.Pose3(gtsam.Rot3(np.column_stack([x_axis, y_axis, a])), gtsam.Point3(x, y, z))


def park_configuration(arm: int) -> np.ndarray:
    """Return the home configuration of an arm (gantry in m, joints zero)."""
    return np.concatenate([np.array(HOME_EXTERNAL_MM[arm]) / 1000.0, np.zeros(6)])
