#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file structure.py
# @brief Post-and-lintel structure specifications built from 2x4 stock on a tabletop.
# @author GTDynamics contributors

"""Turn a post-and-lintel specification into ordered members, the table and grips.

All members share the paper's 2x4 section (3.5 in x 1.5 in). Posts stand on the
table; each lintel lies flat across the tops of two posts. Stock is staged on edge
at the far end of the same table, far from the build site on arm 1's side of the gantry rail, so every member
needs a long gantry transit. Pose conventions follow the paper's truss helpers: a
member's local x axis is its length, y its width and z its thickness.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import gtsam
import numpy as np

import cell
from scene import Box, SphereSet, member_spheres, table_spheres

PROFILE_WIDTH = 3.5 * 0.0254
PROFILE_THICK = 1.5 * 0.0254
# Workbench height: EA_Z bottoms out for a horizontal nailer below about 0.37 m, and the cable needs floor clearance.
TABLE_TOP_Z = 0.91355
TABLE_HEIGHT = TABLE_TOP_Z
GRIP_Z_ABOVE = 0.035
# The gripper's pads reach 0.40 m either side of its centre: both pads hold the post, the upper one
# ending flush with the post's un-nailed top end.
GRIP_FROM_TOP = 0.40
STOCK_SPACING = 0.28
STOCK_MARGIN = 0.10
APPROACH = 0.30


@dataclass
class Member:
    """One piece of wood: sizes, staged and target poses, and the gripper hold."""

    name: str
    kind: str
    size: tuple[float, float, float]
    target: np.ndarray
    staging: np.ndarray | None = None
    grip_staging: gtsam.Pose3 | None = None
    grip: gtsam.Pose3 | None = None
    grip_offset: np.ndarray | None = None
    fixed: bool = False

    def spheres(self, pose: np.ndarray | None = None) -> SphereSet:
        """Return the member's collision spheres at a pose (default its target)."""
        return member_spheres(self.target if pose is None else pose, self.size)


@dataclass
class Structure:
    """A named build: ordered members (fixed ones first) and the one tabletop."""

    name: str
    members: list[Member]
    tables: dict[str, Box]
    site: np.ndarray
    stock_distance: float
    spec: dict = field(default_factory=dict)

    def static_spheres(self) -> list[SphereSet]:
        """Return the tables as obstacle sphere sets."""
        return [table_spheres(box) for box in self.tables.values()]


def flat_pose(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    """Return the pose of a member lying flat with its length along ``yaw``."""
    c, s = math.cos(yaw), math.sin(yaw)
    pose = np.eye(4)
    pose[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    pose[:3, 3] = (x, y, z)
    return pose


def on_edge_pose(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    """Return the pose of a member on edge: length along ``yaw``, width vertical."""
    c, s = math.cos(yaw), math.sin(yaw)
    pose = np.eye(4)
    pose[:3, 0] = (c, s, 0.0)
    pose[:3, 1] = (0.0, 0.0, 1.0)
    pose[:3, 2] = (s, -c, 0.0)
    pose[:3, 3] = (x, y, z)
    return pose


def standing_pose(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    """Return the pose of an upright member whose wide face points along ``yaw``."""
    c, s = math.cos(yaw), math.sin(yaw)
    pose = np.eye(4)
    pose[:3, 0] = (0.0, 0.0, 1.0)
    pose[:3, 1] = (c, s, 0.0)
    pose[:3, 2] = (-s, c, 0.0)
    pose[:3, 3] = (x, y, z)
    return pose


def standoff(goal: gtsam.Pose3, distance: float = APPROACH) -> gtsam.Pose3:
    """Return a pose ``distance`` back along the goal's approach (z) axis."""
    z_axis = goal.rotation().matrix()[:, 2]
    point = np.asarray(goal.translation()) - distance * z_axis
    return gtsam.Pose3(goal.rotation(), gtsam.Point3(*point))


def lifted(goal: gtsam.Pose3, dz: float) -> gtsam.Pose3:
    """Return a pose raised ``dz`` straight up in the world."""
    p = np.asarray(goal.translation())
    return gtsam.Pose3(goal.rotation(), gtsam.Point3(p[0], p[1], p[2] + dz))


def add_grip(
    member: Member, stage_x: float, stage_y: float, grip_y: float, yaw: float
) -> None:
    """Set the staged grip, the grip at the target and the in-hand offset of a member.

    The grip is a downward tool goal above the staged member on edge. Carrying it
    through the staging-to-target transform gives the hold at the target.
    """
    stage_z = TABLE_TOP_Z + PROFILE_WIDTH / 2.0
    member.grip_staging = cell.downward_goal(
        stage_x, grip_y, stage_z + GRIP_Z_ABOVE, yaw
    )
    grip_local = np.linalg.inv(member.staging) @ member.grip_staging.matrix()
    member.grip = gtsam.Pose3(member.target @ grip_local)
    member.grip_offset = np.linalg.inv(member.grip_staging.matrix()) @ member.staging


def build_structure(spec: dict) -> Structure:
    """Build the members and table of a post-and-lintel specification.

    ``spec`` keys: ``name``; ``site`` (x, y of the structure centre); ``stock_distance``
    (m from the site to the furthest staged member, toward arm 1 along -x); ``posts`` a list of
    ``{x, y, height}`` offsets from the site; ``lintels`` a list of ``{between: [i, j], overhang}``
    joining posts, with ends flush to the posts' outer faces by default; and optional ``base``
    (true) for a fixed ground plank fitting exactly between the first two posts.
    """
    site = np.array(spec.get("site", [6.0, 3.65]), dtype=float)
    stock_distance = float(spec.get("stock_distance", 2.0))
    members: list[Member] = []
    if spec.get("base"):
        first, second = (
            np.array([site[0] + p.get("x", 0.0), site[1] + p["y"]])
            for p in spec["posts"][:2]
        )
        direction = second - first
        length = float(np.linalg.norm(direction)) - PROFILE_THICK
        center = (first + second) / 2.0
        pose = flat_pose(
            center[0],
            center[1],
            TABLE_TOP_Z + PROFILE_THICK / 2.0,
            math.atan2(direction[1], direction[0]),
        )
        members.append(
            Member(
                "base", "base", (length, PROFILE_WIDTH, PROFILE_THICK), pose, fixed=True
            )
        )
    posts = spec["posts"]
    movable: list[Member] = []
    for index, post in enumerate(posts):
        height = float(post["height"])
        x, y = site[0] + post.get("x", 0.0), site[1] + post["y"]
        pose = standing_pose(x, y, TABLE_TOP_Z + height / 2.0, math.pi)
        movable.append(
            Member(
                f"post_{index + 1}",
                "post",
                (height, PROFILE_WIDTH, PROFILE_THICK),
                pose,
            )
        )
    for index, lintel in enumerate(spec.get("lintels", [])):
        a, b = lintel["between"]
        pa, pb = movable[a].target[:3, 3], movable[b].target[:3, 3]
        direction = pb[:2] - pa[:2]
        span = float(np.linalg.norm(direction))
        length = span + 2.0 * float(lintel.get("overhang", PROFILE_THICK / 2.0))
        top = max(movable[a].size[0], movable[b].size[0]) + TABLE_TOP_Z
        z = top + PROFILE_THICK / 2.0
        center = (pa[:2] + pb[:2]) / 2.0
        yaw = math.atan2(direction[1], direction[0])
        pose = flat_pose(center[0], center[1], z, yaw)
        movable.append(
            Member(
                f"lintel_{index + 1}",
                "lintel",
                (length, PROFILE_WIDTH, PROFILE_THICK),
                pose,
            )
        )
    stock_x = site[0] - stock_distance + (len(movable) - 1) / 2.0 * STOCK_SPACING
    for slot, member in enumerate(movable):
        length = member.size[0]
        x = stock_x + (slot - (len(movable) - 1) / 2.0) * STOCK_SPACING
        y_center = site[1]
        z = TABLE_TOP_Z + PROFILE_WIDTH / 2.0
        member.staging = on_edge_pose(x, y_center, z, math.pi / 2.0)
        end = y_center + length / 2.0
        grip_y = end - GRIP_FROM_TOP if member.kind == "post" else y_center
        add_grip(member, x, y_center, grip_y, math.pi / 2.0)
    members.extend(movable)
    return Structure(
        spec.get("name", "structure"),
        members,
        _tables(members, movable),
        site,
        stock_distance,
        spec,
    )


def footprint(pose: np.ndarray, size) -> np.ndarray:
    """Return the world x, y of a member's eight corners."""
    signs = np.array(
        [[i, j, k] for i in (-0.5, 0.5) for j in (-0.5, 0.5) for k in (-0.5, 0.5)]
    )
    return ((signs * np.asarray(size)) @ pose[:3, :3].T + pose[:3, 3])[:, :2]


def table_box(points: np.ndarray, margin: float = 0.0) -> Box:
    """Return a table from the floor to the work height over the bounding rectangle of ``points``."""
    low, high = points.min(axis=0) - margin, points.max(axis=0) + margin
    half_top = TABLE_HEIGHT / 2.0
    return Box.axis_aligned(
        [*(low + high) / 2.0, half_top], [*(high - low) / 2.0, half_top]
    )


def _tables(members: list[Member], movable: list[Member]) -> dict[str, Box]:
    """Return a workbench with the assembled structure's footprint and a platform holding the stock."""
    built = np.vstack([footprint(m.target, m.size) for m in members])
    stock = np.vstack([footprint(m.staging, m.size) for m in movable])
    return {"workbench": table_box(built), "platform": table_box(stock, STOCK_MARGIN)}


PRESETS = {
    "portal": {
        "name": "Single portal",
        "site": [6.0, 3.65],
        "stock_distance": 2.0,
        "base": True,
        "posts": [{"y": -1.14, "height": 1.2}, {"y": 1.14, "height": 1.2}],
        "lintels": [{"between": [0, 1]}],
    },
    "double-portal": {
        "name": "Double portal",
        "site": [6.0, 3.65],
        "stock_distance": 2.0,
        "posts": [
            {"y": -1.1, "height": 1.0},
            {"y": 0.0, "height": 1.0},
            {"y": 1.1, "height": 1.0},
        ],
        "lintels": [
            {"between": [0, 1], "overhang": 0.06},
            {"between": [1, 2], "overhang": 0.06},
        ],
    },
    "skewed-portal": {
        "name": "Skewed portal",
        "site": [6.0, 3.65],
        "stock_distance": 2.0,
        "posts": [
            {"x": -0.35, "y": -0.9, "height": 1.1},
            {"x": 0.45, "y": 0.8, "height": 1.1},
        ],
        "lintels": [{"between": [0, 1], "overhang": 0.1}],
    },
}
