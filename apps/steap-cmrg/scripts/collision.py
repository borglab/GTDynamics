#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file collision.py
# @brief Sphere collision models for the CMRG arms, bridges, tools and carried wood.
# @author GTDynamics contributors

"""Fit collision spheres to the cell meshes and assemble GTDynamics query-point models.

Spheres are fitted along each collision mesh's long axis (``trimesh`` supplies
the mesh cross sections, a capability GTSAM and GTDynamics do not provide) and
stored in each link's CoM frame, the frame ``PointOnLink`` expects. The fitted
table is cached because the fit takes about a minute.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import gtdynamics as gtd
import gtsam
import numpy as np
import trimesh

import cell
import urdf_display as ud
from scene import chain_spheres

CACHE_FILE = Path(__file__).resolve().parents[1] / ".cache" / "arm_spheres.json"
SPHERE_MARGIN = 0.02
# Links whose spheres feed cross-arm and placed-block checks; the upper arm alone is held off the floor.
COLLISION_LINKS = ("link_1", "link_2", "link_3", "link_4", "link_5", "link_6",
                   "xy_cart", "robot1_base", "robot2_base")
FLOOR_LINKS = ("link_1", "link_2", "link_3")
CROSS_ARM_PAD = 0.1
BRIDGE_LINKS = {1: "bridge1", 2: "bridge2"}
BRIDGE_PAD = 0.2


def section_radius(mesh: trimesh.Trimesh, axis: int, other: list[int], position: float):
    """Return the max radius and centroid of the mesh cross section at ``position``."""
    normal, origin = np.zeros(3), np.zeros(3)
    normal[axis] = 1.0
    origin[axis] = position
    section = mesh.section(plane_origin=origin, plane_normal=normal)
    if section is None:
        return None, None
    points = section.vertices[:, other]
    centroid = points.mean(axis=0)
    return float(np.linalg.norm(points - centroid, axis=1).max()), centroid


def fit_mesh_spheres(mesh: trimesh.Trimesh, margin: float):
    """March spheres along a mesh's longest axis, overlapping through each cross section."""
    axis = int(np.argmax(mesh.extents))
    other = [i for i in range(3) if i != axis]
    low, high = mesh.bounds[0, axis], mesh.bounds[1, axis]
    slab = np.max(mesh.extents) / 50
    centers, radii = [], []
    position = low
    while position < high:
        radius, centroid = section_radius(mesh, axis, other, position)
        if radius is None:
            position += slab
            continue
        padded = radius + margin
        center = np.empty(3)
        center[axis] = position
        center[other] = centroid
        centers.append(center.tolist())
        radii.append(padded)
        position += 2 * np.sqrt(max(padded**2 - radius**2, 1e-9))
    return centers, radii


def collision_entries(prefixes: tuple[str, ...]) -> list[dict]:
    """Return ``{link, mesh, origin}`` for URDF links whose names start with a prefix."""
    root = ET.parse(cell.URDF_FILE).getroot()
    entries = []
    for link in root.findall("link"):
        name = link.get("name").lower()
        collision = link.find("collision")
        mesh = collision.find("geometry/mesh") if collision is not None else None
        if name.startswith(prefixes) and mesh is not None:
            origin = collision.find("origin")
            xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()]) if origin is not None else np.zeros(3)
            rpy = np.array([float(v) for v in origin.get("rpy", "0 0 0").split()]) if origin is not None else np.zeros(3)
            entries.append({"link": name, "mesh": ud.resolve_mesh(mesh.get("filename")),
                            "origin": ud.transform(xyz, ud.rotation_from_rpy(rpy))})
    return entries


def fit_arm_spheres(prefixes: tuple[str, ...]) -> dict[str, dict]:
    """Fit spheres to every collision mesh of an arm and express them in link CoM frames."""
    table = {}
    for entry in collision_entries(prefixes):
        centers, radii = fit_mesh_spheres(trimesh.load(entry["mesh"], force="mesh"), SPHERE_MARGIN)
        to_com = cell.link_centroid_offset(entry["link"]).inverse().matrix() @ entry["origin"]
        table[entry["link"]] = {
            "centers": [(to_com @ np.append(c, 1.0))[:3].tolist() for c in centers],
            "radii": radii,
        }
    return table


def arm_sphere_tables() -> dict[int, dict]:
    """Return the cached or freshly fitted sphere table of each arm and its bridge."""
    if CACHE_FILE.exists():
        raw = json.loads(CACHE_FILE.read_text())
        return {int(arm): table for arm, table in raw.items()}
    tables = {arm: fit_arm_spheres((f"robot{arm}_", f"bridge{arm}")) for arm in (1, 2)}
    CACHE_FILE.parent.mkdir(parents=True, exist_ok=True)
    CACHE_FILE.write_text(json.dumps(tables))
    return tables


def with_tool(table: dict[str, dict], arm: int) -> dict[str, dict]:
    """Return a copy of the arm's sphere table with its tool spheres on link_6."""
    copy = {k: {"centers": list(v["centers"]), "radii": list(v["radii"])} for k, v in table.items()}
    centers, radii = cell.tool_spheres_in_com(arm)
    entry = copy[f"robot{arm}_link_6"]
    entry["centers"] += [c.tolist() for c in centers]
    entry["radii"] += radii
    return copy


@dataclass
class CollisionModels:
    """Query-point models, radii and pairs consumed by the planner factors."""

    model: gtd.RobotQueryPoints
    radii: np.ndarray
    self_pairs: gtd.SelfCollisionPairs
    self_split: int
    floor_model: gtd.RobotQueryPoints
    floor_radii: np.ndarray
    floor_split: int
    arm_points: dict[int, list]
    arm_radii: dict[int, list]
    bridge_model: gtd.RobotQueryPoints
    bridge_radii: np.ndarray
    bridge_pairs: gtd.SelfCollisionPairs


def build_collision_models(robot: gtd.Robot, joint_names: list[str]) -> CollisionModels:
    """Assemble the two-arm sphere models, cross-arm pairs and bridge pairs."""
    tables = arm_sphere_tables()
    spheres = {arm: with_tool(tables[arm], arm) for arm in (1, 2)}
    joints = [robot.joint(name) for name in joint_names]
    points, floor_points = gtd.PointOnLinks(), gtd.PointOnLinks()
    radii, floor_radii, owner, floor_owner = [], [], [], []
    arm_points, arm_radii = {1: [], 2: []}, {1: [], 2: []}
    for arm in (1, 2):
        for link_name, sphere in spheres[arm].items():
            if not link_name.endswith(COLLISION_LINKS):
                continue
            try:
                link = robot.link(link_name)
            except RuntimeError:
                continue  # fixed-joint links lump into their parent
            on_floor = link_name.endswith(FLOOR_LINKS)
            for center, radius in zip(sphere["centers"], sphere["radii"]):
                center = np.asarray(center)
                points.append(gtd.PointOnLink(link, center))
                radii.append(radius)
                owner.append(arm)
                arm_points[arm].append((link, center))
                arm_radii[arm].append(radius)
                if on_floor:
                    floor_points.append(gtd.PointOnLink(link, center))
                    floor_radii.append(radius)
                    floor_owner.append(arm)
    pairs = gtd.SelfCollisionPairs()
    for a in range(len(owner)):
        for b in range(a + 1, len(owner)):
            if owner[a] != owner[b]:
                pairs.append(gtd.SelfCollisionPair(a, b, CROSS_ARM_PAD))
    bridge_model, bridge_radii, bridge_pairs = _bridge_model(robot, joints, tables)
    return CollisionModels(
        gtd.RobotQueryPoints(robot, cell.BASE_LINK, joints, points), np.array(radii), pairs,
        sum(1 for o in owner if o == 1),
        gtd.RobotQueryPoints(robot, cell.BASE_LINK, joints, floor_points), np.array(floor_radii),
        sum(1 for o in floor_owner if o == 1),
        arm_points, arm_radii, bridge_model, bridge_radii, bridge_pairs)


def _bridge_model(robot, joints, tables):
    points, radii, owner = gtd.PointOnLinks(), [], []
    for arm in (1, 2):
        sphere = tables[arm][BRIDGE_LINKS[arm]]
        link = robot.link(BRIDGE_LINKS[arm])
        for center, radius in zip(sphere["centers"], sphere["radii"]):
            points.append(gtd.PointOnLink(link, np.asarray(center)))
            radii.append(radius)
            owner.append(arm)
    pairs = gtd.SelfCollisionPairs()
    for a in range(len(owner)):
        for b in range(a + 1, len(owner)):
            if owner[a] != owner[b]:
                pairs.append(gtd.SelfCollisionPair(a, b, BRIDGE_PAD))
    return gtd.RobotQueryPoints(robot, cell.BASE_LINK, joints, points), np.array(radii), pairs


def carried_spheres(size: tuple[float, float, float]) -> tuple[np.ndarray, float]:
    """Return sphere centres along a member's length (member frame) and one shared radius.

    ``size`` is (length, width, thickness); the length runs along the member x axis.
    """
    return chain_spheres(size)


def carried_points(spec: cell.ArmSpec, size, grip_offset: np.ndarray):
    """Return carried-member spheres as (link, centre) tuples in the arm link CoM frame.

    ``grip_offset`` maps the member frame into the tool frame at the grip.
    """
    centers, radius = carried_spheres(size)
    carry = spec.tool_t.compose(cell.pose_from_matrix(grip_offset))
    points = [(spec.link, np.asarray(carry.transformFrom(gtsam.Point3(*c)))) for c in centers]
    return points, radius


def carried_model(robot, specs, points) -> gtd.RobotQueryPoints:
    """Return query points for a rigidly carried member."""
    carried = gtd.PointOnLinks()
    for link, center in points:
        carried.append(gtd.PointOnLink(link, center))
    joints = [robot.joint(name) for spec in specs for name in spec.q_names]
    return gtd.RobotQueryPoints(robot, cell.BASE_LINK, joints, carried)
