#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file urdf_display.py
# @brief Convert a URDF with STL meshes into the shared browser display geometry.
# @author GTDynamics contributors

"""Build browser display geometry and link poses from a URDF with STL meshes.

The output uses the same field names as ``examples/g1_mjlab/browser_geometry.py``
so that ``apps/shared/mujocoRobot.js::createRobotVisual`` can render it without
loading MuJoCo. Bodies are URDF link frames; quaternions in ``geom_quat`` are
``wxyz`` and saved body poses are ``xyzw``, matching the shared conventions.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

URDF_ROOT = Path(__file__).resolve().parents[3] / "models" / "urdfs"
MESH_TYPE = 7
DEFAULT_RGBA = (0.7, 0.7, 0.7, 1.0)


@dataclass
class Visual:
    """One mesh drawn in a link frame: STL path, pose (m, rad) and colour."""

    mesh: Path
    xyz: np.ndarray
    rpy: np.ndarray
    rgba: tuple[float, float, float, float]


@dataclass
class Joint:
    """A URDF joint: parent link, origin transform in the parent, and axis."""

    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


@dataclass
class UrdfModel:
    """Links in file order, their visuals, and the joints connecting them."""

    links: list[str]
    visuals: dict[str, list[Visual]]
    joints: list[Joint]
    _by_child: dict[str, Joint] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self._by_child = {joint.child: joint for joint in self.joints}


def rotation_from_rpy(rpy: np.ndarray) -> np.ndarray:
    """Return the 3x3 rotation for URDF roll-pitch-yaw (Rz @ Ry @ Rx), in rad."""
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the Rodrigues rotation about a unit axis by angle (rad)."""
    x, y, z = axis / np.linalg.norm(axis)
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit quaternion ``[x, y, z, w]``."""
    trace = np.trace(rotation)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k])
        q = np.zeros(4)
        q[i] = 0.25 * s
        q[j] = (rotation[j, i] + rotation[i, j]) / s
        q[k] = (rotation[k, i] + rotation[i, k]) / s
        w = (rotation[k, j] - rotation[j, k]) / s
        x, y, z = q[0], q[1], q[2]
    quaternion = np.array([x, y, z, w])
    return quaternion / np.linalg.norm(quaternion)


def transform(xyz: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """Return the 4x4 homogeneous transform with the given translation and rotation."""
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = xyz
    return matrix


def read_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read a binary STL into welded vertices (N, 3) and triangle indices (M, 3)."""
    data = Path(path).read_bytes()
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
    if len(data) != 84 + 50 * count:
        raise ValueError(f"{path} is not a binary STL")
    record = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    triangles = np.frombuffer(data, dtype=record, count=count, offset=84)["v"]
    vertices, faces = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    return vertices.astype(np.float32), faces.reshape(-1, 3).astype(np.int32)


def resolve_mesh(uri: str, root: Path = URDF_ROOT) -> Path:
    """Resolve ``package://pkg/path`` under the models/urdfs directory."""
    prefix = "package://"
    if not uri.startswith(prefix):
        raise ValueError(f"unsupported mesh URI: {uri}")
    return root / uri[len(prefix) :]


def _floats(text: str | None, default: str) -> np.ndarray:
    return np.array([float(value) for value in (text or default).split()])


def _origin(element: ET.Element | None) -> tuple[np.ndarray, np.ndarray]:
    if element is None:
        return np.zeros(3), np.zeros(3)
    return _floats(element.get("xyz"), "0 0 0"), _floats(element.get("rpy"), "0 0 0")


def parse_urdf(path: Path, root: Path = URDF_ROOT) -> UrdfModel:
    """Parse links, visual meshes with colours, and joints from a URDF file."""
    tree = ET.parse(path).getroot()
    colors = {}
    for material in tree.findall("material"):
        color = material.find("color")
        if color is not None:
            colors[material.get("name")] = tuple(
                _floats(color.get("rgba"), "0.7 0.7 0.7 1")
            )
    links, visuals = [], {}
    for link in tree.findall("link"):
        name = link.get("name")
        links.append(name)
        visuals[name] = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            xyz, rpy = _origin(visual.find("origin"))
            material = visual.find("material")
            rgba = DEFAULT_RGBA
            if material is not None:
                color = material.find("color")
                rgba = (
                    tuple(_floats(color.get("rgba"), "0.7 0.7 0.7 1"))
                    if color is not None
                    else colors.get(material.get("name"), DEFAULT_RGBA)
                )
            visuals[name].append(
                Visual(resolve_mesh(mesh.get("filename"), root), xyz, rpy, rgba)
            )
    joints = []
    for joint in tree.findall("joint"):
        xyz, rpy = _origin(joint.find("origin"))
        axis = joint.find("axis")
        joints.append(
            Joint(
                name=joint.get("name"),
                kind=joint.get("type"),
                parent=joint.find("parent").get("link"),
                child=joint.find("child").get("link"),
                origin=transform(xyz, rotation_from_rpy(rpy)),
                axis=_floats(axis.get("xyz") if axis is not None else None, "1 0 0"),
            )
        )
    return UrdfModel(links, visuals, joints)


def link_transforms(
    model: UrdfModel, joint_values: dict[str, float] | None = None
) -> dict[str, np.ndarray]:
    """Return world 4x4 link-frame transforms for joint values (m or rad by name)."""
    values = joint_values or {}
    world: dict[str, np.ndarray] = {}

    def solve(link: str) -> np.ndarray:
        if link in world:
            return world[link]
        joint = model._by_child.get(link)
        if joint is None:
            world[link] = np.eye(4)
            return world[link]
        value = values.get(joint.name, 0.0)
        motion = np.eye(4)
        if joint.kind in ("revolute", "continuous"):
            motion[:3, :3] = axis_angle_rotation(joint.axis, value)
        elif joint.kind == "prismatic":
            motion[:3, 3] = joint.axis / np.linalg.norm(joint.axis) * value
        world[link] = solve(joint.parent) @ joint.origin @ motion
        return world[link]

    for link in model.links:
        solve(link)
    return world


def display_geometry(
    model: UrdfModel,
    extra_visuals: dict[str, list[Visual]] | None = None,
) -> dict:
    """Return the shared ``createRobotVisual`` fields for the model's visuals.

    ``extra_visuals`` adds meshes to existing links, for example tools on tool0.
    """
    merged = {link: list(model.visuals[link]) for link in model.links}
    for link, extras in (extra_visuals or {}).items():
        merged[link].extend(extras)
    geometry = {name: [] for name in _GEOM_FIELDS}
    vertex_blocks, face_blocks = [], []
    vertex_count = face_count = 0
    mesh_cache: dict[Path, int] = {}
    for body, link in enumerate(model.links):
        for visual in merged[link]:
            if visual.mesh not in mesh_cache:
                vertices, faces = read_stl(visual.mesh)
                mesh_cache[visual.mesh] = len(mesh_cache)
                geometry["mesh_vertadr"].append(vertex_count)
                geometry["mesh_vertnum"].append(len(vertices))
                geometry["mesh_faceadr"].append(face_count)
                geometry["mesh_facenum"].append(len(faces))
                vertex_blocks.append(vertices.ravel())
                face_blocks.append(faces.ravel())
                vertex_count += len(vertices)
                face_count += len(faces)
            quaternion = matrix_to_quaternion_xyzw(rotation_from_rpy(visual.rpy))
            geometry["geom_group"].append(2)
            geometry["geom_type"].append(MESH_TYPE)
            geometry["geom_size"].extend([0.0, 0.0, 0.0])
            geometry["geom_dataid"].append(mesh_cache[visual.mesh])
            geometry["geom_matid"].append(-1)
            geometry["geom_rgba"].extend(visual.rgba)
            geometry["geom_pos"].extend(visual.xyz.tolist())
            geometry["geom_quat"].extend(
                [quaternion[3], quaternion[0], quaternion[1], quaternion[2]]
            )
            geometry["geom_bodyid"].append(body)
    geometry["mat_rgba"] = []
    geometry["mesh_vert"] = np.concatenate(vertex_blocks).round(5).tolist()
    geometry["mesh_face"] = np.concatenate(face_blocks).tolist()
    geometry["nbody"] = len(model.links)
    geometry["ngeom"] = len(geometry["geom_type"])
    return geometry


_GEOM_FIELDS = (
    "geom_group",
    "geom_type",
    "geom_size",
    "geom_dataid",
    "geom_matid",
    "geom_rgba",
    "geom_pos",
    "geom_quat",
    "geom_bodyid",
    "mesh_vertadr",
    "mesh_vertnum",
    "mesh_faceadr",
    "mesh_facenum",
)
