#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2020, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415
# All Rights Reserved
# See LICENSE for the license information
# --------------------------------------------------------------------------
## @file pack_meshes.py
# @brief Pack a robot's collision STLs into a SCATE app's meshes.json.
# @author Harrison Jenkins

"""pack_meshes.py - write the meshes.json a SCATE app draws the arm with.

The viewer places each mesh in the URDF frame of the link the replay names
(replay.json "links"). This script reads the URDF's meshes, moves their
vertices into that frame (the mesh <origin>, plus any fixed joints between
the mesh's link and the drawn link, e.g. the UR10e's base_link_inertia under
base_link), and keys them by file name without extension ("upperarm", ...).

Two kinds of mesh:
  - collision (default): the <collision> STLs, simplified shapes, stored as
    base64 binary STL strings;
  - --visual: the <visual> COLLADA (.dae) meshes, with the robot's real
    shape and colors, stored per material as {"color", "positions" (float32),
    "indices" (uint16 or uint32, see "index_bytes")}, base64.

Usage:  python3 pack_meshes.py URDF REPLAY_JSON OUT_JSON [--visual]
Example (from apps/scate):
  python3 shared/scripts/pack_meshes.py ../../models/urdfs/ur10e/ur10e.urdf \\
      ur10e/public/replay.json ur10e/public/meshes.json --visual
"""

import argparse
import base64
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def origin_transform(element):
    """4x4 transform of a URDF <origin> element (identity if absent)."""
    xyz, rpy = [0.0] * 3, [0.0] * 3
    if element is not None:
        xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    rotation_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rotation_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    transform = np.eye(4)
    transform[:3, :3] = rotation_z @ rotation_y @ rotation_x
    transform[:3, 3] = xyz
    return transform


def read_stl_triangles(path):
    """Vertices of a binary STL as an (n, 3, 3) array."""
    data = Path(path).read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if 84 + 50 * count != len(data):
        raise ValueError(f"{path} is not a binary STL")
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    return np.frombuffer(data, dtype=record, count=count, offset=84)["vertices"].astype(float)


def write_stl(triangles):
    """Binary STL bytes; normals are left zero (the viewer recomputes them)."""
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    out = np.zeros(len(triangles), dtype=record)
    out["vertices"] = triangles
    return b"\0" * 80 + struct.pack("<I", len(triangles)) + out.tobytes()


def frame_in_drawn_link(link, parent_joint, drawn_links):
    """(drawn link, transform from `link` to it), walking up fixed joints only."""
    transform = np.eye(4)
    while link not in drawn_links:
        joint = parent_joint.get(link)
        if joint is None or joint.get("type") != "fixed":
            return None, None
        transform = origin_transform(joint.find("origin")) @ transform
        link = joint.find("parent").get("link")
    return link, transform


COLLADA = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def collada_parts(path):
    """Triangles of a COLLADA file grouped by material: {material: (color, vertices, normals)}.

    vertices and normals are (n, 3, 3) arrays in the file's scene frame (node
    matrices applied, scaled to meters).
    """
    root = ET.parse(path).getroot()
    unit = root.find("c:asset/c:unit", COLLADA)
    scale = float(unit.get("meter")) if unit is not None else 1.0
    colors = {}
    effects = {e.get("id"): e for e in root.findall(".//c:effect", COLLADA)}
    for material in root.findall(".//c:material", COLLADA):
        effect = effects[material.find("c:instance_effect", COLLADA).get("url")[1:]]
        diffuse = effect.find(".//c:diffuse/c:color", COLLADA)
        rgb = [float(x) for x in diffuse.text.split()[:3]] if diffuse is not None else [0.7] * 3
        colors[material.get("id")] = rgb

    geometries = {}
    for geometry in root.findall(".//c:geometry", COLLADA):
        mesh = geometry.find("c:mesh", COLLADA)
        sources = {
            source.get("id"): np.array(source.find("c:float_array", COLLADA).text.split(), float)
            for source in mesh.findall("c:source", COLLADA)
        }
        vertices = mesh.find("c:vertices", COLLADA)
        positions = sources[vertices.find('c:input[@semantic="POSITION"]', COLLADA).get("source")[1:]]
        positions = positions.reshape(-1, 3) * scale
        primitives = []
        for primitive in mesh.findall("c:triangles", COLLADA) + mesh.findall("c:polylist", COLLADA):
            vcount = primitive.find("c:vcount", COLLADA)
            if vcount is not None and set(vcount.text.split()) != {"3"}:
                raise ValueError(f"{path}: only triangles are supported")
            inputs = primitive.findall("c:input", COLLADA)
            stride = max(int(i.get("offset")) for i in inputs) + 1
            indices = np.array(primitive.find("c:p", COLLADA).text.split(), int).reshape(-1, stride)
            vertex_offset = next(int(i.get("offset")) for i in inputs if i.get("semantic") == "VERTEX")
            normal_input = next((i for i in inputs if i.get("semantic") == "NORMAL"), None)
            tri_vertices = positions[indices[:, vertex_offset]].reshape(-1, 3, 3)
            if normal_input is not None:
                normals = sources[normal_input.get("source")[1:]].reshape(-1, 3)
                tri_normals = normals[indices[:, int(normal_input.get("offset"))]].reshape(-1, 3, 3)
            else:
                face = np.cross(tri_vertices[:, 1] - tri_vertices[:, 0], tri_vertices[:, 2] - tri_vertices[:, 0])
                face /= np.linalg.norm(face, axis=1, keepdims=True) + 1e-12
                tri_normals = np.repeat(face[:, None], 3, axis=1)
            primitives.append((primitive.get("material"), tri_vertices, tri_normals))
        geometries[geometry.get("id")] = primitives

    parts = {}

    def walk(node, matrix):
        node_matrix = node.find("c:matrix", COLLADA)
        if node_matrix is not None:
            matrix = matrix @ np.array(node_matrix.text.split(), float).reshape(4, 4)
        for instance in node.findall("c:instance_geometry", COLLADA):
            bindings = {
                bound.get("symbol"): bound.get("target")[1:]
                for bound in instance.findall(".//c:instance_material", COLLADA)
            }
            for symbol, vertices, normals in geometries[instance.get("url")[1:]]:
                material = bindings.get(symbol, symbol)
                moved = vertices.reshape(-1, 3) @ matrix[:3, :3].T * 1.0 + matrix[:3, 3] * scale
                turned = normals.reshape(-1, 3) @ matrix[:3, :3].T
                color, old_v, old_n = parts.get(material, (colors.get(material, [0.7] * 3), [], []))
                old_v.append(moved.reshape(-1, 3, 3))
                old_n.append(turned.reshape(-1, 3, 3))
                parts[material] = (color, old_v, old_n)
        for child in node.findall("c:node", COLLADA):
            walk(child, matrix)

    for scene in root.findall(".//c:visual_scene", COLLADA):
        for node in scene.findall("c:node", COLLADA):
            walk(node, np.eye(4))
    return {m: (c, np.concatenate(v), np.concatenate(n)) for m, (c, v, n) in parts.items()}


def encode(array, dtype):
    """Base64 of an array's little-endian bytes."""
    return base64.b64encode(np.ascontiguousarray(array, dtype=dtype).tobytes()).decode("ascii")


def visual_parts(path, transform):
    """
    A COLLADA mesh as indexed per-material parts, moved by transform (4x4).
    Corners at the same position (to 10 micrometers) share a vertex; normals
    are not stored (the viewer rebuilds them with a crease angle, which keeps
    sharp edges sharp). Indices are uint16 when a part has under 65536
    vertices, else uint32 ("index_bytes").
    """
    parts = []
    for color, vertices, _ in collada_parts(path).values():
        flat_v = vertices.reshape(-1, 3) @ transform[:3, :3].T + transform[:3, 3]
        keys = np.round(flat_v * 1e5).astype(np.int64)
        _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
        index_bytes = 2 if len(first) < 65536 else 4
        parts.append({
            "color": [round(c, 4) for c in color],
            "positions": encode(flat_v[first], "<f4"),
            "indices": encode(inverse.reshape(-1), "<u2" if index_bytes == 2 else "<u4"),
            "index_bytes": index_bytes,
        })
    return parts


def pack(urdf_path, drawn_links, visual=False):
    """meshes.json content: every mesh, in the frame of its drawn link."""
    root = ET.parse(urdf_path).getroot()
    parent_joint = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    meshes, placement = {}, {}
    kind = "visual" if visual else "collision"
    for link in root.findall("link"):
        for element in link.findall(kind):
            mesh = element.find("geometry/mesh")
            if mesh is None:
                continue
            drawn, drawnTlink = frame_in_drawn_link(link.get("name"), parent_joint, drawn_links)
            if drawn is None:
                raise ValueError(f"link {link.get('name')} is not rigidly attached to a drawn link")
            transform = drawnTlink @ origin_transform(element.find("origin"))
            path = Path(urdf_path).parent / mesh.get("filename")
            name = Path(mesh.get("filename")).stem
            if visual:
                meshes[name] = {"parts": visual_parts(path, transform)}
            else:
                triangles = read_stl_triangles(path)
                moved = triangles @ transform[:3, :3].T + transform[:3, 3]
                meshes[name] = base64.b64encode(write_stl(moved)).decode("ascii")
            placement[name] = drawn
    return {"source": Path(urdf_path).name, "kind": kind, "frames": placement, "meshes": meshes}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("urdf")
    parser.add_argument("replay", help="replay.json whose 'links' are the drawn frames")
    parser.add_argument("out")
    parser.add_argument("--visual", action="store_true", help="pack the colored visual meshes")
    args = parser.parse_args()
    drawn_links = set(json.loads(Path(args.replay).read_text())["links"])
    packed = pack(args.urdf, drawn_links, visual=args.visual)
    Path(args.out).write_text(json.dumps(packed, separators=(",", ":")))
    print(f"wrote {args.out}: " + ", ".join(f"{k} -> {v}" for k, v in packed["frames"].items()))


if __name__ == "__main__":
    main()
