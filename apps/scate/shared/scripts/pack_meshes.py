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
(replay.json "links"). This script reads every <collision> mesh of the URDF,
moves its vertices into that frame (the collision <origin>, plus any fixed
joints between the mesh's link and the drawn link, e.g. the UR10e's
base_link_inertia under base_link), and stores the result as base64 binary
STL keyed by the STL's file name without extension ("upperarm", ...).

Usage:  python3 pack_meshes.py URDF REPLAY_JSON OUT_JSON
Example (from apps/scate):
  python3 shared/scripts/pack_meshes.py ../../models/urdfs/ur10e/ur10e.urdf \\
      ur10e/public/replay.json ur10e/public/meshes.json
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


def pack(urdf_path, drawn_links):
    """meshes.json content: every collision mesh, in the frame of its drawn link."""
    root = ET.parse(urdf_path).getroot()
    parent_joint = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    meshes, placement = {}, {}
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            drawn, drawnTlink = frame_in_drawn_link(link.get("name"), parent_joint, drawn_links)
            if drawn is None:
                raise ValueError(f"link {link.get('name')} is not rigidly attached to a drawn link")
            transform = drawnTlink @ origin_transform(collision.find("origin"))
            triangles = read_stl_triangles(Path(urdf_path).parent / mesh.get("filename"))
            moved = triangles @ transform[:3, :3].T + transform[:3, 3]
            name = Path(mesh.get("filename")).stem
            meshes[name] = base64.b64encode(write_stl(moved)).decode("ascii")
            placement[name] = drawn
    return {"source": Path(urdf_path).name, "frames": placement, "meshes": meshes}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("urdf")
    parser.add_argument("replay", help="replay.json whose 'links' are the drawn frames")
    parser.add_argument("out")
    args = parser.parse_args()
    drawn_links = set(json.loads(Path(args.replay).read_text())["links"])
    packed = pack(args.urdf, drawn_links)
    Path(args.out).write_text(json.dumps(packed, separators=(",", ":")))
    print(f"wrote {args.out}: " + ", ".join(f"{k} -> {v}" for k, v in packed["frames"].items()))


if __name__ == "__main__":
    main()
