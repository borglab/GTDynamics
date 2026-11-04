# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file sole_support.py
# @brief Derive conservative sole support patches from canonical G1 geometry.
# @author GTDynamics contributors

"""Derive conservative G1 sole support rectangles from canonical capsule geometry."""

from __future__ import annotations

import gtsam
import mujoco
import numpy as np
from numpy.typing import NDArray

from g1_model import sole_frame


def planar_hull(points: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return counterclockwise vertices of a planar convex hull, in metres.

    Andrew's monotone chain removes duplicates and interior collinear points.
    The support-patch calculation needs only this two-dimensional boundary.
    """
    ordered = np.unique(points, axis=0)
    boundary = []
    for sequence in (ordered, ordered[::-1]):
        half = []
        for point in sequence:
            while len(half) >= 2:
                first, second = half[-1] - half[-2], point - half[-1]
                if first[0] * second[1] - first[1] * second[0] > 0:
                    break
                half.pop()
            half.append(point)
        boundary.extend(half[:-1])
    if len(boundary) < 3:
        raise ValueError("The support footprint must have nonzero planar area")
    return np.asarray(boundary)


def support_rectangle(model: mujoco.MjModel, side: str) -> tuple[float, float]:
    """Inscribed symmetric rectangle in the projected capsule support hull.

    A 5% margin avoids replacing rounded capsule footprints by an optimistic
    bounding box. This is a planar conservative support-patch assumption.
    """
    body = model.body(f"robot/{side}_ankle_roll_link").id
    origin = sole_frame(model, side)[:2, 3]
    points = []
    angles = np.linspace(0, 2 * np.pi, 128, endpoint=False)
    circle = np.column_stack([np.cos(angles), np.sin(angles)])
    for geom in range(model.ngeom):
        if (
            model.geom_bodyid[geom] != body
            or model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_CAPSULE
        ):
            continue
        axis = gtsam.Rot3.Quaternion(*model.geom_quat[geom]).matrix()[:, 2]
        for sign in (-1, 1):
            center = model.geom_pos[geom] + sign * axis * model.geom_size[geom, 1]
            points.extend(center[:2] - origin + circle * model.geom_size[geom, 0])
    points = np.asarray(points)
    half = np.minimum(-points.min(axis=0), points.max(axis=0))
    hull = planar_hull(points)
    edges = np.roll(hull, -1, axis=0) - hull
    normals = np.column_stack([edges[:, 1], -edges[:, 0]])
    # Each CCW hull edge gives an outward normal n and the bound n.x <= n.p.
    # A symmetric rectangle's support along n is abs(n) @ half.
    offsets = np.sum(normals * hull, axis=1)
    if np.any(offsets <= 0) or np.any(half <= 0):
        raise ValueError("The sole origin must lie inside the support footprint")
    scale = np.min(offsets / (np.abs(normals) @ half))
    return tuple((half * min(1.0, scale) * 0.95).tolist())
