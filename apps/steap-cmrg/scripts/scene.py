#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file scene.py
# @brief Sphere-based obstacle models and signed distance fields for the CMRG planner.
# @author GTDynamics contributors

"""Floor and obstacle signed distance fields built from spheres.

Every obstacle is a union of equal-radius spheres, the same model used for the
arms, tools and carried plank: the tables and each placed 2x4. The field stores
the exact distance to the union on a uniform grid over the whole workspace,
which GTDynamics interpolates trilinearly. The obstacle cost fails closed off
the grid, so the grid must span every place the robot can reach. ``Box`` is only
a footprint used to size tables and to draw them in the browser.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import gtdynamics as gtd
import numpy as np
from scipy.spatial import cKDTree

FIELD_PITCH = 0.1
FLOOR_PITCH = 0.1
WORKSPACE = np.array([14.4, 7.3, 5.5])
CHAIN_SPACING = 0.05
CHAIN_MARGIN = 0.004
TABLE_SPHERE_RADIUS = 0.05


@dataclass(frozen=True)
class Box:
    """An axis-aligned footprint: centre and half extents (used to size and draw tables)."""

    center: np.ndarray
    half: np.ndarray

    @staticmethod
    def axis_aligned(center, half) -> "Box":
        """Return a box from its centre and half extents."""
        return Box(np.asarray(center, dtype=float), np.asarray(half, dtype=float))


@dataclass
class SphereSet:
    """Equal-radius obstacle spheres: world centres (N, 3) and one radius."""

    centers: np.ndarray
    radius: float
    tree: cKDTree = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.centers = np.asarray(self.centers, dtype=float).reshape(-1, 3)
        self.tree = cKDTree(self.centers)

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        """Return the distance from (N, 3) points to the union surface (negative inside)."""
        return self.tree.query(points)[0] - self.radius


def chain_spheres(size) -> tuple[np.ndarray, float]:
    """Return local sphere centres along a 2x4's length (x) and the shared bounding radius.

    ``size`` is (length, width, thickness). Each sphere bounds the cross section.
    """
    length, width, thickness = size
    radius = float(np.hypot(width / 2.0, thickness / 2.0) + CHAIN_MARGIN)
    count = max(2, int(round(length / CHAIN_SPACING)) + 1)
    xs = np.linspace(-length / 2.0, length / 2.0, count)
    return np.column_stack([xs, np.zeros(count), np.zeros(count)]), radius


def member_spheres(pose: np.ndarray, size) -> SphereSet:
    """Return a member's collision spheres at a 4x4 world pose."""
    local, radius = chain_spheres(size)
    return SphereSet(local @ pose[:3, :3].T + pose[:3, 3], radius)


def table_spheres(box: Box) -> SphereSet:
    """Return small spheres filling a table's volume, each touching but not crossing its faces."""
    radius = min(TABLE_SPHERE_RADIUS, float(np.min(box.half)))
    axes = []
    for center, half in zip(box.center, box.half):
        count = int(np.ceil(2.0 * (half - radius) / radius)) + 1
        axes.append(np.linspace(center - half + radius, center + half - radius, count))
    mesh = np.meshgrid(*axes, indexing="ij")
    return SphereSet(np.stack([axis.ravel() for axis in mesh], axis=1), radius)


def sphere_signed_distance(
    points: np.ndarray, obstacles: list[SphereSet]
) -> np.ndarray:
    """Return the distance from points to the nearest obstacle sphere surface."""
    return np.min([group.signed_distance(points) for group in obstacles], axis=0)


def grid_points(lower: np.ndarray, upper: np.ndarray, pitch: float) -> np.ndarray:
    """Return (N, 3) node positions of a uniform grid covering [lower, upper]."""
    counts = np.ceil((upper - lower) / pitch).astype(int) + 1
    axes = [lower[i] + pitch * np.arange(counts[i]) for i in range(3)]
    mesh = np.meshgrid(*axes, indexing="ij")
    return np.stack([axis.ravel() for axis in mesh], axis=1)


def make_field(points: np.ndarray, distances: np.ndarray) -> gtd.SignedDistanceField:
    """Wrap grid positions (N, 3) and distances (N,) in a GTDynamics field."""
    return gtd.SignedDistanceField(points.T, distances.reshape(-1, 1), 1e-6)


def floor_field() -> gtd.SignedDistanceField:
    """Return the floor plane field (distance is height above z = 0) over the workspace."""
    points = grid_points(np.zeros(3), WORKSPACE, FLOOR_PITCH)
    return make_field(points, points[:, 2].copy())


def sphere_field(
    obstacles: list[SphereSet], pitch: float = FIELD_PITCH
) -> gtd.SignedDistanceField | None:
    """Return the signed distance field of a union of sphere sets over the workspace, or None."""
    if not obstacles:
        return None
    points = grid_points(np.zeros(3), WORKSPACE, pitch)
    return make_field(points, sphere_signed_distance(points, obstacles))
