#!/usr/bin/env python3
# ----------------------------------------------------------------------------
# GTDynamics Copyright 2026, Georgia Tech Research Corporation,
# Atlanta, Georgia 30332-0415. All Rights Reserved.
# See LICENSE for the license information.
# ----------------------------------------------------------------------------

## @file cable_display.py
# @brief Drawn dress-cable shape: chord, network residual and rigid exit guides.
# @author GTDynamics contributors

"""Predict the drawn cable as the BAR Lab Grasshopper NN cable component does.

Ported from BAR-robot-cell ``procedural/rhino_tests/nn_cable.py``. The curve is
the chord between the anchors plus the network's link-1-frame residual. Nodes
inside the rigid 50 mm guide at each end are clamped to the straight line along
that end's exit normal.
"""

from __future__ import annotations

import gtdynamics as gtd
import numpy as np
from scipy.interpolate import CubicSpline

import cell
import planner as pl

GUIDE_SEGMENTS = 2
GUIDE_STEP = 0.025
LABEL_SEGMENTS = 58.0


class CableDisplay:
    """Evaluates each arm's learned cable from world link transforms."""

    def __init__(self):
        self.networks = {arm: gtd.MLP(str(pl.CABLE_WEIGHTS[arm])) for arm in (1, 2)}
        self.arc = {}
        for arm in (1, 2):
            metadata = pl.read_cable_metadata(pl.CABLE_WEIGHTS[arm])
            self.arc[arm] = np.array([float(v) for v in metadata["node_arc"]])

    def nodes(self, world: dict[str, np.ndarray], q: np.ndarray, arm: int) -> np.ndarray:
        """Return the (16, 3) world Chebyshev nodes of an arm's cable at configuration ``q``."""
        block = 9 * (arm - 1)
        residuals = np.asarray(self.networks[arm].forward(q[block + 4 : block + 9])).reshape(-1, 3)
        link_1, link_3, link_6 = (world[f"robot{arm}_link_{n}"] for n in (1, 3, 6))
        anchors = cell.CABLE_ANCHORS[arm]
        end_3 = link_3[:3, :3] @ anchors["link_3"] + link_3[:3, 3]
        end_6 = link_6[:3, :3] @ anchors["link_6"] + link_6[:3, 3]
        normal_3 = link_3[:3, :3] @ cell.CABLE_NORMALS["link_3"]
        normal_6 = link_6[:3, :3] @ cell.CABLE_NORMALS["link_6"]
        arc = self.arc[arm]
        out = np.zeros((len(arc), 3))
        for j, s in enumerate(arc):
            point = (1.0 - s) * end_3 + s * end_6
            if 0 < j < len(arc) - 1:
                point = point + link_1[:3, :3] @ residuals[j - 1]
                from_3, from_6 = s * LABEL_SEGMENTS, (1.0 - s) * LABEL_SEGMENTS
                if from_3 <= GUIDE_SEGMENTS:
                    point = end_3 + normal_3 * (from_3 * GUIDE_STEP)
                elif from_6 <= GUIDE_SEGMENTS:
                    point = end_6 + normal_6 * (from_6 * GUIDE_STEP)
            out[j] = point
        return out

    def samples(self, world: dict[str, np.ndarray], q: np.ndarray, arm: int, count: int) -> np.ndarray:
        """Return ``count`` world points along a cubic through the nodes, by chord length."""
        nodes = self.nodes(world, q, arm)
        chord = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(nodes, axis=0), axis=1))))
        return CubicSpline(chord, nodes)(np.linspace(0.0, chord[-1], count))
