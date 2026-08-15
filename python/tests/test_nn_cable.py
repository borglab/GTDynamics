"""
GTDynamics Copyright 2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

Neural-network cable factor on the bar_lab platform: an MLP loaded from a
weight file predicts the cable shape between two robot1 links, and one factor
keeps the sampled cable clear of a sphere obstacle. Mirrors
gtdynamics/gpmp2/tests/testNNCableFactor.cpp.
Author: Karthik Shaji
"""

import os
import tempfile
import unittest
# pylint: disable=no-name-in-module, import-error, no-member
from pathlib import Path

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd

# The nine movable joints of robot1, gantry prismatic then arm revolute.
ROBOT1_JOINTS = [
    "bridge1_joint_EA_X", "robot1_joint_EA_Y", "robot1_joint_EA_Z",
    "robot1_joint_1", "robot1_joint_2", "robot1_joint_3", "robot1_joint_4",
    "robot1_joint_5", "robot1_joint_6"
]

Q_START = np.array([2.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0])

# The network reads the five distal arm joints, as the trained model does.
INPUT_INDICES = [4, 5, 6, 7, 8]
CHEB_NODES = 6
NUM_SAMPLES = 9
EPSILON = 0.10
CABLE_RADIUS = 0.02


def write_zero_weight_file(path, bias):
    """A 5 -> 4 -> len(bias) network with zero weights and the given bias."""
    n_out = len(bias)
    lines = [
        "# synthetic test model, row major",
        "input_dim 5",
        "hidden_dims 4",
        "activation relu",
        f"output_dim {n_out}",
        f"cheb_nodes {CHEB_NODES}",
        "layers 2",
        "layer0_weight 4 5",
    ]
    lines += ["0"] * 20
    lines += ["layer0_bias 4"] + ["0"] * 4
    lines += [f"layer1_weight {n_out} 4"] + ["0"] * (n_out * 4)
    lines += [f"layer1_bias {n_out}"] + [repr(value) for value in bias]
    with open(path, "w", encoding="ascii") as handle:
        handle.write("\n".join(lines) + "\n")


def grid_positions(origin, cell, counts):
    """Return the 3 x N node positions of a uniform grid."""
    axes = [origin[i] + cell * np.arange(counts[i]) for i in range(3)]
    grid = np.meshgrid(*axes, indexing="ij")
    return np.vstack([axis.ravel() for axis in grid])


def sphere_sdf(center, radius, origin, cell, counts):
    """Sample the exact signed distance to a sphere onto a uniform grid."""
    positions = grid_positions(origin, cell, counts)
    distances = np.linalg.norm(positions - center.reshape(3, 1),
                               axis=0) - radius
    return gtd.SignedDistanceField(positions, distances)


class TestNNCable(GtsamTestCase):
    """An MLP-predicted cable between robot1's link 3 and link 6."""

    def setUp(self):
        self.robot = gtd.CreateRobotFromFile(
            str(Path(gtd.URDF_PATH) / "bar_lab.urdf"))
        self.joints = [self.robot.joint(name) for name in ROBOT1_JOINTS]

        n_out = 3 * (CHEB_NODES - 2)
        self.weight_file = os.path.join(tempfile.gettempdir(),
                                        "gtd_test_nn_cable.txt")
        write_zero_weight_file(self.weight_file, np.zeros(n_out))
        self.addCleanup(os.remove, self.weight_file)
        self.mlp = gtd.MLP(self.weight_file)

        self.attachment0 = gtd.PointOnLink(self.robot.link("robot1_link_3"),
                                           np.array([0.1, 0.0, 0.05]))
        self.attachment1 = gtd.PointOnLink(self.robot.link("robot1_link_6"),
                                           np.array([0.0, 0.0, 0.1]))
        self.cable = gtd.NNCableSpline(self.robot, "columns", self.joints,
                                       self.attachment0, self.attachment1,
                                       self.robot.link("robot1_link_1"),
                                       self.mlp, INPUT_INDICES, CHEB_NODES,
                                       NUM_SAMPLES)

    def test_mlp(self):
        """The loaded network has the declared shape and runs on raw inputs."""
        self.assertEqual(self.mlp.inputDim(), 5)
        self.assertEqual(self.mlp.outputDim(), 3 * (CHEB_NODES - 2))
        self.assertEqual(self.mlp.nrLayers(), 2)
        np.testing.assert_allclose(self.mlp.forward(np.ones(5)),
                                   np.zeros(3 * (CHEB_NODES - 2)), atol=1e-12)

    def test_chord_when_residual_zero(self):
        """With zero residuals every sample lies on the endpoint chord."""
        self.assertEqual(self.cable.dof(), 9)
        self.assertEqual(self.cable.numSamples(), NUM_SAMPLES)

        points = gtd.PointOnLinks()
        points.append(self.attachment0)
        points.append(self.attachment1)
        endpoints = gtd.RobotQueryPoints(self.robot, "columns", self.joints,
                                         points).worldPoints(Q_START)

        pts = self.cable.worldPoints(Q_START)
        self.assertEqual(pts.shape, (3, NUM_SAMPLES))
        for m in range(NUM_SAMPLES):
            s = m / (NUM_SAMPLES - 1)
            chord = (1.0 - s) * endpoints[:, 0] + s * endpoints[:, 1]
            np.testing.assert_allclose(pts[:, m], chord, atol=1e-9)

    def test_factor_pushes_cable_off_obstacle(self):
        """A sphere on the cable is in collision; optimizing clears it."""
        pts = self.cable.worldPoints(Q_START)
        center = pts[:, NUM_SAMPLES // 2] + np.array([0.015, 0.01, 0.0])
        origin = center - 1.0 + 0.5 * 0.05
        sdf = sphere_sdf(center, 0.15, origin, 0.05, (41, 41, 41))

        factor = gtd.NNCableFactor(X(0), self.cable, sdf, 0.01, EPSILON,
                                   CABLE_RADIUS)
        values = gtsam.Values()
        values.insert(X(0), Q_START)
        initial_error = factor.error(values)
        self.assertGreater(initial_error, 0.0)

        graph = gtsam.NonlinearFactorGraph()
        graph.add(factor)
        # A weak prior keeps the configuration near its start.
        graph.add(
            gtsam.PriorFactorVector(X(0), Q_START,
                                    gtsam.noiseModel.Isotropic.Sigma(9, 0.5)))
        result = gtsam.LevenbergMarquardtOptimizer(graph, values).optimize()

        final = gtsam.Values()
        final.insert(X(0), result.atVector(X(0)))
        self.assertLess(factor.error(final), initial_error)

    def test_rejects_bad_radii(self):
        """A radii vector of the wrong length is rejected."""
        pts = self.cable.worldPoints(Q_START)
        center = pts[:, NUM_SAMPLES // 2]
        sdf = sphere_sdf(center, 0.15, center - 1.0, 0.05, (41, 41, 41))
        with self.assertRaises(ValueError):
            gtd.NNCableFactor(X(0), self.cable, sdf, 0.01, EPSILON,
                              np.zeros(NUM_SAMPLES - 1))


if __name__ == "__main__":
    unittest.main()
