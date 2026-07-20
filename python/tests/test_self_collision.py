"""
GTDynamics Copyright 2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

Self collision on the bar_lab platform: one factor over the full 18-DOF state
that keeps the two arms clear of each other. Mirrors
gtdynamics/gpmp2/tests/testSelfCollisionFactor.cpp.
Author: Karthik Shaji
"""

import unittest
# pylint: disable=no-name-in-module, import-error, no-member
from pathlib import Path

import gtsam
import numpy as np
from gtsam.symbol_shorthand import X
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd

# The eighteen joints of both arms, robot1's nine then robot2's nine.
BOTH_ARM_JOINTS = [
    "bridge1_joint_EA_X", "robot1_joint_EA_Y", "robot1_joint_EA_Z",
    "robot1_joint_1", "robot1_joint_2", "robot1_joint_3", "robot1_joint_4",
    "robot1_joint_5", "robot1_joint_6",
    "bridge2_joint_EA_X", "robot2_joint_EA_Y", "robot2_joint_EA_Z",
    "robot2_joint_1", "robot2_joint_2", "robot2_joint_3", "robot2_joint_4",
    "robot2_joint_5", "robot2_joint_6"
]

ARM = [0.2, -0.5, -1.0, 0.3, 0.5, 0.2]
# Bridges 0.2 m apart on the rail, so the two arm bases nearly coincide.
Q_CLOSE = np.array([5.0, 3.0, 1.0] + ARM + [5.2, 3.0, 1.0] + ARM)
EPSILON = 0.4


class TestSelfCollision(GtsamTestCase):
    """One 18-DOF factor keeping the two arm bases apart."""

    def setUp(self):
        self.robot = gtd.CreateRobotFromFile(
            str(Path(gtd.URDF_PATH) / "bar_lab.urdf"))
        joints = [self.robot.joint(name) for name in BOTH_ARM_JOINTS]

        # A query point on each arm's base; these move with the gantry only, so
        # the pair is reliably close when the bridges are.
        points = gtd.PointOnLinks()
        points.append(
            gtd.PointOnLink(self.robot.link("robot1_base"), np.zeros(3)))
        points.append(
            gtd.PointOnLink(self.robot.link("robot2_base"), np.zeros(3)))

        self.model = gtd.RobotQueryPoints(self.robot, "columns", joints, points)

    def make_factor(self, sigma=0.01):
        pairs = gtd.SelfCollisionPairs()
        pairs.append(gtd.SelfCollisionPair(0, 1, EPSILON))
        return gtd.SelfCollisionFactor(X(0), self.model, pairs, np.zeros(2),
                                       sigma)

    def separation(self, q):
        pts = self.model.worldPoints(q)
        return np.linalg.norm(pts[:, 0] - pts[:, 1])

    def test_construct(self):
        """The factor reports one pair over the 18-DOF state."""
        factor = self.make_factor()
        self.assertEqual(factor.nrPairs(), 1)
        self.assertEqual(self.model.dof(), 18)

    def test_pushes_arms_apart(self):
        """The bases start within epsilon; the factor drives them apart."""
        self.assertLess(self.separation(Q_CLOSE), EPSILON)  # in collision

        graph = gtsam.NonlinearFactorGraph()
        graph.add(self.make_factor())
        # A weak prior keeps the rest of the configuration near its start.
        graph.add(
            gtsam.PriorFactorVector(X(0), Q_CLOSE,
                                    gtsam.noiseModel.Isotropic.Sigma(18, 0.5)))

        init = gtsam.Values()
        init.insert(X(0), Q_CLOSE)
        result = gtsam.LevenbergMarquardtOptimizer(graph, init).optimize()

        self.assertGreater(self.separation(result.atVector(X(0))),
                           EPSILON - 0.02)

    def test_radii_inflate_the_points(self):
        """A clear pair becomes a collision once the points carry a radius."""
        d = self.separation(Q_CLOSE)
        eps = d - 0.05  # bases clear by 0.05 without any radius
        pairs = gtd.SelfCollisionPairs()
        pairs.append(gtd.SelfCollisionPair(0, 1, eps))

        values = gtsam.Values()
        values.insert(X(0), Q_CLOSE)
        clear = gtd.SelfCollisionFactor(X(0), self.model, pairs, np.zeros(2),
                                        0.1)
        inflated = gtd.SelfCollisionFactor(X(0), self.model, pairs,
                                           np.array([0.1, 0.1]), 0.1)

        self.assertAlmostEqual(clear.error(values), 0.0, places=9)
        self.assertGreater(inflated.error(values), 0.0)

    def test_rejects_bad_radii(self):
        """A radii vector of the wrong length is rejected."""
        pairs = gtd.SelfCollisionPairs()
        pairs.append(gtd.SelfCollisionPair(0, 1, EPSILON))
        with self.assertRaises(ValueError):
            gtd.SelfCollisionFactor(X(0), self.model, pairs, np.zeros(3), 0.01)


if __name__ == "__main__":
    unittest.main()
