"""
GTDynamics Copyright 2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

Unit tests for planning a trajectory around a spherical obstacle, mirroring
gtdynamics/gpmp2/tests/testObstacleFactors.cpp.
Author: Karthik Shaji
"""

import unittest
# pylint: disable=no-name-in-module, import-error, no-member
from pathlib import Path

import gtsam
import numpy as np
from gtsam.symbol_shorthand import V, X
from gtsam.utils.test_case import GtsamTestCase

import gtdynamics as gtd

CELL = 0.05
RADIUS = 0.15
EPSILON = 0.10
COST_SIGMA = 0.01

# The nine movable joints of robot1, in the order q indexes them: the three
# gantry prismatic joints, then the six arm revolute joints.
ROBOT1_JOINTS = [
    "bridge1_joint_EA_X", "robot1_joint_EA_Y", "robot1_joint_EA_Z",
    "robot1_joint_1", "robot1_joint_2", "robot1_joint_3", "robot1_joint_4",
    "robot1_joint_5", "robot1_joint_6"
]

Q_START = np.array([2.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0])
Q_GOAL = np.array([3.0, 2.0, 1.0, 0.0, -0.5, -1.0, 0.0, 0.5, 0.0])


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
    # Each distance carries the position it was sampled at, so the order the
    # columns arrive in cannot transpose the field.
    return gtd.SignedDistanceField(positions, distances)


class TestSignedDistanceField(GtsamTestCase):
    """Test the signed distance field built from positions and distances."""

    def setUp(self):
        self.center = np.array([0.5, 0.0, 1.25])
        self.origin = np.array([0.3, -0.2, 1.1])
        self.counts = (6, 5, 4)

    def test_interpolates_the_sphere(self):
        """Trilinear interpolation must recover the analytic distance."""
        sdf = sphere_sdf(self.center, RADIUS, self.origin, CELL, self.counts)
        self.assertEqual(sdf.xCount(), 6)
        self.assertEqual(sdf.yCount(), 5)
        self.assertEqual(sdf.zCount(), 4)
        self.assertAlmostEqual(sdf.cellSize(), CELL, places=9)
        np.testing.assert_allclose(sdf.origin(), self.origin, atol=1e-9)

        probe = self.center + np.array([0.5 * CELL, 0.5 * CELL, 0.5 * CELL])
        expected = np.linalg.norm(probe - self.center) - RADIUS
        self.assertAlmostEqual(sdf.getSignedDistance(probe), expected, places=2)

    def test_column_order_does_not_matter(self):
        """Shuffling the columns must give back an identical field."""
        positions = grid_positions(self.origin, CELL, self.counts)
        distances = np.linalg.norm(positions - self.center.reshape(3, 1),
                                   axis=0) - RADIUS

        ordered = gtd.SignedDistanceField(positions, distances)
        permutation = np.random.default_rng(42).permutation(positions.shape[1])
        shuffled = gtd.SignedDistanceField(positions[:, permutation],
                                           distances[permutation])
        self.assertTrue(ordered.equals(shuffled, 1e-9))

    def test_rejects_a_non_grid(self):
        """Positions that are not the nodes of a uniform cubic grid throw."""
        positions = grid_positions(self.origin, CELL, self.counts)
        distances = np.zeros(positions.shape[1])

        stretched = positions.copy()
        stretched[2, :] *= 2.0  # z spacing no longer matches x and y
        with self.assertRaises(ValueError):
            gtd.SignedDistanceField(stretched, distances)

        duplicated = positions.copy()
        duplicated[:, 1] = duplicated[:, 0]
        with self.assertRaises(ValueError):
            gtd.SignedDistanceField(duplicated, distances)

        with self.assertRaises(ValueError):
            gtd.SignedDistanceField(positions, distances[:-1])


class TestObstaclePlanning(GtsamTestCase):
    """Plan a trajectory for robot1 of bar_lab around a spherical obstacle."""

    def setUp(self):
        self.robot = gtd.CreateRobotFromFile(
            str(Path(gtd.URDF_PATH) / "bar_lab.urdf"))

        joints = [self.robot.joint(name) for name in ROBOT1_JOINTS]

        # Two query points on the wrist: the link CoM, and a point out along
        # the tool. Neither carries a radius; the standoff lives in epsilon.
        link6 = self.robot.link("robot1_link_6")
        points = gtd.PointOnLinks()
        points.append(gtd.PointOnLink(link6, np.array([0.0, 0.0, 0.0])))
        points.append(gtd.PointOnLink(link6, np.array([0.0, 0.0, 0.1])))

        # Only robot1's joints are given, so the bridge2 subtree is never
        # traversed and robot2 is absent from the model entirely.
        self.model = gtd.RobotQueryPoints(self.robot, "columns", joints, points)

    def clearance(self, q, center):
        """Distance from the closest query point at q to the sphere centre."""
        points = self.model.worldPoints(q)
        return np.linalg.norm(points - center.reshape(3, 1), axis=0).min()

    def test_query_points(self):
        """The model spans nine joints and carries two query points."""
        self.assertEqual(self.model.dof(), 9)
        self.assertEqual(self.model.nrPoints(), 2)
        self.assertEqual(self.model.worldPoints(Q_START).shape, (3, 2))

    def test_plan_around_sphere(self):
        """The trajectory bows around a sphere planted on its straight line."""
        dof, n_states = self.model.dof(), 5
        total_time = 2.0
        delta_t = total_time / (n_states - 1)
        velocity = (Q_GOAL - Q_START) / total_time

        # Straight line initialisation, and the wrist positions it sweeps.
        line = [
            Q_START + (k / (n_states - 1)) * (Q_GOAL - Q_START)
            for k in range(n_states)
        ]
        swept = np.hstack([self.model.worldPoints(q) for q in line])

        # Plant the sphere on the wrist at the midpoint of the straight line,
        # so the initialisation starts deep inside it.
        center = self.model.worldPoints(line[n_states // 2])[:, 0]

        # Size the grid to contain the whole swept path with room to detour. A
        # point that leaves the grid reads as free space.
        pad = 0.7
        origin = swept.min(axis=1) - pad
        extent = (swept.max(axis=1) + pad) - origin
        counts = np.ceil(extent / CELL).astype(int) + 1
        sdf = sphere_sdf(center, RADIUS, origin, CELL, counts)

        # The field really does describe the sphere we planted.
        outside = center + np.array([0.4, 0.0, 0.0])
        self.assertAlmostEqual(sdf.getSignedDistance(outside), 0.4 - RADIUS,
                               places=2)

        # The problem is only feasible if the pinned endpoints are already
        # clear, and only meaningful if the straight line is not.
        self.assertLess(self.clearance(line[n_states // 2], center),
                        RADIUS + EPSILON)
        self.assertGreater(self.clearance(Q_START, center), RADIUS + EPSILON)
        self.assertGreater(self.clearance(Q_GOAL, center), RADIUS + EPSILON)

        graph = gtsam.NonlinearFactorGraph()
        qc_model = gtsam.noiseModel.Isotropic.Sigma(dof, 1.0)
        endpoint_model = gtsam.noiseModel.Isotropic.Sigma(dof, 1e-4)

        graph.add(gtsam.PriorFactorVector(X(0), Q_START, endpoint_model))
        graph.add(gtsam.PriorFactorVector(V(0), np.zeros(dof), endpoint_model))
        graph.add(
            gtsam.PriorFactorVector(X(n_states - 1), Q_GOAL, endpoint_model))
        graph.add(
            gtsam.PriorFactorVector(V(n_states - 1), np.zeros(dof),
                                    endpoint_model))

        for k in range(n_states):
            graph.add(
                gtd.ObstacleSDFFactor(X(k), self.model, sdf, COST_SIGMA,
                                      EPSILON))
        # The interpolated obstacle cost would only be a Gaussian process
        # posterior mean because this prior, with the same Qc_model and
        # delta_t, joins the same support states.
        for k in range(n_states - 1):
            graph.add(
                gtd.GPLinearPrior(X(k), V(k), X(k + 1), V(k + 1), delta_t,
                                  qc_model))

        initial = gtsam.Values()
        for k in range(n_states):
            initial.insert(X(k), line[k])
            initial.insert(V(k), velocity)

        params = gtsam.LevenbergMarquardtParams()
        params.setMaxIterations(100)
        result = gtsam.LevenbergMarquardtOptimizer(graph, initial,
                                                   params).optimize()

        # The endpoints are held, and every query point at every support state
        # has been pushed clear of the sphere by at least epsilon. Checking
        # this against the analytic sphere rather than the sampled field means
        # a point that escaped the grid cannot pass by reading as free space.
        np.testing.assert_allclose(result.atVector(X(0)), Q_START, atol=1e-3)
        np.testing.assert_allclose(result.atVector(X(n_states - 1)), Q_GOAL,
                                   atol=1e-3)
        for k in range(n_states):
            points = self.model.worldPoints(result.atVector(X(k)))
            distances = np.linalg.norm(points - center.reshape(3, 1), axis=0)
            self.assertTrue(np.all(distances > RADIUS + EPSILON - 0.02))


if __name__ == "__main__":
    unittest.main()
