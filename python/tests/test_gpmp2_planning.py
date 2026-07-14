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

# Joint angle and velocity limits of robot1, in the order q indexes them.
Q_LOWER = np.array(
    [0.0, 0.0, 0.0, -2.96706, -1.13446, -3.14159, -5.23599, -2.26893, -5.23599])
Q_UPPER = np.array(
    [13.0, 7.3, 1.75, 2.96706, 2.44346, 1.22173, 5.23599, 2.26893, 5.23599])
V_LIMIT = np.array(
    [2.618, 2.618, 2.618, 2.618, 2.618, 2.618, 6.2832, 6.2832, 7.854])
LIMIT_THRESH = np.full(9, 1e-3)


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


def spheres_sdf(centers, radius, origin, cell, counts):
    """Signed distance to the nearest of several spheres, on a uniform grid."""
    positions = grid_positions(origin, cell, counts)
    distances = np.min([
        np.linalg.norm(positions - c.reshape(3, 1), axis=0) - radius
        for c in centers
    ], axis=0)
    return gtd.SignedDistanceField(positions, distances)


# All eighteen joints of bar_lab, robot1's nine then robot2's nine. The tree
# branches at the shared rail, so one model spanning all of them reaches both
# arms from the same base.
BOTH_ARM_JOINTS = ROBOT1_JOINTS + [
    "bridge2_joint_EA_X", "robot2_joint_EA_Y", "robot2_joint_EA_Z",
    "robot2_joint_1", "robot2_joint_2", "robot2_joint_3", "robot2_joint_4",
    "robot2_joint_5", "robot2_joint_6"
]

# The two arms sit on opposite sides of the rail, robot1 near x = 2 and robot2
# near x = 9, so their workspaces are disjoint and no arm-vs-arm cost is needed.
_ARM = [0.0, -0.5, -1.0, 0.0, 0.5, 0.0]
Q_START_18 = np.array([2.0, 2.0, 1.0] + _ARM + [9.0, 3.0, 1.0] + _ARM)
Q_GOAL_18 = np.array([3.0, 2.0, 1.0] + _ARM + [10.0, 3.0, 1.0] + _ARM)


class TestSignedDistanceField(GtsamTestCase):
    """Test the signed distance field built from positions and distances."""

    def setUp(self):
        self.center = np.array([0.5, 0.0, 1.25])
        # Counts differ per axis to catch a transposed field, and the grid is
        # centred on the sphere so a probe near its surface stays in range.
        self.counts = (13, 11, 9)
        self.origin = self.center - CELL * (np.array(self.counts) - 1) / 2.0

    def test_interpolates_the_sphere(self):
        """Trilinear interpolation must recover the analytic distance."""
        sdf = sphere_sdf(self.center, RADIUS, self.origin, CELL, self.counts)
        self.assertEqual(sdf.xCount(), 13)
        self.assertEqual(sdf.yCount(), 11)
        self.assertEqual(sdf.zCount(), 9)
        self.assertAlmostEqual(sdf.cellSize(), CELL, places=9)
        np.testing.assert_allclose(sdf.origin(), self.origin, atol=1e-9)

        probe = self.center + np.array([0.25, 0.0, 0.0])
        self.assertAlmostEqual(sdf.getSignedDistance(probe), 0.25 - RADIUS,
                               places=2)

    def test_rejects_a_query_outside_the_grid(self):
        """A point beyond the grid is out of range, not silently free space."""
        sdf = sphere_sdf(self.center, RADIUS, self.origin, CELL, self.counts)
        with self.assertRaises(RuntimeError):
            sdf.getSignedDistance(self.center + np.array([10.0, 0.0, 0.0]))

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

        # Only robot1's joints are given, so ignore bridge2 subtree
        self.model = gtd.RobotQueryPoints(self.robot, "columns", joints, points)

    def clearance(self, q, center):
        """Distance from the closest query point at q to the sphere centre."""
        points = self.model.worldPoints(q)
        return np.linalg.norm(points - center.reshape(3, 1), axis=0).min()

    def sphere_problem(self, n_states):
        """Straight line initialisation, and a sphere planted on its midpoint."""
        line = [
            Q_START + (k / (n_states - 1)) * (Q_GOAL - Q_START)
            for k in range(n_states)
        ]
        swept = np.hstack([self.model.worldPoints(q) for q in line])

        # The sphere sits on the wrist at the midpoint of the straight line, so
        # the initialisation starts deep inside it.
        center = self.model.worldPoints(line[n_states // 2])[:, 0]

        # The grid holds the whole swept path with room to detour, since a
        # point that leaves it reads as free space.
        pad = 0.7
        origin = swept.min(axis=1) - pad
        extent = (swept.max(axis=1) + pad) - origin
        counts = np.ceil(extent / CELL).astype(int) + 1
        sdf = sphere_sdf(center, RADIUS, origin, CELL, counts)

        # The problem is only feasible if the pinned endpoints are already
        # clear, and only meaningful if the straight line is not.
        self.assertLess(self.clearance(line[n_states // 2], center),
                        RADIUS + EPSILON)
        self.assertGreater(self.clearance(Q_START, center), RADIUS + EPSILON)
        self.assertGreater(self.clearance(Q_GOAL, center), RADIUS + EPSILON)
        return line, center, sdf

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

        line, center, sdf = self.sphere_problem(n_states)

        # The field really does describe the sphere we planted.
        outside = center + np.array([0.4, 0.0, 0.0])
        self.assertAlmostEqual(sdf.getSignedDistance(outside), 0.4 - RADIUS,
                               places=2)

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

    def test_plan_under_limits(self):
        """The same detour, now respecting joint angle and velocity limits."""
        dof, n_states = self.model.dof(), 5
        total_time = 2.0
        delta_t = total_time / (n_states - 1)
        velocity = (Q_GOAL - Q_START) / total_time

        line, center, sdf = self.sphere_problem(n_states)

        # A minimum acceleration move of one metre in two seconds, starting and
        # ending at rest, peaks at 0.75 m/s, so the gantry's velocity limit is
        # tightened below that to make the velocity factor bind.
        v_limit = V_LIMIT.copy()
        v_limit[0] = 0.7

        graph = gtsam.NonlinearFactorGraph()
        qc_model = gtsam.noiseModel.Isotropic.Sigma(dof, 1.0)
        endpoint_model = gtsam.noiseModel.Isotropic.Sigma(dof, 1e-4)
        limit_model = gtsam.noiseModel.Isotropic.Sigma(dof, 1e-3)

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
            graph.add(
                gtd.JointLimitFactorVector(X(k), limit_model, Q_LOWER, Q_UPPER,
                                           LIMIT_THRESH))
            graph.add(
                gtd.VelocityLimitFactorVector(V(k), limit_model, v_limit,
                                              LIMIT_THRESH))
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

        np.testing.assert_allclose(result.atVector(X(0)), Q_START, atol=1e-3)
        np.testing.assert_allclose(result.atVector(X(n_states - 1)), Q_GOAL,
                                   atol=1e-3)

        for k in range(n_states):
            q, v = result.atVector(X(k)), result.atVector(V(k))
            self.assertTrue(np.all(q > Q_LOWER - 0.02))
            self.assertTrue(np.all(q < Q_UPPER + 0.02))
            self.assertTrue(np.all(np.abs(v) < v_limit + 0.02))

            points = self.model.worldPoints(q)
            distances = np.linalg.norm(points - center.reshape(3, 1), axis=0)
            self.assertTrue(np.all(distances > RADIUS + EPSILON - 0.02))

        # The gantry velocity limit is below the unconstrained peak, so it had
        # to bind: without it the midpoint would run at about 0.75 m/s.
        self.assertGreater(abs(result.atVector(V(n_states // 2))[0]), 0.3)


class TestPlanning18Dof(GtsamTestCase):
    """Plan both arms of bar_lab at once on a single 18-DOF factor graph."""

    def setUp(self):
        self.robot = gtd.CreateRobotFromFile(
            str(Path(gtd.URDF_PATH) / "bar_lab.urdf"))
        joints = [self.robot.joint(name) for name in BOTH_ARM_JOINTS]

        # One query point on each arm's wrist.
        points = gtd.PointOnLinks()
        points.append(
            gtd.PointOnLink(self.robot.link("robot1_link_6"), np.zeros(3)))
        points.append(
            gtd.PointOnLink(self.robot.link("robot2_link_6"), np.zeros(3)))

        self.model = gtd.RobotQueryPoints(self.robot, "columns", joints, points)
        self.dof = self.model.dof()

    def clearances(self, q, centers):
        """Distance from each wrist to its own sphere centre."""
        points = self.model.worldPoints(q)
        return np.array([
            np.linalg.norm(points[:, i] - centers[i])
            for i in range(len(centers))
        ])

    def test_query_points(self):
        """The model spans eighteen joints and carries two query points."""
        self.assertEqual(self.model.dof(), 18)
        self.assertEqual(self.model.nrPoints(), 2)
        self.assertEqual(self.model.worldPoints(Q_START_18).shape, (3, 2))

    def test_plan_both_arms(self):
        """Both arms detour their own sphere in a single optimisation."""
        dof, n_states = self.dof, 5
        total_time = 2.0
        delta_t = total_time / (n_states - 1)
        velocity = (Q_GOAL_18 - Q_START_18) / total_time

        line = [
            Q_START_18 + (k / (n_states - 1)) * (Q_GOAL_18 - Q_START_18)
            for k in range(n_states)
        ]
        swept = np.hstack([self.model.worldPoints(q) for q in line])

        # One sphere on each arm's wrist at the midpoint of its straight line.
        mid = self.model.worldPoints(line[n_states // 2])
        centers = [mid[:, 0], mid[:, 1]]

        pad = 0.7
        origin = swept.min(axis=1) - pad
        extent = (swept.max(axis=1) + pad) - origin
        counts = np.ceil(extent / CELL).astype(int) + 1
        sdf = spheres_sdf(centers, RADIUS, origin, CELL, counts)

        # Both arms start in collision at the midpoint, both endpoints clear.
        self.assertTrue(
            np.all(self.clearances(line[n_states // 2], centers) < RADIUS +
                   EPSILON))
        self.assertTrue(
            np.all(self.clearances(Q_START_18, centers) > RADIUS + EPSILON))
        self.assertTrue(
            np.all(self.clearances(Q_GOAL_18, centers) > RADIUS + EPSILON))

        graph = gtsam.NonlinearFactorGraph()
        qc_model = gtsam.noiseModel.Isotropic.Sigma(dof, 1.0)
        endpoint_model = gtsam.noiseModel.Isotropic.Sigma(dof, 1e-4)

        graph.add(gtsam.PriorFactorVector(X(0), Q_START_18, endpoint_model))
        graph.add(gtsam.PriorFactorVector(V(0), np.zeros(dof), endpoint_model))
        graph.add(
            gtsam.PriorFactorVector(X(n_states - 1), Q_GOAL_18, endpoint_model))
        graph.add(
            gtsam.PriorFactorVector(V(n_states - 1), np.zeros(dof),
                                    endpoint_model))
        for k in range(n_states):
            graph.add(
                gtd.ObstacleSDFFactor(X(k), self.model, sdf, COST_SIGMA,
                                      EPSILON))
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

        np.testing.assert_allclose(result.atVector(X(0)), Q_START_18, atol=1e-3)
        np.testing.assert_allclose(result.atVector(X(n_states - 1)), Q_GOAL_18,
                                   atol=1e-3)

        # Each arm's wrist has been pushed clear of its own sphere at every
        # state, checked against the analytic spheres.
        for k in range(n_states):
            clear = self.clearances(result.atVector(X(k)), centers)
            self.assertTrue(np.all(clear > RADIUS + EPSILON - 0.02))


if __name__ == "__main__":
    unittest.main()
