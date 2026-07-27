"""
GTDynamics Copyright 2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

Simultaneous trajectory estimation and planning: a GPMP2 plan solved and then
updated with measurement factors through iSAM2. Mirrors
gtdynamics/gpmp2/tests/testSTEAP.cpp.
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
N_STATES = 5
DELTA_T = 0.5

# The nine movable joints of robot1, gantry prismatic then arm revolute.
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
    return gtd.SignedDistanceField(positions, distances)


def steap_params():
    """iSAM2 is non-linear, so a few updates reach the batch solution."""
    params = gtsam.ISAM2Params()
    params.setOptimizationParams(gtsam.ISAM2DoglegParams())
    params.setRelinearizeThreshold(0.01)
    params.relinearizeSkip = 1
    return params


def iterate(isam, iterations=10):
    """Drive iSAM2 to convergence and return the current estimate."""
    for _ in range(iterations):
        isam.update()
    return isam.calculateEstimate()


class TestSteap(GtsamTestCase):
    """A GPMP2 plan updated with measurement factors through iSAM2."""

    def setUp(self):
        self.robot = gtd.CreateRobotFromFile(
            str(Path(gtd.URDF_PATH) / "bar_lab.urdf"))
        joints = [self.robot.joint(name) for name in ROBOT1_JOINTS]

        link6 = self.robot.link("robot1_link_6")
        points = gtd.PointOnLinks()
        points.append(gtd.PointOnLink(link6, np.array([0.0, 0.0, 0.0])))
        points.append(gtd.PointOnLink(link6, np.array([0.0, 0.0, 0.1])))

        # Only robot1's joints are given, so ignore the bridge2 subtree.
        self.model = gtd.RobotQueryPoints(self.robot, "columns", joints, points)
        self.dof = self.model.dof()

        # Straight line initialisation, and the wrist positions it sweeps.
        self.line = [
            Q_START + (k / (N_STATES - 1)) * (Q_GOAL - Q_START)
            for k in range(N_STATES)
        ]
        swept = np.hstack([self.model.worldPoints(q) for q in self.line])

        # The sphere sits on the wrist at the midpoint of the straight line.
        self.center = self.model.worldPoints(self.line[N_STATES // 2])[:, 0]

        pad = 0.7
        origin = swept.min(axis=1) - pad
        extent = (swept.max(axis=1) + pad) - origin
        counts = np.ceil(extent / CELL).astype(int) + 1
        self.sdf = sphere_sdf(self.center, RADIUS, origin, CELL, counts)

    def build_plan_graph(self):
        """The GPMP2 graph, returning the position of the goal pose factor."""
        graph = gtsam.NonlinearFactorGraph()
        qc_model = gtsam.noiseModel.Isotropic.Sigma(self.dof, 1.0)
        fix = gtsam.noiseModel.Isotropic.Sigma(self.dof, 1e-4)

        graph.add(gtsam.PriorFactorVector(X(0), Q_START, fix))
        graph.add(gtsam.PriorFactorVector(V(0), np.zeros(self.dof), fix))
        goal_pos = graph.size()
        graph.add(gtsam.PriorFactorVector(X(N_STATES - 1), Q_GOAL, fix))
        graph.add(
            gtsam.PriorFactorVector(V(N_STATES - 1), np.zeros(self.dof), fix))

        for k in range(N_STATES):
            graph.add(
                gtd.ObstacleSDFFactor(X(k), self.model, self.sdf, COST_SIGMA,
                                      EPSILON))
        for k in range(N_STATES - 1):
            graph.add(
                gtd.GPLinearPrior(X(k), V(k), X(k + 1), V(k + 1), DELTA_T,
                                  qc_model))
        return graph, goal_pos

    def straight_line_init(self):
        velocity = (Q_GOAL - Q_START) / (N_STATES * DELTA_T)
        init = gtsam.Values()
        for k in range(N_STATES):
            init.insert(X(k), self.line[k])
            init.insert(V(k), velocity)
        return init

    def test_initial_plan_is_collision_free(self):
        """Before any measurement, STEAP is exactly the GPMP2 plan."""
        graph, _ = self.build_plan_graph()
        isam = gtsam.ISAM2(steap_params())
        isam.update(graph, self.straight_line_init())
        result = iterate(isam)

        np.testing.assert_allclose(result.atVector(X(0)), Q_START, atol=1e-3)
        np.testing.assert_allclose(result.atVector(X(N_STATES - 1)), Q_GOAL,
                                   atol=1e-3)
        for k in range(N_STATES):
            points = self.model.worldPoints(result.atVector(X(k)))
            distances = np.linalg.norm(points - self.center.reshape(3, 1),
                                       axis=0)
            self.assertTrue(np.all(distances > RADIUS + EPSILON - 0.02))

    def test_measurement_updates_trajectory(self):
        """A measurement factor pulls an interior state, endpoints held."""
        graph, _ = self.build_plan_graph()
        isam = gtsam.ISAM2(steap_params())
        isam.update(graph, self.straight_line_init())
        plan = iterate(isam)

        step = N_STATES // 2
        # A noisy execution landed the robot a third of a metre further along
        # the rail than planned, a collision-free config the sensor reports.
        measured = plan.atVector(X(step)).copy()
        measured[0] += 0.35

        measurement = gtsam.NonlinearFactorGraph()
        measurement.add(
            gtsam.PriorFactorVector(
                X(step), measured,
                gtsam.noiseModel.Isotropic.Sigma(self.dof, 1e-4)))
        isam.update(measurement, gtsam.Values())
        updated = iterate(isam)

        # The tight measurement dominates, so the state snaps onto it.
        np.testing.assert_allclose(updated.atVector(X(step)), measured,
                                   atol=3e-2)
        # The endpoints are untouched by an interior measurement.
        np.testing.assert_allclose(updated.atVector(X(0)), Q_START, atol=1e-3)
        np.testing.assert_allclose(updated.atVector(X(N_STATES - 1)), Q_GOAL,
                                   atol=1e-3)

    def test_goal_factor_swapped_for_measurement(self):
        """At the end, the goal factor is replaced by a pose measurement."""
        graph, goal_pos = self.build_plan_graph()
        isam = gtsam.ISAM2(steap_params())
        first = isam.update(graph, self.straight_line_init())
        iterate(isam)
        goal_index = first.getNewFactorsIndices()[goal_pos]

        # The robot arrived a little short of the planned goal.
        arrived = Q_GOAL.copy()
        arrived[0] -= 0.2

        measurement = gtsam.NonlinearFactorGraph()
        measurement.add(
            gtsam.PriorFactorVector(
                X(N_STATES - 1), arrived,
                gtsam.noiseModel.Isotropic.Sigma(self.dof, 1e-4)))
        isam.update(measurement, gtsam.Values(), [goal_index])
        updated = iterate(isam)

        # With the goal factor gone, the final state follows the measurement.
        np.testing.assert_allclose(updated.atVector(X(N_STATES - 1)), arrived,
                                   atol=3e-2)
        np.testing.assert_allclose(updated.atVector(X(0)), Q_START, atol=1e-3)


if __name__ == "__main__":
    unittest.main()
