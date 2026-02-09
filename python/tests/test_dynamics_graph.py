"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_dynamics_graph.py
 * @brief Unit tests for dynamics graph.
 * @author Frank Dellaert, Varun Agrawal, Mandy Xie, Alejandro Escontrela, and Yetong Zhang
"""

import os.path as osp
import unittest

import gtdynamics as gtd
import gtsam
import numpy as np


class TestDynamicsGraph(unittest.TestCase):
    """Unit tests for DynamicsGraph."""
    def test_dynamics_graph(self):
        """Test construction of DynamicsGraph."""

        # load example robot
        SDF_PATH = osp.join(osp.dirname(osp.realpath(__file__)), "..", "..",
                            "models", "sdfs")
        simple_rr = gtd.CreateRobotFromFile(
            osp.join(SDF_PATH, "test", "simple_rr.sdf"), "simple_rr_sdf")

        # check links and joints
        self.assertEqual(simple_rr.numLinks(), 3)
        self.assertEqual(simple_rr.numJoints(), 2)
        self.assertEqual(len(simple_rr.links()), 3)
        self.assertEqual(simple_rr.links()[0].name(), "link_0")

        # test building dynamics graph
        graph_builder = gtd.DynamicsGraph()
        graph = graph_builder.dynamicsFactorGraph(simple_rr, 0, None, None)
        self.assertEqual(graph.size(), 13)

    def test_objective_factors(self):
        noise1 = gtsam.noiseModel.Unit.Create(1)
        noise6 = gtsam.noiseModel.Unit.Create(6)
        graph = gtsam.NonlinearFactorGraph()
        graph.push_back(gtd.LinkObjectives(1, k=777)\
            .pose(gtsam.Pose3(), noise6)\
            .twistAccel(np.zeros(6), noise6))
        self.assertEqual(graph.size(), 2)
        graph.push_back(gtd.JointObjectives(2, 777)\
            .angle(0., noise1)\
            .velocity(0., noise1))
        self.assertEqual(graph.size(), 4)
        self.assertEqual(graph.keys().size(), 4)
        graph.push_back(gtd.JointObjectives(2, 777)\
            .acceleration(0., noise1)\
            .angle(0., noise1))  # duplicate angle
        self.assertEqual(graph.size(), 6)
        self.assertEqual(graph.keys().size(), 5)
        # optional time index and noise model
        graph.push_back(gtd.JointObjectives(2)\
            .acceleration(0.)\
            .angle(0.))
        graph.push_back(gtd.LinkObjectives(1)\
            .pose(gtsam.Pose3())\
            .twistAccel(np.zeros(6)))
        self.assertEqual(graph.size(), 10)
        self.assertEqual(graph.keys().size(), 9)


if __name__ == "__main__":
    unittest.main()
