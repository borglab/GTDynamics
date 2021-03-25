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


class TestDynamicsGraph(unittest.TestCase):
    """Unit tests for DynamicsGraph."""
    def test_dynamics_graph(self):
        """Test construction of DynamicsGraph."""

        # load example robot
        SDF_PATH = osp.join(osp.dirname(osp.realpath(__file__)), "..", "..",
                            "sdfs")
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


if __name__ == "__main__":
    unittest.main()
