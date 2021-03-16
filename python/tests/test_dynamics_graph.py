"""Unit tests for dynamics graph."""

import unittest

import numpy as np

import gtdynamics as gtd


class TestDynamicsGraph(unittest.TestCase):
    def test_dynamics_graph(self):
        """ Testing for DynamicsGraph. """

        # load example robot
        SDF_PATH = "sdfs/"
        simple_rr = gtd.CreateRobotFromFile(
            SDF_PATH + "/test/simple_rr.sdf", "simple_rr_sdf")

        # check links and joints
        self.assertEqual(simple_rr.numLinks(), 3)
        self.assertEqual(simple_rr.numJoints(), 2)
        self.assertEqual(len(simple_rr.links()), 3)
        self.assertEqual(simple_rr.links()[0].name(), "link_0")

        # test building dynamics graph
        graph_builder = gtd.DynamicsGraph()
        graph = graph_builder.dynamicsFactorGraph(
            simple_rr, 0, None, None, None, None)
        self.assertEqual(graph.size(), 13)


if __name__ == "__main__":
    unittest.main()
