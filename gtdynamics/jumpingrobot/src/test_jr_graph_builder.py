"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_jr_graph_builder.py
 * @brief Unit test for jumping robot graph builder.
 * @author Yetong Zhang
"""

import inspect
import os.path as osp
import sys
import unittest

import gtdynamics as gtd
import gtsam
import numpy as np

currentdir = osp.dirname(osp.abspath(inspect.getfile(inspect.currentframe())))
parentdir = osp.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.actuation_graph_builder import ActuationGraphBuilder
from src.jr_graph_builder import JRGraphBuilder
from src.jumping_robot import Actuator, JumpingRobot
from src.robot_graph_builder import RobotGraphBuilder


class TestJRGraphBuilder(unittest.TestCase):
    """ Tests for JRGraphBuilder. """
    def setUp(self):
        """ Set up the graph builder and jumping robot. """
        self.yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
        self.init_config = JumpingRobot.create_init_config()
        self.jr = JumpingRobot(self.yaml_file_path, self.init_config)
        self.jr_graph_builder = JRGraphBuilder()

    def test_collocation_graph_size(self):
        """ Test graph sizes. """
        step_phases = [0, 0, 3, 3]

        # collocation on mass: (4 + 1) * 4
        actuator_graph_builder = self.jr_graph_builder.actuation_graph_builder
        graph_actuation_col = actuator_graph_builder.collocation_graph(
            self.jr, step_phases)
        self.assertEqual(graph_actuation_col.size(), 20)

        # collocation on joints: 4 * 2 * 2
        # collocation on torso: 2 * 4
        robot_graph_builder = self.jr_graph_builder.robot_graph_builder
        graph_robot_col = robot_graph_builder.collocation_graph(
            self.jr, step_phases)
        self.assertEqual(graph_robot_col.size(), 24)

        # collocation on time: 4
        graph_col = self.jr_graph_builder.collocation_graph(
            self.jr, step_phases)
        self.assertEqual(graph_col.size(), 48)


if __name__ == "__main__":
    unittest.main()
