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

import unittest
import gtsam
import gtdynamics as gtd
import numpy as np

import os, sys, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.jumping_robot import Actuator, JumpingRobot
from src.robot_graph_builder import RobotGraphBuilder
from src.actuation_graph_builder import ActuationGraphBuilder
from src.jr_graph_builder import JRGraphBuilder
from src.jr_values import JRValues


class TestJRGraphBuilder(unittest.TestCase):
    """ Tests for JRGraphBuilder. """
    def setUp(self):
        """ Set up the graph builder and jumping robot. """
        self.yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
        self.init_config = JumpingRobot.create_init_config()
        self.controls = JumpingRobot.create_controls()
        self.jr = JumpingRobot(self.yaml_file_path, self.init_config, 0)
        self.jr_ground = self.jr.jr_with_phase(0)
        self.jr_air = self.jr.jr_with_phase(3)
        self.jr_graph_builder = JRGraphBuilder()
        self.robot_graph_builder = self.jr_graph_builder.robot_graph_builder
        self.actuation_graph_builder = self.jr_graph_builder.actuation_graph_builder
        self.collocation = gtd.CollocationScheme.Trapezoidal


    def test_collocation_graph_size(self):
        """ Test collocation graph sizes. """
        step_phases = [0, 0, 3, 3]

        # collocation on mass: (4 + 1) * 4
        graph_actuation_col = self.actuation_graph_builder.collocation_graph(
            self.jr, step_phases, self.collocation)
        self.assertEqual(graph_actuation_col.size(), 20)

        # collocation on joints: 4 * 2 * 2
        # collocation on torso: 2 * 4
        graph_robot_col = self.robot_graph_builder.collocation_graph(
            self.jr, step_phases, self.collocation)
        self.assertEqual(graph_robot_col.size(), 24)

        # collocation on time: 4
        graph_col = self.jr_graph_builder.collocation_graph(
            self.jr, step_phases, self.collocation)
        self.assertEqual(graph_col.size(), 48)

    def test_actuation_dynamics_graph_size(self):
        """ Test actuation dynamics graph size. """

        # actuator dynamics: 4 * 5
        # actuator mass flow: 4 * 2
        # source dynamics: 1
        graph_actuation_dynamics = self.actuation_graph_builder.dynamics_graph(self.jr, 0)
        self.assertEqual(graph_actuation_dynamics.size(), 29)
    
    def test_robot_dynamics_graph_size(self):
        """ Test robot frame dynamics graph size. """
        # q-level: 6 + 1
        # v-level: 6 + 1
        # a-level: 6 + 1
        # wrench-eq: 6
        # torque: 6
        # wrench: 5
        # wrench planar: 6
        # torque prior: 2
        graph_robot_dynamics_ground = self.robot_graph_builder.dynamics_graph(self.jr_ground, 0)
        self.assertEqual(graph_robot_dynamics_ground.size(), 46)

        # q-level: 4
        # v-level: 4
        # a-level: 4
        # wrench-eq: 4
        # torque: 4
        # wrench: 5
        # wrench planar: 4
        graph_robot_dynamics_air = self.robot_graph_builder.dynamics_graph(self.jr_air, 0)
        self.assertEqual(graph_robot_dynamics_air.size(), 29)

        # dynamics ground: 46
        # guard: 2
        k = 10
        graph_robot_dynamics_transition = self.robot_graph_builder.transition_dynamics_graph(self.jr_ground, self.jr_air, k)
        self.assertEqual(graph_robot_dynamics_transition.size(), 48)

    def test_prior_graph_size(self):
        """ Test prior graph size. """
        init_config_values = JRValues.init_config_values(self.jr)

        # torso pose and twist: 2
        graph_robot_prior = self.robot_graph_builder.prior_graph(self.jr, init_config_values, 0)
        self.assertEqual(graph_robot_prior.size(), 2)

        # m: 4 + 1
        # Vs: 1
        graph_actuation_prior = self.actuation_graph_builder.prior_graph(self.jr, init_config_values, 0)
        self.assertEqual(graph_actuation_prior.size(), 6)

        # Tc, To: 8
        graph_control_prior = self.jr_graph_builder.control_priors(self.jr, self.controls)
        self.assertEqual(graph_control_prior.size(), 8)

    def test_measurement_graph_size(self):
        """ Test measurement graph size of 1 frame. """
        pixels_all_frames = np.zeros((1, 5, 2, 2))
        pressures_all_frames = [[0, 0, 0, 0, 0]]

        graph = self.jr_graph_builder.measurement_graph_builder.measurement_graph(
            self.jr, pixels_all_frames, pressures_all_frames)

        # markers: 20
        # pressures: 5
        self.assertEqual(graph.size(), 25)

    def test_sys_id_graph(self):
        """ Test system identification graph size of 1 step. """
        pixels_all_frames = np.zeros((2, 5, 2, 2))
        pressures_all_frames = np.zeros((2, 5))
        step_phases = [0]

        graph = self.jr_graph_builder.sysid_graph(
            self.jr, self.controls, step_phases, pixels_all_frames, pressures_all_frames)

        # trajectory priors: 2 + 5 + 1 + 1
        # control priros: 8
        # robot dynamics: 46
        # actuation dynamics: 29 * 2
        # measurement: 25 * 2
        # collocation: 48
        self.assertEqual(graph.size(), 219)

if __name__ == "__main__":
    unittest.main()
