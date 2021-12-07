"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_jr_simulator.py
 * @brief Unit test for jumping robot simulator.
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
from src.jr_simulator import JRSimulator
from src.jr_values import JRValues
from src.jumping_robot import Actuator, JumpingRobot
from src.robot_graph_builder import RobotGraphBuilder


class TestJRSimulator(unittest.TestCase):
    def setUp(self):
        """ Set up the simulator. """
        self.yaml_file_path = osp.join(parentdir, "yaml", "robot_config.yaml")
        self.init_config = JumpingRobot.create_init_config()
        self.jr_simulator = JRSimulator(self.yaml_file_path, self.init_config)

    def robot(self):
        """ Return the robot model. """
        return self.jr_simulator.jr.robot

    def cal_jr_accels(self, theta, torque_hip, torque_knee):
        """ Compute groundtruth joint accelerations from virtual work. """
        m1 = self.robot().link("shank_r").mass()
        m2 = self.robot().link("thigh_r").mass()
        m3 = self.robot().link("torso").mass()
        link_radius = self.jr_simulator.jr.params["morphology"]["r_cyl"]
        l_link = self.jr_simulator.jr.params["morphology"]["l"][0]

        g = 9.8
        moment = (0.5 * m1 + 1.5 * m2 + 1.0 * m3) * g * l_link * np.sin(theta)
        J1 = (l_link ** 2 + 3 * link_radius ** 2) * 1.0 / 12 * m1
        J2 = (l_link ** 2 + 3 * link_radius ** 2) * 1.0 / 12 * m2
        J = l_link ** 2 * (1.0 / 4 * m1 + (1.0 / 4 + 2 * np.sin(theta) ** 2) * m2 + 2 * np.sin(theta) ** 2 * m3)

        acc = (torque_hip - torque_knee * 2 - moment) / (J + J1 + J2)
        expected_q_accels = {"foot_r": acc, "knee_r": -2*acc, "hip_r": acc,
                             "hip_l": acc, "knee_l": -2*acc, "foot_l": acc}
        return expected_q_accels

    def test_robot_forward_dynamics(self):
        """ Test forward dynamics of robot frame: specify the angles,
            joint vels and torques, check the joint accelerations. """
        # specify joint angles, joint vels, torques
        theta = np.pi/3
        torque_hip = 0
        torque_knee = 0
        qs = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
        vs = [0., 0., 0., 0., 0., 0.]
        torques = [0., torque_knee, torque_hip, torque_hip, torque_knee, 0.]

        # construct known values
        values = gtsam.Values()
        k = 0
        for joint in self.robot().joints():
            j = joint.id()
            gtd.InsertTorque(values, j, k, torques[j])
            gtd.InsertJointAngle(values, j, k, qs[j])
            gtd.InsertJointVel(values, j, k, vs[j])
        torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.55))
        torso_i = self.robot().link("torso").id()
        gtd.InsertPose(values, torso_i, k, torso_pose)
        gtd.InsertTwist(values, torso_i, k, np.zeros(6))

        # step forward dynamics
        self.jr_simulator.step_robot_dynamics(k, values)

        # check joint accelerations
        q_accels = gtd.DynamicsGraph.jointAccelsMap(self.robot(),
                                                    values, k)
        expected_q_accels = self.cal_jr_accels(theta, torque_hip, torque_knee)
        for joint in self.robot().joints():
            name = joint.name()
            self.assertAlmostEqual(q_accels[name],
                                   expected_q_accels[name], places=7)

    def test_actuation_forward_dynamics(self):
        """ Test forward dynamics of actuator: specify mass, time, controls,
            check torques. """

        # create controls
        Tos = [0, 0, 0, 0]
        Tcs = [1, 1, 1, 1]
        P_s_0 = 65 * 6894.76 / 1e3
        controls = JumpingRobot.create_controls(Tos, Tcs, P_s_0)

        # create init values of known variables
        values = JRValues.init_config_values(self.jr_simulator.jr, controls)
        k = 0

        # compute dynamics
        self.jr_simulator.step_actuation_dynamics(k, values)

        torques = []
        for actuator in self.jr_simulator.jr.actuators:
            j = actuator.j
            torque = gtd.Torque(values, j, k)
            torques.append(torque)
            self.assertAlmostEqual(torque, 0, places=7)
        # TODO(yetong): check torques, pressures, etc


if __name__ == "__main__":
    unittest.main()
