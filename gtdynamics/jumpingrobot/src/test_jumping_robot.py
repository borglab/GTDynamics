"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_jumping_robot.py
 * @brief Unit test for jumping robot.
 * @author Yetong Zhang
"""

import inspect
import os.path as osp
import sys

currentdir = osp.dirname(
    osp.abspath(inspect.getfile(inspect.currentframe())))
parentdir = osp.dirname(currentdir)
sys.path.insert(0, parentdir)

import unittest

import gtdynamics as gtd
import gtsam
import numpy as np

from src.jumping_robot import Actuator, JumpingRobot


class TestPneumaticBindings(unittest.TestCase):
    """Exercise factor helpers with their default optional Jacobians."""

    def test_mass_flow(self):
        model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        factor = gtd.MassFlowRateFactor(
            0, 1, 2, model, 0.1575 * 0.0254, 74 * 0.0254,
            1.8377e-5, 1e-5, 1 / (287.0550 * 296.15))
        self.assertAlmostEqual(
            factor.computeExpectedMassFlow(100., 65 * 6.89476, 1e-6),
            0.000624547, places=8)

    def test_valve_flow(self):
        model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        factor = gtd.ValveControlFactor(0, 1, 2, 3, 4, model, 0.1)
        self.assertAlmostEqual(
            factor.computeExpectedTrueMassFlow(0.8, 0.7, 1.2, 2.),
            1.4261447373358271)

    def test_actuator_volume(self):
        model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        factor = gtd.ActuatorVolumeFactor(
            0, 1, model, d_tube=0.004, l_tube=1.)
        self.assertAlmostEqual(factor.computeVolume(0.),
                               4.243e-5 + np.pi * 0.002 ** 2)


class TestJumpingRobot(unittest.TestCase):
    """ Tests for jumping robot. """
    def setUp(self):
        """ Set up the jumping robot. """
        self.yaml_file_path = osp.join(parentdir, "yaml", "robot_config.yaml")
        self.init_config = JumpingRobot.create_init_config()
        self.jr = JumpingRobot(self.yaml_file_path, self.init_config)

    def test_links_joints(self):
        """ Test number of links and joints. """
        self.assertEqual(self.jr.robot.numLinks(), 6)
        self.assertEqual(self.jr.robot.numJoints(), 6)

    def test_forward_kinematics(self):
        """ Test forward kinematics of jumping robot. """
        values = gtsam.Values()
        k = 0
        theta = np.pi / 3
        qs = [-theta, 2 * theta, -theta, -theta, 2 * theta, -theta]
        for joint in self.jr.robot.joints():
            j = joint.id()
            gtd.InsertJointAngle(values, j, k, qs[j])
            gtd.InsertJointVel(values, j, k, 0.)

        fk_results = self.jr.robot.forwardKinematics(values, k)
        torso_i = self.jr.robot.link("torso").id()
        torso_pose = gtd.Pose(fk_results, torso_i, k)
        expected_torso_pose = gtsam.Pose3(gtsam.Rot3(),
                                          gtsam.Point3(0, 0, 0.55))
        self.assertTrue(torso_pose.equals(expected_torso_pose, tol=1e-5))


if __name__ == "__main__":
    unittest.main()
