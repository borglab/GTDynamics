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

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 

from src.jumping_robot import Actuator, JumpingRobot
from src.jr_visualizer import visualize_jr
import unittest

import gtsam
import gtdynamics as gtd
import numpy as np

class TestJumpingRobot(unittest.TestCase):
    """ Tests for jumping robot. """
    def __init__(self, *args, **kwargs):
        """ Constructor. """
        super(TestJumpingRobot, self).__init__(*args, **kwargs)
        self.yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
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
        theta = np.pi/3
        qs = [-theta, 2 * theta, -theta, -theta, 2 * theta, -theta]
        for joint in self.jr.robot.joints():
            j = joint.id()
            gtd.InsertJointAngleDouble(values, j, k, qs[j])
            gtd.InsertJointVelDouble(values, j, k , 0.)

        fk_results = self.jr.robot.forwardKinematics(values, k)
        torso_i = self.jr.robot.link("torso").id()
        torso_pose = gtd.Pose(fk_results, torso_i, k)
        expected_torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.55))
        self.assertTrue(torso_pose.equals(expected_torso_pose, tol=1e-5))
        
        # visualize_jr(fk_results, self.jr, k)



if __name__ == "__main__":
    unittest.main()
