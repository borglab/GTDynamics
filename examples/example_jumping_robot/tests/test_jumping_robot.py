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

    def test_jumping_robot(self):
        """ Test creating jumping robot. """
        jr = JumpingRobot(self.yaml_file_path, self.init_config)
        self.assertEqual(jr.robot.numLinks(), 6)
        self.assertEqual(jr.robot.numJoints(), 6)

if __name__ == "__main__":
    unittest.main()
