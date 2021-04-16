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
from src.jr_values import JRValues
import unittest

import gtsam
import gtdynamics as gtd
import numpy as np


class TestJRValues(unittest.TestCase):
    """ Tests for jumping robot. """

    def __init__(self, *args, **kwargs):
        """ Constructor. """
        super(TestJRValues, self).__init__(*args, **kwargs)
        self.yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
        self.init_config = JumpingRobot.create_init_config()
        self.jr = JumpingRobot(self.yaml_file_path, self.init_config)

    def test_compute_mass_flow(self):
        j = 1
        k = 0

        values = gtsam.Values()
        values.insertDouble(Actuator.ValveOpenTimeKey(j), 0)
        values.insertDouble(Actuator.ValveCloseTimeKey(j), 1)
        P_a_key = Actuator.PressureKey(j, k)
        P_s_key = Actuator.SourcePressureKey(k)
        t_key = gtd.TimeKey(k).key()
        values.insertDouble(P_a_key, 101.325)
        values.insertDouble(P_s_key, 65 * 6894.76/1000)
        values.insertDouble(t_key, 0.0001)

        mdot, mdot_sigma = JRValues.compute_mass_flow(self.jr, values, j, k)
        self.assertAlmostEqual(mdot, 0.005350431488007809, places=7)
        self.assertAlmostEqual(mdot_sigma, 0.0028088651752360754, places=7)


if __name__ == "__main__":
    unittest.main()