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
from gtsam import Pose3, Rot3
import numpy as np
from cdpr_planar import Cdpr
from cdpr_planar_controller import CdprController
from cdpr_planar_sim import cdpr_sim

class TestCdprPlanar(unittest.TestCase):
    """Unit tests for planar CDPR"""
    def testConstructor(self):
        cdpr = Cdpr()
    
    def testTrajFollow(self):
        cdpr = Cdpr()
        xDes = [Pose3(Rot3(), (1.5+k/20.0, 0, 1.5)) for k in range(10)]
        controller = CdprController(cdpr, xdes=xDes, dt=0.1)
        result = cdpr_sim(cdpr, controller)
        xAct = [gtd.Pose(result, cdpr.eelink().id(), k) for k in range(10)]
        self.assertEqual(xDes, xAct, "didn't achieve desired trajectory")

if __name__ == "__main__":
    unittest.main()
