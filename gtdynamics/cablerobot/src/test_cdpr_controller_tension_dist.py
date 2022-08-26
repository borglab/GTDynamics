"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  test_cdpr_controller_tension_dist.py
@brief Unit tests for CDPR.
@author Frank Dellaert
@author Gerry Chen
"""

import unittest

import gtdynamics as gtd
import gtsam
from gtsam import Pose3, Rot3
import numpy as np
from cdpr_planar import Cdpr
from cdpr_controller_tension_dist import CdprControllerTensionDist as CdprController
from cdpr_planar_sim import CdprSimulator
from gtsam.utils.test_case import GtsamTestCase

class TestCdprControllerTensionDist(GtsamTestCase):
    def testTrajFollow(self):
        """Tests trajectory tracking controller
        """
        cdpr = Cdpr()
        cdpr.params.collocation_mode = 1
        x0 = gtsam.Values()
        gtd.InsertPose(x0, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(x0, cdpr.ee_id(), 0, np.zeros(6))

        x_des = [Pose3(Rot3(), (1.5+k/20.0, 0, 1.5)) for k in range(9)]
        x_des = x_des[0:1] + x_des
        controller = CdprController(cdpr, x0=x0, pdes=x_des, dt=0.1)

        sim = CdprSimulator(cdpr, x0, controller, dt=0.1)
        result = sim.run(N=9)
        pAct = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(10)]

        if False:
            [gtd.InsertTorqueDouble(result, ji, 9, np.nan) for ji in range(4)]
            print()
            for k, (des, act) in enumerate(zip(x_des, pAct)):
                print(('k: {:d}  --  des: {:.3f}, {:.3f}, {:.3f}  --  act: {:.3f}, {:.3f}, {:.3f}' +
                       '  --  u: {:.3e},   {:.3e},   {:.3e},   {:.3e}').format(
                           k, *des.translation(), *act.translation(),
                           *[gtd.Torque(result, ji, k) for ji in range(4)]))

        for k, (des, act) in enumerate(zip(x_des, pAct)):
            self.gtsamAssertEquals(des, act, tol=1e-2)

if __name__ == "__main__":
    unittest.main()
