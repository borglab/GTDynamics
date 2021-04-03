"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  test_cdpr_planar_controller.py
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
from cdpr_planar_controller import CdprController
from cdpr_planar_sim import cdpr_sim
from gtsam.utils.test_case import GtsamTestCase

class TestCdprPlanar(GtsamTestCase):
    def testTrajFollow(self):
        """Tests trajectory tracking controller
        """
        cdpr = Cdpr()

        x0 = gtsam.Values()
        gtd.InsertPose(x0, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(x0, cdpr.ee_id(), 0, np.zeros(6))

        pDes = [Pose3(Rot3(), (1.5+k/20.0, 0, 1.5)) for k in range(9)]
        pDes = pDes[0:1] + pDes
        controller = CdprController(cdpr, x0=x0, pdes=pDes, dt=0.1)

        result = cdpr_sim(cdpr, x0, controller, dt=0.1, N=10)
        pAct = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(10)]

        if False:
            print()
            for k, (des, act) in enumerate(zip(pDes, pAct)):
                print(('k: {:d}  --  des: {:.3f}, {:.3f}, {:.3f}  --  act: {:.3f}, {:.3f}, {:.3f}' +
                       '  --  u: {:.3e},   {:.3e},   {:.3e},   {:.3e}').format(
                           k, *des.translation(), *act.translation(),
                           *[gtd.TorqueDouble(result, ji, k) for ji in range(4)]))

        for k, (des, act) in enumerate(zip(pDes, pAct)):
            self.gtsamAssertEquals(des, act)

if __name__ == "__main__":
    unittest.main()
