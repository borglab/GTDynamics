"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  test_cdpr_controller_ilqr.py
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
from cdpr_controller_ilqr import CdprControllerIlqr
from cdpr_planar_sim import CdprSimulator
from gtsam.utils.test_case import GtsamTestCase

class TestCdprControllerIlqr(GtsamTestCase):
    def testTrajFollow(self):
        """Tests trajectory tracking controller
        """
        cdpr = Cdpr()

        x0 = gtsam.Values()
        gtd.InsertPose(x0, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(x0, cdpr.ee_id(), 0, np.zeros(6))

        x_des = [Pose3(Rot3(), (1.5+k/20.0, 0, 1.5)) for k in range(9)]
        x_des = x_des[0:1] + x_des
        controller = CdprControllerIlqr(cdpr, x0=x0, pdes=x_des, dt=0.1)

        sim = CdprSimulator(cdpr, x0, controller, dt=0.1)
        result = sim.run(N=10)
        pAct = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(10)]

        if False:
            print()
            for k, (des, act) in enumerate(zip(x_des, pAct)):
                print(('k: {:d}  --  des: {:.3f}, {:.3f}, {:.3f}  --  act: {:.3f}, {:.3f}, {:.3f}' +
                       '  --  u: {:.3e},   {:.3e},   {:.3e},   {:.3e}').format(
                           k, *des.translation(), *act.translation(),
                           *[gtd.TorqueDouble(result, ji, k) for ji in range(4)]))

        for k, (des, act) in enumerate(zip(x_des, pAct)):
            self.gtsamAssertEquals(des, act)

    def testGainsNearConstrained(self):
        """Tests locally linear, time-varying feedback gains
        """
        cdpr = Cdpr()
        dt = 0.01

        x0 = gtsam.Values()
        gtd.InsertPose(x0, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(x0, cdpr.ee_id(), 0, np.zeros(6))
        x_des = [Pose3(Rot3(), (1.5, 0, 1.5)),
                 Pose3(Rot3(), (1.5, 0, 1.5)),
                 Pose3(Rot3(), (1.5, 0, 1.5))]  # don't move
        controller = CdprControllerIlqr(cdpr, x0=x0, pdes=x_des, dt=dt, Q=np.ones(6)*1e12, R=np.array([1.]))
        print(controller.fg.error(controller.result))

        # notation: x_K_y means xstar = K * dy
        #           where xstar is optimal x and dy is (desired_y - actual_y)
        #           note: when multiplying gain matrices together, be mindful of negative signs
        # position gain (Kp) - time 0, cable 0, pose gain
        actual_0c0_K_0x = controller.gains_ff[0][0][0:1, 6:]
        expected_1v_K_0x = np.diag([0, -1, 0, -1, 0, -1]) / dt  # v at t=1 in response to x at t=0
        expected_0c0_K_1v = np.array([0, 1e9, 0,
                                      -1 / np.sqrt(2) / 2, 0, 1 / np.sqrt(2) / 2]).reshape((1, -1)) * \
                            cdpr.params.mass / dt  # cable 0 tension at t=0 in response to v at t=1
        expected_0c0_K_0x = -expected_0c0_K_1v @ expected_1v_K_0x
        self.gtsamAssertEquals(actual_0c0_K_0x[:, 3:], expected_0c0_K_0x[:, 3:], tol=1/dt)

        # velocity gain (Kd) - time 0, cable 0, twist gain
        actual_0c0_K_0v = controller.gains_ff[0][0][0:1, :6]
        expected_0c0_K_0v = 2 * expected_0c0_K_1v
        self.gtsamAssertEquals(actual_0c0_K_0v[:, 3:], expected_0c0_K_0v[:, 3:], tol=dt)

    def testRun(self):
        """Tests that controller will not "compile" (aka run without errors)
        """
        cdpr = Cdpr()
        dt = 0.01

        x0 = gtsam.Values()
        gtd.InsertPose(x0, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(x0, cdpr.ee_id(), 0, np.zeros(6))
        x_des = [Pose3(Rot3(), (1.5, 0, 1.5))]
        controller = CdprControllerIlqr(cdpr, x0=x0, pdes=x_des, dt=dt)


if __name__ == "__main__":
    unittest.main()
