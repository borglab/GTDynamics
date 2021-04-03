"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  test_cdpr_planar_sim.py
@brief Unit tests for CDPR simulation.
@author Frank Dellaert
@author Gerry Chen
"""

import unittest

import gtdynamics as gtd
import gtsam
from gtsam import Pose3, Rot3
import numpy as np
from cdpr_planar import Cdpr
from cdpr_planar_controller import CdprControllerBase
from cdpr_planar_sim import cdpr_sim
from gtsam.utils.test_case import GtsamTestCase

class TestCdprPlanar(GtsamTestCase):
    def testSim(self):
        """Tests the simulation: given a controller and initial state, it will run through and
        simulate the system over multiple timesteps
        """
        class DummyController(CdprControllerBase):
            def update(self, values, t):
                tau = gtsam.Values()
                gtd.InsertTorqueDouble(tau, 0, t, 1.)
                gtd.InsertTorqueDouble(tau, 1, t, 1.)
                gtd.InsertTorqueDouble(tau, 2, t, 0.)
                gtd.InsertTorqueDouble(tau, 3, t, 0.)
                return tau
        Tf = 1
        dt = 0.1
        cdpr = Cdpr()
        controller = DummyController()
        # initial state
        xInit = gtsam.Values()
        gtd.InsertPose(xInit, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(xInit, cdpr.ee_id(), 0, np.zeros(6))
        # run simulation
        result = cdpr_sim(cdpr, xInit, controller, dt=dt, N=int(Tf/dt))
        # check correctness
        pAct = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(10)]
        x = 1.5
        xdot = 0
        for k in range(10):
            pExp = Pose3(Rot3(), (x, 0, 1.5))
            self.gtsamAssertEquals(pExp, pAct[k], tol=0)
            dx, dy = 3 - x - 0.15, 1.35  # (dx, dy) represents the cable vector
            xddot = 2 * dx / np.sqrt(dx**2 + dy**2)
            x += xdot * dt
            xdot += xddot * dt

if __name__ == "__main__":
    unittest.main()