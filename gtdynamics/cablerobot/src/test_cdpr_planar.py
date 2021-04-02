"""
GTDynamics Copyright 2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  test_cdpr_planar.py
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
    """Unit tests for planar CDPR"""
    def testConstructor(self):
        cdpr = Cdpr()

    def testKinematics(self):
        cdpr = Cdpr()
        kfg = cdpr.kinematics_factors(ks=[0])
        def zeroValues():
            values = gtsam.Values()
            for j in range(4):
                gtd.InsertJointAngleDouble(values, j, 0, 3.0)
                gtd.InsertJointVelDouble(values, j, 0, 17.0 + np.random.rand(1))
            gtd.InsertPose(values, cdpr.ee_id(), 0, Pose3())
            gtd.InsertTwist(values, cdpr.ee_id(), 0, (0,0,0,0,0,1.4142))
            return values
        # things needed to define FK
        values = gtsam.Values()
        for j, l, ldot in zip(range(4), [1.35 * np.sqrt(2),]*4, [-1, 1, 1, -1.]):
            gtd.InsertJointAngleDouble(values, j, 0, l)
            gtd.InsertJointVelDouble(values, j, 0, ldot)
        # things needed to define IK
        gtd.InsertPose(values, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(values, cdpr.ee_id(), 0, (0, 0, 0, 0, 0, -np.sqrt(2)))
        # check that solution has 0 residual
        self.assertEqual(0.0, kfg.error(values))
        # try optimizing IK
        ikgraph = gtsam.NonlinearFactorGraph(kfg)
        ikgraph.push_back(cdpr.priors_ik(
            [0], [gtd.Pose(values, cdpr.ee_id(), 0)], [gtd.Twist(values, cdpr.ee_id(), 0)]))
        ikres = gtsam.LevenbergMarquardtOptimizer(ikgraph, zeroValues()).optimize()
        self.gtsamAssertEquals(ikres, values)  # should match with full sol
        # try optimizing FK
        fkgraph = gtsam.NonlinearFactorGraph(kfg)
        fkgraph.push_back(cdpr.priors_fk([0],
                                         [[gtd.JointAngleDouble(values, ji, 0) for ji in range(4)]],
                                         [[gtd.JointVelDouble(values, ji, 0) for ji in range(4)]]))
        params = gtsam.LevenbergMarquardtParams()
        params.setAbsoluteErrorTol(1e-20)  # FK less sensitive so we need to decrease the tolerance
        fkres = gtsam.LevenbergMarquardtOptimizer(fkgraph, zeroValues(), params).optimize()
        self.gtsamAssertEquals(fkres, values, tol=1e-5)  # should match with full sol

    def testDynamicsInstantaneous(self):
        cdpr = Cdpr()
        dfg = cdpr.dynamics_factors(ks=[0])
        values = gtsam.Values()
        # things needed to define kinematic
        gtd.InsertPose(values, cdpr.ee_id(), 0, Pose3(Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(values, cdpr.ee_id(), 0, np.zeros(6))
        # things needed to define ID
        for j, tau in zip(range(4), [1, 0, 0, 1]):
            gtd.InsertTorqueDouble(values, j, 0, tau)
        # things needed to define FD
        gtd.InsertTwistAccel(values, cdpr.ee_id(), 0, (0, 0, 0, 0, 0, -np.sqrt(2)))
        # things needed intermediaries
        gtd.InsertWrench(values, cdpr.ee_id(), 0, [0,0,0,1/np.sqrt(2),0,-1/np.sqrt(2)])
        gtd.InsertWrench(values, cdpr.ee_id(), 1, [0,0,0,0,0,0])
        gtd.InsertWrench(values, cdpr.ee_id(), 2, [0,0,0,0,0,0])
        gtd.InsertWrench(values, cdpr.ee_id(), 3, [0,0,0,-1/np.sqrt(2),0,-1/np.sqrt(2)])
        # check FD/ID is valid
        self.assertAlmostEqual(0.0, dfg.error(values), 10)
        # build FD graph
        dfg.push_back(
            cdpr.priors_ik([0], [gtd.Pose(values, cdpr.ee_id(), 0)],
                           [gtd.Twist(values, cdpr.ee_id(), 0)]))
        dfg.push_back(cdpr.priors_fd([0], [gtd.TwistAccel(values, cdpr.ee_id(), 0)]))
        # redundancy resolution
        dfg.push_back(
            gtd.PriorFactorDouble(
                gtd.internal.TorqueKey(1, 0).key(), 0.0,
                gtsam.noiseModel.Unit.Create(1)))
        dfg.push_back(
            gtd.PriorFactorDouble(
                gtd.internal.TorqueKey(2, 0).key(), 0.0,
                gtsam.noiseModel.Unit.Create(1)))
        # initialize and solve
        init = gtsam.Values(values)
        for ji in range(4):
            init.erase(gtd.internal.TorqueKey(ji, 0).key())
            gtd.InsertTorqueDouble(init, ji, 0, -1)
        results = gtsam.LevenbergMarquardtOptimizer(dfg, init).optimize()
        self.gtsamAssertEquals(results, values)

    def testDynamicsCollocation(self):
        cdpr = Cdpr()
        # kinematics
        fg = cdpr.kinematics_factors(ks=[0, 1, 2])
        # dynamics
        fg.push_back(cdpr.dynamics_factors(ks=[0, 1, 2]))
        # collocation
        fg.push_back(cdpr.collocation_factors(ks=[0, 1], dt=0.01))
        # initial state
        fg.push_back(
            cdpr.priors_ik(ks=[0], Ts=[Pose3(Rot3(), (1.5, 0, 1.5))], Vs=[np.zeros(6)]))
        # torque inputs (ID priors)
        fg.push_back(cdpr.priors_id(ks=[0, 1, 2], torquess=[[1,1,0,0],]*3))
        # construct initial guess
        init = gtsam.Values()
        init.insertDouble(0, 0.01)
        for t in range(3):
            for j in range(4):
                gtd.InsertJointAngleDouble(init, j, t, 1)
                gtd.InsertJointVelDouble(init, j, t, 1)
                gtd.InsertTorqueDouble(init, j, t, 1)
                gtd.InsertWrench(init, cdpr.ee_id(), j, t, np.ones(6))
            gtd.InsertPose(init, cdpr.ee_id(), t, Pose3(Rot3(), (1.5, 1, 1.5)))
            gtd.InsertTwist(init, cdpr.ee_id(), t, np.ones(6))
            gtd.InsertTwistAccel(init, cdpr.ee_id(), t, np.ones(6))
        # optimize
        optimizer = gtsam.LevenbergMarquardtOptimizer(fg, init)
        result = optimizer.optimize()
        # correctness checks:
        # timestep 0
        self.gtsamAssertEquals(gtd.Pose(result, cdpr.ee_id(), 0), Pose3(Rot3(), (1.5, 0, 1.5)))
        # timestep 1 (euler collocation)
        self.gtsamAssertEquals(gtd.Pose(result, cdpr.ee_id(), 1),
                               Pose3(Rot3(), (1.5, 0, 1.5)), tol=1e-3)
        self.gtsamAssertEquals(gtd.Twist(result, cdpr.ee_id(), 1),
                               np.array([0, 0, 0, np.sqrt(2) * 0.01, 0, 0]), tol=1e-3)
        # timestep 2
        self.gtsamAssertEquals(
            gtd.Pose(result, cdpr.ee_id(), 2),
            Pose3(Rot3(), (1.5 + np.sqrt(2) * 0.0001, 0, 1.5)))

    def testSim(self):
        class DummyController:
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

    @unittest.SkipTest
    def testTrajFollow(self):
        cdpr = Cdpr()

        pDes = [Pose3(Rot3(), (1.5+k/20.0, 0, 1.5)) for k in range(10)]
        controller = CdprController(cdpr, pdes=pDes, dt=0.1)

        xInit = gtsam.Values()
        for ji in range(4):
            gtd.InsertJointAngleDouble(xInit, ji, 0, 1.5 * np.sqrt(2))
            gtd.InsertJointVelDouble(xInit, ji, 0, 0.0)
        result = cdpr_sim(cdpr, xInit, controller, dt=0.1)

        pAct = [gtd.Pose(result, cdpr.ee_id(), k) for k in range(10)]
        self.assertEqual(pDes, pAct, "didn't achieve desired trajectory")

if __name__ == "__main__":
    unittest.main()
