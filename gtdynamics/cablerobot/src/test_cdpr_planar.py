"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
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
from gtsam.utils.test_case import GtsamTestCase

class TestCdprPlanar(GtsamTestCase):
    """Unit tests for planar CDPR"""
    def testConstructor(self):
        cdpr = Cdpr()

    def testKinematics(self):
        """Unit test kinematics factors and priors corresponding to forward and inverse kinematics
        """
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
        ik1 = cdpr.priors_ik(ks=[0],
                             Ts=[gtd.Pose(values, cdpr.ee_id(), 0)],
                             Vs=[gtd.Twist(values, cdpr.ee_id(), 0)])
        ik2 = cdpr.priors_ik(ks=[0], values=values)
        self.gtsamAssertEquals(ik1, ik2)
        ikgraph.push_back(ik1)
        ikres = gtsam.LevenbergMarquardtOptimizer(ikgraph, zeroValues()).optimize()
        self.gtsamAssertEquals(ikres, values)  # should match with full sol
        # try optimizing FK
        fkgraph = gtsam.NonlinearFactorGraph(kfg)
        fk1 = cdpr.priors_fk(ks=[0],
                             ls=[[gtd.JointAngleDouble(values, ji, 0) for ji in range(4)]],
                             ldots=[[gtd.JointVelDouble(values, ji, 0) for ji in range(4)]])
        fk2 = cdpr.priors_fk(ks=[0], values=values)
        self.gtsamAssertEquals(fk1, fk2)
        fkgraph.push_back(fk1)
        params = gtsam.LevenbergMarquardtParams()
        params.setAbsoluteErrorTol(1e-20)  # FK less sensitive so we need to decrease the tolerance
        fkres = gtsam.LevenbergMarquardtOptimizer(fkgraph, zeroValues(), params).optimize()
        self.gtsamAssertEquals(fkres, values, tol=1e-5)  # should match with full sol

    def testDynamicsInstantaneous(self):
        """Test dynamics factors: relates torque to wrench+twistaccel.  Also tests ID solving
        """
        cdpr = Cdpr()
        dfg = cdpr.dynamics_factors(ks=[0])
        values = gtsam.Values()
        # things needed to define IK
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
        id1 = cdpr.priors_id(ks=[0], VAs=[gtd.TwistAccel(values, cdpr.ee_id(), 0)])
        id2 = cdpr.priors_id(ks=[0], values=values)
        self.gtsamAssertEquals(id1, id2)
        dfg.push_back(id1)
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
        # check FD priors functions
        fd1 = cdpr.priors_fd(ks=[0], torquess=[[gtd.TorqueDouble(results, ji, 0) for ji in range(4)]])
        fd2 = cdpr.priors_fd(ks=[0], values=results)
        self.gtsamAssertEquals(fd1, fd2)

    def testDynamicsCollocation(self):
        """Test dynamics factors across multiple timesteps by using collocation.
        """
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
        # torque inputs (FD priors)
        fg.push_back(cdpr.priors_fd(ks=[0, 1, 2], torquess=[[1,1,0,0],]*3))
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

if __name__ == "__main__":
    unittest.main()
