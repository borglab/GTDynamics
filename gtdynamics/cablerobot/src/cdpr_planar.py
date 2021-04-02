"""
GTDynamics Copyright 2020, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  cdpr_planar.py
@brief Cable-Driven Parallel Robot
@author Frank Dellaert
@author Gerry Chen
"""

import gtdynamics as gtd
import gtsam
from gtsam import Pose3, Rot3
import numpy as np

class CdprParams:
    def __init__(self):
        self.frameLocs = np.array([[3., 0., 0.], [3., 0., 3.], [0., 0., 3.], [0., 0., 0.]])
        s = 0.15
        self.eeLocs = np.array([[s, 0., -s], [s, 0., s], [-s, 0., s], [-s, 0, -s]])
        self.mass = 1.0
        self.inertia = np.eye(3)

class Cdpr:
    def __init__(self, params=CdprParams()):
        self.params = params
        ee = gtd.Link(1, "ee", params.mass, params.inertia, Pose3(), Pose3())
        self.robot = gtd.Robot({'ee' : ee}, {})
        self.costmodel_l = gtsam.noiseModel.Constrained.All(1)
        self.costmodel_ldot = gtsam.noiseModel.Constrained.All(1)
        self.costmodel_wrench = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_torque = gtsam.noiseModel.Constrained.All(1)
        self.costmodel_twistcollo = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_posecollo = gtsam.noiseModel.Constrained.All(6)

    def eelink(self):
        return self.robot.link('ee')
    def ee_id(self):
        return self.eelink().id()

    def kinematics_factors(self, ks=[]):
        kfg = gtsam.NonlinearFactorGraph()
        for k in ks:
            for ji in range(4):
                kfg.push_back(gtd.CableLenFactor(gtd.internal.JointAngleKey(ji, k).key(),
                                                 gtd.internal.PoseKey(self.ee_id(), k).key(),
                                                 self.costmodel_l,
                                                 self.params.frameLocs[ji], self.params.eeLocs[ji]))
                kfg += gtd.CableVelFactor(gtd.internal.JointVelKey(ji, k),
                                          gtd.internal.PoseKey(self.ee_id(), k),
                                          gtd.internal.TwistKey(self.ee_id(), k),
                                          self.costmodel_ldot, self.params.frameLocs[ji])
        return kfg

    def dynamics_factors(self, ks=[]):
        dfg = gtsam.NonlinearFactorGraph()
        for k in ks:
            dfg += gtd.WrenchFactor4(gtd.internal.TwistKey(self.ee_id(), k),
                                     gtd.internal.TwistAccelKey(self.ee_id(), k),
                                     gtd.internal.WrenchKey(self.ee_id(), 0, k),
                                     gtd.internal.WrenchKey(self.ee_id(), 1, k),
                                     gtd.internal.WrenchKey(self.ee_id(), 2, k),
                                     gtd.internal.WrenchKey(self.ee_id(), 3, k),
                                     gtd.internal.PoseKey(self.ee_id(), k),
                                     self.costmodel_wrench, self.params.inertia)
            for ji in range(4):
                dfg += gtd.CableTensionFactor(gtd.internal.TorqueKey(ji, k),
                                              gtd.internal.WrenchKey(self.ee_id(), ji, k),
                                              gtd.internal.PoseKey(self.ee_id(), k),
                                              self.costmodel_torque, self.params.frameLocs[ji])
        for k in ks[:-1]:
            dfg += gtd.EulerPoseColloFactor(gtd.internal.PoseKey(self.ee_id(), k),
                                            gtd.internal.PoseKey(self.ee_id(), k+1),
                                            gtd.internal.TwistKey(self.ee_id(), k),
                                            0, self.costmodel_posecollo)
            dfg += gtd.EulerTwistColloFactor(gtd.internal.TwistKey(self.ee_id(), k),
                                             gtd.internal.TwistKey(self.ee_id(), k+1),
                                             gtd.internal.TwistAccelKey(self.ee_id(), k),
                                             0, self.costmodel_twistcollo)
        dfg += gtsam.PriorFactorVector(0, [0], self.costmodel_dt)
        return dfg

    def dynamics_factors(self):
        pass
