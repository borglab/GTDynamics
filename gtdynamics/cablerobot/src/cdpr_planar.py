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
        self.gravity = np.zeros((3, 1))

class Cdpr:
    def __init__(self, params=CdprParams()):
        self.params = params
        ee = gtd.Link(1, "ee", params.mass, params.inertia, Pose3(), Pose3())
        self.robot = gtd.Robot({'ee' : ee}, {})
        self.costmodel_l = gtsam.noiseModel.Constrained.All(1)
        self.costmodel_ldot = gtsam.noiseModel.Constrained.All(1)
        self.costmodel_wrench = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_torque = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_twistcollo = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_posecollo = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_prior_l = gtsam.noiseModel.Constrained.All(1)
        self.costmodel_prior_ldot = gtsam.noiseModel.Constrained.All(1)
        self.costmodel_prior_tau = gtsam.noiseModel.Constrained.All(1)
        self.costmodel_prior_pose = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_prior_twist = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_prior_twistaccel = gtsam.noiseModel.Constrained.All(6)
        self.costmodel_planar_pose = gtsam.noiseModel.Constrained.All(3)
        self.costmodel_planar_twist = gtsam.noiseModel.Constrained.All(3)
        self.costmodel_dt = gtsam.noiseModel.Constrained.All(1)

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
                kfg.push_back(gtd.CableVelFactor(gtd.internal.JointVelKey(ji, k).key(),
                                                 gtd.internal.PoseKey(self.ee_id(), k).key(),
                                                 gtd.internal.TwistKey(self.ee_id(), k).key(),
                                                 self.costmodel_ldot,
                                                 self.params.frameLocs[ji], self.params.eeLocs[ji]))
            # constrain out-of-plane movements
            kfg.push_back(
                gtsam.LinearContainerFactor(
                    gtsam.JacobianFactor(
                        gtd.internal.PoseKey(self.ee_id(), k).key(),
                        np.array([[1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0],
                                  [0, 0, 0, 0, 1, 0.]]), np.zeros(3),
                        self.costmodel_planar_pose)))
            kfg.push_back(
                gtsam.LinearContainerFactor(
                    gtsam.JacobianFactor(
                        gtd.internal.TwistKey(self.ee_id(), k).key(),
                        np.array([[1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0],
                                  [0, 0, 0, 0, 1, 0.]]), np.zeros(3),
                        self.costmodel_planar_twist)))
        return kfg

    def dynamics_factors(self, ks=[], dt=0.01):
        dfg = gtsam.NonlinearFactorGraph()
        for k in ks:
            wf = gtd.WrenchFactor(
                    gtd.internal.TwistKey(self.ee_id(), k).key(),
                    gtd.internal.TwistAccelKey(self.ee_id(), k).key(),
                    [
                        gtd.internal.WrenchKey(self.ee_id(), 0, k),
                        gtd.internal.WrenchKey(self.ee_id(), 1, k),
                        gtd.internal.WrenchKey(self.ee_id(), 2, k),
                        gtd.internal.WrenchKey(self.ee_id(), 3, k)
                    ],
                    gtd.internal.PoseKey(self.ee_id(), k).key(),
                    self.costmodel_wrench, self.eelink().inertiaMatrix(), self.params.gravity)
            dfg.push_back(wf)
            for ji in range(4):
                dfg.push_back(
                    gtd.CableTensionFactor(
                        gtd.internal.TorqueKey(ji, k).key(),
                        gtd.internal.PoseKey(self.ee_id(), k).key(),
                        gtd.internal.WrenchKey(self.ee_id(), ji, k).key(),
                        self.costmodel_torque, self.params.frameLocs[ji], self.params.eeLocs[ji]))
        for k in ks[:-1]:
            dfg.push_back(
                gtd.EulerPoseColloFactor(
                    gtd.internal.PoseKey(self.ee_id(), k),
                    gtd.internal.PoseKey(self.ee_id(), k + 1),
                    gtd.internal.TwistKey(self.ee_id(), k), 0,
                    self.costmodel_posecollo))
            dfg.push_back(
                gtd.EulerTwistColloFactor(
                    gtd.internal.TwistKey(self.ee_id(), k),
                    gtd.internal.TwistKey(self.ee_id(), k + 1),
                    gtd.internal.TwistAccelKey(self.ee_id(), k), 0,
                    self.costmodel_twistcollo))
        dfg.push_back(gtsam.PriorFactorVector(0, [dt], self.costmodel_dt))
        return dfg

    def priors_fk(self, ks=[], ls=[[]], ldots=[[]]):
        graph = gtsam.NonlinearFactorGraph()
        for k, l, ldot in zip(ks, ls, ldots):
            for ji, (lval, ldotval) in enumerate(zip(l, ldot)):
                graph.push_back(gtd.PriorFactorDouble(gtd.internal.JointAngleKey(ji, k).key(),
                                                      lval, self.costmodel_prior_l))
                graph.push_back(gtd.PriorFactorDouble(gtd.internal.JointVelKey(ji, k).key(),
                                                      ldotval, self.costmodel_prior_ldot))
        return graph

    def priors_ik(self, ks=[], Ts=[], Vs=[]):
        graph = gtsam.NonlinearFactorGraph()
        for k, T, V in zip(ks, Ts, Vs):
            graph.push_back(gtsam.PriorFactorPose3(gtd.internal.PoseKey(self.ee_id(), k).key(),
                                                   T, self.costmodel_prior_pose))
            graph.push_back(gtd.PriorFactorVector6(gtd.internal.TwistKey(self.ee_id(), k).key(),
                                                   V, self.costmodel_prior_twist))
        return graph

    # note: I am not using the strict definitions for forward/inverse dynamics.
    # priors_fd solves for torques given twistaccel (no joint accel)
    # priors_id solves for twistaccel (no joint accel) given torques
    def priors_id(self, ks=[], torquess=[[]]):
        graph = gtsam.NonlinearFactorGraph()
        for k, torques in zip(ks, torquess):
            for ji, torque in enumerate(torques):
                graph.push_back(gtd.PriorFactorDouble(gtd.internal.TorqueKey(ji, k).key(),
                                                      torque, self.costmodel_prior_tau))
        return graph

    def priors_fd(self, ks=[], VAs=[]):
        graph = gtsam.NonlinearFactorGraph()
        for k, VA in zip(ks, VAs):
            graph.push_back(gtsam.PriorFactorVector6(
                gtd.internal.TwistAccelKey(self.ee_id(), k).key(),
                VA, self.costmodel_prior_twistaccel))
        return graph
