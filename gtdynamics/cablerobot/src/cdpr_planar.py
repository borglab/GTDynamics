"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
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
    """Parameters relevant to cable robot geometry and properties
    """
    def __init__(self):
        self.a_locs = np.array([[3., 0., 0.], [3., 0., 3.], [0., 0., 3.], [0., 0., 0.]])
        s = 0.15
        self.b_locs = np.array([[s, 0., -s], [s, 0., s], [-s, 0., s], [-s, 0, -s]])
        self.mass = 1.0
        self.inertia = np.eye(3)
        self.gravity = np.zeros((3, 1))

class Cdpr:
    """Utility functions for a planar cable robot; mostly assembles together factors.
    This class is coded functionally.
    """
    def __init__(self, params=CdprParams()):
        self.params = params
        ee = gtd.Link(1, "ee", params.mass, params.inertia, Pose3(), Pose3())
        self.robot = gtd.Robot({'ee': ee}, {})
        self.costmodel_l = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        self.costmodel_ldot = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        self.costmodel_wrench = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        self.costmodel_torque = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        self.costmodel_twistcollo = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        self.costmodel_posecollo = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        self.costmodel_prior_l = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        self.costmodel_prior_ldot = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        self.costmodel_prior_tau = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        self.costmodel_prior_pose = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        self.costmodel_prior_twist = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        self.costmodel_prior_twistaccel = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        self.costmodel_planar_pose = gtsam.noiseModel.Isotropic.Sigma(3, 0.001)
        self.costmodel_planar_twist = gtsam.noiseModel.Isotropic.Sigma(3, 0.001)
        self.costmodel_dt = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)

    def eelink(self):
        """Link object for the end-effector

        Returns:
            gtd.Link: End effector
        """
        return self.robot.link('ee')
    def ee_id(self):
        """id for the end-effector

        Returns:
            int: The end effector's link id
        """
        return self.eelink().id()

    def all_factors(self, N, dt):
        """All the factors needed to specify the CDPR evolution, except for the priors.  i.e.:
        1. Initial state priors (e.g. Pose/Twist at the initial time step for IK, or l/ldot for FK)
        2. "control"/"dynamics" priors (e.g. Torques at every time step for ID, or twistAccels for
        FD)

        Args:
            N (int): number of time steps
            dt (float): the time step duration

        Returns:
            gtsam.NonlinearFactorGraph: the factor graph
        """
        fg = gtsam.NonlinearFactorGraph()
        fg.push_back(self.kinematics_factors(ks=range(N)))
        fg.push_back(self.dynamics_factors(ks=range(N)))
        fg.push_back(self.collocation_factors(ks=range(N-1), dt=dt))
        return fg

    def kinematics_factors(self, ks=[]):
        """Creates the factors necessary for kinematics, which includes the CableLengthFactors and
        CableVelocityFactors.  Since this is a planar CDPR, it also adds factors which constrain
        motion to the xz plane.
        Primary variables:          length/lengthdot <--> Pose/Twist
        Intermediate variables:     None
        Prerequisite variables:     None

        Args:
            ks (list, optional): list of time step indices.

        Returns:
            gtsam.NonlinearFactorGraph: The factors for kinematics
        """
        kfg = gtsam.NonlinearFactorGraph()
        for k in ks:
            for ji in range(4):
                kfg.push_back(
                    gtd.CableLengthFactor(
                        gtd.internal.JointAngleKey(ji, k).key(),
                        gtd.internal.PoseKey(self.ee_id(), k).key(),  #
                        self.costmodel_l,
                        self.params.a_locs[ji],
                        self.params.b_locs[ji]))
                kfg.push_back(
                    gtd.CableVelocityFactor(
                        gtd.internal.JointVelKey(ji, k).key(),
                        gtd.internal.PoseKey(self.ee_id(), k).key(),
                        gtd.internal.TwistKey(self.ee_id(), k).key(),  #
                        self.costmodel_ldot,
                        self.params.a_locs[ji],
                        self.params.b_locs[ji]))
            # constrain out-of-plane movements
            zeroT = gtsam.Values(); gtd.InsertPose(zeroT, self.ee_id(), k, Pose3())
            kfg.push_back(gtsam.LinearContainerFactor(gtsam.JacobianFactor(
                gtd.internal.PoseKey(self.ee_id(), k).key(),
                np.array([[1, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0.]]),
                np.zeros(3),
                self.costmodel_planar_pose), zeroT))
            zeroV = gtsam.Values(); gtd.InsertTwist(zeroV, self.ee_id(), k, np.zeros(6))
            kfg.push_back(gtsam.LinearContainerFactor(gtsam.JacobianFactor(
                gtd.internal.TwistKey(self.ee_id(), k).key(),
                np.array([[1, 0, 0, 0, 0, 0],
                          [0, 0, 1, 0, 0, 0],
                          [0, 0, 0, 0, 1, 0.]]),
                np.zeros(3),
                self.costmodel_planar_twist), zeroV))
        return kfg

    def dynamics_factors(self, ks=[]):
        """Creates factors necessary for dynamics calculations.  Specifically, consists of the
        generalized version of F=ma and calculates wrenches from cable tensions.
        Primary variables:          Torque <--> TwistAccel
        Intermediate variables:     Wrenches
        Prerequisite variables:     Pose, Twist


        Args:
            ks (list, optional): Time step indices. Defaults to [].

        Returns:
            gtsam.NonlinearFactorGraph: The dynamics factors
        """
        dfg = gtsam.NonlinearFactorGraph()
        for k in ks:
            gtd.addWrenchFactor(dfg,
                    self.costmodel_wrench, self.eelink(),
                    [
                        gtd.internal.WrenchKey(self.ee_id(), 0, k),
                        gtd.internal.WrenchKey(self.ee_id(), 1, k),
                        gtd.internal.WrenchKey(self.ee_id(), 2, k),
                        gtd.internal.WrenchKey(self.ee_id(), 3, k)
                    ], k, self.params.gravity)
            for ji in range(4):
                dfg.push_back(
                    gtd.CableTensionFactor(
                        gtd.internal.TorqueKey(ji, k).key(),
                        gtd.internal.PoseKey(self.ee_id(), k).key(),
                        gtd.internal.WrenchKey(self.ee_id(), ji, k).key(),
                        self.costmodel_torque, self.params.a_locs[ji], self.params.b_locs[ji]))
        return dfg

    def collocation_factors(self, ks=[], dt=0.01):
        """Create collocation factors to relate dynamics across time.  xdot = v, and vdot = a
        Primary variables:      TwistAccel(now)/Twist(now) <--> Twist(next)/Pose(next)
        Intermediate variables: dt (key 0) TODO(gerry): make a factor where this is a constant
        Prerequisite variables: None

        Args:
            ks (list, optional): Time step indices. Defaults to [].
            dt (float, optional): time between each time step. Defaults to 0.01.

        Returns:
            gtsam.NonlinearFactorGraph: the collocation factors
        """
        dfg = gtsam.NonlinearFactorGraph()
        for k in ks:
            dfg.push_back(
                gtd.EulerPoseCollocationFactor(
                    gtd.internal.PoseKey(self.ee_id(), k).key(),
                    gtd.internal.PoseKey(self.ee_id(), k + 1).key(),
                    gtd.internal.TwistKey(self.ee_id(), k).key(), 0,
                    self.costmodel_posecollo))
            dfg.push_back(
                gtd.EulerTwistCollocationFactor(
                    gtd.internal.TwistKey(self.ee_id(), k).key(),
                    gtd.internal.TwistKey(self.ee_id(), k + 1).key(),
                    gtd.internal.TwistAccelKey(self.ee_id(), k).key(), 0,
                    self.costmodel_twistcollo))
        dfg.push_back(gtd.PriorFactorDouble(0, dt, self.costmodel_dt))
        return dfg

    def priors_fk(self, ks=[], ls=[[]], ldots=[[]]):
        """Creates prior factors which correspond to solving the forward kinematics problem by
        specifying the joint angles and velocities.  To be used with kinematics_factors to optimize
        for the Pose and Twist.

        Args:
            ks (list, optional): Time step indices. Defaults to [].
            ls (list, optional): List of list joint angles for each time step. Defaults to [[]].
            ldots (list, optional): List of list of joint velocities for each time step. Defaults to
            [[]].

        Returns:
            gtsam.NonlinearFactorGraph: The forward kinematics prior factors
        """
        graph = gtsam.NonlinearFactorGraph()
        for k, l, ldot in zip(ks, ls, ldots):
            for ji, (lval, ldotval) in enumerate(zip(l, ldot)):
                graph.push_back(gtd.PriorFactorDouble(gtd.internal.JointAngleKey(ji, k).key(),
                                                      lval, self.costmodel_prior_l))
                graph.push_back(gtd.PriorFactorDouble(gtd.internal.JointVelKey(ji, k).key(),
                                                      ldotval, self.costmodel_prior_ldot))
        return graph

    def priors_ik(self, ks=[], Ts=[], Vs=[]):
        """Creates prior factors which correspond to solving the inverse kinematics problem by
        specifying the Pose/Twist of the end effector.  To be used with kinematics_factors to
        optimize for the joint angles and velocities.

        Args:
            ks (list, optional): Time step indices. Defaults to [].
            Ts (list, optional): List of Poses for each time step. Defaults to [[]].
            Vs (list, optional): List of Twists for each time step. Defaults to [[]].

        Returns:
            gtsam.NonlinearFactorGraph: The inverve kinematics prior factors
        """
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
        """Creates factors roughly corresponding to the inverse dynamics problem.  While strictly
        inverse dynamics in Lynch & Park refers to the problem of calculating joint accelerations
        given joint torques, temproarily this function is more convenient which directly relates
        constrains joint torques (to obtain twist accelerations when used with dynamics_factors).

        Args:
            ks (list, optional): Time step indices. Defaults to [].
            torquess (list, optional): List of list of joint torques for each time step. Defaults to
            [[]].

        Returns:
            gtsam.NonlinearFactorGraph: The inverse dynamics prior factors
        """
        graph = gtsam.NonlinearFactorGraph()
        for k, torques in zip(ks, torquess):
            for ji, torque in enumerate(torques):
                graph.push_back(gtd.PriorFactorDouble(gtd.internal.TorqueKey(ji, k).key(),
                                                      torque, self.costmodel_prior_tau))
        return graph

    def priors_fd(self, ks=[], VAs=[]):
        """Creates factors roughly corresponding to the forward dynamics problem.  While strictly
        forward dynamics in Lynch & Park refers to the problem of calculating joint torques given
        joint accelerations, temproarily this function is more convenient which directly relates
        constraints TwistAccelerations (to obtain joint torques when used with dynamics_factors).

        Args:
            ks (list, optional): Time step indices. Defaults to [].
            VAs (list, optional): List of twist accelerations for each time step. Defaults to [[]].

        Returns:
            gtsam.NonlinearFactorGraph: The forward dynamics prior factors
        """
        graph = gtsam.NonlinearFactorGraph()
        for k, VA in zip(ks, VAs):
            graph.push_back(gtd.PriorFactorVector6(
                gtd.internal.TwistAccelKey(self.ee_id(), k).key(),
                VA, self.costmodel_prior_twistaccel))
        return graph
