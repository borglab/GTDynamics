"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  cdpr_controller_tension_dist.py
@brief Controller for a cable robot based on computing the wrench necessary to track the desired
trajectory exactly followed by applying a tension distribution algorithm to compute the necessary
torque to achieve said wrench.
@author Frank Dellaert
@author Gerry Chen
"""

import gtsam
import gtdynamics as gtd
import numpy as np
import utils
from cdpr_controller import CdprControllerBase
from cdpr_planar import Cdpr

class CdprControllerTensionDist(CdprControllerBase):
    """Does whatever wrench is needed to follow the desired trajectory exactly.  Uses a simple
    least-squares minimum effort tension distribution algorithm (using GTSAM) to find the tensions
    necessary to enact such a wrench.
    """
    def __init__(self, cdpr, x0, pdes=[], dt=0.01, Q=None, R=np.array([1.])):
        """constructor

        Args:
            cdpr (Cdpr): cable robot object
            x0 (gtsam.Values): initial state
            pdes (list, optional): list of desired poses. Defaults to [].
            dt (float, optional): time step duration. Defaults to 0.01.
            R (np.ndarray, optional): Control cost (as a 1-vector). Defaults to np.array([1.]).
        """
        self.cdpr = cdpr
        self.pdes = pdes
        self.dt = dt
        self.R = R
        N = len(pdes)

    def update(self, values, t):
        """New control: returns the entire results vector, which contains the optimal open-loop
        control from the optimal trajectory.
        """
        return self.solve_one_step(self.cdpr,
                                   self.cdpr.ee_id(),
                                   self.pdes[t+1],
                                   t,
                                   TVnow=values,
                                   dt=self.dt,
                                   R=self.R)

    @staticmethod
    def solve_one_step(cdpr, lid, Tgoal, k, TVnow=None, lldotnow=None, dt=0.01, R=np.ones(1), debug=False):
        """Creates the factor graph for the tension distribution problem.  This essentially consists
        of creating a factor graph that describes the CDPR dynamics for this one timestep, then
        adding control cost factors.  Either the current pose/twist may be specified, or the current
        cable lengths/velocities may be specified (in which case FK will be used to calculate the
        pose/twist).

        Args:
            cdpr (Cdpr): cable robot object
            Tgoal (gtsam.Pose3): goal pose for the next time step
            TVnow (gtsam.Values, optional): The current pose and twist
            lldotnow (gtsam.Values, optional): The current cable lengths / speeds
            dt (float): time step duration
            R (np.ndarray): The control cost

        Returns:
            gtsam.Values: a Values object containing the control torques
        """

        # First solve for required TwistAccel
        VAres = CdprControllerTensionDist.solve_twist_accel(cdpr, lid, Tgoal, k, TVnow, lldotnow,
                                                            dt)

        # Now solve for required torques
        result = CdprControllerTensionDist.solve_torques(cdpr, lid, k, VAres, VAres, dt, R)

        # Debug
        if debug:
            print("My goal for k = {:d} is:".format(k), Tgoal.translation())
            print("\cur pose: ", gtd.Pose(TVnow, cdpr.ee_id(), k).translation())
            print("\tcur Twist: ", gtd.Twist(TVnow, cdpr.ee_id(), k)[3:])
            print("\tcur pose VA: ", gtd.Pose(VAres, cdpr.ee_id(), k).translation())
            print("\tnext pose VA: ", gtd.Pose(VAres, cdpr.ee_id(), k + 1).translation())
            print("\tcur Twist VA: ", gtd.Twist(VAres, cdpr.ee_id(), k)[3:])
            print("\tnext Twist VA: ", gtd.Twist(VAres, cdpr.ee_id(), k + 1)[3:])
            print("\tcur TwistAccel VA: ", gtd.TwistAccel(VAres, cdpr.ee_id(), k)[3:])
            print("\tcur TwistAccel res: ", gtd.TwistAccel(result, cdpr.ee_id(), k)[3:])
            print("\tcur Torques res: ", [gtd.TorqueDouble(result, ji, k) for ji in range(4)])

        return result

    @staticmethod
    def solve_twist_accel(cdpr, lid, Tgoal, k, TVnow=None, lldotnow=None, dt=0.01):
        fg = gtsam.NonlinearFactorGraph()

        # IK: either solve for current pose T given measurements, or use open-loop solution
        if lldotnow is not None:
            fg.push_back(cdpr.kinematics_factors(ks=[k]))
            fg.push_back(cdpr.priors_fk(ks=[k], values=lldotnow))
        else:
            fg.push_back(cdpr.priors_ik(ks=[k], values=TVnow))
        # pose constraints: must reach next pose T
        fg.push_back(gtsam.PriorFactorPose3(gtd.PoseKey(lid, k+1).key(),
                                            Tgoal, cdpr.costmodel_prior_pose))
        # collocation: given current+next Ts, solve for current+next Vs and current VAs
        fg.push_back(cdpr.collocation_factors(ks=[k], dt=dt))

        xk = gtsam.Values()
        xk.insert(0, dt)
        if lldotnow is not None:
            utils.InsertJointAngles(xk, k, lldotnow)
            utils.InsertJointVels(xk, k, lldotnow)
            gtd.InsertPose(xk, lid, k, gtsam.Pose3())
            gtd.InsertTwist(xk, lid, k, np.zeros(6))
        else:
            utils.InsertPose(xk, lid, k, TVnow)
            utils.InsertTwist(xk, lid, k, TVnow)
        gtd.InsertPose(xk, lid, k+1, Tgoal)
        gtd.InsertTwist(xk, lid, k+1, np.zeros(6))
        gtd.InsertTwistAccel(xk, lid, k, np.zeros(6))

        return gtsam.LevenbergMarquardtOptimizer(fg, xk, utils.MyLMParams()).optimize()

    @staticmethod
    def solve_torques(cdpr: Cdpr, lid, k, VAnow, TVnow, dt, R):
        fg = gtsam.NonlinearFactorGraph()
        # priors
        fg.push_back(cdpr.priors_ik(ks=[k], values=TVnow))
        fg.push_back(cdpr.priors_id_va(ks=[k], values=VAnow))

        # dynamics: given VA, solve for torque/wrenches
        fg.push_back(cdpr.dynamics_factors(ks=[k]))
        # redundancy resolution: control costs
        for ji in range(4):
            fg.push_back(
                gtd.PriorFactorDouble(
                    gtd.TorqueKey(ji, k).key(),  #
                    0.0,
                    gtsam.noiseModel.Diagonal.Precisions(R)))

        # tmp initial guess
        xk = gtsam.Values()
        utils.InsertPose(xk, lid, k, TVnow)
        utils.InsertTwist(xk, lid, k, TVnow)
        gtd.InsertTwistAccel(xk, lid, k, np.zeros(6))
        for ji in range(4):
            gtd.InsertJointVel(xk, ji, k, 0)
            gtd.InsertJointAccel(xk, ji, k, 1)
            gtd.InsertTorque(xk, ji, k, 1)
            gtd.InsertTension(xk, ji, k, 50)
            gtd.InsertWrench(xk, lid, ji, k, np.zeros(6))

        # optimize
        return gtsam.LevenbergMarquardtOptimizer(fg, xk, utils.MyLMParams()).optimize()
