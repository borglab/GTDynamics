"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  cdpr_planar_controller.py
@brief Optimal controller for a cable robot.  Solved by creating a factor graph and adding state
objectives and control costs, then optimizing
@author Frank Dellaert
@author Gerry Chen
"""

import gtsam
import gtdynamics as gtd
import numpy as np
import utils

class CdprControllerBase:
    """Interface for cable robot controllers
    """
    @property
    def update(self, values, t):
        """gives the new control input given current measurements

        Args:
            values (gtsam.Values): values object will contain at least the current Pose and Twist,
            but should often also include the current joint angles and velocities
            t (int): The current time index (discrete time index)

        Returns:
            gtsam.Values: A values object which contains the joint torques for this time step.

        Raises:
            NotImplementedError: Derived classes must override this function
        """
        raise NotImplementedError("CdprControllers need to implement the `update` function")

class CdprController(CdprControllerBase):
    """Precomputes the open-loop trajectory
    then just calls on that for each update.
    """
    def __init__(self, cdpr, x0, pdes=[], dt=0.01, Q=None, R=np.array([1.])):
        """constructor

        Args:
            cdpr (Cdpr): cable robot object
            x0 (gtsam.Values): initial state
            pdes (list, optional): list of desired poses. Defaults to [].
            dt (float, optional): time step duration. Defaults to 0.01.
            Q (np.ndarray, optional): State objective cost (as a vector). Defaults to None.
            R (np.ndarray, optional): Control cost (as a 1-vector). Defaults to np.array([1.]).
        """        
        self.cdpr = cdpr
        self.pdes = pdes
        self.dt = dt

        # iLQR factor graph
        # dynamics
        N = len(pdes)
        fg = cdpr.priors_ik(ks=[0],
                            Ts=[gtd.Pose(x0, cdpr.ee_id(), 0)],
                            Vs=[gtd.Twist(x0, cdpr.ee_id(), 0)])
        fg.push_back(cdpr.kinematics_factors(ks=range(N)))
        fg.push_back(cdpr.dynamics_factors(ks=range(N)))
        fg.push_back(cdpr.collocation_factors(ks=range(N-1), dt=dt))
        # control costs
        for k in range(N):
            for ji in range(4):
                fg.push_back(
                    gtd.PriorFactorDouble(gtd.internal.TorqueKey(ji, k).key(), 0.0,
                                          gtsam.noiseModel.Diagonal.Precisions(R)))
        # state objective costs
        if Q is None:
            cost_x = gtsam.noiseModel.Constrained.All(6)
        else:
            cost_x = gtsam.noiseModel.Diagonal.Precisions(Q)
            # cost_x = gtsam.noiseModel.Diagonal.Precisions(np.array([0, 1, 0, 100, 0, 100.]))
        for k in range(N):
            fg.push_back(
                gtsam.PriorFactorPose3(
                    gtd.internal.PoseKey(cdpr.ee_id(), k).key(), pdes[k], cost_x))
        # initial guess
        init = utils.zerovalues(cdpr.ee_id(), range(N), dt=dt)
        # optimize
        self.optimizer = gtsam.LevenbergMarquardtOptimizer(fg, init)
        self.result = self.optimizer.optimize()
        self.fg = fg

    def update(self, values, t):
        return self.result
