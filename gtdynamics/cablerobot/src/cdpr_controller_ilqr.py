"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  cdpr_controller_ilqr.py
@brief Optimal controller for a cable robot.  Solved by creating a factor graph and adding state
objectives and control costs, then optimizing
@author Frank Dellaert
@author Gerry Chen
"""

import gtsam
import gtdynamics as gtd
import numpy as np
import utils
from cdpr_controller import CdprControllerBase
from scipy.linalg import solve_triangular
import time


class CdprControllerIlqr(CdprControllerBase):
    """Precomputes the open-loop trajectory then just calls on that for each update.
    """
    def __init__(self,
                 cdpr,
                 x0,
                 pdes=[],
                 dt=0.01,
                 Q=None,
                 R=np.array([1.]),
                 x_guess=None,
                 debug=False):
        """constructor

        Args:
            cdpr (Cdpr): cable robot object
            x0 (gtsam.Values): initial state
            pdes (list, optional): list of desired poses. Defaults to [].
            dt (float, optional): time step duration. Defaults to 0.01.
            Q (np.ndarray, optional): State objective cost (as a vector). Defaults to None, which
            denotes a constrained noise model.
            R (np.ndarray, optional): Control cost (as a 1-vector). Defaults to np.array([1.]).
        """
        self.cdpr = cdpr
        self.pdes = pdes
        self.dt = dt

        # create iLQR graph
        fg = self.create_ilqr_fg(cdpr, x0, pdes, dt, Q, R)

        # initial guess
        if x_guess is None:
            x_guess = gtsam.Values()
            for k, pdes_ in enumerate(pdes):
                gtd.InsertPose(x_guess, cdpr.ee_id(), k, pdes_)
                for ji in range(4):
                    l = np.linalg.norm(cdpr.params.a_locs[ji] - pdes_.translation())
                    gtd.InsertJointAngle(x_guess, ji, k, l)
        x_guess = utils.InitValues(fg, x_guess, dt=dt)

        # optimize
        params = utils.MyLMParams(None)
        # params.setRelativeErrorTol(1e-10)
        # params.setAbsoluteErrorTol(1e-10)
        self.optimizer = gtsam.LevenbergMarquardtOptimizer(fg, x_guess, params)
        def optimize(optimizer): # so this shows up in the profiler
            return optimizer.optimize()
        tstart = time.time()
        self.result = optimize(self.optimizer)
        tend = time.time()

        # gains
        self.gains_ff = self.extract_gains_ff(cdpr, fg, self.result, len(self.pdes))

        # print debug info
        if debug:
            print("FG size: ", fg.size())
            print("\tNumber of variables: ", len(x_guess.keys()))
            print("\tNumber of timesteps: ", len(pdes))
            print("\tTime to optimize: ", tend - tstart)

    def update(self, values, t):
        """New control: returns the entire results vector, which contains the optimal open-loop
        control from the optimal trajectory.
        """
        K, uff, Vff, Tff = self.gains_ff[t]
        # compute dx
        Vhat = gtd.Twist(values, self.cdpr.ee_id(), t)
        That = gtd.Pose(values, self.cdpr.ee_id(), t)
        dx = np.hstack((Vhat - Vff, Tff.localCoordinates(That)))
        #compute u
        u = K @ dx + uff
        # populate into values object
        result = gtsam.Values()
        for ji in range(4):
            gtd.InsertTorque(result, ji, t, u[ji])
        return result

    @staticmethod
    def create_ilqr_fg(cdpr, x0, pdes, dt, Q, R):
        """Creates the factor graph for the iLQR problem.  This essentially consists of creating a
        factor graph that describes the CDPR dynamics over all time steps, then adding state
        objective and control cost factors.

        Args:
            cdpr (Cdpr): cable robot object
            x0 (gtsam.Values): initial Pose/Twist of the cable robot
            pdes (List[gtsam.Pose3]): desired poses of the cdpr
            dt (float): time step duration
            Q (Union[np.ndarray, None]): The state objective cost (None for hard constraint)
            R (np.ndarray): The control cost

        Returns:
            gtsam.NonlinearFactorGraph: The factor graph corresponding to the iLQR problem.
        """
        N = len(pdes)
        # initial conditions
        fg = cdpr.priors_ik(ks=[0],
                            Ts=[gtd.Pose(x0, cdpr.ee_id(), 0)],
                            Vs=[gtd.Twist(x0, cdpr.ee_id(), 0)])
        # dynamics
        fg.push_back(cdpr.all_factors(N, dt))
        # control costs
        tmid = (cdpr.params.tmin + cdpr.params.tmax) / 2
        for k in range(N):
            for ji in range(4):
                fg.push_back(
                    gtd.PriorFactorDouble(gtd.TorqueKey(ji, k).key(), tmid,
                                          gtsam.noiseModel.Diagonal.Precisions(R)))
        # state objective costs
        cost_x = gtsam.noiseModel.Isotropic.Sigma(6, 0.001) if Q is None else \
            gtsam.noiseModel.Diagonal.Precisions(Q)
        for k in range(N):
            fg.push_back(
                gtsam.PriorFactorPose3(
                    gtd.PoseKey(cdpr.ee_id(), k).key(), pdes[k], cost_x))
        return fg

    @staticmethod
    def extract_bayesnets(cdpr, fg, openloop_results, N):
        lid = cdpr.ee_id()
        gfg = fg.linearize(openloop_results)

        # ordering
        ordering = []
        # ordering.append(gtsam.Ordering())
        for t in range(N - 1, -1, -1):
            # stuff we don't care about
            ordering.append(gtsam.Ordering())
            for ji in range(4):
                ordering[-1].push_back(gtd.JointAngleKey(ji, t).key())
            for ji in range(4):
                ordering[-1].push_back(gtd.JointVelKey(ji, t).key())
            for ji in range(4):
                ordering[-1].push_back(gtd.JointAccelKey(ji, t).key())
            ordering[-1].push_back(gtd.TwistAccelKey(lid, t).key())
            for ji in range(4):
                ordering[-1].push_back(gtd.WrenchKey(lid, ji, t).key())
            for ji in range(4):
                ordering[-1].push_back(gtd.TensionKey(ji, t).key())
            # control variables
            ordering.append(gtsam.Ordering())
            for ji in range(4):
                ordering[-1].push_back(gtd.TorqueKey(ji, t).key())
            # measurement inputs
            ordering.append(gtsam.Ordering())
            ordering[-1].push_back(gtd.TwistKey(lid, t).key())
            ordering[-1].push_back(gtd.PoseKey(lid, t).key())
        ordering.append(gtsam.Ordering())
        ordering[-1].push_back(0)

        # eliminate
        return gtd.BlockEliminateSequential(gfg, ordering), reversed(range(3, 4*N, 4))

    @staticmethod
    def extract_gains(cdpr, fg, openloop_results, N):
        """Extracts the locally linear optimal feedback control gains

        Args:
            cdpr (Cdpr): cable robot object
            fg (gtsam.NonlinearFactorGraph): The iLQR factor graph
            openloop_results (gtsam.Values): The open-loop optimal trajectory and controls
            N (int): The total number of timesteps in the iLQR factor graph
        Returns:
            gains (List[np.ndarray]): The feedback gains in the form u = K*(x-xff) + uff
        """
        lid = cdpr.ee_id()
        net, u_inds = CdprControllerIlqr.extract_bayesnets(cdpr, fg, openloop_results, N)

        # extract_gains
        gains = [None for t in range(N)]
        # for t, neti in enumerate(u_inds):
        # 0 mod 3: ("stuff we do care about") -> ("stuff we don't care about")
        # 1 mod 3: (twist, pose) -> torques
        # 2 mod 3: (prev state) -> (twist, pose)   aka collocation update
        for t, (neti, netu, netx) in enumerate(
                zip(
                    reversed(range(0, 3 * N, 3)),  #
                    reversed(range(1, 3 * N, 3)),
                    reversed(range(2, 3 * N, 3)))):
            ucond = net.at(netu)
            u_K_x = solve_triangular(ucond.R(), -ucond.S()[:, -6:])
            u_K_v = solve_triangular(ucond.R(), -ucond.S()[:, -12:-6])
            gains[t] = np.hstack((u_K_v, u_K_x))
        return gains

    @staticmethod
    def extract_uff(results, N):
        """Extracts the feedforward control terms in a more python-friendly format

        Args:
            results (gtsam.Values): contains the result of optimizing an iLQR factor graph
            N (int): The total number of timesteps in the iLQR factor graph

        Returns:
            List[np.ndarray]: Feedforward control terms.
        """
        uff = []
        for t in range(N):
            uff.append(np.array([gtd.Torque(results, ji, t) for ji in range(4)]))
        return uff

    @staticmethod
    def extract_xff(cdpr, results, N):
        """Extracts the feedforward trajectory terms in a more python-friendly format

        Args:
            cdpr (Cdpr): cable robot
            results (gtsam.Values): contains the result of optimizing an iLQR factor graph
            N (int): The total number of timesteps in the iLQR factor graph

        Returns:
            List[Tuple[np.ndarray, gtsam.Pose3]]: Feedforward twist and pose terms, where each
            element of the list contains a tuple of (Twist, Pose).
        """
        xff = []
        for t in range(N):
            xff.append((
                gtd.Twist(results, cdpr.ee_id(), t),  #
                gtd.Pose(results, cdpr.ee_id(), t)))
        return xff

    @staticmethod
    def extract_gains_ff(cdpr, fg, openloop_results, N):
        """Extracts both the gains and the feedforward controls/trajectory terms of the form
        u = K*(x-xff) + uff

        Args:
            cdpr (Cdpr): cable robot object
            fg (gtsam.NonlinearFactorGraph): The iLQR factor graph
            openloop_results (gtsam.Values): The open-loop optimal trajectory and controls
            N (int): The total number of timesteps in the iLQR factor graph
        
        Returns:
            List[Tuple[np.ndarray, np.ndarray, np.ndarray, gtsam.Pose3]]: Gains and feedforward
            terms as a list of tuples: (K, tension_ff, twist_ff, pose_ff) where u = K*(x-xff) + uff
            and x stacks twist on top of pose.
        """
        return list(
            zip(CdprControllerIlqr.extract_gains(cdpr, fg, openloop_results, N),
                CdprControllerIlqr.extract_uff(openloop_results, N),
                *zip(*CdprControllerIlqr.extract_xff(cdpr, openloop_results, N))))
