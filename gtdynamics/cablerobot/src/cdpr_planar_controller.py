import gtsam
import gtdynamics as gtd
import numpy as np
import utils

class CdprController:
    def __init__(self, cdpr, x0, pdes=[], dt=0.01):
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
                                          gtsam.noiseModel.Diagonal.Precisions(np.array([1]))))
        # state objective costs
        # cost_x = gtsam.noiseModel.Diagonal.Precisions(np.array([0, 1, 0, 100, 0, 100.]))
        cost_x = gtsam.noiseModel.Constrained.All(6)
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
