import gtsam
import gtdynamics as gtd
import numpy as np

def zerovalues(lid, t, dt):
    init = gtsam.Values()
    init.insertDouble(0, dt)
    for j in range(4):
        gtd.InsertJointAngleDouble(init, j, t, 0)
        gtd.InsertJointVelDouble(init, j, t, 0)
        gtd.InsertTorqueDouble(init, j, t, 0)
        gtd.InsertWrench(init, lid, j, t, np.zeros(6))
    gtd.InsertPose(init, lid, t, Pose3(Rot3(), (1.5, 0, 1.5)))
    gtd.InsertTwist(init, lid, t, np.zeros(6))
    gtd.InsertTwistAccel(init, lid, t, np.zeros(6))

def cdpr_sim(cdpr, xInit, controller, dt=0.01, N=100, verbose=False):
    fg = gtsam.NonlinearFactorGraph()
    x = gtsam.Values(xInit)
    for k in range(N):
        if verbose:
            print('time step: {:4d}   --   EE position: ({:.2f}, {:.2f}, {:.2f})'.format(
                k,
                *gtd.Pose(x, cdpr.ee_id(), k).translation()))
        # IK for this time step, graph
        fg.push_back(cdpr.kinematics_factors(ks=[k]))
        fg.push_back(
            cdpr.priors_ik(ks=[k],
                           Ts=[gtd.Pose(x, cdpr.ee_id(), k)],
                           Vs=[gtd.Twist(x, cdpr.ee_id(), k)]))
        # IK initial estimate
        for j in range(4):
            gtd.InsertJointAngleDouble(x, j, k, 0)
            gtd.InsertJointVelDouble(x, j, k, 0)
        # IK solve
        result = gtsam.LevenbergMarquardtOptimizer(fg, x).optimize()
        assert abs(fg.error(result)) < 1e-20, "inverse kinematics didn't converge"
        x.update(result)
        if k == 0:
            x.insertDouble(0, dt)
        # controller
        u = controller.update(x, k)
        # ID for this timestep + collocation to next time step
        fg.push_back(cdpr.dynamics_factors(ks=[k]))
        fg.push_back(cdpr.collocation_factors(ks=[k], dt=dt))
        # ID priors (torque inputs)
        fg.push_back(
            cdpr.priors_id(ks=[k], torquess=[[gtd.TorqueDouble(u, ji, k) for ji in range(4)]]))
        # ID initial guess
        for ji in range(4):
            gtd.InsertTorqueDouble(x, ji, k, gtd.TorqueDouble(u, ji, k))
            gtd.InsertWrench(x, cdpr.ee_id(), ji, k, np.zeros(6))
        gtd.InsertPose(x, cdpr.ee_id(), k+1, gtsam.Pose3(gtsam.Rot3(), (1.5, 0, 1.5)))
        gtd.InsertTwist(x, cdpr.ee_id(), k+1, np.zeros(6))
        gtd.InsertTwistAccel(x, cdpr.ee_id(), k, np.zeros(6))
        # optimize
        result = gtsam.LevenbergMarquardtOptimizer(fg, x).optimize()
        assert abs(fg.error(result)) < 1e-20, "dynamics simulation didn't converge"
        x.update(result)
        # print(result)

    return x
