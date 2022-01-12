"""
GTDynamics Copyright 2021, Georgia Tech Research Corporation,
Atlanta, Georgia 30332-0415
All Rights Reserved
See LICENSE for the license information

@file  cdpr_planar_sim.py
@brief Simulation for a planar cable robot.  Runs dynamics forward in time to simulate a cable robot
@author Frank Dellaert
@author Gerry Chen
"""

import gtsam
import gtdynamics as gtd
import numpy as np
from utils import MyLMParams, InitValues

class CdprSimulator:
    """Simulates a cable robot forward in time, given a robot, initial state, and controller.

    Pose/Twist  ->  l/ldot  ->  torques  ->  Wrenches/TwistAccel  -->  next Pose/Twist
                IK     Controller        ID                   Collocation

    Example usage:
        sim = CdprSimulation(robot, x0, controller)
        for i in range(10):
            results = sim.step()
            print(gtd.Pose(results, robot.ee_id(), i))
        sim.reset()
        results = sim.run()

    Args:
        cdpr (Cdpr): cable robot object
        xInit (gtsam.Values): initial state
        controller (Controller): controller object
        dt (float, optional): time step duration
        N (int, optional): number of time steps to simulate
        verbose (bool, optional): whether or not to print out debug info
    
    Returns:
        gtsam.Values: The set of fully-resolved variables for the robot over tehe specified time
        steps
    """
    def __init__(self, cdpr, x0, controller, dt=0.01):
        """Constructs a simulator object

        Args:
            cdpr (Cdpr): cable robot object
            x0 (gtsam.Values): initial state (must contain Pose and Twist at t=0)
            controller (CdprControllerBase): controller object
            dt (float, optional): time step duration. Defaults to 0.01.
        """
        self.cdpr = cdpr
        self.x0 = x0
        self.controller = controller
        self.dt = dt
        self.reset()

    @staticmethod
    def update_kinematics(cdpr, x, k):
        """Runs IK to solve for the cable lengths and velocities at time step k.  Modifies x
        inplace!!!

        Args:
            x (gtsam.Values): Values object containing at least the current Pose and current Twist
            k (int): current time step

        Returns:
            tuple(gtsam.NonlinearFactorGraph, gtsam.Values): the factor graph used to perform IK
            and the solution values which includes the current Pose, Twist, JointAngles, and
            JointVels.
        """
        fg = gtsam.NonlinearFactorGraph()
        lid = cdpr.ee_id()
        # local copy of values
        xk = gtsam.Values()
        gtd.InsertPose(xk, lid, k, gtd.Pose(x, lid, k))
        gtd.InsertTwist(xk, lid, k, gtd.Twist(x, lid, k))
        # IK for this time step, graph
        fg.push_back(cdpr.kinematics_factors(ks=[k]))
        fg.push_back(cdpr.priors_ik(ks=[k], values=xk))
        # IK initial estimate
        for j in range(4):
            gtd.InsertJointAngle(x, j, k, 0)
            gtd.InsertJointVel(x, j, k, 0)
        # IK solve
        result = gtsam.LevenbergMarquardtOptimizer(fg, xk, MyLMParams()).optimize()
        assert abs(fg.error(result)) < 1e-20, "inverse kinematics didn't converge"
        xk.update(result)
        return fg, xk

    @staticmethod
    def update_dynamics(cdpr, x, u, k, dt):
        """Runs ID to solve for the twistAccel, and also runs collocation to get the next timestep
        Pose/Twist.  Modifies x inplace!!!

        Args:
            cdpr (Cdpr): the cable robot
            x (gtsam.Values): Values object containing at least the current Pose and Twist
            u (gtsam.Values): The current joint torques
            k (int): The current time index
            dt (float): the time slice duration

        Returns:
            tuple(gtsam.NonlinearFactorGraph, gtsam.Values): the factor graph used to update the
            dynamics, and the solution Values which consists of the Pose, Twist, torque, TwistAccel,
            next Pose, and next Twist.
        """
        fg = gtsam.NonlinearFactorGraph()
        lid = cdpr.ee_id()
        # local copy of values
        xd = gtsam.Values()
        xd.insert(0, dt)
        gtd.InsertPose(xd, lid, k, gtd.Pose(x, lid, k))
        gtd.InsertTwist(xd, lid, k, gtd.Twist(x, lid, k))
        # FD for this timestep + collocation to next time step
        fg.push_back(cdpr.dynamics_factors(ks=[k]))
        fg.push_back(cdpr.collocation_factors(ks=[k], dt=dt))
        # priors (pose/twist and torque inputs)
        fg.push_back(cdpr.priors_ik(ks=[k], values=xd))
        fg.push_back(cdpr.priors_fd(ks=[k], values=u))
        # FD initial guess
        for ji in range(4):
            gtd.InsertJointVel(xd, ji, k, 0)
            gtd.InsertJointAccel(xd, ji, k, 0)
            gtd.InsertTorque(xd, ji, k, gtd.Torque(u, ji, k))
            gtd.InsertTension(xd, ji, k,
                                    gtd.Torque(u, ji, k) / cdpr.params.winch_params.radius_)
            gtd.InsertWrench(xd, cdpr.ee_id(), ji, k, np.zeros(6))
        gtd.InsertPose(xd, cdpr.ee_id(), k + 1, gtd.Pose(x, lid, k))
        gtd.InsertTwist(xd, cdpr.ee_id(), k + 1, gtd.Twist(x, lid, k))
        gtd.InsertTwistAccel(xd, cdpr.ee_id(), k, np.zeros(6))
        # optimize
        result = gtsam.LevenbergMarquardtOptimizer(fg, xd, MyLMParams()).optimize()
        assert abs(fg.error(result)) < 1e-15, "dynamics simulation didn't converge: {:.5e}".format(
            abs(fg.error(result)))
        xd.update(result)
        return fg, xd

    def step(self, verbose=False):
        """Performs one time step of the simulation, which consists of:
        1. Using IK to calculate the current cable lengths and velocities
        2. Calling the controller to get the torque inputs
        3. Updating the dynamics to get the current TwistAccel and next Pose+Twist.

        Args:
            verbose (bool, optional): True to print debug information. Defaults to False.

        Returns:
            gtsam.Values: The new values object containing the current state and next Pose+Twist.
        """
        # setup
        x, lid, k, dt = self.x, self.cdpr.ee_id(), self.k, self.dt

        # kinematics
        _, xk = self.update_kinematics(self.cdpr, x, k)
        # controller
        u = self.controller.update(xk, k)
        # dynamics
        _, xd = self.update_dynamics(self.cdpr, x, u, k, dt)

        # update x
        for ji in range(4):
            gtd.InsertJointAngleDouble(x, ji, k, gtd.JointAngleDouble(xk, ji, k))
            gtd.InsertJointVelDouble(x, ji, k, gtd.JointVelDouble(xk, ji, k))
            gtd.InsertTorqueDouble(x, ji, k, gtd.TorqueDouble(u, ji, k))
        gtd.InsertTwistAccel(x, lid, k, gtd.TwistAccel(xd, lid, k))
        gtd.InsertPose(x, lid, k + 1, gtd.Pose(xd, lid, k + 1))
        gtd.InsertTwist(x, lid, k + 1, gtd.Twist(xd, lid, k + 1))

        # debug
        if verbose:
            print('time step: {:4d}'.format(k), end='  --  ')
            print('EE position: ({:.2f}, {:.2f}, {:.2f})'.format(
                *gtd.Pose(x, lid, k).translation()), end='  --  ')
            print('next: ({:.2f}, {:.2f}, {:.2f})'.format(
                *gtd.Pose(x, lid, k+1).translation()), end='  --  ')
            print('control torques: {:.2e},   {:.2e},   {:.2e},   {:.2e}'.format(
                *[gtd.Torque(u, ji, k) for ji in range(4)]))

        self.k += 1
        return x

    def run(self, N=100, verbose=False):
        """Runs the simulation

        Args:
            N (int, optional): Number of iterations to run. Defaults to 100.
            verbose (bool, optional): True to print debug information. Defaults to False.

        Returns:
            gtsam.Values: The values object containing all the data from the simulation.
        """
        for _ in range(N):
            self.step(verbose=verbose)
        return self.x

    def reset(self):
        self.fg = gtsam.NonlinearFactorGraph()
        self.x = gtsam.Values(self.x0)
        self.k = 0
