"""
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file main.py
 * @brief Trajectory optimization for an inverted pendulum.
 * @author Alejandro Escontrela and Frank Dellaert
"""

import argparse
import math

import gtdynamics as gtd
import gtsam
import numpy as np
import pandas as pd

Isotropic = gtsam.noiseModel.Isotropic


def jointAngleKey(id, t=0):
    return gtd.internal.JointAngleKey(id, t).key()


def jointVelKey(id, t=0):
    return gtd.internal.JointVelKey(id, t).key()


def jointAccelKey(id, t=0):
    return gtd.internal.JointAccelKey(id, t).key()


def torqueKey(id, t=0):
    return gtd.internal.TorqueKey(id, t).key()


def run(args):
    # Load the inverted pendulum.
    ip = gtd.CreateRobotFromFile("inverted_pendulum.urdf", "inverted_pendulum")
    j1_id = ip.joint("j1").id()
    ip.fixLink("l1")

    T = 3.0  # seconds
    dt = 1. / 100  # Time horizon (s) and timestep duration (s).
    t_steps = math.ceil(T / dt)  # Timesteps.

    # Noise models:
    dynamics_model = Isotropic.Sigma(1, 1e-5)    # Dynamics constraints.
    objectives_model = Isotropic.Sigma(1, 1e-2)  # Objectives.
    control_model = Isotropic.Sigma(1, 1e-1)     # Controls.

    # Specify initial conditions and goal state.
    theta_i = 0
    dtheta_i = 0
    ddtheta_i = 0
    theta_T = math.pi
    dtheta_T = 0
    ddtheta_T = 0

    # Create trajectory factor graph.
    gravity = (0, 0, -9.8)
    planar_axis = (1, 0, 0)
    graph_builder = gtd.DynamicsGraph(gravity, planar_axis)
    graph = graph_builder.trajectoryFG(ip, t_steps, dt)

    # Add initial conditions to trajectory factor graph.
    graph.addPriorDouble(jointAngleKey(j1_id, 0), theta_i, dynamics_model)
    graph.addPriorDouble(jointVelKey(j1_id, 0), dtheta_i, dynamics_model)

    # Add state and min torque objectives to trajectory factor graph.
    graph.addPriorDouble(jointVelKey(j1_id, t_steps),
                         dtheta_T, objectives_model)
    graph.addPriorDouble(jointAccelKey(j1_id, t_steps),
                         dtheta_T, objectives_model)
    apply_theta_objective_all_dt = False
    graph.addPriorDouble(jointAngleKey(j1_id, t_steps),
                         theta_T, objectives_model)
    if apply_theta_objective_all_dt:
        for t in range(t_steps+1):
            graph.addPriorDouble(jointAngleKey(j1_id, t),
                                 theta_T, objectives_model)

    for t in range(t_steps+1):
        graph.add(gtd.MinTorqueFactor(torqueKey(j1_id, t), control_model))

    # Initialize solution.
    init_vals = gtd.ZeroValuesTrajectory(ip, t_steps, 0, 0.0, None)
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_vals, params)
    results = optimizer.optimize()

    # TODO(frank): Log the joint angles, velocities, accels, torques, and current goal pose.
    def time(values, id, t):
        return t*dt
    data = {key: [fn(results, j1_id, t) for t in range(t_steps+1)]
            for (key, fn) in [("t", time), ("theta", gtd.JointAngleDouble), ("dtheta", gtd.JointVelDouble),
                              ("ddtheta", gtd.JointAccelDouble), ("tau", gtd.TorqueDouble)]}
    df = pd.DataFrame(data)
    df.to_csv("traj.csv", index=False, float_format="%.6f")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run inverted pendulum.')
    args = parser.parse_args()
    run(args)
