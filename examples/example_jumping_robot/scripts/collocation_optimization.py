"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  collocation_optimization.py
 * @brief Create a vertical jumping trajectory, simulate to get initial values,
          optimize to satisfy all collcoation constraints.
 * @author Yetong Zhang
"""

import gtsam
import gtdynamics as gtd
import numpy as np

import os, sys, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.jumping_robot import Actuator, JumpingRobot
from src.jr_graph_builder import JRGraphBuilder
from src.jr_simulator import JRSimulator


def vertical_jump_simulation(jr, controls):
    """ Simulate vertical jump trajectory. """
    num_steps = 1
    dt = 0.005
    jr_simulator = JRSimulator(jr.yaml_file_path, jr.init_config)
    values, step_phases = jr_simulator.simulate(num_steps, dt, controls)
    values.insertDouble(gtd.PhaseKey(0).key(), dt)
    # values.insertDouble(gtd.PhaseKey(3).key(), dt)
    return values, step_phases

def vertical_jump_collocation(jr, controls, init_values, step_phases):
    """ Collocation optimization for vertical jump. """
    jr_graph_builder = JRGraphBuilder()

    # trajectory graph
    collocation = gtd.CollocationScheme.Euler
    graph = jr_graph_builder.trajectory_graph(jr, step_phases, collocation)

    # control priors
    graph.push_back(jr_graph_builder.control_priors(jr, controls))

    # goal factors
    num_steps = len(step_phases)
    phase0_key = gtd.PhaseKey(0).key()
    graph.add(gtd.PriorFactorDouble(phase0_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))
    # graph.push_back(jr_graph_builder.vertical_jump_goal_factors(jr, num_steps))

    
    for f_idx in range(graph.size()):
        factor = graph.at(f_idx)
        if factor.error(init_values) > 1:
            graph_tmp = gtsam.NonlinearFactorGraph()
            graph_tmp.add(factor)
            gtd.DynamicsGraph.printGraph(graph_tmp)
            print("error", factor.error(init_values))

    # optimization
    # params = gtsam.LevenbergMarquardtParams()
    # params.setVerbosityLM("SUMMARY")
    # optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, params)

    print("init error: ", graph.error(init_values))
    optimizer = gtsam.GaussNewtonOptimizer(graph, init_values)
    results = optimizer.optimize()


    return results

def main():
    """ Main file. """
    # create jumping robot
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"
    theta = np.pi/3
    rest_angles = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
    init_angles = rest_angles
    init_vels = [0, 0, 0, 0, 0, 0]
    torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.55))
    torso_twist = np.zeros(6)
    P_s_0 = 65 * 6894.76/1000
    init_config = JumpingRobot.create_init_config(
        torso_pose, torso_twist, rest_angles, init_angles, init_vels, P_s_0)
    jr = JumpingRobot(yaml_file_path, init_config)

    # create controls
    Tos = [0, 0, 0, 0]
    Tcs = [1, 1, 1, 1]
    controls = JumpingRobot.create_controls(Tos, Tcs)

    # simulation
    sim_values, step_phases = vertical_jump_simulation(jr, controls)

    print("step_phases", step_phases)

    # collocation optimization
    collo_values = vertical_jump_collocation(jr, controls, sim_values, step_phases)

    # visualize

if __name__ == "__main__":
    main()