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
from src.helpers import OptimizeLM
from src.jr_visualizer import visualize_jr_trajectory

dt = 1e-3

def vertical_jump_simulation(jr, controls):
    """ Simulate vertical jump trajectory. """
    jr_simulator = JRSimulator(jr.yaml_file_path, jr.init_config)
    sim_values, step_phases = jr_simulator.simulate_to_high(dt, controls)
    phase0_key = gtd.PhaseKey(0).key()
    phase3_key = gtd.PhaseKey(3).key()
    sim_values.insertDouble(phase0_key, dt)
    sim_values.insertDouble(phase3_key, dt)
    return sim_values, step_phases



def vertical_jump_optimization(jr, controls, sim_values, step_phases):
    """ Collocation optimization for vertical jump. """
    jr_graph_builder = JRGraphBuilder()
    collocation = gtd.CollocationScheme.Trapezoidal
    graph = jr_graph_builder.trajectory_graph(jr, step_phases, collocation)
    # graph.push_back(jr_graph_builder.control_priors(jr, controls))

    # goal factors
    num_steps = len(step_phases)
    graph.push_back(jr_graph_builder.vertical_jump_max_height_factors(jr, num_steps))

    goal_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 1.52))
    graph.push_back(jr_graph_builder.target_pose_goal_factor(jr, num_steps, goal_pose))

    # symmetry factor for control
    graph.push_back(jr_graph_builder.control_symmetry_factors(jr))
    

    # for f_idx in range(graph.size()):
    #     factor = graph.at(f_idx)
    #     if factor.error(init_values) > 1:
    #         graph_tmp = gtsam.NonlinearFactorGraph()
    #         graph_tmp.add(factor)
    #         gtd.DynamicsGraph.printGraph(graph_tmp)
    #         print("error", factor.error(init_values))

    # graph1 = gtsam.NonlinearFactorGraph(graph)
    # graph1.push_back(jr_graph_builder.control_priors(jr, controls))
    # results1 = OptimizeLM(graph1, sim_values)
    # results = OptimizeLM(graph, results1)

    results = OptimizeLM(graph, sim_values)

    return results


def get_final_torso_pose(jr, values, step_phases):
    num_steps = len(step_phases)
    torso_i = jr.robot.link("torso").id()
    torso_pose = gtd.Pose(values, torso_i, num_steps)
    return torso_pose


def main():
    """ Main file. """
    # create jumping robot
    yaml_file_path = JumpingRobot.icra_yaml()
    init_config = JumpingRobot.icra_init_config()
    jr = JumpingRobot(yaml_file_path, init_config)

    # create controls
    Tos = [0, 0, 0, 0]
    Tcs = [0.098, 0.098, 0.098, 0.098]
    controls = JumpingRobot.create_controls(Tos, Tcs)

    # simulation
    sim_values, step_phases = vertical_jump_simulation(jr, controls)
    print("final torso pose:\n", get_final_torso_pose(jr, sim_values, step_phases))

    # collocation optimization
    collo_values = vertical_jump_optimization(jr, controls, sim_values, step_phases)

    # visualize
    visualize_jr_trajectory(collo_values, jr, len(step_phases), dt)

if __name__ == "__main__":
    main()