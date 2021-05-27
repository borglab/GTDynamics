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


def vertical_jump_simulation(jr, controls):
    """ Simulate vertical jump trajectory. """
    dt = 0.01
    jr_simulator = JRSimulator(jr.yaml_file_path, jr.init_config)
    sim_values, step_phases = jr_simulator.simulate_to_high(dt, controls)
    phase0_key = gtd.PhaseKey(0).key()
    phase3_key = gtd.PhaseKey(3).key()
    sim_values.insertDouble(phase0_key, dt)
    sim_values.insertDouble(phase3_key, dt)
    return sim_values, step_phases

def vertical_jump_collocation(jr, controls, sim_values, step_phases):
    """ Collocation optimization for vertical jump. """
    jr_graph_builder = JRGraphBuilder()
    collocation = gtd.CollocationScheme.Trapezoidal
    graph = jr_graph_builder.trajectory_graph(jr, step_phases, collocation)
    graph.push_back(jr_graph_builder.control_priors(jr, controls))

    time_cost_model = jr_graph_builder.robot_graph_builder.graph_builder.opt().time_cost_model
    phase3_key = gtd.PhaseKey(3).key()
    dt = sim_values.atDouble(phase3_key)
    graph.add(gtd.PriorFactorDouble(phase3_key, dt, time_cost_model))

    # # goal factors
    # num_steps = len(step_phases)
    # phase0_key = gtd.PhaseKey(0).key()
    # graph.add(gtd.PriorFactorDouble(phase0_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))
    # # graph.push_back(jr_graph_builder.vertical_jump_goal_factors(jr, num_steps))

    
    # for f_idx in range(graph.size()):
    #     factor = graph.at(f_idx)
    #     if factor.error(init_values) > 1:
    #         graph_tmp = gtsam.NonlinearFactorGraph()
    #         graph_tmp.add(factor)
    #         gtd.DynamicsGraph.printGraph(graph_tmp)
    #         print("error", factor.error(init_values))

    results = OptimizeLM(graph, sim_values)

    return results

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

    print("step_phases", step_phases)

    # collocation optimization
    collo_values = vertical_jump_collocation(jr, controls, sim_values, step_phases)

    # visualize

if __name__ == "__main__":
    main()