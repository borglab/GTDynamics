"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  simulation.py
 * @brief Simulate a vertical jump trajectory.
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


def vertical_jump_simulation(jr, controls, dt):
    """ Simulate vertical jump trajectory. """
    jr_simulator = JRSimulator(jr)
    sim_values, step_phases = jr_simulator.simulate_to_high(dt, controls)
    phase0_key = gtd.PhaseKey(0).key()
    phase3_key = gtd.PhaseKey(3).key()
    sim_values.insertDouble(phase0_key, dt)
    sim_values.insertDouble(phase3_key, dt)
    return sim_values, step_phases

def main():
    """ Main file. """
    # create jumping robot
    yaml_file_path = JumpingRobot.icra_yaml()
    init_config = JumpingRobot.icra_init_config()
    jr = JumpingRobot.from_yaml(yaml_file_path, init_config)

    # create controls
    Tos = [0, 0, 0, 0]
    Tcs = [0.098, 0.098, 0.098, 0.098]
    controls = JumpingRobot.create_controls(Tos, Tcs)

    # simulation
    dt = 0.01
    sim_values, step_phases = vertical_jump_simulation(jr, controls, dt)

    # visualize
    visualize_jr_trajectory(sim_values, jr, len(step_phases), dt)

if __name__ == "__main__":
    main()
