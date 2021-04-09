"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  jr_graph_builder.py
 * @brief Create factor graphs for the jumping robot.
 * @author Yetong Zhang
"""



import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,currentdir) 

import gtdynamics as gtd
import gtsam
from gtsam import noiseModel, NonlinearFactorGraph
import numpy as np

from jumping_robot import Actuator, JumpingRobot
from actuation_graph_builder import ActuationGraphBuilder
from robot_graph_builder import RobotGraphBuilder


class JRGraphBuilder:
    """ Class that constructs factor graphs for a jumping robot. """

    def __init__(self):
        """Initialize the graph builder, specify all noise models."""
        self.robot_graph_builder = RobotGraphBuilder()
        self.actuation_graph_builder = ActuationGraphBuilder()

    def collocation_graph(self, jr: JumpingRobot, step_phases: list):
        """ Create a factor graph containing collocation constraints. """
        graph = self.actuation_graph_builder.collocation_graph(jr, step_phases)
        graph.add(self.robot_graph_builder.collocation_graph(jr, step_phases))

        # add collocation factors for time
        for time_step in range(len(step_phases)):
            phase = step_phases[time_step]
            k_prev = time_step
            k_curr = time_step+1
            dt_key = gtd.PhaseKey(phase).key()
            time_prev_key = gtd.TimeKey(k_prev).key()
            time_curr_key = gtd.TImeKey(k_curr).key()
            time_col_cost_model = self.robot_graph_builder.graph_builder.opt().time_cost_model
            graph.add(gtd.TimeColloFactor(
                time_prev_key, time_curr_key, dt_key, time_col_cost_model))

        return graph

    def dynamics_graph(self, jr: JumpingRobot, k: int) -> NonlinearFactorGraph:
        """ Create a factor graph containing dynamcis constraints for 
            the robot, actuators and source tank at a certain time step
        """
        graph = self.actuation_graph_builder.dynamics_graph(jr, k)
        graph.add(self.robot_graph_builder.dynamics_graph(jr, k))
        return graph

