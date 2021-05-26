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

import os
import sys
import inspect
currentdir = os.path.dirname(os.path.abspath(
    inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, currentdir)


from jr_values import JRValues
from measurement_graph_builder import MeasurementGraphBuilder
from robot_graph_builder import RobotGraphBuilder
from actuation_graph_builder import ActuationGraphBuilder
from jumping_robot import Actuator, JumpingRobot
import numpy as np
from gtsam import noiseModel, NonlinearFactorGraph
import gtsam
import gtdynamics as gtd


class JRGraphBuilder:
    """ Class that constructs factor graphs for a jumping robot. """

    def __init__(self):
        """Initialize the graph builder, specify all noise models."""
        self.robot_graph_builder = RobotGraphBuilder()
        self.actuation_graph_builder = ActuationGraphBuilder()
        self.measurement_graph_builder = MeasurementGraphBuilder()

    def collocation_graph(self, jr: JumpingRobot, step_phases: list, collocation):
        """ Create a factor graph containing collocation constraints. """
        graph = self.actuation_graph_builder.collocation_graph(
            jr, step_phases, collocation)
        graph.push_back(self.robot_graph_builder.collocation_graph(
            jr, step_phases, collocation))

        # add collocation factors for time
        for time_step in range(len(step_phases)):
            phase = step_phases[time_step]
            k_prev = time_step
            k_curr = time_step+1
            dt_key = gtd.PhaseKey(phase).key()
            time_prev_key = gtd.TimeKey(k_prev).key()
            time_curr_key = gtd.TimeKey(k_curr).key()
            time_col_cost_model = self.robot_graph_builder.graph_builder.opt().time_cost_model
            gtd.AddTimeCollocationFactor(graph, time_prev_key, time_curr_key,
                                         dt_key, time_col_cost_model)

        return graph

    def dynamics_graph(self, jr: JumpingRobot, k: int, sysid=False) -> NonlinearFactorGraph:
        """ Create a factor graph containing dynamcis constraints for 
            the robot, actuators and source tank at a certain time step.
        """
        graph = self.actuation_graph_builder.dynamics_graph(jr, k, sysid)
        graph.push_back(self.robot_graph_builder.dynamics_graph(jr, k))
        return graph

    def transition_dynamics_graph(self, prev_jr, new_jr, k, sysid=False):
        """ Dynamics graph for transition node. """
        graph = self.actuation_graph_builder.dynamics_graph(prev_jr, k, sysid)
        graph.push_back(
            self.robot_graph_builder.transition_dynamics_graph(prev_jr, new_jr, k))
        return graph

    def control_priors_actuator(self, jr, actuator, controls):
        """ Create prior factors for control variables of an actuator. """
        graph = NonlinearFactorGraph()
        j = actuator.j
        name = actuator.name
        prior_time_cost_model = self.actuation_graph_builder.prior_time_cost_model
        To_key = Actuator.ValveOpenTimeKey(j)
        Tc_key = Actuator.ValveCloseTimeKey(j)
        graph.add(gtd.PriorFactorDouble(
            To_key, controls["Tos"][name], prior_time_cost_model))
        graph.add(gtd.PriorFactorDouble(
            Tc_key, controls["Tcs"][name], prior_time_cost_model))
        return graph

    def control_priors(self, jr, controls):
        """ Create prior factors for control variables (To, Tc). """
        graph = NonlinearFactorGraph()
        for actuator in jr.actuators:
            graph.push_back(self.control_priors_actuator(
                jr, actuator, controls))
        return graph

    def vertical_jump_goal_factors(self, jr, k):
        """ Add goal factor for vertical jumps, at step k. 
            The twist of torso reduces to 0.
        """
        graph = NonlinearFactorGraph()
        torso_i = jr.robot.link("torso").id()
        torso_twist_key = gtd.internal.TwistKey(torso_i, k).key()
        target_twist = np.zeros(6)
        bv_cost_model = self.robot_graph_builder.graph_builder.opt().bv_cost_model
        graph.add(gtd.PriorFactorVector6(
            torso_twist_key, target_twist, bv_cost_model))
        return graph

    def target_pose_goal_factor(self, jr, k, pose):
        """ Add goal factor for torso height at step k """
        graph = NonlinearFactorGraph()
        torso_i = jr.robot.link("torso").id()
        torso_pose_key = gtd.internal.PoseKey(torso_i, k).key()
        p_cost_model = self.robot_graph_builder.graph_builder.opt().p_cost_model
        graph.add(gtsam.PriorFactorPose3(
            torso_pose_key, pose, p_cost_model))
        return graph

    def time_prior(self):
        graph = NonlinearFactorGraph()
        t0_key = gtd.TimeKey(0).key()
        # t0 = init_config_values.atDouble(t0_key)
        time_cost_model = self.robot_graph_builder.graph_builder.opt().time_cost_model
        graph.add(gtd.PriorFactorDouble(t0_key, 0.0, time_cost_model))
        return graph

    def trajectory_priors(self, jr):
        graph = NonlinearFactorGraph()
        init_config_values = JRValues.init_config_values(jr)
        graph.push_back(self.robot_graph_builder.prior_graph(
            jr, init_config_values, 0))
        graph.push_back(self.actuation_graph_builder.prior_graph(
            jr, init_config_values, 0))
        graph.push_back(self.time_prior())
        return graph

    def trajectory_graph(self, jr, step_phases, collocation=gtd.CollocationScheme.Euler):
        """ Create a factor graph consisting of all factors represeting
            the robot trajectory.
        """
        # prior factors for init configuration, control and time
        graph = self.trajectory_priors(jr)

        # dynamics graph at each step
        if len(step_phases) == 0:
            graph.push_back(self.dynamics_graph(jr, 0))
            return graph

        jr = jr.jr_with_phase(step_phases[0])
        for k in range(len(step_phases)+1):
            prev_phase = step_phases[k-1] if k != 0 else step_phases[0]
            next_phase = step_phases[k] if k != len(
                step_phases) else step_phases[-1]
            if next_phase == prev_phase:
                print(k, prev_phase)
                graph_dynamics = self.dynamics_graph(jr, k)
            else:
                print(k, prev_phase, '->', next_phase)
                new_jr = jr.jr_with_phase(next_phase)
                graph_dynamics = self.transition_dynamics_graph(jr, new_jr, k)
                jr = new_jr
            graph.push_back(graph_dynamics)

        # collocation factors across steps
        graph_collo = self.collocation_graph(jr, step_phases, collocation)
        graph.push_back(graph_collo)

        return graph

    def sys_id_graph(self, jr, step_phases, pixels_all_frames,
                     pressures_all_frames, collocation=gtd.CollocationScheme.Euler):
        """ Graph for system identification. """
        # prior factors for init configuration, control and time
        graph = self.trajectory_priors(jr)

        # dynamics graph at each step
        jr = jr.jr_with_phase(step_phases[0])
        for k in range(len(step_phases)+1):
            prev_phase = step_phases[k-1] if k != 0 else step_phases[0]
            next_phase = step_phases[k] if k != len(
                step_phases) else step_phases[-1]
            if next_phase == prev_phase:
                # print(k, prev_phase)
                graph_dynamics = self.dynamics_graph(jr, k, sysid=True)
            else:
                # print(k, prev_phase, '->', next_phase)
                new_jr = jr.jr_with_phase(next_phase)
                graph_dynamics = self.transition_dynamics_graph(
                    jr, new_jr, k, sysid=True)
                jr = new_jr
            graph.push_back(graph_dynamics)

        # collocation factors across steps
        graph_collo = self.collocation_graph(jr, step_phases, collocation)
        graph.push_back(graph_collo)

        # measurements
        graph.push_back(self.measurement_graph_builder.camera_priors(jr))
        graph.push_back(self.measurement_graph_builder.measurement_graph(
            jr, pixels_all_frames, pressures_all_frames))

        return graph
