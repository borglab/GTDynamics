"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  actuation_graph_builder.py
 * @brief Create factor graphs for the jumping robot frame.
 * @author Yetong Zhang
"""

import gtdynamics as gtd
from gtsam import noiseModel, NonlinearFactorGraph
import numpy as np

import os, sys, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, currentdir)

from jumping_robot import Actuator, JumpingRobot


class RobotGraphBuilder:
    """ Class that constructs dynamics factor graphs for a jumping robot. """

    def __init__(self):
        self.graph_builder = self.construct_graph_builder()

    @staticmethod
    def construct_graph_builder() -> gtd.DynamicsGraph:
        """construct GTDynamics graph builder by specifying noise models"""
        opt = gtd.OptimizerSetting()
        opt.bv_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.ba_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.p_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.v_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.a_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.linear_a_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.f_cost_model = noiseModel.Isotropic.Sigma(6, 0.01)
        opt.linear_f_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.fa_cost_model = noiseModel.Isotropic.Sigma(6, 0.01)
        opt.t_cost_model = noiseModel.Isotropic.Sigma(1, 0.01)
        opt.linear_t_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.cp_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.cfriction_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.cv_cost_model = noiseModel.Isotropic.Sigma(3, 0.001)
        opt.ca_cost_model = noiseModel.Isotropic.Sigma(3, 0.001)
        opt.cm_cost_model = noiseModel.Isotropic.Sigma(3, 0.001)
        opt.planar_cost_model = noiseModel.Isotropic.Sigma(3, 0.001)
        opt.linear_planar_cost_model = noiseModel.Isotropic.Sigma(3, 0.001)
        opt.prior_q_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.prior_qv_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.prior_qa_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.prior_t_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.q_col_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.v_col_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        opt.pose_col_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.twist_col_cost_model = noiseModel.Isotropic.Sigma(6, 0.001)
        opt.time_cost_model = noiseModel.Isotropic.Sigma(1, 0.0001)
        opt.jl_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        gravity = np.array([0, 0, -9.8])
        planar_axis = np.array([1, 0, 0])
        return gtd.DynamicsGraph(opt, gravity, planar_axis)

    def dynamics_graph(self, jr: JumpingRobot, k: int) -> NonlinearFactorGraph:
        """ Create a factor graph of dynamcis constraints for robot frame. """
        graph = self.graph_builder.dynamicsFactorGraph(jr.robot, k, None, None)
        joint_names = [joint.name() for joint in jr.robot.joints()]
        for name in ["foot_l", "foot_r"]:
            if name in joint_names:
                j = jr.robot.joint(name).id()
                torque_key = gtd.internal.TorqueKey(j, k).key()
                graph.add(gtd.PriorFactorDouble(torque_key, 0.0,
                                                self.graph_builder.opt().t_cost_model))
        return graph

    def collocation_graph(self, jr: JumpingRobot, step_phases: list):
        """ Create a factor graph containing collocation constraints.
            - For ground phase, only collocation on the torso link, which is
                enough to determine the constraints for the next step.
                Additional constraints may cause conflict.
            - For air phase, collocation on torso link and all joints.
            - TODO(yetong): For single leg contact, collocation on all joints
        """
        graph = NonlinearFactorGraph()
        for time_step in range(len(step_phases)):
            phase = step_phases[time_step]
            k_prev = time_step
            k_curr = time_step+1
            dt_key = gtd.PhaseKey(phase).key()

            # collocation on joint angles
            if phase == 3:
                collo_joint_names = ["hip_r", "hip_l", "knee_r", "knee_l"]
                for name in collo_joint_names:
                    joint = self.jr.robot.joint(name)
                    j = joint.id()
                    q_col_cost_model = self.graph_builder.opt().q_col_cost_model
                    v_col_cost_model = self.graph_builder.opt().v_col_cost_model
                    graph.add(gtd.TrapezoidalScalarColloFactor.Velocity(
                        j, k_curr, dt_key, q_col_cost_model))
                    graph.add(gtd.TrapezoidalScalarColloFactor.Angle(
                        j, k_curr, dt_key, v_col_cost_model))

            # collocation on torso link
            link = self.jr.robot.link("torso")
            i = link.id()
            pose_prev_key = gtd.internal.PoseKey(i, k_prev).key()
            pose_curr_key = gtd.internal.PoseKey(i, k_curr).key()
            twist_prev_key = gtd.internal.TwistKey(i, k_prev).key()
            twist_curr_key = gtd.internal.TwistKey(i, k_curr).key()
            twistaccel_prev_key = gtd.internal.TwistAccelKey(i, k_prev).key()
            twistaccel_curr_key = gtd.internal.TwistAccelKey(i, k_curr).key()

            pose_col_cost_model = self.graph_builder.opt().pose_col_cost_model
            graph.add(gtd.TrapezoidalPoseColloFactor(
                pose_prev_key, pose_curr_key, twist_prev_key, twist_curr_key,
                dt_key, pose_col_cost_model))
            twist_col_cost_model = self.graph_builder.opt().twist_col_cost_model
            graph.add(gtd.TrapezoidalPoseColloFactor(
                twist_prev_key, twist_curr_key, twistaccel_prev_key,
                twistaccel_curr_key, dt_key, twist_col_cost_model))

        return graph
