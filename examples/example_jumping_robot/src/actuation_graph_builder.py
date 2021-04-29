"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  actuation_graph_builder.py
 * @brief Create actuation factor graphs for the jumping robot.
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


class ActuationGraphBuilder:
    """ Class that constructs actuation graph for the jumping robot. """
    def __init__(self):
        # ratio = 1e-3
        # self.pressure_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e2)
        # self.force_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e2)
        # self.balance_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e2)
        # self.torque_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e0)
        # self.prior_pressure_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e2)
        # self.prior_q_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e0)
        # self.prior_time_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e-1)
        # self.prior_v_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e0)

        # self.gas_law_model = noiseModel.Isotropic.Sigma(1, ratio * 1e1)
        # self.mass_rate_model = noiseModel.Isotropic.Sigma(1, ratio * 1e-3)
        # self.volume_model = noiseModel.Isotropic.Sigma(1, ratio * 1e-4)
        # self.prior_m_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e-3)
        # self.m_col_cost_model = noiseModel.Isotropic.Sigma(1, ratio * 1e-3)

        self.pressure_cost_model = noiseModel.Isotropic.Sigma(1, 0.1)
        self.force_cost_model = noiseModel.Isotropic.Sigma(1, 0.01)
        self.balance_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        self.torque_cost_model = noiseModel.Isotropic.Sigma(1, 0.01)
        self.prior_pressure_cost_model = noiseModel.Isotropic.Sigma(1, 0.01)
        self.prior_q_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)
        self.prior_time_cost_model = noiseModel.Isotropic.Sigma(1, 0.0001)
        self.prior_v_cost_model = noiseModel.Isotropic.Sigma(1, 0.001)

        self.gas_law_model = noiseModel.Isotropic.Sigma(1, 0.0001)
        self.mass_rate_model = noiseModel.Isotropic.Sigma(1, 1e-5)
        self.volume_model = noiseModel.Isotropic.Sigma(1, 1e-7)
        self.prior_m_cost_model = noiseModel.Isotropic.Sigma(1, 1e-7)
        self.m_col_cost_model = noiseModel.Isotropic.Sigma(1, 1e-7)


    def source_dynamics_graph(self, jr: JumpingRobot, k: int) -> NonlinearFactorGraph:
        """ Create a factor graph containing source dynamics constraints at step k. """
        m_s_key = Actuator.SourceMassKey(k)
        P_s_key = Actuator.SourcePressureKey(k)
        V_s_key = Actuator.SourceVolumeKey()
        graph = NonlinearFactorGraph()
        graph.add(gtd.GasLawFactor(P_s_key, V_s_key, m_s_key, self.gas_law_model, jr.gas_constant))
        return graph
    
    def actuator_dynamics_graph(self, jr: JumpingRobot, actuator: Actuator, k: int) -> NonlinearFactorGraph:
        """ Create a factor graph containing actuator dynamics constraints at step k. """
        d_tube = jr.params["pneumatic"]["d_tube_valve_musc"] * 0.0254
        l_tube = jr.params["pneumatic"]["l_tube_valve_musc"] * 0.0254
        ka = actuator.config["k_anta"]
        kt = actuator.config["k_tendon"]
        q_anta_limit = actuator.config["q_anta_limit"]
        b = actuator.config["b"]
        radius = actuator.config["rad0"]
        q_rest = jr.init_config["qs_rest"][actuator.name]

        j = actuator.j
        m_a_key = Actuator.MassKey(j, k)
        P_a_key = Actuator.PressureKey(j, k)
        V_a_key = Actuator.VolumeKey(j, k)
        delta_x_key = Actuator.ContractionKey(j, k)
        f_a_key = Actuator.ForceKey(j, k)
        torque_key = gtd.internal.TorqueKey(j, k).key()
        q_key = gtd.internal.JointAngleKey(j, k).key()
        v_key = gtd.internal.JointVelKey(j, k).key()

        graph = NonlinearFactorGraph()
        graph.add(gtd.GasLawFactor(P_a_key, V_a_key, m_a_key, self.gas_law_model, jr.gas_constant))
        graph.add(gtd.ActuatorVolumeFactor(V_a_key, delta_x_key, self.volume_model, d_tube, l_tube))
        graph.add(gtd.SmoothActuatorFactor(delta_x_key, P_a_key, f_a_key, self.force_cost_model))
        graph.add(gtd.ForceBalanceFactor(delta_x_key, q_key, f_a_key, self.balance_cost_model, kt, radius, q_rest, actuator.positive))
        graph.add(gtd.JointTorqueFactor(q_key, v_key, f_a_key, torque_key, self.torque_cost_model, q_anta_limit, ka, radius, b, actuator.positive))
        return graph

    def mass_flow_graph(self, jr, actuator, k):
        """ Create a factor graph containing mass flow dynamics constraints at step k. """
        d_tube = jr.params["pneumatic"]["d_tube_valve_musc"] * 0.0254
        l_tube = jr.params["pneumatic"]["l_tube_valve_musc"] * 0.0254
        mu = jr.params["pneumatic"]["mu_tube"]
        epsilon = jr.params["pneumatic"]["eps_tube"]
        ct = jr.params["pneumatic"]["time_constant_valve"]
        Rs = jr.params["pneumatic"]["Rs"]
        temp = jr.params["pneumatic"]["T"]
        k_const = 1.0 / (Rs * temp)

        j = actuator.j
        P_s_key = Actuator.SourcePressureKey(k)
        P_a_key = Actuator.PressureKey(j, k)
        mdot_key = Actuator.MassRateOpenKey(j, k)
        mdot_sigma_key = Actuator.MassRateActualKey(j, k)
        To_a_key = Actuator.ValveOpenTimeKey(j)
        Tc_a_key = Actuator.ValveCloseTimeKey(j)
        t_key = gtd.TimeKey(k).key()

        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtd.MassFlowRateFactor(P_a_key, P_s_key, mdot_key, self.mass_rate_model, d_tube, l_tube, mu, epsilon, k_const))
        graph.add(gtd.ValveControlFactor(t_key, To_a_key, Tc_a_key, mdot_key, mdot_sigma_key, self.mass_rate_model, ct))
        return graph

    def dynamics_graph(self, jr: JumpingRobot, k: int) -> NonlinearFactorGraph:
        """ Create a factor graph containing all actuation dynamics constraints at step k. """
        graph = self.source_dynamics_graph(jr, k)
        for actuator in jr.actuators:
            graph.push_back(self.actuator_dynamics_graph(jr, actuator, k))
            graph.push_back(self.mass_flow_graph(jr, actuator, k))
        return graph

    def prior_graph_actuator(self, jr, actuator, values, k):
        graph = NonlinearFactorGraph()
        j = actuator.j
        m_a_key = Actuator.MassKey(j, k)
        m_a = values.atDouble(m_a_key)
        graph.add(gtd.PriorFactorDouble(m_a_key, m_a, self.prior_m_cost_model))
        return graph

    def prior_graph_source(self, values, k):
        graph = NonlinearFactorGraph()
        m_s_key = Actuator.SourceMassKey(k)
        m_s = values.atDouble(m_s_key)
        graph.add(gtd.PriorFactorDouble(m_s_key, m_s, self.prior_m_cost_model))
        V_s_key = Actuator.SourceVolumeKey()
        V_s = values.atDouble(V_s_key)
        graph.add(gtd.PriorFactorDouble(V_s_key, V_s, self.volume_model))
        return graph

    def prior_graph(self, jr, values, k):
        """ Priors for actuation part. (m_a, m_s, V_s) """
        graph = NonlinearFactorGraph()
        for actuator in jr.actuators:
            graph.push_back(self.prior_graph_actuator(jr, actuator, values, k))
        graph.push_back(self.prior_graph_source(values, k))
        return graph

    def prior_graph_joint(self, values, j, k):
        graph = NonlinearFactorGraph()
        q_key = gtd.internal.JointAngleKey(j, k).key()
        v_key = gtd.internal.JointVelKey(j, k).key()
        graph.add(gtd.PriorFactorDouble(q_key, values.atDouble(q_key), self.prior_q_cost_model))
        graph.add(gtd.PriorFactorDouble(v_key, values.atDouble(v_key), self.prior_v_cost_model))
        return graph

    def collocation_graph(self, jr: JumpingRobot, step_phases: list, collocation) -> NonlinearFactorGraph:
        """ Create a factor graph containing collocation constraints on actuation variables. """
        graph = NonlinearFactorGraph()
        # for time_step in range(len(step_phases)):
        #     phase = step_phases[time_step]
        #     k_prev = time_step
        #     k_curr = time_step+1
        #     dt_key = gtd.PhaseKey(phase).key()

        #     # collcoation on actuator mass
        #     mdot_prev_keys = []
        #     mdot_curr_keys = []
        #     for actuator in jr.actuators:
        #         j = actuator.j
        #         mdot_prev_key = Actuator.MassRateActualKey(j, k_prev)
        #         mdot_curr_key = Actuator.MassRateActualKey(j, k_curr)
        #         mdot_prev_keys.append(mdot_prev_key)
        #         mdot_curr_keys.append(mdot_curr_key)
        #         m_a_prev_key = Actuator.MassKey(j, k_prev)
        #         m_a_curr_key = Actuator.MassKey(j, k_curr)
        #         gtd.DynamicsGraph.addMultiPhaseCollocationFactorDouble(
        #             graph, m_a_prev_key, m_a_curr_key, mdot_prev_key,
        #             mdot_curr_key, dt_key, self.m_col_cost_model, collocation)

        #     # collocation on source mass
        #     m_s_prev_key = Actuator.SourceMassKey(k_prev)
        #     m_s_curr_key = Actuator.SourceMassKey(k_curr)
        #     is_euler = collocation == gtd.CollocationScheme.Euler
        #     gtd.AddSourceMassCollocationFactor(graph, mdot_prev_keys,
        #                                        mdot_curr_keys, m_s_prev_key,
        #                                        m_s_curr_key, dt_key,
        #                                        is_euler, self.m_col_cost_model)
        return graph
