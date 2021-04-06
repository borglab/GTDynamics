"""
@file   jr_graphs.py
@brief  create factor graphs for a jumping robot
@author Yetong Zhang
"""

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,currentdir) 

import gtdynamics as gtd
import gtsam
import numpy as np

from jumping_robot import Actuator, JumpingRobot

class JRGraphBuilder:
    """ class that constructs factor graphs for a jumping robot """

    def __init__(self):
        """initialize the graph builder, specify all noise models"""
        self.graph_builder = self.get_graph_builder()
        self.pressure_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.1)
        self.force_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.01)
        self.balance_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        self.torque_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.01)
        self.prior_pressure_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.01)
        self.prior_valve_t_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)
        self.prior_q_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        self.prior_time_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)
        self.prior_v_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)  

        self.gass_law_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)  
        self.mass_rate_model = gtsam.noiseModel.Isotropic.Sigma(1, 1e-5)  
        self.volume_model = gtsam.noiseModel.Isotropic.Sigma(1, 1e-7)  
        self.prior_m_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 1e-7)  
        self.m_col_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 1e-7)  
        self.mass_rate_obj_model = gtsam.noiseModel.Isotropic.Sigma(1, 1.0)


    @staticmethod
    def get_graph_builder() -> gtd.DynamicsGraph:
        """construct GTDynamics graph builder by specifying noise models"""
        opt = gtd.OptimizerSetting()
        opt.bv_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.ba_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.p_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.v_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.a_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.linear_a_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.f_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.01)
        opt.linear_f_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.fa_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.01)
        opt.t_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.01)
        opt.linear_t_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.cp_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.cfriction_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.cv_cost_model = gtsam.noiseModel.Isotropic.Sigma(3, 0.001)
        opt.ca_cost_model = gtsam.noiseModel.Isotropic.Sigma(3, 0.001)
        opt.cm_cost_model = gtsam.noiseModel.Isotropic.Sigma(3, 0.001)
        opt.planar_cost_model = gtsam.noiseModel.Isotropic.Sigma(3, 0.001)
        opt.linear_planar_cost_model = gtsam.noiseModel.Isotropic.Sigma(3, 0.001)
        opt.prior_q_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.prior_qv_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.prior_qa_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.prior_t_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.q_col_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.v_col_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        opt.pose_col_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.twist_col_cost_model = gtsam.noiseModel.Isotropic.Sigma(6, 0.001)
        opt.time_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)
        opt.jl_cost_model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        gravity = np.array([0, 0, -9.8])
        planar_axis = np.array([1, 0, 0])
        return gtd.DynamicsGraph(opt, gravity, planar_axis)

    def step_robot_dynamics_graph(self, jr: JumpingRobot, k: int) -> gtsam.NonlinearFactorGraph:
        """ Create factor graph containing dynamcis constraints for the jumping robot at a certain time step

        Args:
            jr (JumpingRobot): jumping robot
            k (int): time step index

        Returns:
            gtsam.NonlinearFactorGraph: graph of dynamcis constraints
        """
        graph = self.graph_builder.dynamicsFactorGraph(jr.robot, k, None, None)
        joint_names = []
        for joint in jr.robot.joints():
            joint_names.append(joint.name())
        for name in ["foot_l", "foot_r"]:
            if name in joint_names:
                j = jr.robot.joint(name).id()
                torque_key = gtd.internal.TorqueKey(j, k).key()
                graph.add(gtd.PriorFactorDouble(torque_key, 0.0, self.graph_builder.opt().prior_q_cost_model))
        return graph

    def step_source_dynamics_graph(self, jr: JumpingRobot, k: int) -> gtsam.NonlinearFactorGraph:
        """ Create factor graph containing source dynamics constraints at step k """
        m_s_key = Actuator.SourceMassKey(k)
        P_s_key = Actuator.SourcePressureKey(k)
        V_s_key = Actuator.SourceVolumeKey()
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtd.GassLawFactor(P_s_key, V_s_key, m_s_key, jr.gas_constant))
        return graph
    
    def step_actuator_dynamics_graph(self, jr: JumpingRobot, actuator: Actuator, k: int) -> gtsam.NonlinearFactorGraph:
        """ Create factor graph containing actuator dynamics constraints at step k """
        j = actuator.j

        d_tube = jr.params["pneumatic"]["d_tube_valve_musc"] * 0.0254
        l_tube = jr.params["pneumatic"]["l_tube_valve_musc"] * 0.0254
        mu = jr.params["pneumatic"]["mu_tube"]
        epsilon = jr.params["pneumatic"]["eps_tube"]
        ct = jr.params["pneumatic"]["time_constant_valve"]
        x0_coeffs = np.array([3.05583930e+00, 7.58361626e-02, -4.91579771e-04, 1.42792618e-06, -1.54817477e-09])
        f0_coeffs = np.array([0, 1.966409])
        k_coeffs = np.array([0, 0.35541599])

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
        P_s_key = Actuator.SourcePressureKey(k)

        mdot_key = Actuator.MassRateOpenKey(j, k)
        mdot_sigma_key = Actuator.MassRateActualKey(j, k)

        delta_x_key = Actuator.ContractionKey(j, k)
        f_a_key = Actuator.ForceKey(j, k)
        torque_key = gtd.internal.TorqueKey(j, k).key()
        q_key = gtd.internal.JointAngleKey(j, k).key()
        v_key = gtd.internal.JointVelKey(j, k).key()
        
        To_a_key = Actuator.ValveOpenTimeKey(j)
        Tc_a_key = Actuator.ValveCloseTimeKey(j)
        t_key = gtd.TimeKey(k).key()

        graph = gtsam.NonlinearFactorGraph()
        graph.push_back(gtd.MassFlowRateFactor(P_a_key, P_s_key, mdot_key, self.mass_rate_model, d_tube, l_tube, mu, epsilon, k))
        graph.add(gtd.ValveControlFactor(t_key, To_a_key, Tc_a_key, mdot_key, mdot_sigma_key, self.mass_rate_model, ct))
        graph.add(gtd.GassLawFactor(P_a_key, V_a_key, m_a_key, self.gass_law_model, jr.gas_constant))
        graph.add(gtd.ActuatorVolumeFactor(V_a_key, delta_x_key, self.volume_model, d_tube, l_tube))
        graph.add(gtd.SmoothActuatorFactor(delta_x_key, P_a_key, f_a_key, self.force_cost_model, x0_coeffs, k_coeffs, f0_coeffs))
        graph.add(gtd.ForceBalanceFactor(delta_x_key, q_key, f_a_key, self.balance_cost_model, kt, radius, q_rest, actuator.positive))
        graph.add(gtd.JointTorqueFactor(q_key, v_key, f_a_key, torque_key, self.torque_cost_model, q_anta_limit, ka, radius, b, actuator.positive))
        return graph

    def step_actuation_dynamics_graph(self, jr: JumpingRobot, k: int) -> gtsam.NonlinearFactorGraph:
        """ Create factor graph containing all actuation dynamics constraints at step k """
        graph = self.step_source_dynamics_graph(jr, k)
        for actuator in jr.actuators:
            graph.add(self.step_actuator_dynamics_graph(jr, actuator, k))
        return graph

    def collocation_graph(self, jr: JumpingRobot, step_phases: list):
        """ Create factor graph containing collocation constraints """
        graph = gtsam.NonlinearFactorGraph()
        for time_step in range(len(step_phases)):
            phase = step_phases[time_step]
            k_prev = time_step
            k_curr = time_step+1
            dt_key = gtd.PhaseKey(phase).key()

            # collcoation on actuator mass
            mdot_prev_keys = []
            mdot_curr_keys = []
            for actuator in jr.actuators:
                j = actuator.j
                mdot_prev_key = Actuator.MassRateActualKey(j, k_prev)
                mdot_curr_key = Actuator.MassRateActualKey(j, k_curr)
                mdot_prev_keys.append(mdot_prev_key)
                mdot_curr_keys.append(mdot_curr_key)
                m_a_prev_key = Actuator.MassKey(j, k_prev)
                m_a_curr_key = Actuator.MassKey(j, k_curr)
                graph.add(gtd.TrapezoidalScalarColloFactor(
                    m_a_prev_key, m_a_curr_key, mdot_prev_key, mdot_curr_key, dt_key, self.m_col_cost_model))

            # collocation on source mass
            m_s_prev_key = Actuator.SourceMassKey(k_prev)
            m_s_curr_key = Actuator.SourceMassKey(k_curr)
            graph.add(gtd.SourceMassColloFactor(
                m_s_prev_key, m_s_curr_key, mdot_prev_keys[0], mdot_prev_keys[1], mdot_prev_keys[2], mdot_prev_keys[3],
                mdot_curr_keys[0], mdot_curr_keys[1], mdot_curr_keys[2], mdot_curr_keys[3], dt_key, self.m_col_cost_model   
            ))

            # collocation on joint angles
            collo_joint_names = ["hip_r", "hip_l"]
            if phase == 3:
                collo_joint_names += ["knee_r", "knee_l"]
            for name in collo_joint_names:
                joint = self.jr.robot.joint(name)
                j = joint.id()
                q_prev_key = gtd.internal.JointAngleKey(j, k_prev).key()
                q_curr_key = gtd.internal.JointAngleKey(j, k_curr).key()
                v_prev_key = gtd.internal.JointVelKey(j, k_prev).key()
                v_curr_key = gtd.internal.JointVelKey(j, k_curr).key()
                a_prev_key = gtd.internal.JointAccelKey(j, k_prev).key()
                a_curr_key = gtd.internal.JointAccelKey(j, k_curr).key()

                q_col_cost_model = self.graph_builder.opt().q_col_cost_model
                graph.add(gtd.TrapezoidalScalarColloFactor(
                    q_prev_key, q_curr_key, v_prev_key, v_curr_key, dt_key, q_col_cost_model))
                v_col_cost_model = self.graph_builder.opt().v_col_cost_model
                graph.add(gtd.TrapezoidalScalarColloFactor(
                    v_prev_key, v_curr_key, a_prev_key, a_curr_key, dt_key, v_col_cost_model))

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
                pose_prev_key, pose_curr_key, twist_prev_key, twist_curr_key, dt_key, pose_col_cost_model))
            twist_col_cost_model = self.graph_builder.opt().twist_col_cost_model
            graph.add(gtd.TrapezoidalPoseColloFactor(
                twist_prev_key, twist_curr_key, twistaccel_prev_key, twistaccel_curr_key, dt_key, pose_col_cost_model))

        return graph
