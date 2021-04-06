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
    def __init__(self):
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
    def get_graph_builder():
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

    def step_robot_dynamics_graph(self, jr, k):
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

    def step_source_dynamics_graph(self, jr, k):
        m_s_key = Actuator.SourceMassKey(k)
        P_s_key = Actuator.SourcePressureKey(k)
        V_s_key = Actuator.SourceVolumeKey()
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtd.GassLawFactor(P_s_key, V_s_key, m_s_key, jr.gas_constant))
        return graph
    
    def step_actuator_dynamics_graph(self, jr, actuator, k):
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

    def step_actuation_dynamics_graph(self, jr, k):
        graph = self.step_source_dynamics_graph(jr, k)
        for actuator in jr.actuators:
            graph.add(self.step_actuator_dynamics_graph(jr, actuator, k))
        return graph

    def collocation_graph(self):
        graph = gtsam.NonlinearFactorGraph()
        return graph
