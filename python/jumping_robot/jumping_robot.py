import gtdynamics as gtd
import gtsam
import numpy as np
import yaml

class Actuator:
    def __init__(self, j, actuator_config, positive):
        self.j = j
        self.config = actuator_config
        self.positive = positive

    @staticmethod
    def StartTimeKey(j):
        return gtd.DynamicsSymbol.JointSymbol("ti", j, 0).key()

    @staticmethod
    def PressureKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("P", j, t).key()
        
    @staticmethod
    def SourcePressureKey(t):
        return gtd.DynamicsSymbol.SimpleSymbol("Ps", t).key()
        
    @staticmethod
    def InitSourcePressureKey():
        return gtd.DynamicsSymbol.SimpleSymbol("Pi", 0).key()
        
    @staticmethod
    def ContractionKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("dx", j, t).key()
        
    @staticmethod
    def ForceKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("fr", j, t).key()
        
    @staticmethod
    def MassKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("m", j, t).key()
        
    @staticmethod
    def SourceMassKey(t):
        return gtd.DynamicsSymbol.SimpleSymbol("ms", t).key()
        
    @staticmethod
    def MassRateOpenKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("mo", j, t).key()
        
    @staticmethod
    def MassRateActualKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("md", j, t).key()
        
    @staticmethod
    def VolumeKey(j, t):
        return gtd.DynamicsSymbol.JointSymbol("vo", j, t).key()
        
    @staticmethod
    def SourceVolumeKey():
        return gtd.DynamicsSymbol.SimpleSymbol("vs", 0).key()
        
    @staticmethod
    def ValveOpenTimeKey(j):
        return gtd.DynamicsSymbol.JointSymbol("to", j, 0).key()
        
    @staticmethod
    def ValveCloseTimeKey(j):
        return gtd.DynamicsSymbol.JointSymbol("tc", j, 0).key()


'''class that stores all jumping robot properties'''
class JumpingRobot:
    def __init__(self, yaml_file_path):
        self.params = self.load_file(yaml_file_path)
        self.robot = self.create_jumping_robot(self.params)
        self.actuators = [Actuator(self.robot.joint("knee_r").id(), self.params["knee"], False),
                          Actuator(self.robot.joint("hip_r").id(), self.params["hip"], True),
                          Actuator(self.robot.joint("hip_l").id(), self.params["hip"], True),
                          Actuator(self.robot.joint("knee_l").id(), self.params["knee"], False)]

    @staticmethod
    def load_file(yaml_file_path):
        with open(yaml_file_path) as file:
            params = yaml.load(file, Loader=yaml.FullLoader)
            return params

    @staticmethod
    def create_jumping_robot(params):
        length_list = params["morphology"]["l"]
        mass_list = params["morphology"]["m"]
        link_radius = params["morphology"]["r_cyl"]
        foot_distance = params["morphology"]["foot_dist"]

        link_poses, joint_poses = JumpingRobot.compute_poses(length_list, foot_distance)
        ground = gtd.Link(0, "ground", 1, np.eye(3), gtsam.Pose3(), gtsam.Pose3(), True)
        shank_r = JumpingRobot.construct_link(1, "shank_r", mass_list[4], length_list[4], link_radius, link_poses["shank_r"])
        thigh_r = JumpingRobot.construct_link(2, "thigh_r", mass_list[3], length_list[3], link_radius, link_poses["thigh_r"])
        torso = JumpingRobot.construct_link(3, "torso", mass_list[2], length_list[2], link_radius, link_poses["torso"])
        thigh_l = JumpingRobot.construct_link(4, "thigh_l", mass_list[1], length_list[1], link_radius, link_poses["thigh_l"])
        shank_l = JumpingRobot.construct_link(5, "shank_l", mass_list[0], length_list[0], link_radius, link_poses["shank_l"])

        axis = np.array([1, 0, 0])
        foot_r = gtd.RevoluteJoint(0, "foot_r", joint_poses["foot_r"], ground, shank_r, gtd.JointParams(), axis)
        knee_r = gtd.RevoluteJoint(1, "knee_r", joint_poses["knee_r"], shank_r, thigh_r, gtd.JointParams(), axis)
        hip_r = gtd.RevoluteJoint(2, "hip_r", joint_poses["hip_r"], thigh_r, torso, gtd.JointParams(), axis)
        hip_l = gtd.RevoluteJoint(3, "hip_l", joint_poses["hip_l"], thigh_l, torso, gtd.JointParams(), axis)
        knee_l = gtd.RevoluteJoint(4, "knee_l", joint_poses["knee_l"], shank_l, thigh_l, gtd.JointParams(), axis)
        foot_l = gtd.RevoluteJoint(5, "foot_l", joint_poses["foot_l"], ground, shank_l, gtd.JointParams(), axis)

        links = [ground, shank_r, thigh_r, torso, thigh_l, shank_l]
        joints = [foot_r, knee_r, hip_r, hip_l, knee_l, foot_l]
        link_dict = {}
        joint_dict = {}
        for joint in joints:
            joint_dict[joint.name()] = joint
            joint.parent().addJoint(joint)
            joint.child().addJoint(joint)
        for link in links:
            link_dict[link.name()] = link
        return gtd.Robot(link_dict, joint_dict)

    @staticmethod
    def compute_poses(length_list, foot_distance):
        length_right = length_list[-1] + length_list[-2]
        length_left = length_list[0] + length_list[1]
        length_mid = length_list[2]

        init_values = gtsam.Values()
        init_values.insert(0, gtsam.Pose2(foot_distance/2, 0, 0))
        init_values.insert(1, gtsam.Pose2(foot_distance/2, length_right, 0))
        init_values.insert(2, gtsam.Pose2(-foot_distance/2, length_left, 0))
        init_values.insert(3, gtsam.Pose2(-foot_distance/2, 0, 0))

        range_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1]))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1, 1, 1]))
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.RangeFactorPose2(0, 1, length_right, range_noise))
        graph.add(gtsam.RangeFactorPose2(1, 2, length_mid, range_noise))
        graph.add(gtsam.RangeFactorPose2(2, 3, length_left, range_noise))
        graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(foot_distance/2, 0, 0), prior_noise))
        graph.add(gtsam.PriorFactorPose2(3, gtsam.Pose2(-foot_distance/2, 0, 0), prior_noise))

        result = gtsam.LevenbergMarquardtOptimizer(graph, init_values).optimize()
        p0 = result.atPose2(0)
        p1 = result.atPose2(1)
        p2 = result.atPose2(2)
        p3 = result.atPose2(3)
        rot_r = gtsam.Rot3.Rx(np.arctan2(p1.y() - p0.y(), p1.x() - p0.x()))
        rot_m = gtsam.Rot3.Rx(np.arctan2(p2.y() - p1.y(), p2.x() - p1.x()))
        rot_l = gtsam.Rot3.Rx(np.arctan2(p3.y() - p2.y(), p3.x() - p2.x()))

        link_poses = {}
        link_poses["shank_r"] = gtsam.Pose3(rot_r, gtsam.Point3(0, p0.x(), p0.y()))
        link_poses["thigh_r"] = gtsam.Pose3(rot_r, gtsam.Point3(0, (p0.x() + p1.x())/2, (p0.y() + p1.y())/2))
        link_poses["torso"] = gtsam.Pose3(rot_m, gtsam.Point3(0, p1.x(), p1.y()))
        link_poses["thigh_l"] = gtsam.Pose3(rot_l, gtsam.Point3(0, p2.x(), p2.y()))
        link_poses["shank_l"] = gtsam.Pose3(rot_l, gtsam.Point3(0, (p2.x() + p3.x())/2, (p2.y()+p3.y())/2))

        joint_poses = {}
        joint_poses["foot_r"] = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, p0.x(), p0.y()))
        joint_poses["knee_r"] = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, (p0.x() + p1.x())/2, (p0.y() + p1.y())/2))
        joint_poses["hip_r"] = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, p1.x(), p1.y()))
        joint_poses["hip_l"] = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, p2.x(), p2.y()))
        joint_poses["knee_l"] = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, (p2.x() + p3.x())/2, (p2.y()+p3.y())/2))
        joint_poses["foot_l"] = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, p3.x(), p3.y()))
        return link_poses, joint_poses

    @staticmethod
    def construct_link(link_id, link_name, mass, length, radius, pose):
        com = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, length/2, 0))
        return gtd.Link(link_id, link_name, mass, JumpingRobot.get_link_inertia(mass, length, radius), pose, com, False)

    @staticmethod
    def get_link_inertia(mass, length, radius):
        Ixx = 1/12 * mass * (3*radius**2 + length**2)
        Iyy = 1/2 * mass * radius**2
        Izz = Ixx
        inertia = np.diag([Ixx, Iyy, Izz])
        return inertia

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

    def step_robot_dynamics_graph(self, jumping_robot, k):
        return self.graph_builder.dynamicsFactorGraph(jumping_robot, k)

    def step_actuator_dynamics_graph(self, jumping_robot, k):
        
        m_s_key = Actuator.SourceMassKey(k)
        P_s_key = Actuator.SourcePressureKey(k)
        V_s_key = Actuator.SourceVolumeKey()

        Rs = jumping_robot.params["pneumatic"]["Rs"]
        temperature = jumping_robot.params["pneumatic"]["T"]
        gas_constant = Rs * temperature
        d_tube = jumping_robot.params["pneumatic"]["d_tube_valve_musc"] * 0.0254
        l_tube = jumping_robot.params["pneumatic"]["l_tube_valve_musc"] * 0.0254
        mu = jumping_robot.params["pneumatic"]["mu_tube"]
        epsilon = jumping_robot.params["pneumatic"]["eps_tube"]
        ct = jumping_robot.params["pneumatic"]["time_constant_valve"]
        x0_coeffs = np.array([3.05583930e+00, 7.58361626e-02, -4.91579771e-04, 1.42792618e-06, -1.54817477e-09])
        f0_coeffs = np.array([0, 1.966409])
        k_coeffs = np.array([0, 0.35541599])

        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtd.GassLawFactor(P_s_key, V_s_key, m_s_key, gas_constant))

        for actuator in jumping_robot.actuators:
            ka = actuator.config["k_anta"]
            kt = actuator.config["k_tendon"]
            q_anta_limit = actuator.config["q_anta_limit"]
            b = actuator.config["b"]
            radius = actuator.config["rad0"]

            j = actuator.j
            m_a_key = Actuator.MassKey(j, k)
            P_a_key = Actuator.PressureKey(j, k)
            V_a_key = Actuator.VolumeKey(j, k)

            mdot_key = Actuator.MassRateOpenKey(j, k)
            mdot_sigma_key = Actuator.MassRateActualKey(j, k)

            delta_x_key = Actuator.ContractionKey(j, k)
            f_a_key = Actuator.ForceKey(j, k)
            torque_key = gtd.internal.TorqueKey(j, k)
            q_key = gtd.internal.JointAngleKey(j, k)
            v_key = gtd.internal.JointVelKey(j, k)
            
            To_a_key = Actuator.ValveOpenTimeKey(j)
            Tc_a_key = Actuator.ValveCloseTimeKey(j)
            t_key = gtd.TimeKey(k)

            graph.add(gtd.MassFlowRateFactor(P_a_key, P_s_key, mdot_key, self.mass_rate_model, d_tube, l_tube, mu, epsilon, k))
            graph.add(gtd.ValveControlFactor(t_key, To_a_key, Tc_a_key, mdot_key, mdot_sigma_key, self.mass_rate_model, ct))
            graph.add(gtd.GassLawFactor(P_a_key, V_a_key, m_a_key, self.gass_law_model, gas_constant))
            graph.add(gtd.ActuatorVolumeFactor(V_a_key, delta_x_key, self.volume_model, d_tube, l_tube))
            graph.add(gtd.SmoothActuatorFactor(delta_x_key, P_a_key, f_a_key, self.force_cost_model, x0_coeffs, k_coeffs, f0_coeffs))
            graph.add(gtd.ForceBalanceFactor(delta_x_key, q_key, f_a_key, self.balance_cost_model, kt, radius, q_rest, actuator.positive))
            graph.add(gtd.JointTorqueFactor(q_key, v_key, f_a_key, torque_key, self.torque_cost_model, q_anta_limit, ka, radius, b, actuator.positive))

        return graph

    def collocation_graph(self):
        graph = gtsam.NonlinearFactorGraph()
        return graph


class JRSimulator:
    def __init__(self, yaml_file_path):
        self.yaml_file_path = yaml_file_path
        self.jr_graph_builder = JRGraphBuilder()
        self.reset()
    
    def reset(self):
        self.jr = JumpingRobot(self.yaml_file_path)
        self.phase = 0  # 0 ground, 1 left, 2 right, 3 air
        self.step_phases = [self.phase]
        self.values = gtsam.Values()
        # add values for initial configuration

    def step_collocation(self, k, dt):
        for joint in self.jr.robot.joints():
            j = joint.id()
            q_prev = self.values.atDouble(gtd.internal.JointAngleKey(j, k-1))
            v_prev = self.values.atDouble(gtd.internal.JointVelKey(j, k-1))
            a_prev = self.values.atDouble(gtd.internal.JointAccelKey(j, k-1))
            v_curr = v_prev + a_prev * dt
            q_curr = q_prev + v_prev * dt + 0.5 * a_prev * dt * dt
            self.values.insert(gtd.internal.JointAngleKey(j, k), q_curr)
            self.values.insert(gtd.internal.JointVelKey(j, k), q_curr)
        if self.phase == 3:
            i = self.jr.robot.link("torso").id()
            pose_torso_prev = self.values.atPose3(gtd.internal.PoseKey(i, k-1))
            twist_torso_prev = self.values.atVector(gtd.internal.TwistKey(i, k-1))
            twistaccel_torso_prev = self.values.atVector(gtd.internal.TwistAccelKey(i, k-1))
            twist_torso_curr = twist_torso_prev + twistaccel_torso_prev * dt
            pose_torso_curr = pose_torso_prev * gtsam.Pose3.Expmap(dt * twist_torso_prev + 0.5*twistaccel_torso_prev * dt * dt)
            self.values.insert(gtd.internal.PoseKey(i, k), pose_torso_curr)
            self.values.insert(gtd.internal.TwistKey(i, k), twist_torso_curr)
        
        total_m_out = 0
        for actuator in self.jr.actuators:
            j = actuator.j
            m_a_prev = self.values.atDouble(Actuator.MassKey(j, k-1))
            mdot_a_prev = self.values.atDouble(Actuator.MassRateActualKey(j, k-1))
            m_a_curr = m_a_prev + mdot_a_prev * dt
            self.values.insert(Actuator.MassKey(j, k), m_a_curr)
            total_m_out += mdot_a_prev * dt
        m_s_prev = self.values.atDouble(Actuator.SourceMassKey(k-1))
        m_s_curr = m_s_prev - total_m_out
        self.values.insert(Actuator.SourceMassKey(k), m_s_curr)

    def step_actuator_dynamics(self, k):
        return

    def step_robot_dynamics(self, k):
        fk_results = self.jr.robot.forwardKinematics()
        dynamics_values = self.jr_graph_builder.linearSolveFD(self.jr.robot, k, fk_results)
        self.values.insert(fk_results)
        self.values.insert(dynamics_values)

    def step_phase_change(self, k):
        return

    def simulate(num_steps, dt, P_s_init, T_o, T_c):
        for k in range(num_steps):
            if k!=0:
                self.step_collocation()
            self.step_actuator_dynamics()
            self.step_robot_dynamics()
            self.step_phase_change()
        return self.values    

if __name__=="__main__":
    yaml_file_path = "python/jumping_robot/robot_config.yaml"
    jr = JumpingRobot(yaml_file_path)

    jr_graph_builder = JRGraphBuilder()
