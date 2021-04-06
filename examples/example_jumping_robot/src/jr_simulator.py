import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir) 
sys.path.insert(0,currentdir) 

import gtdynamics as gtd
import gtsam
import numpy as np

from helpers import mergeValues

from jumping_robot import Actuator, JumpingRobot
from jr_graphs import JRGraphBuilder

class JRSimulator:
    def __init__(self, yaml_file_path, initial_config, controls):
        self.yaml_file_path = yaml_file_path
        self.jr_graph_builder = JRGraphBuilder()
        self.reset(initial_config, controls)
    
    def reset(self, initial_config, controls):
        self.jr = JumpingRobot(self.yaml_file_path, initial_config)
        self.phase = 0  # 0 ground, 1 left, 2 right, 3 air
        self.step_phases = [self.phase]
        self.time = 0
        self.values = gtsam.Values()

        # add values for initial configuration
        V_s = self.jr.params["pneumatic"]["v_source"]
        P_s_0 = controls["P_s_0"]
        m_s_0 = V_s * P_s_0 / self.jr.gas_constant
        m_a_0 = self.jr.params["pneumatic"]["init_mass"]
        self.values.insertDouble(Actuator.SourceVolumeKey(), V_s)
        self.values.insertDouble(Actuator.SourceMassKey(0), m_s_0)
        for joint in self.jr.robot.joints():
            j = joint.id()
            name = joint.name()
            q_key = gtd.internal.JointAngleKey(j, 0).key()
            v_key = gtd.internal.JointVelKey(j, 0).key()
            q = float(initial_config["qs"][name])
            v = float(initial_config["vs"][name])
            self.values.insertDouble(q_key, q)
            self.values.insertDouble(v_key, v)
        for actuator in self.jr.actuators:
            j = actuator.j
            self.values.insertDouble(Actuator.MassKey(j, 0), m_a_0)
            To_key = Actuator.ValveOpenTimeKey(j)
            Tc_key = Actuator.ValveCloseTimeKey(j)
            self.values.insertDouble(To_key, float(controls["Tos"][name]))
            self.values.insertDouble(Tc_key, float(controls["Tcs"][name]))

    def step_collocation(self, k, dt):
        for joint in self.jr.robot.joints():
            j = joint.id()
            q_prev = self.values.atDouble(gtd.internal.JointAngleKey(j, k-1))
            v_prev = self.values.atDouble(gtd.internal.JointVelKey(j, k-1))
            a_prev = self.values.atDouble(gtd.internal.JointAccelKey(j, k-1))
            v_curr = v_prev + a_prev * dt
            q_curr = q_prev + v_prev * dt + 0.5 * a_prev * dt * dt
            self.values.insertDouble(gtd.internal.JointAngleKey(j, k), q_curr)
            self.values.insertDouble(gtd.internal.JointVelKey(j, k), q_curr)
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
        m_s = self.values.atDouble(Actuator.SourceMassKey(k))
        V_s = self.values.atDouble(Actuator.SourceVolumeKey())
        P_s = m_s * self.jr.gas_constant / V_s
        P_s_key = Actuator.SourcePressureKey(k)
        t_key = gtd.TimeKey(k).key()
        for actuator in self.jr.actuators:
            j = actuator.j
            
            graph = self.jr_graph_builder.step_actuator_dynamics_graph(self.jr, actuator, k)
            m_a_key = Actuator.MassKey(j, k)
            q_key = gtd.internal.JointAngleKey(j, k).key()
            v_key = gtd.internal.JointVelKey(j, k).key()
            To_key = Actuator.ValveOpenTimeKey(j)
            Tc_key = Actuator.ValveCloseTimeKey(j)
            P_a_key = Actuator.PressureKey(j, k)
            V_a_key = Actuator.VolumeKey(j, k)
            delta_x_key = Actuator.ContractionKey(j, k)
            f_a_key = Actuator.ForceKey(j, k)
            torque_key = gtd.internal.TorqueKey(j, k).key()
            mdot_key = Actuator.MassRateOpenKey(j, k)
            mdot_sigma_key = Actuator.MassRateActualKey(j, k)
            m_a = self.values.atDouble(m_a_key)
            q = self.values.atDouble(q_key)
            v = self.values.atDouble(v_key)
            To = self.values.atDouble(To_key)
            Tc = self.values.atDouble(Tc_key)
            graph.add(gtd.PriorFactorDouble(m_a_key, m_a, self.jr_graph_builder.prior_m_cost_model))
            graph.add(gtd.PriorFactorDouble(P_s_key, P_s, self.jr_graph_builder.prior_pressure_cost_model))
            graph.add(gtd.PriorFactorDouble(q_key, q, self.jr_graph_builder.prior_q_cost_model))
            graph.add(gtd.PriorFactorDouble(v_key, v, self.jr_graph_builder.prior_v_cost_model))
            graph.add(gtd.PriorFactorDouble(t_key, self.time, self.jr_graph_builder.prior_time_cost_model))
            graph.add(gtd.PriorFactorDouble(To_key, To, self.jr_graph_builder.prior_time_cost_model))
            graph.add(gtd.PriorFactorDouble(Tc_key, Tc, self.jr_graph_builder.prior_time_cost_model))

            init_values = gtsam.Values()
            init_values.insertDouble(P_s_key, P_s)
            init_values.insertDouble(q_key, q)
            init_values.insertDouble(v_key, v)
            init_values.insertDouble(m_a_key, m_a)
            init_values.insertDouble(t_key, self.time)
            init_values.insertDouble(To_key, To)
            init_values.insertDouble(Tc_key, Tc)
            if k == 0:
                init_values.insertDouble(P_a_key, 10.0)
                init_values.insertDouble(delta_x_key, 2.0)
                init_values.insertDouble(f_a_key, 0.0)
                init_values.insertDouble(torque_key, 0.0)
                init_values.insertDouble(mdot_key, 1e-2)
                init_values.insertDouble(mdot_sigma_key, 1e-2)
                init_values.insertDouble(V_a_key, 1e-5)
            else:
                init_values.insertDouble(P_a_key, self.values.atDouble(Actuator.PressureKey(j, k-1)))
                init_values.insertDouble(x_key, self.values.atDouble(Actuator.ContractionKey(j, k-1)))
                init_values.insertDouble(f_key, self.values.atDouble(Actuator.ForceKey(j, k-1)))
                init_values.insertDouble(torque_key, self.values.atDouble(gtd.internal.TorqueKey(j, k-1)))
                init_values.insertDouble(mdot_key, self.values.atDouble(Actuator.MassRateOpenKey(j, k-1)))
                init_values.insertDouble(mdot_sigma_key, self.values.atDouble(Actuator.MassRateActualKey(j, k-1)))
                init_values.insertDouble(V_a_key, self.values.atDouble(Actuator.VolumeKey(j, k-1)))
            result = gtsam.LevenbergMarquardtOptimizer(graph, init_values).optimize()
            mergeValues(self.values, result)

    def step_robot_dynamics(self, k):
        # perform forward kinematics
        if self.phase == 3:
            fk_results = self.jr.robot.forwardKinematics(self.values, k, "torso")
        else:
            fk_results = self.jr.robot.forwardKinematics(self.values, k)
        mergeValues(self.values, fk_results)

        if self.phase == 0:
            for name in ["foot_l", "foot_r"]:
                j = self.jr.robot.joint(name).id()
                torque_key = gtd.internal.TorqueKey(j, k).key()
                self.values.insertDouble(torque_key, 0.0)

        # construct dynamcis graph for the time step
        opt = self.jr_graph_builder.graph_builder.opt()
        graph = self.jr_graph_builder.step_robot_dynamics_graph(self.jr, k)
        for acutator in self.jr.actuators:
            j = acutator.j
            torque_key = gtd.internal.TorqueKey(j, k).key()
            torque = self.values.atDouble(torque_key)
            graph.add(gtd.PriorFactorDouble(torque_key, torque, opt.prior_q_cost_model))
        
        # construct initial values
        init_values = gtsam.Values()
        for joint in self.jr.robot.joints():
            j = joint.id()
            gtd.InsertJointAngleDouble(init_values, j, k, gtd.JointAngleDouble(fk_results, j, k))
            gtd.InsertJointVelDouble(init_values, j, k, gtd.JointVelDouble(fk_results, j, k))
            gtd.InsertTorqueDouble(init_values, j, k, gtd.TorqueDouble(self.values, j, k))
        for link in self.jr.robot.links():
            i = link.id()
            gtd.InsertPose(init_values, i, k, gtd.Pose(fk_results, i, k))
            gtd.InsertTwist(init_values, i, k, gtd.Twist(fk_results, i, k))

        if k==0:
            # assign zeros to unknown values
            for joint in self.jr.robot.joints():
                j = joint.id()
                gtd.InsertJointAccelDouble(init_values, j, k, 0.0)
                i1 = joint.parent().id()
                i2 = joint.child().id()
                gtd.InsertWrench(init_values, i1, j, k, np.zeros(6))
                gtd.InsertWrench(init_values, i2, j, k, np.zeros(6))
            for link in self.jr.robot.links():
                i = link.id()
                gtd.InsertTwistAccel(init_values, i, k, np.zeros(6))
        else:
            # initialize from previous step
            for joint in self.jr.robot.joints():
                j = joint.id()
                gtd.InsertJointAccelDouble(init_values, j, k, gtd.JointAccelDouble(self.values, j, k-1))
                i1 = joint.parent.id()
                i2 = joint.child.id()
                gtd.InsertWrench(init_values, i1, j, k, gtd.Wrench(self.values, i1, j, k-1))
                gtd.InsertWrench(init_values, i2, j, k, gtd.Wrench(self.values, i2, j, k-1))
            for link in self.jr.robot.links():
                i = link.id()
                gtd.InsertTwistAccel(init_values, i, k, gtd.TwistAccel(self.values, i, k-1))

        # solve
        results = gtsam.LevenbergMarquardtOptimizer(graph, init_values).optimize()
        print("error: ", graph.error(results))

        mergeValues(self.values, results)

        # graph = self.jr_graph_builder.graph_builder.linearDynamicsGraph(self.jr.robot, k, self.values)
        # priors = self.jr_graph_builder.graph_builder.linearFDPriors(self.jr.robot, k, self.values)

        # gtd.DynamicsGraph.printGraph(priors)
        # graph.push_back(priors)
        # print("linear graph:")
        # gtd.DynamicsGraph.printGraph(graph)
        # result = graph.optimize()

        # dynamics_values = self.jr_graph_builder.graph_builder.linearSolveFD(self.jr.robot, k, self.values)
        # mergeValues(self.values, dynamics_values)

    def get_ground_force_z(self, side, k):
        i = self.jr.robot.link("thigh_" + side).id()
        j = self.jr.robot.joint("knee_" + side).id()
        wrench_b = gtd.Wrench(self.values, i, j, k) 
        T_wb = gtd.Pose(self.values, i, k)
        wrench_w = T_wb.inverse().AdjointMap().transpose().dot(wrench_b)
        print(side + " wrench: ", wrench_w.transpose())
        return wrench_w[5]

    def step_phase_change(self, k):
        threshold = 0
        if self.phase == 0:
            f_left = self.get_ground_force_z("l", k)
            f_right = self.get_ground_force_z("r", k)
            if f_left < threshold and f_right < threshold:
                #TODO: remove link and joints
                self.phase = 3
            elif f_left < threshold:
                self.phase = 2
            elif f_right < threshold:
                self.phase = 1
        elif self.phase == 1:
            f_left = self.get_ground_force_z("l", k)
            if f_left < threshold:
                self.phase = 3
        elif self.pahse == 2:
            f_right = self.get_ground_force_z("r", k)
            if f_right < threshold:
                self.phase = 3

    def simulate(self, num_steps, dt):
        for k in range(num_steps):
            if k!=0:
                self.step_collocation(k, dt)
            self.step_actuator_dynamics(k)
            self.step_robot_dynamics(k)
            self.step_phase_change(k)
            self.step_phases.append(self.phase)
            self.time += dt
        return self.values    

if __name__=="__main__":
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"

    theta = 0 # np.pi / 6
    rest_angles = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
    init_angles = rest_angles
    init_vels = [0, 0, 0, 0, 0, 0]
    init_config = JumpingRobot.create_init_config(rest_angles, init_angles, init_vels)
    Tos = [0, 0, 0, 0]
    Tcs = [0, 0, 0, 0]
    P_s_0 = 65 * 6894.76
    controls = JumpingRobot.create_controls(Tos, Tcs, P_s_0)

    jr_simulator = JRSimulator(yaml_file_path, init_config, controls)
    jr_simulator.simulate(1, 0.1)