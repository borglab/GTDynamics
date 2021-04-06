"""
@file   jr_simulator.py
@brief  simulate the jumping robot by solving dynamics of each step
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

from helpers import mergeValues

from jumping_robot import Actuator, JumpingRobot
from jr_graphs import JRGraphBuilder

class JRSimulator:
    """ class for jumping robot simulation """
    def __init__(self, yaml_file_path, init_config):
        self.yaml_file_path = yaml_file_path
        self.jr_graph_builder = JRGraphBuilder()
        self.init_config = init_config
        self.jr = JumpingRobot(yaml_file_path, init_config)
    
    def init_config_values(self, controls) -> gtsam.Values:
        """ values to specify initial configuration """
        values = gtsam.Values()
        V_s = self.jr.params["pneumatic"]["v_source"]
        P_s_0 = controls["P_s_0"]
        m_s_0 = V_s * P_s_0 / self.jr.gas_constant
        m_a_0 = self.jr.params["pneumatic"]["init_mass"]
        values.insertDouble(Actuator.SourceVolumeKey(), V_s)
        values.insertDouble(Actuator.SourceMassKey(0), m_s_0)
        for joint in self.jr.robot.joints():
            j = joint.id()
            name = joint.name()
            q_key = gtd.internal.JointAngleKey(j, 0).key()
            v_key = gtd.internal.JointVelKey(j, 0).key()
            q = float(self.init_config["qs"][name])
            v = float(self.init_config["vs"][name])
            values.insertDouble(q_key, q)
            values.insertDouble(v_key, v)
        for actuator in self.jr.actuators:
            j = actuator.j
            values.insertDouble(Actuator.MassKey(j, 0), m_a_0)
            To_key = Actuator.ValveOpenTimeKey(j)
            Tc_key = Actuator.ValveCloseTimeKey(j)
            values.insertDouble(To_key, float(controls["Tos"][name]))
            values.insertDouble(Tc_key, float(controls["Tcs"][name]))
        return values


    def step_collocation(self, k, dt, values, phase):
        """ perform integration, and add results to values

        Args:
            k (int): current step index
            dt (float): duration of time step
            values (gtsam.Values): contain values and derivatives of previous step
            phase (int): current phase of jumping robot
        """
        # integrate joint angles
        for joint in self.jr.robot.joints():
            j = joint.id()
            q_prev = values.atDouble(gtd.internal.JointAngleKey(j, k-1))
            v_prev = values.atDouble(gtd.internal.JointVelKey(j, k-1))
            a_prev = values.atDouble(gtd.internal.JointAccelKey(j, k-1))
            v_curr = v_prev + a_prev * dt
            q_curr = q_prev + v_prev * dt + 0.5 * a_prev * dt * dt
            values.insertDouble(gtd.internal.JointAngleKey(j, k), q_curr)
            values.insertDouble(gtd.internal.JointVelKey(j, k), q_curr)
        
        # integrate torso link for air phase
        if phase == 3:
            i = self.jr.robot.link("torso").id()
            pose_torso_prev = values.atPose3(gtd.internal.PoseKey(i, k-1))
            twist_torso_prev = values.atVector(gtd.internal.TwistKey(i, k-1))
            twistaccel_torso_prev = values.atVector(gtd.internal.TwistAccelKey(i, k-1))
            twist_torso_curr = twist_torso_prev + twistaccel_torso_prev * dt
            pose_torso_curr = pose_torso_prev * gtsam.Pose3.Expmap(dt * twist_torso_prev + 0.5*twistaccel_torso_prev * dt * dt)
            values.insert(gtd.internal.PoseKey(i, k), pose_torso_curr)
            values.insert(gtd.internal.TwistKey(i, k), twist_torso_curr)
        
        # integrate mass flow
        total_m_out = 0
        for actuator in self.jr.actuators:
            j = actuator.j
            m_a_prev = values.atDouble(Actuator.MassKey(j, k-1))
            mdot_a_prev = values.atDouble(Actuator.MassRateActualKey(j, k-1))
            m_a_curr = m_a_prev + mdot_a_prev * dt
            values.insert(Actuator.MassKey(j, k), m_a_curr)
            total_m_out += mdot_a_prev * dt
        m_s_prev = values.atDouble(Actuator.SourceMassKey(k-1))
        m_s_curr = m_s_prev - total_m_out
        values.insert(Actuator.SourceMassKey(k), m_s_curr)

    def step_actuator_dynamics(self, k, values, curr_time):
        """ perform actuation dynamics by solving the actuation dynamics factor 
            graph of the current step, and add results to values

        Args:
            k (int): current step index
            values (gtsam.Values): values contains q, v, m_a, m_s of 
                                   current step and To, Ti, V_s
            curr_time (float): current time

        Raises:
            Exception: optimization does not converge
        """

        # directly compute source pressure
        m_s = values.atDouble(Actuator.SourceMassKey(k))
        V_s = values.atDouble(Actuator.SourceVolumeKey())
        P_s = m_s * self.jr.gas_constant / V_s
        P_s_key = Actuator.SourcePressureKey(k)
        t_key = gtd.TimeKey(k).key()

        # perform forward dynamics for each actuator by solving its dynamcis factor graph
        for actuator in self.jr.actuators:
            j = actuator.j
            
            # construct graph
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
            m_a = values.atDouble(m_a_key)
            q = values.atDouble(q_key)
            v = values.atDouble(v_key)
            To = values.atDouble(To_key)
            Tc = values.atDouble(Tc_key)
            graph.add(gtd.PriorFactorDouble(m_a_key, m_a, self.jr_graph_builder.prior_m_cost_model))
            graph.add(gtd.PriorFactorDouble(P_s_key, P_s, self.jr_graph_builder.prior_pressure_cost_model))
            graph.add(gtd.PriorFactorDouble(q_key, q, self.jr_graph_builder.prior_q_cost_model))
            graph.add(gtd.PriorFactorDouble(v_key, v, self.jr_graph_builder.prior_v_cost_model))
            graph.add(gtd.PriorFactorDouble(t_key, curr_time, self.jr_graph_builder.prior_time_cost_model))
            graph.add(gtd.PriorFactorDouble(To_key, To, self.jr_graph_builder.prior_time_cost_model))
            graph.add(gtd.PriorFactorDouble(Tc_key, Tc, self.jr_graph_builder.prior_time_cost_model))

            # construct init values
            init_values = gtsam.Values()
            init_values.insertDouble(P_s_key, P_s)
            init_values.insertDouble(q_key, q)
            init_values.insertDouble(v_key, v)
            init_values.insertDouble(m_a_key, m_a)
            init_values.insertDouble(t_key, curr_time)
            init_values.insertDouble(To_key, To)
            init_values.insertDouble(Tc_key, Tc)
            if k == 0:
                # guess
                init_values.insertDouble(P_a_key, 10.0)
                init_values.insertDouble(delta_x_key, 2.0)
                init_values.insertDouble(f_a_key, 0.0)
                init_values.insertDouble(torque_key, 0.0)
                init_values.insertDouble(mdot_key, 1e-2)
                init_values.insertDouble(mdot_sigma_key, 1e-2)
                init_values.insertDouble(V_a_key, 1e-5)
            else:
                # initialize from previous step
                init_values.insertDouble(P_a_key, values.atDouble(Actuator.PressureKey(j, k-1)))
                init_values.insertDouble(x_key, values.atDouble(Actuator.ContractionKey(j, k-1)))
                init_values.insertDouble(f_key, values.atDouble(Actuator.ForceKey(j, k-1)))
                init_values.insertDouble(torque_key, values.atDouble(gtd.internal.TorqueKey(j, k-1)))
                init_values.insertDouble(mdot_key, values.atDouble(Actuator.MassRateOpenKey(j, k-1)))
                init_values.insertDouble(mdot_sigma_key, values.atDouble(Actuator.MassRateActualKey(j, k-1)))
                init_values.insertDouble(V_a_key, values.atDouble(Actuator.VolumeKey(j, k-1)))
            
            # solve the actuator dynamics graph
            result = gtsam.LevenbergMarquardtOptimizer(graph, init_values).optimize()
            if (graph.error(result) > 1e-5):
                print("error: ", graph.error(result))
                raise Exception("optimizing dynamics for actuator does not converge")
            mergeValues(values, result)

    def step_robot_dynamics(self, k, values, phase):
        """ Perform robot dynamics by first performing forward kinematics, 
            then solving the dynamics factor graph of the current step. 
            Add results to values

        Args:
            k (int): current step index
            values (gtsam.Values): values contains q, v, m_a, m_s of 
                                   current step and To, Ti, V_s
            phase ([type]): current phase

        Raises:
            Exception: optimization does not converge
        """

        # perform forward kinematics
        if phase == 3:
            fk_results = self.jr.robot.forwardKinematics(values, k, "torso")
        else:
            fk_results = self.jr.robot.forwardKinematics(values, k)
        mergeValues(values, fk_results)

        if phase == 0:
            for name in ["foot_l", "foot_r"]:
                j = self.jr.robot.joint(name).id()
                torque_key = gtd.internal.TorqueKey(j, k).key()
                values.insertDouble(torque_key, 0.0)

        # construct dynamcis graph for the time step
        opt = self.jr_graph_builder.graph_builder.opt()
        graph = self.jr_graph_builder.step_robot_dynamics_graph(self.jr, k)
        for acutator in self.jr.actuators:
            j = acutator.j
            torque_key = gtd.internal.TorqueKey(j, k).key()
            torque = values.atDouble(torque_key)
            graph.add(gtd.PriorFactorDouble(torque_key, torque, opt.prior_q_cost_model))
        
        # construct initial values
        init_values = gtsam.Values()
        for joint in self.jr.robot.joints():
            j = joint.id()
            gtd.InsertJointAngleDouble(init_values, j, k, gtd.JointAngleDouble(fk_results, j, k))
            gtd.InsertJointVelDouble(init_values, j, k, gtd.JointVelDouble(fk_results, j, k))
            gtd.InsertTorqueDouble(init_values, j, k, gtd.TorqueDouble(values, j, k))
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
                gtd.InsertJointAccelDouble(init_values, j, k, gtd.JointAccelDouble(values, j, k-1))
                i1 = joint.parent.id()
                i2 = joint.child.id()
                gtd.InsertWrench(init_values, i1, j, k, gtd.Wrench(values, i1, j, k-1))
                gtd.InsertWrench(init_values, i2, j, k, gtd.Wrench(values, i2, j, k-1))
            for link in self.jr.robot.links():
                i = link.id()
                gtd.InsertTwistAccel(init_values, i, k, gtd.TwistAccel(values, i, k-1))

        # solve the robot dynamics graph
        results = gtsam.LevenbergMarquardtOptimizer(graph, init_values).optimize()
        if (graph.error(results) > 1e-5):
            print("error: ", graph.error(results))
            raise Exception("optimizing dynamics for robot frame does not converge")
        mergeValues(values, results)


    def get_ground_force_z(self, side, k, values):
        """ get ground reaction force for the specifed foot contact """
        i = self.jr.robot.link("thigh_" + side).id()
        j = self.jr.robot.joint("knee_" + side).id()
        wrench_b = gtd.Wrench(values, i, j, k) 
        T_wb = gtd.Pose(values, i, k)
        wrench_w = T_wb.inverse().AdjointMap().transpose().dot(wrench_b)
        # print(side + " wrench: ", wrench_w.transpose())
        return wrench_w[5]

    def step_phase_change(self, k: int, phase: int, values: gtsam.Values):
        """ check if phase change happens in the step """
        threshold = 0
        new_phase = phase
        if phase == 0:
            f_left = self.get_ground_force_z("l", k)
            f_right = self.get_ground_force_z("r", k)
            if f_left < threshold and f_right < threshold:
                #TODO: remove link and joints
                new_phase = 3
            elif f_left < threshold:
                new_phase = 2
            elif f_right < threshold:
                new_phase = 1
        elif phase == 1:
            f_left = self.get_ground_force_z("l", k)
            if f_left < threshold:
                new_phase = 3
        elif phase == 2:
            f_right = self.get_ground_force_z("r", k)
            if f_right < threshold:
                new_phase = 3
        return new_phase

    def simulate(self, num_steps: int, dt: float, controls):
        """ simulate the trajectory with specified controls

        Args:
            num_steps (int): total number of simulation steps
            dt (float): duration of each step
            controls (Dict): specify control variables

        Returns:
            (gtsam.Values, list): (values for all steps, list of phases for each step)
        """
        self.jr = JumpingRobot(self.yaml_file_path, init_config)
        phase = 0  # 0 ground, 1 left, 2 right, 3 air
        step_phases = [phase]
        curr_time = 0
        values = self.init_config_values(controls)
        for k in range(num_steps):
            if k!=0:
                self.step_collocation(k, dt, values, phase)
            self.step_actuator_dynamics(k, values, curr_time)
            self.step_robot_dynamics(k, values, phase)
            phase = self.step_phase_change(k, phase, values)
            self.step_phases.append(phase)
            curr_time += dt
        return values, step_phases

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

    jr_simulator = JRSimulator(yaml_file_path, init_config)
    values, step_phases = jr_simulator.simulate(1, 0.1, controls)