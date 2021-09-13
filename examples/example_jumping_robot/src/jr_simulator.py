"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  jr_simulator.py
 * @brief Simulate the jumping robot by solving dynamics of each step.
 * @author Yetong Zhang
"""

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)
sys.path.insert(0, currentdir)

import gtdynamics as gtd
import gtsam
import numpy as np

from helpers import mergeValues
from jumping_robot import Actuator, JumpingRobot
from jr_graph_builder import JRGraphBuilder
from jr_values import JRValues
from jr_visualizer import visualize_jr_trajectory, make_plot


class JRSimulator:
    # TODO(yetong) create pneumatics.py and actuator.py, which contain
    # the factors but also the functionality below that is
    # pneumatics/actuator specific.
    """ Class for jumping robot simulation.
        Refer to `example_simulate` on setup and use. """
    def __init__(self, yaml_file_path, init_config):
        """ Constructor, creates init_config and jr. """
        self.yaml_file_path = yaml_file_path
        self.jr_graph_builder = JRGraphBuilder()
        self.init_config = init_config
        self.jr = JumpingRobot(yaml_file_path, init_config)

    def step_integration(self, k, dt, values, include_actuation=True):
        """ Perform integration, and add results to values.

        Args:
            k (int): current step index
            dt (float): duration of time step
            values (gtsam.Values): values and derivatives of previous step
        """
        JRValues.integrate_joints(self.jr, values, k, dt)
        JRValues.integrate_torso(self.jr, values, k, dt)
        if include_actuation:
            JRValues.integrate_mass(self.jr, values, k, dt)

        # integrate time
        t_prev = values.atDouble(gtd.TimeKey(k-1).key())
        t_curr = t_prev + dt
        values.insert(gtd.TimeKey(k).key(), t_curr)

    def step_actuation_dynamics(self, k, values):
        """ Perform actuation dynamics by solving the actuation dynamics factor
            graph of the current step, and add results to values.

        Args:
            k (int): current step index
            values (gtsam.Values): values containing q, v, m_a, m_s of
                                   current step and To, Ti, V_s

        Raises:
            Exception: optimization does not converge
        """

        # directly compute source pressure
        m_s = values.atDouble(Actuator.SourceMassKey(k))
        V_s = values.atDouble(Actuator.SourceVolumeKey())
        P_s = m_s * self.jr.gas_constant / V_s / 1e3
        P_s_key = Actuator.SourcePressureKey(k)

        # perform actuator forward dynamics by solving its dynamcis graph
        for actuator in self.jr.actuators:
            j = actuator.j

            # construct graph
            graph = self.jr_graph_builder.actuation_graph_builder.actuator_dynamics_graph(self.jr, actuator, k)
            m_a_key = Actuator.MassKey(j, k)
            q_key = gtd.internal.JointAngleKey(j, k).key()
            v_key = gtd.internal.JointVelKey(j, k).key()
            m_a = values.atDouble(m_a_key)
            q = values.atDouble(q_key)
            v = values.atDouble(v_key)
            actuation_graph_builder = self.jr_graph_builder.actuation_graph_builder
            graph.add(gtd.PriorFactorDouble(m_a_key, m_a, actuation_graph_builder.prior_m_cost_model))
            graph.add(gtd.PriorFactorDouble(P_s_key, P_s, actuation_graph_builder.prior_pressure_cost_model))
            graph.add(gtd.PriorFactorDouble(q_key, q, actuation_graph_builder.prior_q_cost_model))
            graph.add(gtd.PriorFactorDouble(v_key, v, actuation_graph_builder.prior_v_cost_model))
            # TODO(yetong): check why adding the massflow graph makes it unable to optimize

            # construct init values and optimize
            if k == 0:
                init_values = JRValues.init_values_from_init_config_actuator(self.jr, j, k, values)
            else:
                init_values = JRValues.init_values_from_prev_actuator(j, k, values)
            results = self.optimize(graph, init_values)
            mergeValues(values, results)

            # compute mass flow
            mdot, mdot_sigma = JRValues.compute_mass_flow(self.jr, values, j, k)
            values.insert(Actuator.MassRateOpenKey(j, k), mdot)
            values.insert(Actuator.MassRateActualKey(j, k), mdot_sigma)

    def step_robot_dynamics_by_layer(self, k, values):
        """ In case solving the entire dynamics graph is hard to converge,
            this function provides a more robust method to solve the dynamics
            graph by layers (q, v, dynamics).
        """

        # construct init values
        if k==0:
            init_values = JRValues.init_values_from_fk_robot(self.jr, k, values)
        else:
            init_values = JRValues.init_values_from_prev_robot(self.jr.robot, k, values)

        robot_graph_builder = self.jr_graph_builder.robot_graph_builder
        opt = robot_graph_builder.graph_builder.opt()
        torso_i = self.jr.robot.link("torso").id()
        link_names = [link.name() for link in self.jr.robot.links()]

        # solve q level
        graph_q = robot_graph_builder.graph_builder.qFactors(self.jr.robot, k, None)
        pose_key = gtd.internal.PoseKey(torso_i, k).key()
        torso_pose = gtd.Pose(values, torso_i, k)
        graph_q.add(gtsam.PriorFactorPose3(pose_key, torso_pose, opt.p_cost_model))
        if "ground" not in link_names:
            for joint in self.jr.robot.joints():
                j = joint.id()
                q_key = gtd.internal.JointAngleKey(j, k).key()
                graph_q.add(gtd.PriorFactorDouble(q_key, gtd.JointAngleDouble(values, j, k), opt.prior_q_cost_model))

        init_values_q = gtd.ExtractValues(init_values, graph_q.keys())
        results_q = self.optimize(graph_q, init_values_q)
        mergeValues(init_values, results_q, overwrite=True)

        # solve v level
        graph_v = robot_graph_builder.graph_builder.vFactors(self.jr.robot, k, None)
        twist_key = gtd.internal.TwistKey(torso_i, k).key()
        torso_twist = gtd.Twist(values, torso_i, k)
        graph_v.add(gtd.PriorFactorVector6(twist_key, torso_twist, opt.v_cost_model))
        for joint in self.jr.robot.joints():
            j = joint.id()
            q_key = gtd.internal.JointAngleKey(j, k).key()
            graph_v.add(gtd.PriorFactorDouble(q_key, init_values.atDouble(q_key), opt.prior_q_cost_model))
        if not "ground" in link_names:
            for joint in self.jr.robot.joints():
                j = joint.id()
                v_key = gtd.internal.JointVelKey(j, k).key()
                graph_v.add(gtd.PriorFactorDouble(v_key, gtd.JointVelDouble(values, j, k), opt.prior_qv_cost_model))

        init_values_v = gtd.ExtractValues(init_values, graph_v.keys())
        results_v = self.optimize(graph_v, init_values_v)
        mergeValues(init_values, results_v, overwrite=True)

        # solve dynamics level
        graph_a = robot_graph_builder.graph_builder.aFactors(self.jr.robot, k, None)
        graph_d = robot_graph_builder.graph_builder.dynamicsFactors(self.jr.robot, k, None, None)
        graph_dynamics = graph_a
        graph_dynamics.push_back(graph_d)

        for joint in self.jr.robot.joints():
            j = joint.id()
            q_key = gtd.internal.JointAngleKey(j, k).key()
            v_key = gtd.internal.JointVelKey(j, k).key()
            torque_key = gtd.internal.TorqueKey(j, k).key()
            graph_dynamics.add(gtd.PriorFactorDouble(q_key, init_values.atDouble(q_key), opt.prior_q_cost_model))
            graph_dynamics.add(gtd.PriorFactorDouble(v_key, init_values.atDouble(v_key), opt.prior_qv_cost_model))
            graph_dynamics.add(gtd.PriorFactorDouble(torque_key, init_values.atDouble(torque_key), opt.prior_t_cost_model))
        for link in self.jr.robot.links():
            i = link.id()
            pose_key = gtd.internal.PoseKey(i, k).key()
            twist_key = gtd.internal.TwistKey(i, k).key()
            graph_dynamics_keys = [key for key in gtd.KeySetToKeyVector(graph_dynamics.keys())]
            if pose_key in graph_dynamics_keys:
                graph_dynamics.add(gtsam.PriorFactorPose3(pose_key, init_values.atPose3(pose_key), opt.p_cost_model))
            if twist_key in graph_dynamics_keys:
                graph_dynamics.add(gtd.PriorFactorVector6(twist_key, gtd.Twist(init_values, i, k), opt.v_cost_model))

        init_values_dynamics = gtd.ExtractValues(init_values, graph_dynamics.keys())
        results_dynamics = self.optimize(graph_dynamics, init_values_dynamics)
        mergeValues(init_values, results_dynamics, overwrite=True)
        mergeValues(values, init_values, overwrite=True)

    def step_robot_dynamics(self, k, values):
        """ Perform robot dynamics by first performing forward kinematics,
            then solving the dynamics factor graph of the current step.
            Add results to values

        Args:
            k (int): current step index
            values (gtsam.Values): values contains q, v, m_a, m_s of
                                   current step and To, Ti, V_s

        Raises:
            Exception: forward kinematics disagreement
            Exception: optimization does not converge
        """

        link_names = [link.name() for link in self.jr.robot.links()]
        joint_names = [joint.name() for joint in self.jr.robot.joints()]

        # construct dynamcis graph for the time step
        robot_graph_builder = self.jr_graph_builder.robot_graph_builder
        opt = robot_graph_builder.graph_builder.opt()
        graph = robot_graph_builder.dynamics_graph(self.jr, k)
        for acutator in self.jr.actuators:
            j = acutator.j
            torque_key = gtd.internal.TorqueKey(j, k).key()
            torque = values.atDouble(torque_key)
            graph.add(gtd.PriorFactorDouble(torque_key, torque, opt.prior_t_cost_model))
        
        # prior on torso link pose and twist
        i = self.jr.robot.link("torso").id()
        pose_key = gtd.internal.PoseKey(i, k).key()
        torso_pose = gtd.Pose(values, i, k)
        graph.add(gtsam.PriorFactorPose3(pose_key, torso_pose, opt.p_cost_model))
        twist_key = gtd.internal.TwistKey(i, k).key()
        torso_twist = gtd.Twist(values, i, k)
        graph.add(gtd.PriorFactorVector6(twist_key, torso_twist, opt.v_cost_model))

        # prior for joint angles and vels
        if "ground" not in link_names:
            for joint in self.jr.robot.joints():
                j = joint.id()
                q_key = gtd.internal.JointAngleKey(j, k).key()
                graph.add(gtd.PriorFactorDouble(q_key, gtd.JointAngleDouble(values, j, k), opt.prior_q_cost_model))
                v_key = gtd.internal.JointVelKey(j, k).key()
                graph.add(gtd.PriorFactorDouble(v_key, gtd.JointVelDouble(values, j, k), opt.prior_v_cost_model))

        # construct initial values
        if k==0:
            init_values = JRValues.init_values_from_fk_robot(self.jr, k, values)
        else:
            init_values = JRValues.init_values_from_prev_robot(self.jr.robot, k, values)

        # solve the robot dynamics graph
        results = self.optimize(graph, init_values)
        mergeValues(values, results)

    def optimize(self, graph, init_values, threshold=1e-5):
        """ Run optimization with different optimizers to ensure convergence.
            TODO(yetong): check why each optimizer does not converge for cases
        """
        # optimize
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values)
        results = optimizer.optimize()

        # Check if optimization converges.
        if (graph.error(results) > 1e-5):
            for f_idx in range(graph.size()):
                factor = graph.at(f_idx)
                print()
                graph_tmp = gtsam.NonlinearFactorGraph()
                graph_tmp.add(factor)
                gtd.DynamicsGraph.printGraph(graph_tmp)
                print(factor.error(init_values))

            print("init error: ", graph.error(init_values))
            params = gtsam.LevenbergMarquardtParams()
            params.setVerbosityLM("SUMMARY")
            results = gtsam.LevenbergMarquardtOptimizer(
                graph, init_values, params).optimize()
            print("error: ", graph.error(results))

            print("graph size: ", graph.size())
            print('values size: ', init_values.size())

            results = gtsam.DoglegOptimizer(graph, init_values).optimize()
            print("dogleg error: ", graph.error(results))
            raise Exception("optimizing dynamics does not converge")
        return results

    def step_phase_change(self, k: int, phase: int, values: gtsam.Values):
        """ Check if phase change happens in the step.
            We follow event-driven algorithms in
            Brogliato02amr_simulating_non_smooth
            by checking the contact forcds. """
        threshold = 0
        new_phase = phase
        if phase == 0:
            f_left = JRValues.get_ground_force_z(self.jr, "l", k, values)
            f_right = JRValues.get_ground_force_z(self.jr, "r", k, values)
            if f_left < threshold and f_right < threshold:
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
        if new_phase != phase:
            self.jr = JumpingRobot(self.yaml_file_path,
                                   self.init_config, new_phase)
        return new_phase

    def simulate(self, num_steps: int, dt: float, controls):
        """ Simulate the trajectory with specified controls.

        Args:
            num_steps (int): total number of simulation steps
            dt (float): duration of each step
            controls (Dict): specify control variables

        Returns:
            (gtsam.Values, list): (values for all steps, list of phases for
                                   each step)
        """
        self.jr = JumpingRobot(self.yaml_file_path, self.init_config)
        phase = 0
        step_phases = [phase]

        values = JRValues.init_config_values(self.jr, controls)
        for k in range(num_steps):
            print("step", k, "phase", phase)
            if k != 0:
                self.step_integration(k, dt, values)
            self.step_actuation_dynamics(k, values)
            self.step_robot_dynamics_by_layer(k, values)
            phase = self.step_phase_change(k, phase, values)
            step_phases.append(phase)

        return values, step_phases

    def simulate_with_torque_seq(self, num_steps, dt, torques_seq):
        """ Run simulation with specified torque sequence. """
        controls = JumpingRobot.create_controls()
        self.jr = JumpingRobot(self.yaml_file_path, controls)
        phase = 0
        step_phases = [phase]
        values = self.init_config_values(controls)
        for k in range(num_steps):
            print("step", k, "phase", phase)
            if k != 0:
                self.step_integration(k, dt, values, False)
            for joint in self.jr.robot.joints():
                j = joint.id()
                gtd.InsertTorqueDouble(values, j, k, torques_seq[k][j])
            self.step_robot_dynamics_by_layer(k, values)
            phase = self.step_phase_change(k, phase, values)
            step_phases.append(phase)
        return values, step_phases


def example_simulate():
    """ Show an example robot jumping trajectory """
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config.yaml"

    theta = np.pi/3
    rest_angles = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
    init_angles = rest_angles
    init_vels = [0, 0, 0, 0, 0, 0]
    torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.55))
    torso_twist = np.zeros(6)
    init_config = JumpingRobot.create_init_config(
        torso_pose, torso_twist, rest_angles, init_angles, init_vels)

    num_steps = 100
    dt = 0.005
    jr_simulator = JRSimulator(yaml_file_path, init_config)

    Tos = [0, 0, 0, 0]
    Tcs = [1, 1, 1, 1]
    P_s_0 = 65 * 6894.76/1000
    controls = JumpingRobot.create_controls(Tos, Tcs, P_s_0)
    values, step_phases = jr_simulator.simulate(num_steps, dt, controls)

    # torques_seq = [[0, -5, 5, 5, -5, 0]] * 300
    # values, step_phases = jr_simulator.simulate_with_torque_seq(
    #     num_steps, dt, torques_seq)

    make_plot(values, jr_simulator.jr, num_steps)
    # visualize_jr_trajectory(values, jr_simulator.jr, num_steps, step=1)


if __name__ == "__main__":
    example_simulate()
