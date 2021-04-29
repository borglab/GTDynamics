"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  test_jr_simulator.py
 * @brief Unit test for jumping robot simulator.
 * @author Yetong Zhang
"""

import unittest
import gtsam
import gtdynamics as gtd
import numpy as np

import os, sys, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.jumping_robot import Actuator, JumpingRobot
from src.jr_visualizer import visualize_jr, visualize_jr_trajectory
from src.robot_graph_builder import RobotGraphBuilder
from src.actuation_graph_builder import ActuationGraphBuilder
from src.jr_graph_builder import JRGraphBuilder
from src.jr_values import JRValues
from src.jr_simulator import JRSimulator


class TestJRSimulator(unittest.TestCase):
    def setUp(self):
        """ Set up the simulator. """
        self.yaml_file_path = JumpingRobot.icra_yaml()
        self.init_config = JumpingRobot.icra_init_config()

        self.jr_simulator = JRSimulator(self.yaml_file_path, self.init_config)
        Tos = [0, 0, 0, 0]
        Tcs = [0.098, 0.098, 0.098, 0.098]
        self.controls = JumpingRobot.create_controls(Tos, Tcs)
        self.jr_graph_builder = self.jr_simulator.jr_graph_builder
        self.jr = self.jr_simulator.jr

    def robot(self):
        """ Return the robot model. """
        return self.jr_simulator.jr.robot

    def cal_jr_accels(self, theta, torque_hip, torque_knee):
        """ Compute groundtruth joint accelerations from virtual work. """
        m1 = self.robot().link("shank_r").mass()
        m2 = self.robot().link("thigh_r").mass()
        m3 = self.robot().link("torso").mass()
        link_radius = self.jr_simulator.jr.params["morphology"]["r_cyl"]
        l_link = self.jr_simulator.jr.params["morphology"]["l"][0]

        g = 9.8
        moment = (0.5 * m1 + 1.5 * m2 + 1.0 * m3) * g * l_link * np.sin(theta)
        J1 = (l_link ** 2 + 3 * link_radius ** 2) * 1.0 / 12 * m1
        J2 = (l_link ** 2 + 3 * link_radius ** 2) * 1.0 / 12 * m2
        J = l_link ** 2 * (1.0 / 4 * m1 + (1.0 / 4 + 2 * np.sin(theta) ** 2) * m2 + 2 * np.sin(theta) ** 2 * m3)

        acc = (torque_hip - torque_knee * 2 - moment) / (J + J1 + J2)
        expected_q_accels = {"foot_r": acc, "knee_r": -2*acc, "hip_r": acc,
                             "hip_l": acc, "knee_l": -2*acc, "foot_l": acc}
        return expected_q_accels

    def lm_optimize(self, graph, values):
        """ Run Levenberg-Marquardt optimization. """
        init_values = gtd.ExtractValues(values, graph.keys())
        params = gtsam.LevenbergMarquardtParams()
        # params.SetCeresDefaults
        # params = gtsam.LevenbergMarquardtParams.CeresDefaults()
        params.setlambdaLowerBound(1e-20)
        params.setlambdaUpperBound(1e20)
        params.setVerbosityLM("SUMMARY")
        params.setLinearSolverType("MULTIFRONTAL_QR")
        # params.setLinearSolverType("SEQUENTIAL_QR")
        params.setMaxIterations(20)
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, params)
        results = optimizer.optimize()

        # results = gtd.optimize_LMQR(graph, init_values)

        return results
    
    def gn_optimize(self, graph, values):
        """ Run Gauss-Newton optimization. """
        init_values = gtd.ExtractValues(values, graph.keys())
        optimizer = gtsam.GaussNewtonOptimizer(graph, init_values)
        results = optimizer.optimize()
        return results

    def dl_optimize(self, graph, values):
        """ Run Dogleg optimization. """
        init_values = gtd.ExtractValues(values, graph.keys())
        optimizer = gtsam.DoglegOptimizer(graph, init_values)
        results = optimizer.optimize()
        return results

    def test_robot_forward_dynamics(self):
        """ Test forward dynamics of robot frame: specify the angles,
            joint vels and torques, check the joint accelerations. """

        # forward kinematics
        init_config = JumpingRobot.simple_init_config()
        jr_simulator = JRSimulator(self.yaml_file_path, init_config)
        jr = jr_simulator.jr
        values = JRValues.init_config_values(jr)
        jr_simulator.step_robot_kinematics(0, values)
        
        # forward dynamics
        torque_hip = 0
        torque_knee = 0
        torques = [0., torque_knee, torque_hip, torque_hip, torque_knee, 0.]
        for joint in jr.robot.joints():
            j = joint.id()
            gtd.InsertTorqueDouble(values, j, 0, torques[j])
        jr_simulator.step_robot_dynamics(0, values)

        # check joint accelerations
        q_accels = gtd.DynamicsGraph.jointAccelsMap(self.robot(),
                                                    values, 0)
        theta = np.pi/3
        expected_q_accels = self.cal_jr_accels(theta, torque_hip, torque_knee)
        for joint in self.robot().joints():
            name = joint.name()
            self.assertAlmostEqual(q_accels[name],
                                   expected_q_accels[name], places=7)

    def test_actuation_forward_dynamics(self):
        """ Test forward dynamics of actuator: specify mass, time, controls,
            check torques. """

        # create init values of known variables
        values = JRValues.init_config_values(self.jr_simulator.jr)
        values.insert(JRValues.control_values(self.jr_simulator.jr, self.controls))
        k = 0

        # compute dynamics
        self.jr_simulator.step_actuation_dynamics(k, values)

        torques = []
        for actuator in self.jr_simulator.jr.actuators:
            j = actuator.j
            torque = gtd.TorqueDouble(values, j, k)
            torques.append(torque)
            self.assertAlmostEqual(torque, 0, places=7)
        # TODO(yetong): check torques, pressures, etc


    def test_solve_simple(self):
        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        key = 1
        value = 0.1
        model = gtsam.noiseModel.Isotropic.Sigma(1, 0.001)
        graph.add(gtd.PriorFactorDouble(key, value, model))
        values.insertDouble(key, value)
        self.lm_optimize(graph, values)


    # def test_solving_actuator_cpp(self):
    #     actuator = gtd.PneumaticActuator()
    #     k = 0
    #     prior_values = actuator.priorValues()
    #     graph = gtsam.NonlinearFactorGraph()
    #     graph.push_back(actuator.actuatorFactorGraph(k))
    #     graph.push_back(actuator.actuatorPriorGraph(k, prior_values))
  
    #     init_values = gtsam.Values()
    #     init_values.insert(actuator.actuatorInitValues(k, prior_values))

    #     params = gtsam.LevenbergMarquardtParams()
    #     # params.setlambdaLowerBound(1e-20)
    #     # params.setlambdaUpperBound(1e20)
    #     params.setVerbosityLM("SUMMARY")
    #     # params.setLinearSolverType("MULTIFRONTAL_QR")
    #     # params.setLinearSolverType("SEQUENTIAL_QR")
    #     optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, params)
    #     results = optimizer.optimize()

    def test_solving_actuator(self):
        """ Test solving dynamics factor graph for an actuator at a step. """
        sim_values, phase_steps = self.jr_simulator.simulate(0, 0.1, self.controls)
        actuation_graph_builder = self.jr_graph_builder.actuation_graph_builder

        init_config_values = JRValues.init_config_values(self.jr)
        actuator = self.jr.actuators[0]
        j = actuator.j

        graph = actuation_graph_builder.actuator_dynamics_graph(self.jr, actuator, 0)
        # graph.push_back(actuation_graph_builder.source_dynamics_graph(self.jr, 0))
        graph.push_back(actuation_graph_builder.mass_flow_graph(self.jr, actuator, 0))
        graph.push_back(actuation_graph_builder.prior_graph_actuator(self.jr, actuator, init_config_values, 0))
        P_s_key = Actuator.SourcePressureKey(0)
        graph.add(gtd.PriorFactorDouble(P_s_key, sim_values.atDouble(P_s_key), actuation_graph_builder.prior_pressure_cost_model))
        # graph.push_back(actuation_graph_builder.prior_graph_source(init_config_values, 0))
        graph.push_back(self.jr_graph_builder.control_priors_actuator(self.jr, actuator, self.controls))
        graph.push_back(self.jr_graph_builder.time_prior())

        q_key = gtd.internal.JointAngleKey(j, 0).key()
        v_key = gtd.internal.JointVelKey(j, 0).key()
        graph.add(gtd.PriorFactorDouble(q_key, init_config_values.atDouble(q_key), actuation_graph_builder.prior_q_cost_model))
        graph.add(gtd.PriorFactorDouble(v_key, init_config_values.atDouble(v_key), actuation_graph_builder.prior_v_cost_model))

        self.lm_optimize(graph, sim_values)
        

    def test_solving_actuation(self):
        """ Create dynamics graph of one step, with init values from simulation, and solve it. """
        sim_values, phase_steps = self.jr_simulator.simulate(0, 0.1, self.controls)
        actuation_graph_builder = self.jr_graph_builder.actuation_graph_builder
        init_config_values = JRValues.init_config_values(self.jr)

        graph = actuation_graph_builder.dynamics_graph(self.jr, 0)
        graph.push_back(actuation_graph_builder.prior_graph(self.jr, init_config_values, 0))
        graph.push_back(self.jr_graph_builder.control_priors(self.jr, self.controls))
        graph.push_back(self.jr_graph_builder.time_prior())
        for actuator in self.jr.actuators:
            j = actuator.j
            graph.push_back(actuation_graph_builder.prior_graph_joint(init_config_values, j, 0))

        self.lm_optimize(graph, sim_values)



    def test_solving_robot_dynamics(self):
        """ Create robot dynamics graph of one step, and solve it. """
        sim_values, phase_steps = self.jr_simulator.simulate(0, 0.1, self.controls)
        robot_graph_builder = self.jr_graph_builder.robot_graph_builder
        init_config_values = JRValues.init_config_values(self.jr)

        graph = robot_graph_builder.dynamics_graph(self.jr, 0)
        graph.push_back(robot_graph_builder.prior_graph(self.jr, init_config_values, 0))
        prior_t_cost_model = robot_graph_builder.graph_builder.opt().prior_t_cost_model
        for actuator in self.jr.actuators:
            j = actuator.j
            torque_key = gtd.internal.TorqueKey(j, 0).key()
            graph.push_back(gtd.PriorFactorDouble(torque_key, 0, prior_t_cost_model))
        
        # gtd.DynamicsGraph.printGraph(graph)
        self.lm_optimize(graph, sim_values)

    def test_solving_step(self):
        """ Create dynamics graph of a single step, and solve it. """
        sim_values, phase_steps = self.jr_simulator.simulate(0, 0.1, self.controls)
        actuation_graph_builder = self.jr_graph_builder.actuation_graph_builder

        graph = self.jr_graph_builder.trajectory_graph(self.jr, phase_steps)
        graph.push_back(self.jr_graph_builder.control_priors(self.jr, self.controls))

        # gtd.DynamicsGraph.printGraph(graph)

        self.lm_optimize(graph, sim_values)


    def test_solving_onse_step_collocation(self):
        """ Create trajectory of one step, and solve it. """
        dt = 0.005
        sim_values, phase_steps = self.jr_simulator.simulate(1, dt, self.controls)
        actuation_graph_builder = self.jr_graph_builder.actuation_graph_builder
        phase0_key = gtd.PhaseKey(0).key()
        sim_values.insertDouble(phase0_key, dt)

        collocation = gtd.CollocationScheme.Trapezoidal
        graph = self.jr_graph_builder.trajectory_graph(self.jr, phase_steps, collocation)
        graph.push_back(self.jr_graph_builder.control_priors(self.jr, self.controls))
        graph.add(gtd.PriorFactorDouble(phase0_key, dt, gtsam.noiseModel.Isotropic.Sigma(1, 0.01)))

        results = self.lm_optimize(graph, sim_values)
        self.assertAlmostEqual(graph.error(results), 0, places=5)

    def test_solving_single_phase_collocation(self):
        """ Create trajectory of one step, and solve it. """
        dt = 0.002
        num_steps = 50
        sim_values, phase_steps = self.jr_simulator.simulate(num_steps, dt, self.controls)
        actuation_graph_builder = self.jr_graph_builder.actuation_graph_builder
        phase0_key = gtd.PhaseKey(0).key()
        sim_values.insertDouble(phase0_key, dt)

        collocation = gtd.CollocationScheme.Trapezoidal
        graph = self.jr_graph_builder.trajectory_graph(self.jr, phase_steps, collocation)
        graph.push_back(self.jr_graph_builder.control_priors(self.jr, self.controls))
        graph.add(gtd.PriorFactorDouble(phase0_key, dt, gtsam.noiseModel.Isotropic.Sigma(1, 0.01)))
        
        init_values = gtd.ExtractValues(sim_values, graph.keys())

        results = self.lm_optimize(graph, sim_values)

        print("initial error: ", graph.error(init_values))
        print("resulting error: ", graph.error(results))

        # for factor_idx in range(graph.size()):
        #     factor = graph.at(factor_idx)
        #     error = factor.error(results)
        #     if error>1e-5:
        #         graph_tmp = gtsam.NonlinearFactorGraph()
        #         graph_tmp.add(factor)
        #         gtd.DynamicsGraph.printGraph(graph_tmp)
        #         print("error: ", error, "\n")

        visualize_jr_trajectory(results, self.jr, num_steps, step=1)

        # self.assertAlmostEqual(graph.error(results), 0, places=5)

    def test_solving_vertical_jump_collocation(self):
        """ Create trajectory vertical jump, and solve it. """
        dt = 0.002
        phase0_key = gtd.PhaseKey(0).key()
        sim_values, phase_steps = self.jr_simulator.simulate_to_high(dt, self.controls)
        actuation_graph_builder = self.jr_graph_builder.actuation_graph_builder
        sim_values.insertDouble(phase0_key, dt)
        sim_values.insertDouble(gtd.PhaseKey(3).key(), dt)

        collocation = gtd.CollocationScheme.Euler
        graph = self.jr_graph_builder.trajectory_graph(self.jr, phase_steps, collocation)
        graph.push_back(self.jr_graph_builder.control_priors(self.jr, self.controls))
        # graph.add(gtd.PriorFactorDouble(phase0_key, dt, gtsam.noiseModel.Isotropic.Sigma(1, 0.01)))
        num_steps = len(phase_steps)
        graph.push_back(self.jr_graph_builder.vertical_jump_goal_factors(self.jr, num_steps))

        print("init error:", graph.error(sim_values))
        results = self.lm_optimize(graph, sim_values)
        print("result error:", graph.error(results))

        visualize_jr_trajectory(results, self.jr, num_steps, step=1)

        # self.assertAlmostEqual(graph.error(results), 0, places=5)

        # init_values = gtd.ExtractValues(sim_values, graph.keys())

        # self.lm_optimize(graph, sim_values)

if __name__ == "__main__":
    # unittest.main()
    suite = unittest.TestSuite()
    suite.addTest(TestJRSimulator("test_solving_single_phase_collocation"))
    runner = unittest.TextTestRunner()
    runner.run(suite)
