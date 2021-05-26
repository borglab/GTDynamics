"""
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 *
 * @file  trajectory_optimization.py
 * @brief Optimize for the controls of a vertical jump trajectory.
 * @author Yetong Zhang
"""

import gtsam
import gtdynamics as gtd
import numpy as np

import os, sys, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.jumping_robot import Actuator, JumpingRobot
from src.jr_visualizer import visualize_jr, make_plot, visualize_jr_trajectory, plot_torso_height
from src.robot_graph_builder import RobotGraphBuilder
from src.actuation_graph_builder import ActuationGraphBuilder
from src.jr_graph_builder import JRGraphBuilder
from src.jr_values import JRValues
from src.jr_simulator import JRSimulator
from src.jr_measurements import read_t_valve, read_pressure, \
    read_marker_pix, interp_pressure, interp_marker_pix


def vertical_jump_simulation(jr, num_steps, dt, controls):
    """ Simulate vertical jump trajectory. """
    jr_simulator = JRSimulator(jr.yaml_file_path, jr.init_config)
    values, step_phases = jr_simulator.simulate(num_steps, dt, controls)
    values.insertDouble(gtd.PhaseKey(0).key(), dt)
    values.insertDouble(gtd.PhaseKey(3).key(), dt)
    return values, step_phases


def vertical_jump_trajopt(jr, controls, init_values, step_phases, dt):
    """ Collocation optimization for vertical jump. """
    jump_height = 0.378
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setLinearSolverType("MULTIFRONTAL_QR")
    params.setMaxIterations(20)

    init_values_i = init_values
    step_phases_i = step_phases
    for i in range(1):
        jr_graph_builder = JRGraphBuilder()

        # trajectory graph
        collocation = gtd.CollocationScheme.Euler
        graph = jr_graph_builder.trajectory_graph(jr, step_phases_i, collocation)

        # control priors
        graph.push_back(jr_graph_builder.control_priors(jr, controls))

        # goal factors
        num_steps = len(step_phases)
        phase0_key = gtd.PhaseKey(0).key()
        phase3_key = gtd.PhaseKey(3).key()
        graph.add(gtd.PriorFactorDouble(phase0_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))
        graph.add(gtd.PriorFactorDouble(phase3_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))
        graph.push_back(jr_graph_builder.vertical_jump_goal_factors(jr, num_steps))
        torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 1.1+jump_height))
        graph.push_back(jr_graph_builder.target_pose_goal_factor(jr, num_steps, torso_pose))

        # for f_idx in range(graph.size()):
        #     factor = graph.at(f_idx)
        #     # print(factor)
        #     if factor.error(init_values) > 1:
        #         graph_tmp = gtsam.NonlinearFactorGraph()
        #         graph_tmp.add(factor)
        #         gtd.DynamicsGraph.printGraph(graph_tmp)
        #         print("error", factor.error(init_values))


        # optimization
        optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values_i, params)
        results = optimizer.optimize()
        print("result error: ", graph.error(results))

        Tos = [results.atDouble(Actuator.ValveOpenTimeKey(1)),
            results.atDouble(Actuator.ValveOpenTimeKey(2)),
            results.atDouble(Actuator.ValveOpenTimeKey(3)),
            results.atDouble(Actuator.ValveOpenTimeKey(4))]
        Tcs = [results.atDouble(Actuator.ValveCloseTimeKey(1)),
            results.atDouble(Actuator.ValveCloseTimeKey(2)),
            results.atDouble(Actuator.ValveCloseTimeKey(3)),
            results.atDouble(Actuator.ValveCloseTimeKey(4))]
        controls = JumpingRobot.create_controls(Tos, Tcs)
        sim_values, step_phases = vertical_jump_simulation(jr, num_steps, dt, controls)
        init_values_i = sim_values
        step_phases_i = step_phases


    # initial values
    print("init error:", graph.error(init_values))
    print("jump height:", init_values.atPose3(gtd.internal.PoseKey(
        jr.robot.link("torso").id(), num_steps).key()))
    print("valve open:", init_values.atDouble(Actuator.ValveOpenTimeKey(1)),
        init_values.atDouble(Actuator.ValveOpenTimeKey(2)),
        init_values.atDouble(Actuator.ValveOpenTimeKey(3)),
        init_values.atDouble(Actuator.ValveOpenTimeKey(4)))
    print("valve close:", init_values.atDouble(Actuator.ValveCloseTimeKey(1)),
        init_values.atDouble(Actuator.ValveCloseTimeKey(2)),
        init_values.atDouble(Actuator.ValveCloseTimeKey(3)),
        init_values.atDouble(Actuator.ValveCloseTimeKey(4)))
    print("phase 3:", init_values.atDouble(gtd.PhaseKey(3).key()))


    # result values
    print("result error: ", graph.error(results))
    print("jump height:", results.atPose3(gtd.internal.PoseKey(
        jr.robot.link("torso").id(), num_steps).key()))
    print("valve open:", results.atDouble(Actuator.ValveOpenTimeKey(1)),
        results.atDouble(Actuator.ValveOpenTimeKey(2)),
        results.atDouble(Actuator.ValveOpenTimeKey(3)),
        results.atDouble(Actuator.ValveOpenTimeKey(4)))
    print("valve close:", results.atDouble(Actuator.ValveCloseTimeKey(1)),
        results.atDouble(Actuator.ValveCloseTimeKey(2)),
        results.atDouble(Actuator.ValveCloseTimeKey(3)),
        results.atDouble(Actuator.ValveCloseTimeKey(4)))

    print("phase 3:", results.atDouble(gtd.PhaseKey(3).key()))

    # make_plot(init_values, jr, num_steps)
    # make_plot(results, jr, num_steps)

    visualize_jr_trajectory(init_values, jr, num_steps, dt, step=1)
    visualize_jr_trajectory(results, jr, num_steps, dt, step=1)

    plot_torso_height([init_values, results], jr, num_steps)

    return results


def main():
    """ Main file. """
    t_sim = 0.6
    dt = 0.005

    # create jumping robot
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config_2021-04-05.yaml"
    # theta = np.pi/3
    # rest_angles = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
    # init_angles = rest_angles
    # init_vels = [0, 0, 0, 0, 0, 0]
    # torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.55))
    # torso_twist = np.zeros(6)
    # P_s_0 = 80 * 6894.76/1000 + 101.325 # (kPag)
    # init_config = JumpingRobot.create_init_config(
        # torso_pose, torso_twist, rest_angles, init_angles, init_vels, P_s_0)
    # jr = JumpingRobot(yaml_file_path, init_config)


    # create init config
    P_s_0 = 80 # (psig) source tank initial pressure
    init_config = JumpingRobot.icra_init_config()
    init_config['P_s_0'] = P_s_0 * 6894.76/1000 + 101.325 # (psig to kPa)
    jr = JumpingRobot(yaml_file_path, init_config)


    # create controls
    Tos = [0,   0,   0,   0]
    Tcs = [0.1, 0.1, 0.1, 0.1]
    controls = JumpingRobot.create_controls(Tos, Tcs)

    # simulation
    num_steps = int(t_sim/dt)
    sim_values, step_phases = vertical_jump_simulation(jr, num_steps, dt, controls)

    # print("step_phases", step_phases)

    # collocation optimization
    collo_values = vertical_jump_trajopt(jr, controls, sim_values, step_phases, dt)

    # visualize

if __name__ == "__main__":
    main()