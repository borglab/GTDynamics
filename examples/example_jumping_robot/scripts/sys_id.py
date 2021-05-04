

import gtsam
import gtdynamics as gtd
import numpy as np

import os, sys, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.jumping_robot import Actuator, JumpingRobot
from src.jr_visualizer import visualize_jr
from src.robot_graph_builder import RobotGraphBuilder
from src.actuation_graph_builder import ActuationGraphBuilder
from src.jr_graph_builder import JRGraphBuilder
from src.jr_values import JRValues
from src.jr_simulator import JRSimulator
from src.helpers import read_pressure, read_marker_pix, interp_pressure, interp_marker_pix


def vertical_jump_simulation(jr, num_steps, dt, controls):
    """ Simulate vertical jump trajectory. """
    jr_simulator = JRSimulator(jr.yaml_file_path, jr.init_config)
    values, step_phases = jr_simulator.simulate(num_steps, dt, controls)
    values.insertDouble(gtd.PhaseKey(0).key(), dt)
    # values.insertDouble(gtd.PhaseKey(3).key(), dt)
    return values, step_phases


def vertical_jump_sysid(jr, controls, init_values, step_phases, path_exp):
    """ Collocation optimization for vertical jump. """
    jr_graph_builder = JRGraphBuilder()

    # trajectory graph
    collocation = gtd.CollocationScheme.Euler # gtd.CollocationScheme.Trapezoidal
    graph = jr_graph_builder.trajectory_graph(jr, step_phases, collocation)

    # control priors
    graph.push_back(jr_graph_builder.control_priors(jr, controls))

    # system id factors
    dim_camera = [1920, 1080] # GoPro is 1920 x 1080 (oriented horizontally)
    time_sens, pressure_sens = read_pressure(path_exp)
    time_vid, pix_vid = read_marker_pix(path_exp, dim_camera)
    time_interp = np.arange(0, dt*len(step_phases), dt) #TODO: check
    pressure_interp = interp_pressure(time_exp, pressure_exp, time_interp)
    pix_interp = interp_marker_pix(time_vid, pix_vid, time_interp)
    marker_locations = JumpingRobot.marker_locations()()
    graph.push_back(jr_graph_builder.sys_id_graph(jr, marker_locations, 
        pix_interp, pressure_interp))


    # # goal factors
    # num_steps = len(step_phases)
    # phase0_key = gtd.PhaseKey(0).key()
    # graph.add(gtd.PriorFactorDouble(phase0_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))
    # # graph.push_back(self.jr_graph_builder.vertical_jump_goal_factors(self.jr, num_steps))

    
    for f_idx in range(graph.size()):
        factor = graph.at(f_idx)
        if factor.error(init_values) > 1:
            graph_tmp = gtsam.NonlinearFactorGraph()
            graph_tmp.add(factor)
            gtd.DynamicsGraph.printGraph(graph_tmp)
            print("error", factor.error(init_values))

    # optimization
    # params = gtsam.LevenbergMarquardtParams()
    # params.setVerbosityLM("SUMMARY")
    # optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, params)

    print("init error: ", graph.error(init_values))
    optimizer = gtsam.GaussNewtonOptimizer(graph, init_values)
    results = optimizer.optimize()

    return results



def main():
    """ Main file. """
    # create jumping robot
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config_2021-04-05.yaml"
    theta = np.pi/3
    rest_angles = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
    init_angles = rest_angles
    init_vels = [0, 0, 0, 0, 0, 0]
    torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.55))
    torso_twist = np.zeros(6)
    P_s_0 = 65 * 6894.76/1000
    init_config = JumpingRobot.create_init_config(
        torso_pose, torso_twist, rest_angles, init_angles, init_vels, P_s_0)
    jr = JumpingRobot(yaml_file_path, init_config)

    # create controls TODO: set to experimental controls
    Tos = [0, 0, 0, 0]
    Tcs = [0.1, 0.1, 0.1, 0.1]
    controls = JumpingRobot.create_controls(Tos, Tcs)

    # simulate
    num_steps = 1
    dt = 0.005
    sim_values, step_phases = vertical_jump_simulation(jr, num_steps, dt, controls)
    print("step_phases", step_phases)

    # system id
    path_exp = '/home/cs3630/Documents/system-id-data/0p00_0p12_hipknee-80source 2021-04-05 11-43-47'
    sysid_values = vertical_jump_sysid(jr, controls, sim_values, step_phases, path_exp)


    # visualize TODO


if __name__ == "__main__":
    main()