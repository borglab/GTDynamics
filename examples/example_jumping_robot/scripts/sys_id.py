

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
from src.helpers import read_t_musc, read_pressure, \
    read_marker_pix, interp_pressure, interp_marker_pix


def vertical_jump_simulation(jr, num_steps, dt, controls):
    """ Simulate vertical jump trajectory. """
    jr_simulator = JRSimulator(jr.yaml_file_path, jr.init_config)
    values, step_phases = jr_simulator.simulate(num_steps, dt, controls)
    values.insertDouble(gtd.PhaseKey(0).key(), dt)
    # values.insertDouble(gtd.PhaseKey(3).key(), dt)
    return values, step_phases


def vertical_jump_sysid(jr, controls, init_values, step_phases, path_exp_data, path_cam_params):
    """ Collocation optimization for vertical jump. """
    jr_graph_builder = JRGraphBuilder()

    # trajectory graph
    collocation = gtd.CollocationScheme.Euler # gtd.CollocationScheme.Trapezoidal
    graph = jr_graph_builder.trajectory_graph(jr, step_phases, collocation)

    # control priors
    graph.push_back(jr_graph_builder.control_priors(jr, controls))

    # system id factors
    time_mcu, pressures_mcu = read_pressure(path_exp_data)
    time_vid, pixels_vid = read_marker_pix(path_exp_data, dim_camera)
    pressures_interp = interp_pressure(time_mcu, pressures_mcu, time_interp)
    pixels_interp = interp_marker_pix(time_vid, pixels_vid, time_interp)
    marker_locations = JumpingRobot.marker_locations() #TODO: maybe put marker locations in text file
    graph.push_back(jr_graph_builder.sys_id_graph(jr, marker_locations, 
        pixels_interp, pressures_interp))

    with open(path_cam_params) as file:
        cam_params = yaml.load(file, Loader=yaml.FullLoader)
    graph.push_back(jr_graph_builder.camera_priors(cam_params))

    # add initial system id estimates
    init_values = JRGraphBuilder.sys_id_estimates(jr, init_values, marker_locations,
        pixels_interp, pressures_interp, cam_params)


    # # goal factors
    # num_steps = len(step_phases)
    # phase0_key = gtd.PhaseKey(0).key()
    # graph.add(gtd.PriorFactorDouble(phase0_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))
    # # graph.push_back(self.jr_graph_builder.vertical_jump_goal_factors(self.jr, num_steps))

    # debug
    # for f_idx in range(graph.size()):
    #     factor = graph.at(f_idx)
    #     if factor.error(init_values) > 1:
    #         graph_tmp = gtsam.NonlinearFactorGraph()
    #         graph_tmp.add(factor)
    #         gtd.DynamicsGraph.printGraph(graph_tmp)
    #         print("error", factor.error(init_values))

    # optimization
    print("init error: ", graph.error(init_values))
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, params)
    results = optimizer.optimize()
    print("result error: ", graph.error(results))


    # optimizer = gtsam.GaussNewtonOptimizer(graph, init_values)
    # results = optimizer.optimize()

    return results



def main():
    """ Main file. """
    # parameters
    yaml_file_path = "examples/example_jumping_robot/yaml/robot_config_2021-04-05.yaml"
    path_exp_data = '/home/cs3630/Documents/system-id-data/0p00_0p12_hipknee-80source 2021-04-05 11-43-47'
    path_cam_params = '/home/cs3630/Documents/system-id-data/camera_param.yaml'
    P_s_0 = 80 # (psi) source tank initial pressure
    t_sim = 0.2
    dt = 0.005

    # create jumping robot
    # theta = np.pi/3
    # rest_angles = [-theta, 2 * theta, -theta, -theta, 2*theta, -theta]
    # init_angles = rest_angles
    # init_vels = [0, 0, 0, 0, 0, 0]
    # torso_pose = gtsam.Pose3(gtsam.Rot3(), gtsam.Point3(0, 0, 0.55))
    # torso_twist = np.zeros(6)
    # P_s_0 = P_s_0 * 6894.76/1000 # (psi to kPa) TODO: check that atm pressure is added in later version
    # init_config = JumpingRobot.create_init_config(
    #     torso_pose, torso_twist, rest_angles, init_angles, init_vels, P_s_0)

    init_config = JumpingRobot.icra_init_config()
    # TODO: modify P_s_0 in dict
    jr = JumpingRobot(yaml_file_path, init_config)

    # create controls from experimental valve times
    t_valve = read_t_valve(path_exp_data)
    controls = JumpingRobot.create_controls(t_valve[0,:], t_valve[1,:])

    # simulate
    num_steps = int(t_sim/dt)
    sim_values, step_phases = vertical_jump_simulation(jr, num_steps, dt, controls)
    print("step_phases", step_phases)

    # system id
    time_interp = np.arange(0, dt*len(step_phases), dt)
    sysid_values = vertical_jump_sysid(jr, controls, sim_values, step_phases, 
        path_exp_data, path_cam_params, time_interp)


    # visualize TODO


if __name__ == "__main__":
    main()