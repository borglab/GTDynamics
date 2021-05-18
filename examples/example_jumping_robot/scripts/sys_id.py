

import gtsam
import gtdynamics as gtd
import numpy as np

import os, sys, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0, parentdir)

from src.jumping_robot import Actuator, JumpingRobot
from src.jr_visualizer import visualize_jr, make_plot
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
    # values.insertDouble(gtd.PhaseKey(3).key(), dt)
    return values, step_phases


def vertical_jump_sysid(jr, controls, init_values, step_phases, pixels_all_frames, pressures_all_frames):
    """ System identification optimization for vertical jump. """
    jr_graph_builder = JRGraphBuilder()

    # trajectory graph
    collocation = gtd.CollocationScheme.Euler
    graph = jr_graph_builder.sys_id_graph(jr, step_phases, 
        pixels_all_frames, pressures_all_frames, collocation)
    graph.push_back(jr_graph_builder.control_priors(jr, controls))
    phase0_key = gtd.PhaseKey(0).key()
    graph.add(gtd.PriorFactorDouble(phase0_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))

    # add values for system identification
    num_frames = len(pixels_all_frames)
    init_values_measurements = JRValues.sys_id_estimates(jr, num_frames, init_values)
    init_values.insert(init_values_measurements)


    # debug
    # for f_idx in range(graph.size()):
    #     factor = graph.at(f_idx)
    #     if factor.error(init_values) > 1:
    #         graph_tmp = gtsam.NonlinearFactorGraph()
    #         graph_tmp.add(factor)
    #         gtd.DynamicsGraph.printGraph(graph_tmp)
    #         print("error", factor.error(init_values))

    # values_keys = init_values.keys()
    # for key in gtd.KeySetToKeyVector(graph.keys()):
    #     if not key in values_keys:
    #         print(gtd.DynamicsSymbol(key))


    # values
    print("init error: ", graph.error(init_values))
    print("Tube Diam:", init_values.atDouble(Actuator.TubeDiameterKey())*39.37, "in")
    print("Knee Stiffness:", init_values.atDouble(Actuator.TendonStiffnessKey(1)), "N/m")
    print("Hip Stiffness:", init_values.atDouble(Actuator.TendonStiffnessKey(2)), "N/m")
    print("Joint Damping:", init_values.atDouble(Actuator.DampingKey()), "Nm/rad/s")

    # optimization
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, params)
    results = optimizer.optimize()
    print("\nresult error: ", graph.error(results))

    # values
    print("Tube Diam:", results.atDouble(Actuator.TubeDiameterKey())*39.37, "in")
    print("Knee Stiffness:", results.atDouble(Actuator.TendonStiffnessKey(1)), "N/m")
    print("Hip Stiffness:", results.atDouble(Actuator.TendonStiffnessKey(2)), "N/m")
    print("Joint Damping:", results.atDouble(Actuator.DampingKey()), "Nm/rad/s")

    # optimizer = gtsam.GaussNewtonOptimizer(graph, init_values)
    # results = optimizer.optimize()

    return results



def main():
    """ Main file. """
    # parameters
    jr_folder = "examples/example_jumping_robot/"
    yaml_file_path = jr_folder + "/yaml/robot_config_2021-04-05.yaml"
    path_exp_data = jr_folder + "system-id-data/0p00_0p12_hipknee-80source 2021-04-05 11-43-47"
    
    t_sim = 0.23 # sim up to just before takeoff
    dt = 0.005

    # create init config
    P_s_0 = 80 # (psig) source tank initial pressure
    init_config = JumpingRobot.icra_init_config()
    init_config['P_s_0'] = P_s_0 * 6894.76/1000 + 101.325 # (psig to kPa)
    jr = JumpingRobot(yaml_file_path, init_config)

    # create controls from experimental valve times
    t_valve = read_t_valve(path_exp_data)
    controls = JumpingRobot.create_controls(t_valve[0,:], t_valve[1,:])

    # simulate
    num_steps = int(t_sim/dt)
    # num_steps = 10
    sim_values, step_phases = vertical_jump_simulation(jr, num_steps, dt, controls)
    print("step_phases", step_phases)

    # read experiment and measurement data
    dim_camera = [1920, 1080]
    time_mcu, pressures_mcu = read_pressure(path_exp_data)
    time_vid, pixels_vid = read_marker_pix(path_exp_data, dim_camera)
    time_interp = np.arange(num_steps) * dt
    pixels_interp = interp_marker_pix(time_vid, pixels_vid, time_interp)
    pressures_interp = interp_pressure(time_mcu, pressures_mcu, time_interp) + 101.325 # (kPag)

    # system id
    # time_interp = np.arange(0, dt*len(step_phases), dt)
    sysid_results = vertical_jump_sysid(jr, controls, sim_values, step_phases, 
        pixels_interp, pressures_interp)

    # visualize
    make_plot(sysid_results, jr, num_steps)



if __name__ == "__main__":
    main()