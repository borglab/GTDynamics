

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
    jr_simulator = JRSimulator(jr)
    values, step_phases = jr_simulator.simulate(num_steps, dt, controls)
    values.insertDouble(gtd.PhaseKey(0).key(), dt)
    values.insertDouble(gtd.PhaseKey(3).key(), dt)
    return values, step_phases


def vertical_jump_sysid(jr, controls, init_values, step_phases, pixels_all_frames,
    pressures_all_frames):
    """ System identification optimization for vertical jump. """
    jr_graph_builder = JRGraphBuilder()

    # trajectory graph
    collocation = gtd.CollocationScheme.Trapezoidal
    graph = jr_graph_builder.sys_id_graph(jr, step_phases, 
        pixels_all_frames, pressures_all_frames, collocation)
    graph.push_back(jr_graph_builder.control_priors(jr, controls))
    phase0_key = gtd.PhaseKey(0).key()
    phase3_key = gtd.PhaseKey(3).key()
    graph.add(gtd.PriorFactorDouble(phase0_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))
    graph.add(gtd.PriorFactorDouble(phase3_key, 0.005, gtsam.noiseModel.Isotropic.Sigma(1, 0.0001)))

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

    # optimization
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setLinearSolverType("MULTIFRONTAL_QR")
    params.setMaxIterations(20)
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, init_values, params)
    results = optimizer.optimize()

    # initial values
    print("init error: ", graph.error(init_values))
    print("Tube Diam:", init_values.atDouble(Actuator.TubeDiameterKey())*39.37, "in")
    print("Knee Stiffness:", init_values.atDouble(Actuator.TendonStiffnessKey(1)), "N/m")
    print("Hip Stiffness:", init_values.atDouble(Actuator.TendonStiffnessKey(2)), "N/m")
    print("Joint Damping:", init_values.atDouble(Actuator.DampingKey()), "Nm/rad/s")
    print("Cam pose:", init_values.atPose3(JumpingRobot.CameraPoseKey()))
    print("Cam cal:", init_values.atCal3Bundler(JumpingRobot.CalibrationKey()))

    # results
    print("\nresult error: ", graph.error(results))
    print("Tube Diam:", results.atDouble(Actuator.TubeDiameterKey())*39.37, "in")
    print("Knee Stiffness:", results.atDouble(Actuator.TendonStiffnessKey(1)), "N/m")
    print("Hip Stiffness:", results.atDouble(Actuator.TendonStiffnessKey(2)), "N/m")
    print("Joint Damping:", results.atDouble(Actuator.DampingKey()), "Nm/rad/s")
    print("Cam pose:", results.atPose3(JumpingRobot.CameraPoseKey()))
    print("Cam cal:", results.atCal3Bundler(JumpingRobot.CalibrationKey()))

    return results



def main():
    """ Main file. """
    # parameters
    jr_folder = "examples/example_jumping_robot/"
    yaml_file_path = jr_folder + "/yaml/robot_config_2021-04-05.yaml"
    path_exp_data = jr_folder + "system-id-data/0p00_0p09_hipknee-65source 2021-04-05 11-17-40"
    P_s_0 = 65 # (psig) source tank initial pressure
    t_sim = 0.45 # sim up to before feet collide
    dt = 0.005

    # read experiment and measurement data
    num_steps = int(t_sim/dt)
    time_interp = np.arange(num_steps) * dt
    dim_camera = [1920, 1080]
    time_mcu, pressures_mcu = read_pressure(path_exp_data)
    time_vid, pixels_vid = read_marker_pix(path_exp_data, dim_camera)
    pixels_interp = interp_marker_pix(time_vid, pixels_vid, time_interp)
    pressures_interp = interp_pressure(time_mcu, pressures_mcu, time_interp) + 101.325 # (kPag)

    # create init config
    init_config = JumpingRobot.icra_init_config()
    init_config['P_s_0'] = P_s_0 * 6894.76/1000 + 101.325 # (psig to kPa)
    jr = JumpingRobot.from_yaml(yaml_file_path, init_config)

    # create controls from experimental valve times
    t_valve = read_t_valve(path_exp_data)
    controls = JumpingRobot.create_controls(t_valve[0,:], t_valve[1,:])

    knee_stiffness_list = []
    hip_stiffness_list = []

    # iterative loop
    for i in range(40):
        # simulate
        sim_values, step_phases = vertical_jump_simulation(jr, num_steps, dt, controls)
        print("step_phases", step_phases)

        # system id
        sysid_results = vertical_jump_sysid(jr, controls, sim_values, step_phases, 
            pixels_interp, pressures_interp)

        # update ID params for next iteration
        jr.params["pneumatic"]["d_tube_valve_musc"] = sysid_results.atDouble(Actuator.TubeDiameterKey())*39.3701 # (m to in)
        jr.params["knee"]["b"] = max(0, sysid_results.atDouble(Actuator.DampingKey()))
        jr.params["hip"]["b"] = max(0, sysid_results.atDouble(Actuator.DampingKey()))
        jr.params["knee"]["k_tendon"] = sysid_results.atDouble(Actuator.TendonStiffnessKey(1))
        jr.params["hip"]["k_tendon"] = sysid_results.atDouble(Actuator.TendonStiffnessKey(2))

        knee_stiffness_list.append(sysid_results.atDouble(Actuator.TendonStiffnessKey(1)))
        hip_stiffness_list.append(sysid_results.atDouble(Actuator.TendonStiffnessKey(2)))

        # for j in range(3):
        #     jr.actuators[j].config['b'] = max(0, sysid_results.atDouble(Actuator.DampingKey()))
        #     if j == 0 or j == 3:
        #         jr.actuators[j].config['k_tendon'] = sysid_results.atDouble(Actuator.TendonStiffnessKey(1))
        #     else:
        #         jr.actuators[j].config['k_tendon'] = sysid_results.atDouble(Actuator.TendonStiffnessKey(2))

    print(knee_stiffness_list)
    print(hip_stiffness_list)

    # visualize
    make_plot(sysid_results, jr, num_steps)



if __name__ == "__main__":
    main()