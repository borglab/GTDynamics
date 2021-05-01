/**
 * @file  testJumpingRobot.cpp
 * @brief test trajectory optimization for jumping robot
 * @Author: Yetong Zhang
 */


#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include <iostream>
#include <fstream>

#include <chrono>
#include <ctime>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/jumpingrobot/factorsJumpingRobot.h"

using namespace std;
using namespace gtdynamics;
using namespace gtsam;

int main()
{
  std::cout << "Initializing robot...\n";
  JointValueMap rest_angles, init_angles, init_vels, init_masses;
  double default_angle = 0.14824135537076213;
  double angle_offset = M_PI_2 - default_angle;

  JumpingRobot::Params jr_params;
  jr_params.angle_offset = angle_offset;
  jr_params.Rs = 287.0550;
  jr_params.T = 296.15;
  jr_params.tank_volume = 1.475e-3;


  rest_angles["j0"] = -12.6 / 180.0 * M_PI - default_angle;
  rest_angles["j1"] = 161.7/180.0 * M_PI;
  rest_angles["j2"] = -59.1/180.0 * M_PI - angle_offset;
  rest_angles["j3"] = -59.1/180.0 * M_PI - angle_offset;
  rest_angles["j4"] = 161.7/180.0 * M_PI;
  rest_angles["j5"] = -12.6 / 180.0 * M_PI - default_angle;

  // double theta1 = 0.31217200034782067;
  // rest_angles["j0"] = -default_angle;
  // rest_angles["j1"] = theta1;
  // rest_angles["j2"] = -(theta1-default_angle);
  // rest_angles["j3"] = -(theta1-default_angle);
  // rest_angles["j4"] = theta1;
  // rest_angles["j5"] = -default_angle;

  double initial_mass = 7.873172488131229e-05;

  init_angles = rest_angles;
  init_vels["j0"] = 0;
  init_vels["j1"] = 0;
  init_vels["j2"] = 0;
  init_vels["j3"] = 0;
  init_vels["j4"] = 0;
  init_vels["j5"] = 0;
  init_masses["j1"] = initial_mass;
  init_masses["j2"] = initial_mass;
  init_masses["j3"] = initial_mass;
  init_masses["j4"] = initial_mass;

  JumpingRobot jr(rest_angles, init_angles, init_vels, init_masses, jr_params);
  const auto& graph_builder = jr.getGraphBuilder();

  int num_steps_sim = 200;
  double dt = 0.002;
  double init_tank_pressure = 65.0 * 6.89476 + 101.325;
  Vector open_times = (Vector(4) << 0, 0, 0, 0).finished();
  Vector close_times = (Vector(4) << 0.098, 0.098, 0.098, 0.098).finished();

  std::cout << "simulation...\n";
  JRSimulator jr_simulator(jr);
  // Values results_sim = jr_simulator.simulate(num_steps_sim, dt, init_tank_pressure, open_times, close_times);
  Values results_sim = jr_simulator.simulateToHigh(dt, init_tank_pressure, open_times, close_times);
  vector<JumpingRobot::Phase> phases_sim = jr_simulator.phases();
  std::vector<JumpingRobot::Phase> phase_seq{JumpingRobot::Phase::Ground, JumpingRobot::Phase::Air};
  std::vector<int> phase_steps = jr_simulator.phaseSteps();
  num_steps_sim = std::accumulate(phase_steps.begin(), phase_steps.end(), 0);

  // std::cout << "export trajectory\n";
  // jr.exportTrajectoryMultiPhase(num_steps_sim, dt, phases_sim, results);

  std::cout << "interpolate trajectory...\n";
  // values should include (dynamics variables, actuator variables, time varaibles, dt varaibles, start_time, init_pressure)
  Values results_interp = jr.linearInterpolation(dt, phases_sim, results_sim, phase_seq, phase_steps);
  auto phases_interp = jr.getPhases(phase_seq, phase_steps);

  std::cout << "phase steps" << phase_steps.size() << ", " << phase_steps[0] << ", " << phase_steps[1] << "\n";
  int num_steps = std::accumulate(phase_steps.begin(), phase_steps.end(), 0);
  std::cout << "generate graph...\n";
  // graph should include :
  //        dynamics for each step (guardian factors for transition steps),
  //        initial_state priors,
  //        collocation factors,
  //        actuators & zero-torque for non-actuated joints,
  //        start_time, init_pressure,
  //        time calculation factors

  // dynamics for each step (guardains), collocation factors, actuator graphs
  auto graph = jr.multiPhaseTrajectoryFG(phase_seq, phase_steps, CollocationScheme::Trapezoidal); //TODO: check why trapezoidal causes ILS
  // initial_state priors
  graph.add(jr.initialStatePriors());
  // start_time, init_pressure
  // graph.add(jr.controlPriors(init_tank_pressure, open_times, close_times));
  // prior of dt for second phase
  // auto second_phase = PhaseKey(1);
  // graph.add(gtsam::PriorFactor<double>(second_phase, results_sim.atDouble(second_phase), graph_builder.opt().time_cost_model));

  std::cout << "generate graph...\n";
  // graph should include :
  //        dynamics for each step (guardian factors for transition steps),
  //        initial_state priors,
  //        collocation factors,
  //        actuators & zero-torque for non-actuated joints,
  //        start_time, init_pressure,
  //        time calculation factors

  auto first_phase = PhaseKey(0);
  auto second_phase = PhaseKey(1);
  // graph.add(gtsam::PriorFactor<double>(first_phase, results_interp.atDouble(first_phase), graph_builder.opt().time_cost_model));
  // graph.add(gtsam::PriorFactor<double>(second_phase, results_interp.atDouble(second_phase), graph_builder.opt().time_cost_model));


  Key knee_close_key = ValveCloseTimeKey(9);
  Key hip_close_key = ValveCloseTimeKey(10);

  const vector<PneumaticActuator>& actuators = jr.getActuators();
  graph.add(PriorFactor<double>(SourcePressureKey(0), init_tank_pressure, actuators[0].prior_pressure_cost_model));
  for (int actuator_idx = 0; actuator_idx < actuators.size(); actuator_idx++)
  {
      const auto &actuator = actuators[actuator_idx];
      int j = actuator.j();
      graph.add(gtsam::PriorFactor<double>(ValveOpenTimeKey(j), open_times[actuator_idx],
                                            actuator.prior_valve_t_cost_model));
      if (actuator_idx == 0 || actuator_idx == 3) {
        graph.add(BetweenFactor<double>(ValveCloseTimeKey(j), knee_close_key, 
                                        double(0), actuator.prior_valve_t_cost_model));
      }
      else {
        graph.add(BetweenFactor<double>(ValveCloseTimeKey(j), hip_close_key, 
                                double(0), actuator.prior_valve_t_cost_model));
      }
      
      // graph.add(gtsam::PriorFactor<double>(ValveCloseTimeKey(j), close_times[actuator_idx],
      //                                       actuator.prior_valve_t_cost_model));
  }

  results_interp.insert(knee_close_key, close_times(0));
  results_interp.insert(hip_close_key, close_times(1));

  auto final_pose_key = PoseKey(3,  num_steps);
  auto final_twist_key = TwistKey(3, num_steps);
  gtsam::Pose3 target_pose(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 1.4));
  graph.add(PriorFactor<double>(knee_close_key, close_times(0), actuators[0].prior_valve_t_cost_model));
  graph.add(PriorFactor<gtsam::Pose3>(final_pose_key, target_pose, graph_builder.opt().p_cost_model));
  graph.add(PriorFactor<gtsam::Vector6>(final_twist_key, gtsam::Vector6::Zero(), graph_builder.opt().v_cost_model));

  // gtsam::Ordering colamd = Ordering::Colamd(graph);
  // colamd.print("", gtsam::MultiRobotKeyFormatter);

  std::cout << "optimizing...\n";
  // DynamicsGraph::printValues(results_interp);
  // DynamicsGraph::printGraph(graph);
  
  std::cout << "init error: " << graph.error(results_interp) << "\n";
  std::cout << "simulation final pose: " << results_interp.at<gtsam::Pose3>(PoseKey(3,  num_steps)) << "\n";

  gtsam::LevenbergMarquardtParams params;
  // params.setlambdaUpperBound(1e20);
  params.setLinearSolverType("MULTIFRONTAL_QR");
  params.setVerbosityLM("SUMMARY");
  // params.setAbsoluteErrorTol(1e-10);
  // params.setRelativeErrorTol(1e-8);
  // params.setMaxIterations(20);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, results_interp, params);
  // gtsam::DoglegOptimizer optimizer(graph, results_interp);

  auto start_t = std::chrono::system_clock::now();

  gtsam::Values collocation_result = optimizer.optimize();
  std::cout << "result error: " << graph.error(collocation_result) << "\n";

  auto end_t = std::chrono::system_clock::now();
  std::cout << "time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count() << "\n";

  std::cout << "exporting trajectory...\n";
  jr.exportTrajectoryMultiPhase(num_steps, phases_interp, collocation_result);
  std::cout << "knee close time: " << collocation_result.atDouble(knee_close_key);
  std::cout << "\thip close time: " << collocation_result.atDouble(hip_close_key) << "\n";
  std::cout << "final pose: " << collocation_result.at<gtsam::Pose3>(final_pose_key) << "\n";
}
