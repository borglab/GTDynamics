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

JumpingRobot buildJr() {
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

  init_angles = rest_angles;
  init_vels["j0"] = 0;
  init_vels["j1"] = 0;
  init_vels["j2"] = 0;
  init_vels["j3"] = 0;
  init_vels["j4"] = 0;
  init_vels["j5"] = 0;

  double initial_mass = 7.873172488131229e-05;

  init_masses["j1"] = initial_mass;
  init_masses["j2"] = initial_mass;
  init_masses["j3"] = initial_mass;
  init_masses["j4"] = initial_mass;
  return JumpingRobot(rest_angles, init_angles, init_vels, init_masses, jr_params);
}

void refineHeight(double knee_open_time, double hip_open_time,  double knee_close_time, double hip_close_time, double target_height, double dt) {
}


void optHeight(double init_tank_pressure, double knee_open_time, double hip_open_time,  double knee_close_time, double hip_close_time, double target_height, double dt = 0.002, bool reach_goal=false) {
  std::cout << "\n##################################################\n";
  
  JumpingRobot jr = buildJr();
  const auto& graph_builder = jr.getGraphBuilder();
  Vector open_times = (Vector(4) << knee_open_time, hip_open_time, hip_open_time, knee_open_time).finished();
  Vector close_times = (Vector(4) << knee_close_time, hip_close_time, hip_close_time, knee_close_time).finished();

  std::cout << "simulation...\n";
  JRSimulator jr_simulator(jr);
  // Values results_sim = jr_simulator.simulate(num_steps_sim, dt, init_tank_pressure, open_times, close_times);
  Values results_sim = jr_simulator.simulateToHigh(dt, init_tank_pressure, open_times, close_times);
  vector<JumpingRobot::Phase> phases_sim = jr_simulator.phases();
  std::vector<JumpingRobot::Phase> phase_seq{JumpingRobot::Phase::Ground, JumpingRobot::Phase::Air};
  std::vector<int> phase_steps = jr_simulator.phaseSteps();
  size_t num_steps_sim = std::accumulate(phase_steps.begin(), phase_steps.end(), 0);

  std::cout << "interpolate trajectory...\n";
  Values results_interp = jr.linearInterpolation(dt, phases_sim, results_sim, phase_seq, phase_steps);
  auto phases_interp = jr.getPhases(phase_seq, phase_steps);
  std::cout << "phase steps" << phase_steps.size() << ", " << phase_steps[0] << ", " << phase_steps[1] << "\n";
  int num_steps = std::accumulate(phase_steps.begin(), phase_steps.end(), 0);

  gtsam::Pose3 sim_final_pose = results_interp.at<gtsam::Pose3>(PoseKey(3,  num_steps));
  double sim_final_height = sim_final_pose.translation().z();
  std::cout << "simulation final pose: " << sim_final_pose << "\n";
  double tank_mass_final_sim = results_interp.atDouble(SourceMassKey(num_steps));
  double tank_mass_init_sim = results_interp.atDouble(SourceMassKey(0));
  std::cout << "simulation mass loss: " << tank_mass_init_sim - tank_mass_final_sim << "\n";

  std::cout << "generate graph...\n";
  auto graph = jr.multiPhaseTrajectoryFG(phase_seq, phase_steps, CollocationScheme::Trapezoidal);
  graph.add(jr.initialStatePriors());

  Key knee_open_key = ValveOpenTimeKey(9);
  Key hip_open_key = ValveOpenTimeKey(10);
  Key knee_close_key = ValveCloseTimeKey(9);
  Key hip_close_key = ValveCloseTimeKey(10);

  const vector<PneumaticActuator>& actuators = jr.getActuators();
  graph.add(PriorFactor<double>(SourcePressureKey(0), init_tank_pressure, actuators[0].prior_pressure_cost_model));
  for (int actuator_idx = 0; actuator_idx < actuators.size(); actuator_idx++)
  {
      const auto &actuator = actuators[actuator_idx];
      int j = actuator.j();
      if (actuator_idx == 0 || actuator_idx == 3) {
        graph.add(BetweenFactor<double>(ValveOpenTimeKey(j), knee_open_key, 
                                        double(0), actuator.prior_valve_t_cost_model));
        graph.add(BetweenFactor<double>(ValveCloseTimeKey(j), knee_close_key, 
                                        double(0), actuator.prior_valve_t_cost_model));
      }
      else {
        graph.add(BetweenFactor<double>(ValveOpenTimeKey(j), hip_open_key, 
                                double(0), actuator.prior_valve_t_cost_model));
        graph.add(BetweenFactor<double>(ValveCloseTimeKey(j), hip_close_key, 
                                double(0), actuator.prior_valve_t_cost_model));
      }
  }

  results_interp.insert(knee_close_key, knee_close_time);
  results_interp.insert(hip_close_key, hip_close_time);
  results_interp.insert(knee_open_key, knee_open_time);
  results_interp.insert(hip_open_key, hip_open_time);

  auto final_pose_key = PoseKey(3,  num_steps);
  auto final_twist_key = TwistKey(3, num_steps);

  double threshold = 0.01;
  Pose3 target_pose = Pose3(Rot3::identity(), Point3(0, 0, target_height));
  // if (abs(target_height - sim_final_height) < threshold) {
  //   target_pose = Pose3(Rot3::identity(), Point3(0, 0, target_height));
  // }
  // else if (target_height > sim_final_height) {
  //   target_pose = Pose3(Rot3::identity(), Point3(0, 0, sim_final_height+threshold));
  // }
  // else {
  //   target_pose = Pose3(Rot3::identity(), Point3(0, 0, sim_final_height-threshold));
  // }

  // graph.add(PriorFactor<double>(knee_open_key, 0.0, actuators[0].prior_valve_t_cost_model));
  // graph.add(PriorFactor<double>(hip_open_key, 0.0, actuators[0].prior_valve_t_cost_model));
  graph.add(PriorFactor<double>(knee_open_key, knee_open_time, actuators[0].prior_valve_t_cost_model));
  graph.add(PriorFactor<double>(hip_open_key, hip_open_time, actuators[0].prior_valve_t_cost_model));
  graph.add(PriorFactor<double>(knee_close_key, knee_close_time, actuators[0].prior_valve_t_cost_model));
  graph.add(PriorFactor<gtsam::Pose3>(final_pose_key, target_pose, graph_builder.opt().p_cost_model));
  graph.add(PriorFactor<gtsam::Vector6>(final_twist_key, gtsam::Vector6::Zero(), graph_builder.opt().v_cost_model));

  // graph.add(jr.costFactors(num_steps));

  std::cout << "optimizing...\n";
  std::cout << "init error: " << graph.error(results_interp) << "\n";

  gtsam::LevenbergMarquardtParams params;
  // params.setlambdaUpperBound(1e20);
  params.setLinearSolverType("MULTIFRONTAL_QR");
  params.setVerbosityLM("SUMMARY");
  // params.setAbsoluteErrorTol(1e-10);
  // params.setRelativeErrorTol(1e-8);
  params.setMaxIterations(15);
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, results_interp, params);
  // gtsam::DoglegOptimizer optimizer(graph, results_interp);
  gtsam::Values collocation_result = optimizer.optimize();
  std::cout << "result error: " << graph.error(collocation_result) << "\n";

  double knee_open_time_opt = collocation_result.atDouble(knee_open_key);
  double hip_open_time_opt = collocation_result.atDouble(hip_open_key);
  double knee_close_time_opt = collocation_result.atDouble(knee_close_key);
  double hip_close_time_opt = collocation_result.atDouble(hip_close_key);
  double tank_mass_final_opt = collocation_result.atDouble(SourceMassKey(num_steps));
  double tank_mass_init_opt = collocation_result.atDouble(SourceMassKey(0));
  std::cout << "knee open time: " << knee_open_time_opt
            << "\thip open time: " << hip_open_time_opt
            << "\tknee close time: " << knee_close_time_opt
            << "\thip close time: " << hip_close_time_opt << "\n";
  std::cout << "mass loss: " << tank_mass_init_opt - tank_mass_final_opt << "\n";
  Pose3 final_pose_opt = collocation_result.at<gtsam::Pose3>(final_pose_key);
  std::cout << "final pose: " << final_pose_opt << "\n";
  double height_opt = final_pose_opt.translation().z();

  if (reach_goal) {
    jr.exportPoses(target_height, num_steps, collocation_result);
    return;
  }
  if (abs(height_opt - target_height) < 0.001) {
    return;
    // optHeight(init_tank_pressure, knee_open_time_opt, hip_open_time_opt, knee_close_time_opt, hip_close_time_opt, target_height, dt, true);
  }
  else {
    optHeight(init_tank_pressure, knee_open_time_opt, hip_open_time_opt, knee_close_time_opt, hip_close_time_opt, target_height, dt, false);
  }
}


int main()
{
  // std::vector<double> target_heights{1.2, 1.3, 1.4, 1.5};

  // optHeight(0.0174815, 0.0, 0.0827593, 0.0831586, 1.3, 0.002, true);
  // optHeight(65.0 * 6.89476 + 101.325, 0, 0.0192553, 0.0615197, 0.0706128, 1.2, 0.0005, true);
  optHeight(65.0 * 6.89476 + 101.325, 0.0104335, 0, 0.112054, 0.106954, 1.4, 0.002, true);
  optHeight(80.0 * 6.89476 + 101.325, 0.0176424, 0, 0.133869, 0.123735, 1.5, 0.002, true);

  return 0;

}
