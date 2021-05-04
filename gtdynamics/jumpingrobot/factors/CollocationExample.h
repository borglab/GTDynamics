/**
 * @file  testJumpingRobot.cpp
 * @brief test trajectory optimization for jumping robot
 * @Author: Yetong Zhang
 */


#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <iostream>
#include <fstream>

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/jumpingrobot/factors/JumpingRobot.h"

using namespace std;

using namespace gtsam;

namespace gtdynamics {

Values getExampleGraphValues(NonlinearFactorGraph& graph)
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
  double dt = 0.01;
  double source_pressure = 65.0 * 6.89476 + 101.325;
  Vector open_times = (Vector(4) << 0, 0, 0, 0).finished();
  Vector close_times = (Vector(4) << 0.098, 0.098, 0.098, 0.098).finished();


  std::cout << "simulation...\n";
  JRSimulator jr_simulator(jr);
  // Values results_sim = jr_simulator.simulate(num_steps_sim, dt, source_pressure, open_times, close_times);
  Values results_sim = jr_simulator.simulateToHigh(dt, source_pressure, open_times, close_times);
  vector<JumpingRobot::Phase> phases_sim = jr_simulator.phases();
  std::vector<JumpingRobot::Phase> phase_seq{JumpingRobot::Phase::Ground, JumpingRobot::Phase::Air};
  std::vector<int> phase_steps = jr_simulator.phaseSteps();
  num_steps_sim = std::accumulate(phase_steps.begin(), phase_steps.end(), 0);

  // phase_steps = std::vector<int> {70, 20};

  std::cout << "interpolate trajectory...\n";
  // values should include (dynamics variables, actuator variables, time varaibles, dt varaibles, start_time, init_pressure)
  results_sim = jr.linearInterpolation(dt, phases_sim, results_sim, phase_seq, phase_steps);
  phases_sim = jr.getPhases(phase_seq, phase_steps);

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
  // phase_steps[0] = 1;
  // phase_steps[1] = 1;
  graph.add(jr.multiPhaseTrajectoryFG(phase_seq, phase_steps, CollocationScheme::Trapezoidal));
  // initial_state priors
  graph.add(jr.initialStatePriors());
  // start_time, init_pressure
  graph.add(jr.controlPriors(source_pressure, open_times, close_times));
  // prior of dt for second phase
  auto second_phase = PhaseKey(JumpingRobot::Phase::Air);
  graph.add(gtsam::PriorFactor<double>(second_phase, results_sim.atDouble(second_phase), graph_builder.opt().time_cost_model));

  std::cout << graph.keys().size() << "\n";
  std::cout << results_sim.keys().size() << "\n";

  return results_sim;
}

}