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
  auto graph = jr.multiPhaseTrajectoryFG(phase_seq, phase_steps, CollocationScheme::Trapezoidal);
  // initial_state priors
  graph.add(jr.initialStatePriors());
  // start_time, init_pressure
  graph.add(jr.controlPriors(source_pressure, open_times, close_times));
  // prior of dt for second phase
  auto second_phase = PhaseKey(JumpingRobot::Phase::Air);
  graph.add(gtsam::PriorFactor<double>(second_phase, results_sim.atDouble(second_phase), graph_builder.opt().time_cost_model));

  std::cout << graph.keys().size() << "\n";
  std::cout << results_sim.keys().size() << "\n";

  // auto graph2 = jr.actuatorGraphs(num_steps);
  // std::cout << "actuator error: " << graph2.error(results_sim) << "\n";

  // auto graph1 = jr.multiPhaseTrajectoryFG(phase_seq, phase_steps, CollocationScheme::Euler);
  // std::cout << "dynamics error: " << graph1.error(results_sim) << "\n";

  jr.exportData(num_steps, results_sim);

  GaussianFactorGraph linear_graph = *graph.linearize(results_sim);
  size_t graph_dim = 0;
  for (auto factor:linear_graph) {
    graph_dim += factor->jacobian().second.size();
    // graph_dim += H.rows();
  }
  size_t variable_dim = 0;
  VariableIndex vi(linear_graph);
  for (Key key:results_sim.keys()) {
    auto factor_idx = vi[key][0];
    auto& factor = linear_graph.at(factor_idx);
    variable_dim += factor->getDim(factor->find(key));
  }
  std::cout << graph_dim << ", " << variable_dim << "\n";

  // auto grad = linear_graph.gradientAtZero();
  // grad.print("", GTDKeyFormatter);

  // auto delta = linear_graph.optimize();
  // delta.print("", GTDKeyFormatter);

  // auto vs_key = SourceVolumeKey();
  // for (const auto& factor: linear_graph) {
  //   const auto& keys = factor->keys();
  //   if (std::find(keys.begin(), keys.end(), vs_key) != keys.end()) {
  //     factor->print("", GTDKeyFormatter);
  //   }
  // }

  // results_sim.print("", GTDKeyFormatter);

  // auto q_key = gtdynamics::internal::JointAngleKey(2, 100);
  // double q = results_sim.atDouble(q_key);
  // results_sim.update(q_key, q+0.1);

  // auto v_key = gtdynamics::internal::JointVelKey(2, 100);
  // double v = results_sim.atDouble(v_key);
  // results_sim.update(v_key, v+0.1);

  // auto key = PhaseKey(0);
  // double value = results_sim.atDouble(key);
  // results_sim.update(key, value +0.01);

  std::cout << "optimizing...\n";
  // DynamicsGraph::printValues(results_interp);
  // DynamicsGraph::printGraph(graph);
  // DynamicsGraph::saveGraphMultiSteps("../../visualization/factor_graph.json", graph, results_interp, jr.getRobot(JumpingRobot::Phase::Air), num_steps, false);
  std::cout << "init error: " << graph.error(results_sim) << "\n";
  gtsam::LevenbergMarquardtParams params;
  // gtsam::LevenbergMarquardtParams::SetCeresDefaults(&params);
  // params.setlambdaUpperBound(1e20);
  params.setLinearSolverType("MULTIFRONTAL_QR");
  params.setVerbosityLM("SUMMARY");
  params.setMaxIterations(30);

  std::cout << "graph size: " << graph.size() << "\tvalues size: " << results_sim.size() << "\n";

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, results_sim, params);
  // gtsam::DoglegOptimizer optimizer(graph, results_sim);
  gtsam::Values collocation_result = optimizer.optimize();
  DynamicsGraph::saveGraphMultiSteps("../../visualization/factor_graph.json", graph, collocation_result, jr.getRobot(JumpingRobot::Phase::Air), num_steps, false);
  std::cout << "result error: " << graph.error(collocation_result) << "\n";

  // gtsam::NonlinearConjugateGradientOptimizer gd_optimizer(graph, collocation_result, params);
  // gtsam::Values gd_result = gd_optimizer.optimize();
  // std::cout << "gd result error: " << graph.error(gd_result) << "\n";
  // std::cout << "phase 0 time: " << gd_result.atDouble(PhaseKey(0)) << "\n";
  // std::cout << "phase 1 time: " << gd_result.atDouble(PhaseKey(1)) << "\n";

  std::cout << "simulation final pose: " << collocation_result.at<gtsam::Pose3>(gtdynamics::internal::PoseKey(3,  num_steps)) << "\n";

  std::cout << "exporting trajectory...\n";
  jr.exportTrajectoryMultiPhase(num_steps, phases_sim, collocation_result);
  
}
