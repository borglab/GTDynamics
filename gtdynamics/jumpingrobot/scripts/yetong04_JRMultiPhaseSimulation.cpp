/**
 * @file  testJumpingRobot.cpp
 * @brief test trajectory optimization for jumping robot
 * @Author: Yetong Zhang
 */


#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <iostream>
#include <fstream>

#include "gtdynamics/jumpingrobot/factors/JumpingRobot.h"

using namespace std;
using namespace gtdynamics;
using namespace gtsam;

int main()
{
  // double theta = 45.0 / 180.0 * M_PI;
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

  // auto robot = jr.getRobot(JumpingRobot::Phase::Ground);
  // robot.printRobot();

  int num_steps = 200;
  double dt = 2e-3;
  double source_pressure = 65.0 * 6.89476 + 101.325;
  Vector open_times = (Vector(4) << 0, 0, 0, 0).finished();
  Vector close_times = (Vector(4) << 0.098, 0.098, 0.098, 0.098).finished();

  // std::cout << "simulate\n";
  JRSimulator jr_simulator(jr);
  Values results = jr_simulator.simulate(num_steps, dt, source_pressure, open_times, close_times);
  vector<JumpingRobot::Phase> phases = jr_simulator.phases();

  results.print("", GTDKeyFormatter);

  std::cout << "export trajectory\n";
  jr.exportTrajectoryMultiPhase(num_steps, phases, results);
  jr.exportData(num_steps, results);
  // std::cout << "interpolate trajectory\n";
  // std::vector<JumpingRobot::Phase> phases_out{JumpingRobot::Phase::Ground, JumpingRobot::Phase::Air};
  // std::vector<int> phase_steps_out{20, 20};
  // Values results_interp = jr.linearInterpolation(dt, phases, results, phases_out, phase_steps_out);

  // vector<JumpingRobot::Phase> phases_new;
  // for (int phase_idx = 0; phase_idx < phases_out.size(); phase_idx++)
  // {
  //   JumpingRobot::Phase phase = phases_out[phase_idx];
  //   for (int iter = 0; iter < phase_steps_out[phase_idx]; iter++)
  //   {
  //     phases_new.emplace_back(phase);
  //   }
  // }
  // phases_new.emplace_back(JumpingRobot::Phase::Air);

  // std::cout << "export trajectory\n";
  // jr.exportTrajectoryMultiPhase(40, dt, phases_new, results_interp);
}
