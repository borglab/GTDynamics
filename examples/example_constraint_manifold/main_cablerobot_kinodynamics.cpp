/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
/**
 * @file main_cablerobot_kinodynamics.cpp
 * @brief Cable robot kinodynamics manifold benchmark revived from PR #363.
 * @author Gerry Chen
 * @author Yetong Zhang
 */
#include <gtdynamics/cablerobot/utils/CableRobotUtils.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "CableRobotBenchmark.h"

using namespace gtdynamics;
using namespace gtsam;

/** Run the source trajectory problem with the selected benchmark methods. */
void trajectoryOptimization(size_t numSteps,
                            ConstrainedOptBenchmark::Options options) {
  if (numSteps < 2 ||
      numSteps > static_cast<size_t>(std::numeric_limits<int>::max()))
    throw std::invalid_argument("--num-steps must be between 2 and INT_MAX");
  options = cableRobotBenchmarkOptions(options);
  CableRobotParams params;
  params.timeStep = 0.01;
  params.torqueWeight = 1;
  params.waypointWeight = 100;
  params.initialPose = Pose3(Rot3(), Point3(1.5, 1.0, 0));
  const CableRobot robot(params);

  std::map<int, Pose3> waypoints;
  for (size_t stepIndex = 0; stepIndex < numSteps; ++stepIndex) {
    const double fraction = static_cast<double>(stepIndex) / (numSteps - 1);
    waypoints.emplace(
        stepIndex,
        Pose3(Rot3(), Point3(1.5 + 0.2 * fraction, 1.0 + 0.1 * fraction, 0)));
  }
  const auto constraints = NonlinearEqualityConstraints::FromCostGraph(
      robot.constraintFactors(numSteps));
  const auto costGroups = robot.costFactors(numSteps, waypoints);
  NonlinearFactorGraph costs;
  for (const auto &[name, group] : costGroups) costs.add(group);
  const auto initialValues = robot.initialValues(numSteps);
  auto reportCosts = [&](const Values &values) {
    for (const auto &[name, group] : costGroups) {
      const double error = group.error(values);
      if (!std::isfinite(error))
        throw std::runtime_error("Non-finite " + name + " cost");
      std::cout << name << ": " << error << '\n';
    }
  };
  reportCosts(initialValues);
  // Use the common data directory and portable filenames, including the initial
  // trajectory. The method token only supplies the conventional filename
  // prefix.
  robot.exportTrajectory(
      initialValues, numSteps,
      ConstrainedOptBenchmark::MethodDataPath(
          options, ConstrainedOptBenchmark::Method::SOFT, "_initial_traj.csv"));

  ConstrainedOptBenchmark runner(options);
  runner.setProblemFactory(
      [=]() { return EConsOptProblem(costs, constraints, initialValues); });
  runner.setOuterLmConfig(configureCableRobotLm);
  runner.setMoptFactory([](ConstrainedOptBenchmark::Method) {
    return cableRobotManifoldParams();
  });
  runner.setPenaltyConfig(configureCableRobotPenalty);
  runner.setAugmentedLagrangianConfig(configureCableRobotAugmentedLagrangian);
  runner.setResultCallback([&](ConstrainedOptBenchmark::Method method,
                               const Values &result) {
    reportCosts(result);
    robot.exportTrajectory(
        result, numSteps,
        ConstrainedOptBenchmark::MethodDataPath(options, method, "_traj.csv"));
  });
  std::ostringstream latexOutput;
  runner.run(latexOutput);
  std::cout << latexOutput.str();
}

int main(int argc, char **argv) {
  ConstrainedOptBenchmark::CliDefaults defaults;
  defaults.id = "cable_robot_kinodynamics";
  defaults.enableNumSteps = true;
  defaults.defaultNumSteps = 1000;
  try {
    const auto parsed = ConstrainedOptBenchmark::ParseCli(argc, argv, defaults);
    if (!parsed.unknownArgs.empty())
      throw std::invalid_argument("Unknown option: " +
                                  parsed.unknownArgs.front());
    trajectoryOptimization(parsed.numSteps, parsed.runOptions);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "Error: " << error.what() << '\n';
    ConstrainedOptBenchmark::PrintUsage(std::cerr, argv[0], defaults);
    return 1;
  }
}
