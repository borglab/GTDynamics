/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  main_cartpole.cpp
 * @brief Dynamic trajectory planning problem of rotating up a cart-pole.
 * @author Yetong Zhang
 */

#include "CartPoleUtils.h"

#include <gtdynamics/config.h>
#include <gtdynamics/constrained_optimizer/ConstrainedOptBenchmark.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace gtsam;
using namespace gtdynamics;

namespace {
constexpr double kConstraintUnitScale = 1e-3;
constexpr double kHorizonTime = 2.0;
constexpr double kDt = 1.0 / 100.0;

struct CartPoleArgs {
  ParsedBenchmarkCli benchmark_cli;
};

void PrintUsage(const char* program_name) {
  BenchmarkCliDefaults defaults;
  defaults.id = "cartpole";
  defaults.enableNumSteps = true;
  defaults.defaultNumSteps = static_cast<size_t>(std::ceil(kHorizonTime / kDt));
  defaults.defaultMethods = {BenchmarkMethod::CM_I};
  PrintBenchmarkUsage(std::cout, program_name, defaults);
}

CartPoleArgs ParseArgs(int argc, char** argv) {
  BenchmarkCliDefaults defaults;
  defaults.id = "cartpole";
  defaults.enableNumSteps = true;
  defaults.defaultNumSteps = static_cast<size_t>(std::ceil(kHorizonTime / kDt));
  defaults.defaultMethods = {BenchmarkMethod::CM_I};
  return CartPoleArgs{ParseBenchmarkCli(argc, argv, defaults)};
}
}  // namespace

/** Cart-pole dynamic planning benchmark with shared constrained optimizer runner. */
void dynamic_planning(size_t numSteps, const ConstrainedOptBenchmark::Options& runOptions) {
  CartPole cartpole;

  NonlinearFactorGraph dynamicConstraintsGraph =
      cartpole.getDynamicsGraph(numSteps);
  NonlinearFactorGraph unactuatedGraph = cartpole.getUnactuatedGraph(numSteps);
  NonlinearFactorGraph initStateGraph = cartpole.initStateGraph();

  NonlinearFactorGraph finalStateGraph = cartpole.finalStateGraph(numSteps);
  NonlinearFactorGraph collocationCosts = cartpole.getCollocation(numSteps, kDt);
  NonlinearFactorGraph minTorqueCosts = cartpole.minTorqueCosts(numSteps);

  NonlinearFactorGraph constraintsGraph;
  constraintsGraph.add(dynamicConstraintsGraph);
  constraintsGraph.add(unactuatedGraph);
  constraintsGraph.add(initStateGraph);

  NonlinearFactorGraph costs;
  costs.add(finalStateGraph);
  costs.add(collocationCosts);
  costs.add(minTorqueCosts);

  auto evaluateCosts = [=](const Values& values) {
    std::cout << "collocation costs:\t" << collocationCosts.error(values) << "\n";
    std::cout << "final state costs:\t" << finalStateGraph.error(values) << "\n";
    std::cout << "min torque costs:\t" << minTorqueCosts.error(values) << "\n";
  };

  auto initValues = cartpole.getInitValues(numSteps, "zero");
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraintsGraph);

  evaluateCosts(initValues);
  cartpole.exportTrajectory(
      initValues, numSteps, kDt,
      std::string(kDataPath) + runOptions.id + "_init_traj.csv");

  LevenbergMarquardtParams lmParams;
  lmParams.setlambdaUpperBound(1e10);
  lmParams.setMaxIterations(30);

  ConstrainedOptBenchmark runner(runOptions);
  runner.setProblemFactory(
      [=]() { return EConsOptProblem(costs, constraints, initValues); });
  runner.setOuterLmBaseParams(lmParams);
  runner.setMoptFactory([&](BenchmarkMethod) {
    auto moptParams =
        ConstrainedOptBenchmark::DefaultMoptParamsSV(cartpole.getBasisKeyFunc(true));
    auto retractorParams = moptParams.cc_params->retractor_creator->params();
    retractorParams->check_feasible = true;
    retractorParams->lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    return moptParams;
  });
  runner.setResultCallback([&](BenchmarkMethod method, const Values& result) {
    evaluateCosts(result);
    cartpole.exportTrajectory(
        result, numSteps, kDt,
        BenchmarkMethodDataPath(runOptions, method, "_traj.csv"));
  });

  std::ostringstream latexOs;
  runner.run(latexOs);
  std::cout << latexOs.str();
}

int main(int argc, char** argv) {
  try {
    const CartPoleArgs args = ParseArgs(argc, argv);
    if (!args.benchmark_cli.unknownArgs.empty()) {
      throw std::invalid_argument("Unknown option: " +
                                  args.benchmark_cli.unknownArgs.front());
    }

    auto runOptions = args.benchmark_cli.runOptions;
    runOptions.constraintUnitScale = kConstraintUnitScale;

    std::cout << "Using num_steps=" << args.benchmark_cli.numSteps << "\n";
    dynamic_planning(args.benchmark_cli.numSteps, runOptions);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
