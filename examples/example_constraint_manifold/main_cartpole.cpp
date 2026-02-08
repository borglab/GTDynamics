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
  defaults.benchmark_id = "cartpole";
  defaults.enable_num_steps = true;
  defaults.default_num_steps = static_cast<size_t>(std::ceil(kHorizonTime / kDt));
  defaults.default_methods = {BenchmarkMethod::CM_I};
  PrintBenchmarkUsage(std::cout, program_name, defaults);
}

CartPoleArgs ParseArgs(int argc, char** argv) {
  BenchmarkCliDefaults defaults;
  defaults.benchmark_id = "cartpole";
  defaults.enable_num_steps = true;
  defaults.default_num_steps = static_cast<size_t>(std::ceil(kHorizonTime / kDt));
  defaults.default_methods = {BenchmarkMethod::CM_I};
  return CartPoleArgs{ParseBenchmarkCli(argc, argv, defaults)};
}
}  // namespace

/** Cart-pole dynamic planning benchmark with shared constrained optimizer runner. */
void dynamic_planning(size_t num_steps, const BenchmarkRunOptions& run_options) {
  CartPole cartpole;

  NonlinearFactorGraph dynamic_constraints_graph =
      cartpole.getDynamicsGraph(num_steps);
  NonlinearFactorGraph unactuated_graph =
      cartpole.getUnactuatedGraph(num_steps);
  NonlinearFactorGraph init_state_graph = cartpole.initStateGraph();

  NonlinearFactorGraph final_state_graph =
      cartpole.finalStateGraph(num_steps);
  NonlinearFactorGraph collocation_costs =
      cartpole.getCollocation(num_steps, kDt);
  NonlinearFactorGraph min_torque_costs =
      cartpole.minTorqueCosts(num_steps);

  NonlinearFactorGraph constraints_graph;
  constraints_graph.add(dynamic_constraints_graph);
  constraints_graph.add(unactuated_graph);
  constraints_graph.add(init_state_graph);

  NonlinearFactorGraph costs;
  costs.add(final_state_graph);
  costs.add(collocation_costs);
  costs.add(min_torque_costs);

  auto evaluate_costs = [=](const Values& values) {
    std::cout << "collocation costs:\t" << collocation_costs.error(values) << "\n";
    std::cout << "final state costs:\t" << final_state_graph.error(values) << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values) << "\n";
  };

  auto init_values = cartpole.getInitValues(num_steps, "zero");
  auto constraints =
      gtsam::NonlinearEqualityConstraints::FromCostGraph(constraints_graph);

  evaluate_costs(init_values);
  cartpole.exportTrajectory(
      init_values, num_steps, kDt,
      std::string(kDataPath) + run_options.benchmark_id + "_init_traj.csv");

  LevenbergMarquardtParams lm_params;
  lm_params.setlambdaUpperBound(1e10);
  lm_params.setMaxIterations(30);

  ConstrainedOptBenchmarkRunner runner(run_options);
  runner.setProblemFactory(
      [=]() { return EConsOptProblem(costs, constraints, init_values); });
  runner.setOuterLmBaseParams(lm_params);
  runner.setMoptFactory([&](BenchmarkMethod) {
    auto mopt_params = DefaultMoptParamsSV(cartpole.getBasisKeyFunc(true));
    auto retract_params = mopt_params.cc_params->retractor_creator->params();
    retract_params->check_feasible = true;
    retract_params->lm_params.linearSolverType =
        gtsam::NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
    return mopt_params;
  });
  runner.setResultCallback([&](BenchmarkMethod method, const Values& result) {
    evaluate_costs(result);
    cartpole.exportTrajectory(
        result, num_steps, kDt,
        BenchmarkMethodDataPath(run_options, method, "_traj.csv"));
  });

  std::ostringstream latex_os;
  runner.run(latex_os);
  std::cout << latex_os.str();
}

int main(int argc, char** argv) {
  try {
    const CartPoleArgs args = ParseArgs(argc, argv);
    if (!args.benchmark_cli.unknown_args.empty()) {
      throw std::invalid_argument("Unknown option: " +
                                  args.benchmark_cli.unknown_args.front());
    }

    auto run_options = args.benchmark_cli.run_options;
    run_options.constraint_unit_scale = kConstraintUnitScale;

    std::cout << "Using num_steps=" << args.benchmark_cli.num_steps << "\n";
    dynamic_planning(args.benchmark_cli.num_steps, run_options);
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
