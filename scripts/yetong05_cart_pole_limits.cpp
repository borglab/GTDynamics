#include "gtdynamics/optimizer/EqualityConstraint.h"
#include "gtdynamics/utils/DynamicsSymbol.h"
#include <cmath>
#include <gtdynamics/imanifold/IEOptimizationBenchmark.h>
#include <gtdynamics/optimizer/InequalityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/imanifold/IECartPoleWithLimits.h>
#include <gtdynamics/imanifold/IEGDOptimizer.h>
#include <gtdynamics/imanifold/IELMOptimizer.h>
#include <gtdynamics/optimizer/BarrierOptimizer.h>

#include <iomanip>
#include <string>

using namespace gtsam;
using namespace gtdynamics;


int main(int argc, char **argv) {
  // Setting
  IECartPoleWithLimits cp;
  size_t num_steps = 20;
  double dt = 0.1;

  // Constraints
  EqualityConstraints e_constraints;
  InequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    auto e_constraints_k = cp.eConstraints(k);
    auto i_constraints_k = cp.iConstraints(k);
    e_constraints.add(e_constraints_k);
    i_constraints.add(i_constraints_k);
  }
  e_constraints.add(cp.initStateConstraints());

  // Costs
  NonlinearFactorGraph graph;
  NonlinearFactorGraph collocation_costs = cp.collocationCosts(num_steps, dt);
  NonlinearFactorGraph final_state_graph = cp.finalStateCosts(num_steps);
  NonlinearFactorGraph min_torque_costs = cp.minTorqueCosts(num_steps);
  graph.add(final_state_graph);
  graph.add(collocation_costs);
  graph.add(min_torque_costs);
  auto EvaluateCosts = [=](const Values &values) {
    std::cout << "collocation costs:\t" << collocation_costs.error(values)
              << "\n";
    std::cout << "final state costs:\t" << final_state_graph.error(values)
              << "\n";
    std::cout << "min torque costs:\t" << min_torque_costs.error(values)
              << "\n";
  };

  // Initial Values
  Values initial_values = cp.getInitValuesZero(num_steps);
  EvaluateCosts(initial_values);

  // Problem
  IEConsOptProblem problem(graph, e_constraints, i_constraints, initial_values);

  // Parameters
  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->ecm_params->basis_params->setFixVars();
  iecm_params->ecm_params->basis_key_func = cp.getBasisKeyFunc();
  iecm_params->retractor_creator = std::make_shared<UniversalIERetractorCreator>(std::make_shared<CartPoleWithLimitsRetractor>(cp));
  LevenbergMarquardtParams lm_params;
  lm_params.setMaxIterations(100);
  IELMParams ie_params;

  // optimize IELM
  auto lm_result = OptimizeIELM(problem, lm_params, ie_params, iecm_params);
  Values result_values = IEOptimizer::CollectManifoldValues(
      lm_result.second.back().state.manifolds);
  for (const auto &iter_details : lm_result.second) {
    IEOptimizer::PrintIterDetails(
        iter_details, num_steps, false, IECartPoleWithLimits::PrintValues,
        IECartPoleWithLimits::PrintDelta, gtdynamics::GTDKeyFormatter);
  }
  IECartPoleWithLimits::PrintValues(result_values, num_steps);
  EvaluateCosts(result_values);
  IECartPoleWithLimits::ExportValues(result_values, num_steps, "/Users/yetongzhang/packages/noboost/GTD_ineq/GTDynamics/data/ineq_cartpole_traj.csv");
  
  // Optimize Barrier
  BarrierParameters barrier_params;
  barrier_params.initial_mu = 1e0;
  barrier_params.num_iterations = 20;
  auto barrier_result = OptimizeBarrierMethod(problem, barrier_params);
  EvaluateCosts(barrier_result.second.rbegin()->values);
  barrier_result.first.exportFileWithMu("/Users/yetongzhang/packages/noboost/GTD_ineq/GTDynamics/data/ineq_cartpole_barrier.txt");
  barrier_result.first.printLatex(std::cout);
  lm_result.first.printLatex(std::cout);
  
  return 0;
}
