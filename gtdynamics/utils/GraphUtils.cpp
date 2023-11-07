#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>

#include <iostream>

namespace gtsam {

/* ************************************************************************* */
void PrintGraphWithError(const NonlinearFactorGraph &graph,
                         const Values &values, double error_threshold,
                         const KeyFormatter &key_formatter) {
  std::cout << "total error: " << graph.error(values) << std::endl;
  for (const auto &factor : graph) {
    double error = factor->error(values);
    if (error > error_threshold) {
      std::cout << "====================== factor ==========================\n";
      factor->print("", key_formatter);
      std::cout << "error: " << error << std::endl;
    }
  }
}

/* ************************************************************************* */
void CheckFeasible(const NonlinearFactorGraph &graph, const Values &values,
                   const std::string s, const double feasible_threshold,
                   bool print_details, const KeyFormatter &key_formatter) {

  if (graph.error(values) > feasible_threshold) {
    std::cout << s << "fail: " << graph.error(values) << "\n";
    if (print_details) {
      PrintGraphWithError(graph, values, feasible_threshold, key_formatter);
    }
  }
}

/* ************************************************************************* */
VectorValues SqrtHessianDiagonal(const GaussianFactorGraph &graph,
                                 const LevenbergMarquardtParams &params) {
  VectorValues sqrt_hessian_diagonal;
  if (params.diagonalDamping) {
    sqrt_hessian_diagonal = graph.hessianDiagonal();
    for (auto &[key, value] : sqrt_hessian_diagonal) {
      value = value.cwiseMax(params.minDiagonal)
                  .cwiseMin(params.maxDiagonal)
                  .cwiseSqrt();
    }
  }
  return sqrt_hessian_diagonal;
}


/* ************************************************************************* */
VectorValues SolveLinear(const GaussianFactorGraph &gfg,
                         const NonlinearOptimizerParams &params) {
  // solution of linear solver is an update to the linearization point
  VectorValues delta;

  // Check which solver we are using
  if (params.isMultifrontal()) {
    // Multifrontal QR or Cholesky (decided by params.getEliminationFunction())
    if (params.ordering)
      delta = gfg.optimize(*params.ordering, params.getEliminationFunction());
    else
      delta = gfg.optimize(params.getEliminationFunction());
  } else if (params.isSequential()) {
    // Sequential QR or Cholesky (decided by params.getEliminationFunction())
    if (params.ordering)
      delta = gfg.eliminateSequential(*params.ordering,
                                      params.getEliminationFunction())
                  ->optimize();
    else
      delta = gfg.eliminateSequential(params.orderingType,
                                      params.getEliminationFunction())
                  ->optimize();
  } else if (params.isIterative()) {
    // Conjugate Gradient -> needs params.iterativeParams
    if (!params.iterativeParams)
      throw std::runtime_error(
          "NonlinearOptimizer::solve: cg parameter has to be assigned ...");

    if (auto pcg = std::dynamic_pointer_cast<PCGSolverParameters>(
            params.iterativeParams)) {
      delta = PCGSolver(*pcg).optimize(gfg);
    } else if (auto spcg = std::dynamic_pointer_cast<SubgraphSolverParameters>(
                   params.iterativeParams)) {
      if (!params.ordering)
        throw std::runtime_error("SubgraphSolver needs an ordering");
      delta = SubgraphSolver(gfg, *spcg, *params.ordering).optimize();
    } else {
      throw std::runtime_error(
          "NonlinearOptimizer::solve: special cg parameter type is not handled "
          "in LM solver ...");
    }
  } else {
    throw std::runtime_error(
        "NonlinearOptimizer::solve: Optimization parameter is invalid");
  }

  // return update
  return delta;
}

} // namespace gtsam
