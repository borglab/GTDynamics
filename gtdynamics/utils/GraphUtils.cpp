#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/base/serialization.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphSolver.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

GTSAM_VALUE_EXPORT(double)
GTSAM_VALUE_EXPORT(gtsam::Point3)
GTSAM_VALUE_EXPORT(gtsam::Rot3)
GTSAM_VALUE_EXPORT(gtsam::Pose3)
GTSAM_VALUE_EXPORT(gtsam::Vector6)

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
bool CheckFeasible(const NonlinearFactorGraph &graph, const Values &values,
                   const std::string s, const double feasible_threshold,
                   bool print_details, const KeyFormatter &key_formatter) {

  if (graph.error(values) > feasible_threshold) {
    std::cout << s << "fail: " << graph.error(values) << "\n";
    if (print_details) {
      PrintGraphWithError(graph, values, feasible_threshold, key_formatter);
    }
    return false;
  }
  return true;
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

/* ************************************************************************* */

/* ************************************************************************* */
void ExportValuesToFile(const Values &values, const std::string &file_path) {
  serializeToBinaryFile(values, file_path);
}

/* ************************************************************************* */
Values LoadValuesFromFile(const std::string &file_path) {
  Values values;
  deserializeFromBinaryFile(file_path, values);
  return values;
}

/* ************************************************************************* */
double ComputeErrorNorm(const double &graph_error, const double &sigma) {
  return sqrt(graph_error * 2) * sigma;
}

/* ************************************************************************* */
void IndexSetMapTranslator::insert(size_t index, Key key, size_t index_in_key) {
  decoder.insert({index, {key, index_in_key}});
  encoder.insert({{key, index_in_key}, index});
}

/* ************************************************************************* */
IndexSet
IndexSetMapTranslator::encodeIndices(const IndexSetMap &index_set_map) const {
  IndexSet indices;
  for (const auto &[key, man_indices] : index_set_map) {
    for (const auto &constraint_idx : man_indices) {
      indices.insert(encoder.at({key, constraint_idx}));
    }
  }
  return indices;
}

/* ************************************************************************* */
IndexSetMap
IndexSetMapTranslator::decodeIndices(const IndexSet &indices) const {
  IndexSetMap index_set_map;
  for (const auto &index : indices) {
    const auto [key, constraint_idx] = decoder.at(index);
    index_set_map.addIndex(key, constraint_idx);
  }
  return index_set_map;
}

/* ************************************************************************* */
GaussianFactorGraph ScaledBiasedFactors(const GaussianFactorGraph &graph_in,
                                        double mu, double b_scale) {
  GaussianFactorGraph graph;
  double sigma = 1 / sqrt(mu);
  for (const auto &factor : graph_in) {
    auto [A, b] = factor->jacobian();
    std::map<Key, Matrix> terms;
    size_t start_col = 0;
    for (auto it = factor->begin(); it != factor->end(); it++) {
      size_t num_cols = factor->getDim(it);
      terms.insert({*it, A.middleCols(start_col, num_cols)});
      start_col += num_cols;
    }
    b *= b_scale;
    graph.emplace_shared<JacobianFactor>(
        terms, b, noiseModel::Isotropic::Sigma(b.size(), sigma));
  }
  return graph;
}

/* ************************************************************************* */
JacobianFactor::shared_ptr
ZerobFactor(const JacobianFactor::shared_ptr factor) {
  // auto [A, b] = factor->jacobian();
  std::map<Key, Matrix> terms;
  for (auto it = factor->begin(); it != factor->end(); it++) {
    terms.insert({*it, factor->getA(it)});
  }
  // b = Vector::Zero(b.size());
  // auto A = factor->getA();
  auto b = Vector::Zero(factor->getb().size());
  if (factor->get_model()) {
    return std::make_shared<JacobianFactor>(terms, b, factor->get_model());
  } else {
    return std::make_shared<JacobianFactor>(terms, b,
                                            noiseModel::Unit::Create(b.size()));
  }
}

} // namespace gtsam
