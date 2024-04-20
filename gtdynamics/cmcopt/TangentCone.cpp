/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Tangent Cone.cpp
 * @brief Tangent cone implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/cmcopt/TangentCone.h>
#include <gtdynamics/optimizer/ConvexIQPSolver.h>

using namespace gtdynamics;

namespace gtsam {

/* ************************************************************************* */
std::pair<IndexSet, Vector> TangentCone::project(const Vector &xi) const {
  if (constraints_.size() == 0) {
    return {IndexSet(), xi};
  }

  GaussianFactorGraph graph;
  Key x_key = 1;
  size_t dim = xi.size();
  auto factor = std::make_shared<JacobianFactor>(
      x_key, Matrix::Identity(dim, dim), xi, noiseModel::Unit::Create(dim));
  graph.push_back(factor);

  IndexSet init_active_indices;
  VectorValues init_values;
  init_values.insert(x_key, Vector::Zero(dim));

  auto [values, active_indices, num_solves, solve_successful] =
      SolveConvexIQP(graph, constraints_, init_active_indices, init_values);
  if (!solve_successful) {
    std::cout << "solve failed in project T-cone.\n";
    return {init_active_indices, init_values.at(x_key)};
  }
  return {active_indices, values.at(x_key)};
}

} // namespace gtsam
