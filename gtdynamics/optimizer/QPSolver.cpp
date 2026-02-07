/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  QPSolver.cpp
 * @brief Quadratic programming solver implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/optimizer/QPSolver.h>

namespace gtdynamics {

using gtsam::Key;
using gtsam::JacobianFactor;

std::pair<Vector, Vector> SolveEQP(const Matrix &A_cost, const Vector &b_cost,
                                   const Matrix &A_constraint,
                                   const Vector &b_constraint) {
  gtsam::GaussianFactorGraph graph;
  // Key key = 0;
  // size_t cost_dim = A_cost.rows();
  // size_t constraint_dim = A_constraint.rows();
  // auto cost_noise = noiseModel::Unit::Create(cost_dim);
  // auto constrained_noise = noiseModel::Constrained::All(constraint_dim);

  // graph.emplace_shared<JacobianFactor>(key, A_cost, b_cost, cost_noise);
  // graph.emplace_shared<JacobianFactor>(key, A_constraint, b_constraint,
  // constrained_noise);

  // Vector x = graph.optimize().at(key);

  // // Solve for lambda.
  // GaussianFactorGraph lambda_graph;

  Key x_key = 0;
  Key lambda_key = 1;
  Matrix G = A_cost.transpose() * A_cost;
  Vector c = -b_cost.transpose() * A_cost;
  graph.emplace_shared<JacobianFactor>(
      x_key, G, lambda_key, -A_constraint.transpose(), -c,
      gtsam::noiseModel::Unit::Create(c.size()));
  graph.emplace_shared<JacobianFactor>(
      x_key, A_constraint, b_constraint,
      gtsam::noiseModel::Unit::Create(b_constraint.size()));

  auto result = graph.optimize();
  Vector x = result.at(x_key);
  Vector lambda = result.at(lambda_key);
  return std::make_pair(x, lambda);
}

}  // namespace gtdynamics
