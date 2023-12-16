/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ConvexIQPSolver.cpp
 * @brief Convex IQP solver implementations.
 * @author: Yetong Zhang
 */

#include <gtdynamics/optimizer/ConvexIQPSolver.h>

namespace gtsam {

/* ************************************************************************* */
GaussianFactorGraph
ConstraintGraph(const LinearInequalityConstraints &constraints,
                const IndexSet &active_indices) {
  GaussianFactorGraph graph;
  for (const auto &constraint_idx : active_indices) {
    graph.push_back(constraints.at(constraint_idx)->createConstrainedFactor());
  }
  return graph;
}

/* ************************************************************************* */
VectorValues SolveEQP(const GaussianFactorGraph &cost,
                      const GaussianFactorGraph &constriant_graph) {
  GaussianFactorGraph graph = cost;
  for (const auto &factor : constriant_graph) {
    graph.push_back(factor);
  }
  VectorValues solution = graph.optimize();
  return solution;
}

/* ************************************************************************* */
std::map<size_t, Vector>
ComputeLagrangianMultipliers(const GaussianFactorGraph &cost,
                             const LinearInequalityConstraints &constraints,
                             const VectorValues &x,
                             const IndexSet &active_indices) {
  std::map<size_t, Vector> lambdas;

  auto grad_x = cost.gradient(x);
  std::map<size_t, size_t> start_row_map;
  size_t total_rows = 0;
  for (const auto &idx : active_indices) {
    start_row_map.insert({idx, total_rows});
    total_rows += constraints.at(idx)->dim();
  }
  std::map<Key, size_t> start_col_map;
  size_t total_cols = 0;
  for (const auto &[key, v] : x) {
    start_col_map.insert({key, total_cols});
    total_cols += v.size();
  }

  Matrix jac_mat = Matrix::Zero(total_rows, total_cols);
  for (const auto &idx : active_indices) {
    const auto &constraint = constraints.at(idx);
    size_t start_row = start_row_map.at(idx);
    size_t block_rows = constraint->dim();
    auto multi_jacobian = constraint->jacobian();
    for (const auto &[key, mat] : multi_jacobian) {
      size_t start_col = start_col_map.at(key);
      size_t block_cols = x.at(key).size();
      jac_mat.block(start_row, start_col, block_rows, block_cols) = mat;
    }
  }
  Vector b = Vector::Zero(total_cols);
  for (const auto &[key, grad] : grad_x) {
    size_t start_row = start_col_map.at(key);
    b.middleRows(start_row, grad.size()) = grad;
  }

  Vector lambda = jac_mat.transpose().colPivHouseholderQr().solve(b);
  for (const auto &idx : active_indices) {
    lambdas.insert({idx, lambda.middleRows(start_row_map.at(idx),
                                           constraints.at(idx)->dim())});
  }

  return lambdas;
}

/* ************************************************************************* */
double VectorMax(const Vector &vec) {
  double max_val = vec(0);
  for (size_t i = 1; i < vec.size(); i++) {
    max_val = std::max(max_val, vec(i));
  }
  return max_val;
}

/* ************************************************************************* */
std::pair<size_t, double> GetMinimum(const std::map<size_t, Vector> &vals) {
  if (vals.size() == 0) {
    return {0, 0};
  }
  size_t min_idx = vals.begin()->first;
  double min_val = VectorMax(vals.begin()->second);

  for (const auto &[idx, vec] : vals) {
    double val = VectorMax(vec);
    if (val < min_val) {
      min_val = val;
      min_idx = idx;
    }
  }
  return {min_idx, min_val};
}

/* ************************************************************************* */
std::pair<double, size_t>
ClipUpdate(const VectorValues &x, const VectorValues &p,
           const LinearInequalityConstraints &constraints,
           const IndexSet &active_indices) {
  double alpha = 1;
  size_t blocking_idx = 0;

  for (size_t idx = 0; idx < constraints.size(); idx++) {
    if (active_indices.exists(idx)) {
      continue;
    }
    const auto &constraint = constraints.at(idx);
    auto jacobian = constraint->jacobian();

    for (size_t row_idx = 0; row_idx < constraint->dim(); row_idx++) {
      VectorValues A_i = jacobian.row(row_idx);
      double Aip = A_i.dot(p);
      if (Aip < 0) {
        double clip_rate = -A_i.dot(x) / A_i.dot(p);
        if (clip_rate < alpha) {
          alpha = clip_rate;
          blocking_idx = idx;
        }
      }
    }
  }

  return {alpha, blocking_idx};
}

/* ************************************************************************* */
std::pair<VectorValues, IndexSet>
SolveConvexIQP(const GaussianFactorGraph &cost,
               const LinearInequalityConstraints &constraints,
               const IndexSet &init_active_indices,
               const VectorValues &init_values) {
  VectorValues x = init_values;
  IndexSet active_indices = init_active_indices;

  while (true) {
    // compute update
    auto constraint_graph = ConstraintGraph(constraints, active_indices);
    VectorValues x_new = SolveEQP(cost, constraint_graph);
    VectorValues p = x_new - x;

    if (p.norm() < 1e-5) {
      // no update is made
      auto lambdas = ComputeLagrangianMultipliers(cost, constraints, x_new,
                                                  active_indices);
      auto [min_idx, min_lambda] = GetMinimum(lambdas);
      if (min_lambda >= 0) {
        // all constraints are tight
        return {x_new, active_indices};
      } else {
        // can relieve constriant
        active_indices.erase(min_idx);
      }
    } else {
      // has a non-zero update
      auto [alpha, blocking_idx] =
          ClipUpdate(x, p, constraints, active_indices);
      if (alpha < 1) {
        // new constriants blocking
        active_indices.insert(blocking_idx);
        x = x + alpha * p;
      } else {
        // no constraints blocking
        x = x_new;
      }
    }
  }
}

} // namespace gtsam