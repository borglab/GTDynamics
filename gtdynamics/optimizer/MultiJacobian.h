/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  MultiJacobian.h
 * @brief Jacobian w.r.t. multiple variables.
 * @author: Yetong Zhang
 */

#pragma once

#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace gtsam {

/** Class that stores the jacobian of w.r.t. multiple variables. e.g. x = f(m,
 * n, o). The class will store the jacobians of dx/dm, dx/dn, dx/do. */
class MultiJacobian : public std::map<Key, Matrix> {
 public:
  using Base = std::map<Key, Matrix>;

  /// Default constructor.
  MultiJacobian() : Base() {}

  /// Constructor from jacobian to a single variable.
  MultiJacobian(const Key& key, const Matrix& matrix);

  /// Constructor from jacobian (as identity matrix) to a single variable.
  static MultiJacobian Identity(const Key& key, const size_t& dim);

  /** Add jacobian to a variable. If jacobian to that variable already
   * exists, will add the matrix to the existing jacobian. */
  void addJacobian(const Key& key, const Matrix& matrix);

  /// Insert jacobian by chain-rule: given jacobian of a parent variable, and
  /// the relative jacobian to parent variable, will add the jacobians by chain
  /// rule, e.g., add the jacobians H_relative * parent_jacobian.
  void addJacobianChainRule(const Matrix& H_relative,
                            const MultiJacobian parent_jacobian);

  /// Customizable print function.
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /// Check equality.
  bool equals(const MultiJacobian& other, double tol = 1e-8) const;
};

/** jacobian of multiple variables w.r.t. multiple variables. */
typedef std::map<Key, MultiJacobian> MultiJacobians;

/** Given a bayes net, compute the jacobians of all variables w.r.t. basis
 * variables.
 * @param bn bayes net
 * @param basis_keys basis variables
 * @param var_dim dimension of variables
 * @param jacobians output, jacobians of all variables
 */
void ComputeBayesNetJacobian(const GaussianBayesNet& bn,
                             const KeyVector& basis_keys,
                             const std::map<Key, size_t>& var_dim,
                             MultiJacobians& jacobians);

}  // namespace gtsam
