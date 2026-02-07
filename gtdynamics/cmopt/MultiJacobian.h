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

#include <unordered_map>

namespace gtsam {

/** Class that stores the jacobian of w.r.t. multiple variables. e.g. x = f(m,
 * n, o). The class will store the jacobians of dx/dm, dx/dn, dx/do. */
class MultiJacobian : public std::unordered_map<Key, Matrix> {
 protected:
  int dim_ = -1;
  
 public:
  using Base = std::unordered_map<Key, Matrix>;

  /// Default constructor.
  MultiJacobian() : Base() {}

  /// Constructor from jacobian to a single variable.
  MultiJacobian(const Key& key, const Matrix& matrix);

  /// Constructor from jacobian (as identity matrix) to a single variable.
  static MultiJacobian Identity(const Key& key, const size_t& dim);

  /// Vertical stack of two jacobians.
  static MultiJacobian VerticalStack(const MultiJacobian &jac1,
                                     const MultiJacobian &jac2);

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

  /// Return the i-th row of the jacobian as vector values.
  VectorValues row(const size_t i) const;

  KeySet keys() const;

  size_t numRows() const;

  void operator += (const MultiJacobian& other);
};

MultiJacobian operator*(const Matrix& m, const MultiJacobian& jac);

MultiJacobian operator+(const MultiJacobian& jac1, const MultiJacobian& jac2);

/** jacobian of multiple variables w.r.t. multiple variables. */
typedef std::unordered_map<Key, MultiJacobian> MultiJacobians;

MultiJacobians JacobiansMultiply(const MultiJacobians& multi_jac1, const MultiJacobians& multi_jac2);

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
