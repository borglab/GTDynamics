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

#include <map>
#include <unordered_map>
#include <vector>

namespace gtdynamics {

using gtsam::DefaultKeyFormatter;
using gtsam::GaussianBayesNet;
using gtsam::Key;
using gtsam::KeyFormatter;
using gtsam::KeySet;
using gtsam::KeyVector;
using gtsam::Matrix;
using gtsam::VectorValues;

/**
 * Jacobian map with respect to multiple variables.
 *
 * For an output variable block `x = f(m, n, o)`, this stores partial Jacobians
 * `{dx/dm, dx/dn, dx/do}` keyed by variable key and supports chain-rule
 * composition used by elimination-based tangent basis construction.
 *
 * @see README.md#tangent-basis
 */
class MultiJacobian : public std::unordered_map<Key, Matrix> {
 protected:
  int dim_ = -1;

 public:
  using Base = std::unordered_map<Key, Matrix>;

  /// Default constructor.
  MultiJacobian() : Base() {}

  /**
   * Constructor from a Jacobian to a single variable.
   * @param key Variable key.
   * @param matrix Jacobian matrix for the variable.
   */
  MultiJacobian(const Key& key, const Matrix& matrix);

  /**
   * Construct identity Jacobian for a single variable.
   * @param key Variable key.
   * @param dim Variable dimension.
   * @return Identity multi-variable Jacobian.
   */
  static MultiJacobian Identity(const Key& key, const size_t& dim);

  /**
   * Vertically stack two multi-variable Jacobians.
   * @param jac1 First Jacobian block.
   * @param jac2 Second Jacobian block.
   * @return Vertically stacked Jacobian.
   */
  static MultiJacobian VerticalStack(const MultiJacobian& jac1,
                                     const MultiJacobian& jac2);

  /**
   * Add Jacobian for a variable.
   * @param key Variable key.
   * @param matrix Jacobian block to add.
   * @note If an entry already exists for `key`, matrices are summed.
   */
  void addJacobian(const Key& key, const Matrix& matrix);

  /**
   * Add Jacobian terms by chain rule.
   * @param H_relative Relative Jacobian to the parent variable.
   * @param parent_jacobian Parent Jacobian map.
   */
  void addJacobianChainRule(const Matrix& H_relative,
                            const MultiJacobian parent_jacobian);

  /**
   * Print Jacobian entries.
   * @param s Prefix string.
   * @param keyFormatter Key formatter.
   */
  void print(const std::string& s = "",
             const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

  /**
   * Compare two multi-variable Jacobians.
   * @param other Jacobian to compare with.
   * @param tol Absolute comparison tolerance.
   * @return True if equal within tolerance.
   */
  bool equals(const MultiJacobian& other, double tol = 1e-8) const;

  /// Return the i-th row of the jacobian as vector values.
  VectorValues row(const size_t i) const;

  KeySet keys() const;

  size_t numRows() const;

  void operator+=(const MultiJacobian& other);
};

/**
 * Left-multiply each Jacobian block by a matrix.
 * @param m Left multiplication matrix.
 * @param jac Input Jacobian.
 * @return Resulting Jacobian.
 */
MultiJacobian operator*(const Matrix& m, const MultiJacobian& jac);

/**
 * Add two multi-variable Jacobians.
 * @param jac1 First Jacobian.
 * @param jac2 Second Jacobian.
 * @return Sum Jacobian.
 */
MultiJacobian operator+(const MultiJacobian& jac1, const MultiJacobian& jac2);

/** jacobian of multiple variables w.r.t. multiple variables. */
typedef std::unordered_map<Key, MultiJacobian> MultiJacobians;

/**
 * Compose two Jacobian maps.
 * @param multi_jac1 Outer Jacobian map.
 * @param multi_jac2 Inner Jacobian map.
 * @return Composed Jacobian map.
 */
MultiJacobians JacobiansMultiply(const MultiJacobians& multi_jac1,
                                 const MultiJacobians& multi_jac2);

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

}  // namespace gtdynamics
