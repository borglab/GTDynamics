/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020-2021, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Dynamics.h
 * @brief Wrench calculations for configurations in motion.
 * @author Frank Dellaert, Mandy Xie, Yetong Zhang, and Gerry Chen
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>

namespace gtdynamics {

/// calculate Coriolis term and jacobian w.r.t. joint coordinate twist
gtsam::Vector6 Coriolis(const gtsam::Matrix6 &inertia,
                        const gtsam::Vector6 &twist,
                        gtsam::OptionalJacobian<6, 6> H_twist = boost::none);

/// Matrix vector multiplication.
template <int M, int N>
inline Eigen::Matrix<double, M, 1> MatVecMult(
    const Eigen::Matrix<double, M, N> &constant_matrix,
    const Eigen::Matrix<double, N, 1> &vector,
    gtsam::OptionalJacobian<M, N> H_vector) {
  if (H_vector) {
    *H_vector = constant_matrix;
  }
  return constant_matrix * vector;
}

}  // namespace gtdynamics
