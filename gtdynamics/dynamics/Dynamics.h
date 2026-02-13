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

#include <gtdynamics/dynamics/DynamicsParameters.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtdynamics {

/// calculate Coriolis term and jacobian w.r.t. joint coordinate twist
gtsam::Vector6 Coriolis(const gtsam::Matrix6 &inertia,
                        const gtsam::Vector6 &twist,
                        gtsam::OptionalJacobian<6, 6> H_twist = {});

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

/// Dynamics factors and solvers for moving configurations.
class Dynamics {
 protected:
  const DynamicsParameters p_;

 public:
  /// Constructor.
  Dynamics(const DynamicsParameters& parameters = DynamicsParameters())
      : p_(parameters) {}

  /// Return a-level nonlinear factor graph (acceleration related factors).
  gtsam::NonlinearFactorGraph aFactors(
      const Slice& slice, const Robot& robot,
      const std::optional<PointOnLinks>& contact_points = {}) const;
};

}  // namespace gtdynamics
