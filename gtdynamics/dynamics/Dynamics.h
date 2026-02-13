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
#include <gtdynamics/utils/Interval.h>
#include <gtdynamics/utils/PointOnLink.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtdynamics {

/**
 * Calculate Coriolis wrench term and Jacobian with respect to twist.
 * @param inertia Spatial inertia matrix.
 * @param twist Spatial twist.
 * @param H_twist Optional Jacobian with respect to twist.
 */
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

/**
 * Dynamics factor builder for moving configurations.
 *
 * For the templated API below, `CONTEXT` is typically `Slice`, `Interval`,
 * or `Phase`.
 */
class Dynamics {
 protected:
  const DynamicsParameters p_;

 public:
  /**
   * Constructor.
   * @param parameters Dynamics parameter bundle with mechanics + dynamic-only
   * noise models and settings.
   */
  Dynamics(const DynamicsParameters& parameters = DynamicsParameters())
      : p_(parameters) {}

  /**
   * Return acceleration-level factor graph.
   * @param contact_points Optional contact points with zero-acceleration
   * constraints.
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph aFactors(
      const CONTEXT& context, const Robot& robot,
      const std::optional<PointOnLinks>& contact_points = {}) const;

  /**
   * Return dynamic-only wrench factors for a context.
   * This excludes factor groups provided via the Statics slice interface.
   * @param contact_points Optional contact points that add friction/moment
   * factors and contact wrench keys.
   * @param mu Optional friction coefficient (defaults to 1.0).
   */
  template <class CONTEXT>
  gtsam::NonlinearFactorGraph graph(
      const CONTEXT& context, const Robot& robot,
      const std::optional<PointOnLinks>& contact_points = {},
      const std::optional<double>& mu = {}) const;
};

}  // namespace gtdynamics
