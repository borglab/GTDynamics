/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */

/**
 * @file ContactWrench.h
 * @brief Rectangular sole support patches and affine resultant-wrench cones.
 * @author GTDynamics contributors
 */

#pragma once

#include <gtsam/constrained/NonlinearInequalityConstraint.h>

namespace gtdynamics {

/** A centered rectangular support patch in a material contact frame.
 * The frame has +Z pointing out of the ground into the robot. A wrench acting
 * on the robot is [tau_x,tau_y,tau_z,F_x,F_y,F_z], in N m and N. The rectangle
 * is x in [-halfLength,halfLength], y in [-halfWidth,halfWidth], z=0, metres.
 * All constructor arguments must be finite and strictly positive.
 *
 * The cone is the exact resultant-wrench cone of four corner forces with
 * |f_x|,|f_y| <= muPyramid*f_z and f_z >= 0 at each corner. We set
 * muPyramid = frictionCoefficient/sqrt(2), inscribing the square friction
 * pyramid in the physical isotropic Coulomb cone. Thus the representation is
 * conservative for circular friction, including its coupled yaw bounds.
 *
 * The 17 affine faces enforce normal positivity (1), tangential friction (4),
 * center-of-pressure bounds (4), and coupled yaw-moment bounds (8). The latter
 * prevent independent force, CoP, and yaw limits from accepting impossible
 * combinations. At zero normal force all force and moment components vanish.
 * This models rigid planar support, without a pressure-distribution state,
 * friction identification, adhesion, or independent torsional friction.
 * Formula: Caron, Pham and Nakamura, ICRA 2015, Section 4, Eq. (20),
 * https://arxiv.org/abs/1501.04719; reordered here to torque-first wrenches.
 */
class RectangularContactWrench {
  double halfLength_, halfWidth_, frictionCoefficient_;

 public:
  /** Construct a centered sole support patch in metres with Coulomb friction.
   * Throws std::invalid_argument for nonfinite or nonpositive dimensions or
   * friction coefficient.
   */
  RectangularContactWrench(double halfLength, double halfWidth,
                           double frictionCoefficient);

  /// Support extent along contact +X, metres.
  double halfLength() const { return halfLength_; }
  /// Support extent along contact +Y, metres.
  double halfWidth() const { return halfWidth_; }
  /// Physical isotropic Coulomb friction coefficient, dimensionless.
  double frictionCoefficient() const { return frictionCoefficient_; }
  /// Axis-wise coefficient of the inscribed four-sided friction pyramid.
  double pyramidCoefficient() const;

  /** Return A (17x6), with feasible wrenches satisfying A*wrench <= 0.
   * Rows are -F_z; +/-F_x-mu*F_z; +/-F_y-mu*F_z;
   * +/-tau_x-halfWidth*F_z; +/-tau_y-halfLength*F_z; then the four upper
   * and four lower yaw faces. For each s_x,s_y in {-1,+1}, the upper face is
   * tau_z+s_x*(halfWidth*F_x+mu*tau_x)
   *       +s_y*(halfLength*F_y+mu*tau_y)-mu*(halfLength+halfWidth)*F_z <= 0.
   * The lower face negates tau_z and both mu*tau terms. Normal and friction
   * rows have N units; CoP and yaw rows have N m units.
   */
  gtsam::Matrix matrix() const;

  /** Build 17 separately active scalar GTSAM LinearConstraint inequalities.
   * The key must store a dynamic gtsam::Vector of length six, as required by
   * GTSAM LinearConstraint (Python NumPy vectors already use this storage).
   * They are hard feasibility constraints, not clamped soft costs. Each row
   * is divided by its Euclidean coefficient norm and uses sigma=1, so changing
   * support dimensions does not arbitrarily scale active-face tolerances.
   */
  gtsam::NonlinearInequalityConstraints constraints(gtsam::Key key) const;
};

}  // namespace gtdynamics
