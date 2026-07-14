/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  ObstacleCost.h
 * @brief Hinge loss obstacle cost functions on a signed distance field.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtdynamics/gpmp2/SDFexception.h>
#include <gtdynamics/gpmp2/SignedDistanceField.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

/**
 * Hinge loss obstacle cost for a query point expressed in the same frame as the
 * field. The cost is epsilon - d(point) whenever the signed distance d falls
 * within epsilon of the surface, and zero beyond it, so epsilon is the standoff
 * distance kept from every obstacle in the field.
 *
 * A point outside the grid is treated as free space, which means an undersized
 * field fails open: pad every field by at least epsilon past its geometry.
 *
 * @param point query point, in the frame of the field
 * @param sdf signed distance field
 * @param epsilon standoff distance at which the cost becomes non-zero
 * @param H_point optional Jacobian of the cost with respect to the point
 * @return the hinge loss cost
 */
inline double hingeLossObstacleCost(
    const gtsam::Point3 &point, const SignedDistanceField &sdf, double epsilon,
    gtsam::OptionalJacobian<1, 3> H_point = {}) {
  gtsam::Vector3 field_gradient;
  double dist_signed;
  try {
    dist_signed = sdf.getSignedDistance(point, field_gradient);
  } catch (const SDFQueryOutOfRange &) {
    if (H_point) *H_point = gtsam::Matrix13::Zero();
    return 0.0;
  }

  if (dist_signed > epsilon) {
    if (H_point) *H_point = gtsam::Matrix13::Zero();
    return 0.0;
  }
  // Inside the obstacle, or outside it but closer than epsilon.
  if (H_point) *H_point = -field_gradient.transpose();
  return epsilon - dist_signed;
}

/**
 * Hinge loss obstacle cost for a field rigidly attached to a moving frame s.
 * The query point is given in the world frame and transformed into s before the
 * field is read, so the same field serves as a world obstacle field (with sTw
 * the identity), as the collision geometry of a moving obstacle, or as the
 * collision geometry of a robot link for self collision.
 *
 * @param wTs pose of the field's frame s in the world frame
 * @param point query point, in the world frame
 * @param sdf signed distance field, expressed in frame s
 * @param epsilon standoff distance at which the cost becomes non-zero
 * @param H_pose optional Jacobian of the cost with respect to wTs
 * @param H_point optional Jacobian of the cost with respect to the point
 * @return the hinge loss cost
 */
inline double hingeLossObstacleCost(
    const gtsam::Pose3 &wTs, const gtsam::Point3 &point,
    const SignedDistanceField &sdf, double epsilon,
    gtsam::OptionalJacobian<1, 6> H_pose = {},
    gtsam::OptionalJacobian<1, 3> H_point = {}) {
  gtsam::Matrix36 Hlocal_pose;
  gtsam::Matrix3 Hlocal_point;
  const gtsam::Point3 sP = wTs.transformTo(point, Hlocal_pose, Hlocal_point);

  gtsam::Matrix13 Herr_local;
  const double cost = hingeLossObstacleCost(sP, sdf, epsilon, Herr_local);

  if (H_pose) *H_pose = Herr_local * Hlocal_pose;
  if (H_point) *H_point = Herr_local * Hlocal_point;
  return cost;
}

/**
 * Hinge loss self collision cost between two world frame points. The cost is
 * epsilon - dist whenever the two points are within epsilon of each other, and
 * zero beyond it, so epsilon is the standoff kept between them. Since there is
 * no field, epsilon must fold in both points' radii plus any margin.
 *
 * The two points must never coincide: the distance gradient divides by dist,
 * so a zero distance yields a non-finite Jacobian.
 *
 * @param pA first query point, in the world frame
 * @param pB second query point, in the world frame
 * @param epsilon standoff distance at which the cost becomes non-zero
 * @param H_pA optional Jacobian of the cost with respect to pA
 * @param H_pB optional Jacobian of the cost with respect to pB
 * @return the hinge loss cost
 */
inline double hingeLossSelfCollisionCost(
    const gtsam::Point3 &pA, const gtsam::Point3 &pB, double epsilon,
    gtsam::OptionalJacobian<1, 3> H_pA = {},
    gtsam::OptionalJacobian<1, 3> H_pB = {}) {
  gtsam::Matrix13 H_A, H_B;
  const double dist = gtsam::distance3(pA, pB, H_A, H_B);

  if (dist > epsilon) {
    if (H_pA) *H_pA = gtsam::Matrix13::Zero();
    if (H_pB) *H_pB = gtsam::Matrix13::Zero();
    return 0.0;
  }
  // Closer than epsilon: cost falls as the points separate.
  if (H_pA) *H_pA = -H_A;
  if (H_pB) *H_pB = -H_B;
  return epsilon - dist;
}

}  // namespace gtdynamics
