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

#include <gtdynamics/gpmp2/SDFException.h>
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
 * @param Hpt optional Jacobian of the cost with respect to the point
 * @return the hinge loss cost
 */
inline double hingeLossObstacleCost(
    const gtsam::Point3 &point, const SignedDistanceField &sdf, double epsilon,
    gtsam::OptionalJacobian<1, 3> Hpt = {}) {
  gtsam::Vector3 fieldGradient;
  double signedDist;
  try {
    signedDist = sdf.getSignedDistance(point, fieldGradient);
  } catch (const SDFQueryOutOfRange &) {
    if (Hpt) *Hpt = gtsam::Matrix13::Zero();
    return 0.0;
  }

  if (signedDist > epsilon) {
    if (Hpt) *Hpt = gtsam::Matrix13::Zero();
    return 0.0;
  }
  // Inside the obstacle, or outside it but closer than epsilon.
  if (Hpt) *Hpt = -fieldGradient.transpose();
  return epsilon - signedDist;
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
 * @param Hpose optional Jacobian of the cost with respect to wTs
 * @param Hpt optional Jacobian of the cost with respect to the point
 * @return the hinge loss cost
 */
inline double hingeLossObstacleCost(
    const gtsam::Pose3 &wTs, const gtsam::Point3 &point,
    const SignedDistanceField &sdf, double epsilon,
    gtsam::OptionalJacobian<1, 6> Hpose = {},
    gtsam::OptionalJacobian<1, 3> Hpt = {}) {
  gtsam::Matrix36 HlocalPose;
  gtsam::Matrix3 HlocalPt;
  const gtsam::Point3 sP = wTs.transformTo(point, HlocalPose, HlocalPt);

  gtsam::Matrix13 HerrLocal;
  const double cost = hingeLossObstacleCost(sP, sdf, epsilon, HerrLocal);

  if (Hpose) *Hpose = HerrLocal * HlocalPose;
  if (Hpt) *Hpt = HerrLocal * HlocalPt;
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
 * @param HptA optional Jacobian of the cost with respect to pA
 * @param HptB optional Jacobian of the cost with respect to pB
 * @return the hinge loss cost
 */
inline double hingeLossSelfCollisionCost(
    const gtsam::Point3 &pA, const gtsam::Point3 &pB, double epsilon,
    gtsam::OptionalJacobian<1, 3> HptA = {},
    gtsam::OptionalJacobian<1, 3> HptB = {}) {
  gtsam::Matrix13 HA, HB;
  const double dist = gtsam::distance3(pA, pB, HA, HB);

  if (dist > epsilon) {
    if (HptA) *HptA = gtsam::Matrix13::Zero();
    if (HptB) *HptB = gtsam::Matrix13::Zero();
    return 0.0;
  }
  // Closer than epsilon: cost falls as the points separate.
  if (HptA) *HptA = -HA;
  if (HptB) *HptB = -HB;
  return epsilon - dist;
}

}  // namespace gtdynamics
