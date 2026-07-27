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
 * A point outside the grid fails closed: its distance is extrapolated downward
 * at unit rate from the nearest grid point, so the cost pushes back inside.
 *
 * @param point query point, in the frame of the field
 * @param sdf signed distance field
 * @param epsilon standoff distance at which the cost becomes non-zero
 * @param Hpt optional Jacobian of the cost with respect to the point
 * @return the hinge loss cost
 */
GTSAM_EXPORT double hingeLossObstacleCost(const gtsam::Point3 &point,
                                          const SignedDistanceField &sdf,
                                          double epsilon,
                                          gtsam::OptionalJacobian<1, 3> Hpt = {});

/**
 * Hinge loss obstacle cost for a field rigidly attached to a moving frame s.
 * The query point is given in the world frame and transformed into s before the
 * field is read, so the same field serves as a world obstacle field (with sTw
 * the identity), as the collision geometry of a moving obstacle, or as the
 * collision geometry of a robot link for self collision. Out of grid queries
 * fail closed, as in the frame s overload.
 *
 * @param wTs pose of the field's frame s in the world frame
 * @param point query point, in the world frame
 * @param sdf signed distance field, expressed in frame s
 * @param epsilon standoff distance at which the cost becomes non-zero
 * @param Hpose optional Jacobian of the cost with respect to wTs
 * @param Hpt optional Jacobian of the cost with respect to the point
 * @return the hinge loss cost
 */
GTSAM_EXPORT double hingeLossObstacleCost(const gtsam::Pose3 &wTs,
                                          const gtsam::Point3 &point,
                                          const SignedDistanceField &sdf,
                                          double epsilon,
                                          gtsam::OptionalJacobian<1, 6> Hpose = {},
                                          gtsam::OptionalJacobian<1, 3> Hpt = {});

}  // namespace gtdynamics
