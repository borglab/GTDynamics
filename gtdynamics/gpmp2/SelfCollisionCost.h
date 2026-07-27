/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  SelfCollisionCost.h
 * @brief Hinge loss self collision cost between two world frame points.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

namespace gtdynamics {

/**
 * Hinge loss self collision cost between two world frame points. The cost is
 * epsilon - dist whenever the two points are within epsilon of each other, and
 * zero beyond it, so epsilon is the standoff kept between them. Since there is
 * no field, epsilon must fold in both points' radii plus any margin.
 *
 * Near-coincident points fall back to a fixed separation direction, so the
 * Jacobian stays finite and still pushes the pair apart.
 *
 * @param pA first query point, in the world frame
 * @param pB second query point, in the world frame
 * @param epsilon standoff distance at which the cost becomes non-zero
 * @param HptA optional Jacobian of the cost with respect to pA
 * @param HptB optional Jacobian of the cost with respect to pB
 * @return the hinge loss cost
 */
GTSAM_EXPORT double hingeLossSelfCollisionCost(
    const gtsam::Point3 &pA, const gtsam::Point3 &pB, double epsilon,
    gtsam::OptionalJacobian<1, 3> HptA = {},
    gtsam::OptionalJacobian<1, 3> HptB = {});

}  // namespace gtdynamics
