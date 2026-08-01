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

#include <gtdynamics/gpmp2/RobotQueryPoints.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>

#include <cstddef>
#include <vector>

namespace gtdynamics {

/// One self collision check between two query points, each inflated to a sphere
/// by its radius and kept epsilon apart on top of that.
struct SelfCollisionPair {
  size_t a;        ///< first query point index
  size_t b;        ///< second query point index
  double epsilon;  ///< standoff between the two, added to their radii

  SelfCollisionPair() : a(0), b(0), epsilon(0.0) {}
  SelfCollisionPair(size_t a, size_t b, double epsilon)
      : a(a), b(b), epsilon(epsilon) {}
};

using SelfCollisionPairs = std::vector<SelfCollisionPair>;

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

/// Hinge loss of every pair at configuration q, using
/// pair.epsilon + radii(pair.a) + radii(pair.b) as each pair's standoff. If Hq
/// is non-null, it is filled with the nrPairs x dof Jacobian with respect to q.
GTSAM_EXPORT gtsam::Vector selfCollisionError(const gtsam::Vector &q,
                                              const RobotQueryPoints &robot,
                                              const SelfCollisionPairs &pairs,
                                              const gtsam::Vector &radii,
                                              gtsam::Matrix *Hq = nullptr);

}  // namespace gtdynamics
