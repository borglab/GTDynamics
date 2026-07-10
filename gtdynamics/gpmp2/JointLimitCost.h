/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  JointLimitCost.h
 * @brief Hinge loss joint limit cost function.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

namespace gtdynamics {

/// Hinge loss cost keeping p within its limits, zero until thresh away from
/// either, so thresh is the standoff kept from each limit.
inline double hingeLossJointLimitCost(double p, double down_limit,
                                      double up_limit, double thresh,
                                      double *H_p = nullptr) {
  if (p < down_limit + thresh) {
    if (H_p) *H_p = -1.0;
    return down_limit + thresh - p;

  } else if (p <= up_limit - thresh) {
    if (H_p) *H_p = 0.0;
    return 0.0;

  } else {
    if (H_p) *H_p = 1.0;
    return p - up_limit + thresh;
  }
}

}  // namespace gtdynamics
