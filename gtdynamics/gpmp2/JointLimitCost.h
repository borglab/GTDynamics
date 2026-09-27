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
inline double hingeLossJointLimitCost(double p, double downLimit,
                                      double upLimit, double thresh,
                                      double *Hp = nullptr) {
  if (p < downLimit + thresh) {
    if (Hp) *Hp = -1.0;
    return downLimit + thresh - p;

  } else if (p <= upLimit - thresh) {
    if (Hp) *Hp = 0.0;
    return 0.0;

  } else {
    if (Hp) *Hp = 1.0;
    return p - upLimit + thresh;
  }
}

}  // namespace gtdynamics
