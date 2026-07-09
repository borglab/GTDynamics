/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GPPose3PriorFactor.h
 * @brief Gaussian process prior factor on Pose3.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtdynamics/factors/GPLiePriorFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

/// Gaussian process prior factor on Pose3, with velocity in the tangent space.
using GPPose3Prior = GPLiePrior<gtsam::Pose3>;

}  // namespace gtdynamics
