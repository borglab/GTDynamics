/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  GPPose3Interpolator.h
 * @brief Gaussian process interpolator on Pose3.
 * @author Karthik Shaji - Adapted from gpmp2 by Jing Dong.
 */

#pragma once

#include <gtdynamics/gpmp2/GPLieInterpolator.h>
#include <gtsam/geometry/Pose3.h>

namespace gtdynamics {

/// Gaussian process interpolator on Pose3. Requires a 6-dimensional QcModel,
/// which the GPLieInterpolator constructor enforces.
using GPPose3Interpolator = GPLieInterpolator<gtsam::Pose3>;

}  // namespace gtdynamics
