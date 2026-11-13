/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  PandaIKFast.h
 * @brief Wrapper for the generated IKFast source files for the panda robot.
 * @author Frank Dellaert
 * @author Antoni Jubes
 */

#pragma once

//----------------------------------------------------------------------------//

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <stdio.h>
#include <stdlib.h>

#include <vector>

namespace gtdynamics {

/// Analytical kinematics from Panda link0 to link7, before the hand offset.
class PandaIKFast {
 public:
  /// Construct the stateless IKFast interface.
  PandaIKFast();

  // The robot's number of joints, for the panda it's 7
  static constexpr size_t kNumJoints = 7;

  /**
   * @brief Forward Kinematics on Panda robot using IKFast.
   *
   * @param jointValues Seven joint angles in radians, ordered joint1 to joint7.
   * @return Pose of link7 expressed in link0, with translation in metres.
   */
  static gtsam::Pose3 forward(const gtsam::Vector7& jointValues);

  /**
   * @brief Inverse Kinematics on Panda robot using IKFast. Singularity
   * solutions are not returned.
   *
   * @param bTe Desired link7 pose expressed in link0, in metres.
   * @param theta7 -- the value for the 7th joint angle, working as the
   * parameter of the 1D solution
   * @return std::vector<gtsam::Vector7> -- std vector containing the different
   * solutions for joint angles in radians. Joint limits are not enforced;
   * callers must filter the solutions. An empty vector means no solution for
   * the specified theta7, not necessarily that the pose is unreachable.
   */
  static std::vector<gtsam::Vector7> inverse(const gtsam::Pose3& bTe,
                                             double theta7);
};

}  // namespace gtdynamics
