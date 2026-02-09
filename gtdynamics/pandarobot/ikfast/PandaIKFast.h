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

// Wrapper of IKFast functions for panda robot.
class PandaIKFast {
 public:
  PandaIKFast();

  // The robot's number of joints, for the panda it's 7
  static constexpr size_t kNumJoints = 7;

  /**
   * @brief Forward Kinematics on Panda robot using IKFast.
   *
   * @param joint_values -- gtsam::Vector7 of joint angles' values
   * @return gtsam::Pose3 -- Resulting 3D Pose with the guven joint angles
   */
  static gtsam::Pose3 forward(const gtsam::Vector7& joint_values);

  /**
   * @brief Inverse Kinematics on Panda robot using IKFast. Singularity
   * solutions are not returned.
   *
   * @param bTe -- the desired end-effector pose wrt the base frame
   * @param theta7 -- the value for the 7th joint angle, working as the
   * parameter of the 1D solution
   * @return std::vector<gtsam::Vector7> -- std vector containing the different
   * solutions for joint angles' values
   */
  static std::vector<gtsam::Vector7> inverse(const gtsam::Pose3& bTe,
                                             double theta7);
};

}  // namespace gtdynamics
