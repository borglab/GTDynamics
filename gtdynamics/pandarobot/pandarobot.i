/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file pandarobot.i
 * @brief Python bindings for Panda analytical kinematics.
 * @author GTDynamics contributors
 */

namespace gtdynamics {

#include <gtdynamics/pandarobot/ikfast/PandaIKFast.h>
class PandaIKFast {
  PandaIKFast();
  static gtsam::Pose3 forward(const gtsam::Vector7& jointValues);
  static std::vector<gtsam::Vector7> inverse(const gtsam::Pose3& bTe,
                                          double theta7);
};

}  // namespace gtdynamics
