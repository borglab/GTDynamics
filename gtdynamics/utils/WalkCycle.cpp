/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WalkCycle.cpp
 * @brief Class to store walk cycle.
 * @author: Disha Das, Varun Agrawal, Frank Dellaert
 */

#include <gtdynamics/utils/WalkCycle.h>

namespace gtdynamics {
std::map<std::string, gtsam::Point3> WalkCycle::initContactPointGoal(
    const Robot& robot) const {
  std::map<std::string, gtsam::Point3> cp_goals;

  // Go over all phases, and all contact points
  for (auto&& phase : phases_) {
    for (auto&& kv : phase.contactPoints()) {
      auto link_name = kv.first;
      // If no goal set yet, add it here
      if (cp_goals.count(link_name) == 0) {
        LinkSharedPtr link = robot.link(link_name);
        auto foot_w = link->wTcom().transformFrom(kv.second.point);
        cp_goals.emplace(link_name, foot_w);
      }
    }
  }

  return cp_goals;
}
}  // namespace gtdynamics
