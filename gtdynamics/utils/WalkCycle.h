/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WalkCycle.h
 * @brief Class to store walk cycle.
 * @author: Disha Das, Varun Agrawal
 */

#pragma once

#include <gtdynamics/utils/Phase.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <set>
#include <string>
#include <vector>

namespace gtdynamics {
/**
 * @class WalkCycle class stores the sequence of phases
 * in a walk cycle.
 */
class WalkCycle {
 protected:
  std::vector<Phase> phases_;            ///< Phases in walk cycle
  std::set<std::string> contact_links_;  ///< names of all contact links

 public:
  /// Default Constructor
  WalkCycle() {}

  /**
   * @fn Adds phase in walk cycle
   * @param[in] phase Swing or stance phase in the walk cycle.
   */
  void addPhase(const Phase& phase) {
    for (auto&& kv : phase.contactPoints()) {
      auto link_name = kv.first;
      contact_links_.insert(link_name);
    }
    phases_.push_back(phase);
  }

  /**
   * @fn Returns the initial contact point goal for every contact link.
   * @return Map from link name to goal points.
   */
  std::map<std::string, gtsam::Point3> initContactPointGoal(
      const Robot& robot) const;

  /// Returns vector of phases in the walk cycle
  const std::vector<Phase>& phases() const { return phases_; }

  /// Returns count of phases in the walk cycle
  int numPhases() const { return phases_.size(); }

  /// Return all the contact points.
  const std::set<std::string>& contactLinkNames() const {
    return contact_links_;
  }
};
}  // namespace gtdynamics
