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

#include <string>
#include <vector>

namespace gtdynamics {
/**
 * @class WalkCycle class stores the sequence of phases
 * in a walk cycle.
 */
class WalkCycle {
 protected:
  std::vector<Phase> phases_;     ///< Phases in walk cycle
  ContactPoints contact_points_;  ///< All Contact points in the walk cycle

 public:
  /// Default Constructor
  WalkCycle() {}

  /**
   * @fn Adds phase in walk cycle
   * @param[in] phase Swing or stance phase in the walk cycle.
   */
  void addPhase(const Phase& phase) {
    auto phase_contact_points = phase.contactPoints();
    for (auto&& contact_point : phase_contact_points) {
      // If contact point is not present, add it
      if (contact_points_.find(contact_point.first) == contact_points_.end()) {
        contact_points_.emplace(contact_point.first, contact_point.second);
      } else {
        if (contact_points_[contact_point.first] != contact_point.second)
          throw std::runtime_error("Multiple Contact points for Link " +
                                   contact_point.first + " found!");
      }
    }
    phases_.push_back(phase);
  }

  /// Returns vector of phases in the walk cycle
  const std::vector<Phase>& phases() const { return phases_; }

  /// Returns count of phases in the walk cycle
  int numPhases() const { return phases_.size(); }

  /// Return all the contact points.
  ContactPoints contactPoints() const { return contact_points_; }
};
}  // namespace gtdynamics
