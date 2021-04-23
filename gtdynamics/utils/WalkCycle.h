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
  std::vector<Phase> phases_;     ///< Phases in walk cycle
  ContactPoints contact_points_;  ///< All contact points

 public:
  /// Default Constructor
  WalkCycle() {}

  /// Constructor with phases
  explicit WalkCycle(const std::vector<Phase>& phases) {
    for (auto&& phase : phases) {
      addPhase(phase);
    }
  }

  /**
   * @fn Adds phase in walk cycle
   * @param[in] phase Swing or stance phase in the walk cycle.
   */
  void addPhase(const Phase& phase) {
    for (auto&& kv : phase.contactPoints()) {
      contact_points_.emplace(kv);
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
  const ContactPoints& contactPoints() const { return contact_points_; }

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os,
                                  const WalkCycle& walk_cycle);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s) const;

  /**
   * @fn Returns the swing links for a given phase.
   * @param[in]p    Phase number.
   * @return Vector of swing links.
   */
  std::vector<std::string> swingLinks(size_t p) const;

  /**
   * Add PointGoalFactors for all swing feet in phase `p`, starting at cp_goals.
   * Swing feet are moved according to a pre-determined height trajectory, and
   * moved by the 3D vector step.
   * Factors are added at time step k, default 0.
   */
  gtsam::NonlinearFactorGraph swingObjectives(
      const Robot& robot, size_t p,
      std::map<std::string, gtsam::Point3> cp_goals, const gtsam::Point3& step,
      const gtsam::SharedNoiseModel& cost_model, double ground_height,
      size_t k) const;
};
}  // namespace gtdynamics
