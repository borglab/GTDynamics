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

#include <map>
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
   * @fn Return phase for given phase number p.
   * @param[in]p    Phase number \in [0..numPhases()[.
   * @return Phase instance.
   */
  const Phase& phase(size_t p) const {
    if (p >= numPhases()) {
      throw std::invalid_argument("Trajectory:phase: no such phase");
    }
    return phases_.at(p);
  }

  /// Returns vector of phases in the walk cycle
  const std::vector<Phase>& phases() const { return phases_; }

  /// Returns count of phases in the walk cycle
  size_t numPhases() const { return phases_.size(); }

  /// Returns the number of time steps, summing over all phases.
  size_t numTimeSteps() const {
    size_t num_time_steps = 0;
    for (const Phase& p : phases_) num_time_steps += p.numTimeSteps();
    return num_time_steps;
  }

  /// Return all the contact points.
  const ContactPoints& contactPoints() const { return contact_points_; }

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os,
                                  const WalkCycle& walk_cycle);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s = "") const;

  /**
   * @fn Returns the initial contact point goal for every contact link.
   * @return Map from link name to goal points.
   */
  std::map<std::string, gtsam::Point3> initContactPointGoal(
      const Robot& robot,
      double ground_height = 0) const;

  /**
   * @fn Returns the swing links for a given phase.
   * @param[in]p    Phase number.
   * @return Vector of swing links.
   */
  std::vector<std::string> swingLinks(size_t p) const;

  /**
   * Add PointGoalFactors for all feet as given in cp_goals.
   * @param[in] step 3D vector to move by
   * @param[in] cost_model noise model
   * @param[in] robot needed to get link id and create key
   * @param[in] k_start Factors are added at this time step
   * @param[inout] cp_goals either stance goal or start of swing (updated)
   */
  gtsam::NonlinearFactorGraph contactPointObjectives(
      const gtsam::Point3& step, const gtsam::SharedNoiseModel& cost_model,
      const Robot& robot, size_t k_start,
      std::map<std::string, gtsam::Point3>* cp_goals) const;
};
}  // namespace gtdynamics
