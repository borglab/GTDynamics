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
 * ContactAdjustment is the adjustment made to all the contact points in relation to a given link in the robot.
 *
 * @param link_name  The link in relation to which adjustment is to be made.
 * @param adjustment The adjustment made to the contact points relative to the link COM.
 */
struct ContactAdjustment{
  std::string link_name;
  gtsam::Point3 adjustment;

  ContactAdjustment(const std::string& link_name, const gtsam::Point3 &adjustment)
      : link_name(link_name), adjustment(adjustment) {}
};

using ContactAdjustments = std::vector<ContactAdjustment>;


/**
 * @class WalkCycle class stores the sequence of phases
 * in a walk cycle.
 */
class WalkCycle {
 protected:
  std::vector<Phase> phases_;     ///< Phases in walk cycle
  PointOnLinks contact_points_;  ///< All contact points

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
  void addPhase(const Phase &phase);

  /**
   * @fn Return phase for given phase number p.
   * @param[in]p    Phase number \in [0..numPhases()[.
   * @return Phase instance.
   */
  const Phase& phase(size_t p) const;

  /// Returns vector of phases in the walk cycle
  const std::vector<Phase>& phases() const { return phases_; }

  /// Returns count of phases in the walk cycle
  size_t numPhases() const { return phases_.size(); }

  /// Returns the number of time steps, summing over all phases.
  size_t numTimeSteps() const;

  /// Return all the contact points.
  const PointOnLinks& contactPoints() const { return contact_points_; }

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os,
                                  const WalkCycle& walk_cycle);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s = "") const;

  /**
   * @fn Returns the initial contact point goal for every contact link.
   * @return Map from link name to goal points.
   */
  ContactGoals initContactPointGoal(
      const Robot &robot,
      const ContactAdjustments &contact_adjustments = {}) const;

  /**
   * @fn Returns the swing links for a given phase.
   * @param[in]p    Phase number.
   * @return Vector of swing links.
   */
  std::vector<std::string> swingLinks(size_t p) const;


  //NOTE DISHA: Remove Robot
  /**
   * Add PointGoalFactors for all feet as given in cp_goals.
   * @param[in] step 3D vector to move by
   * @param[in] cost_model noise model
   * @param[in] k_start Factors are added at this time step
   * @param[inout] cp_goals either stance goal or start of swing (updated)
   */
  gtsam::NonlinearFactorGraph contactPointObjectives(
      const gtsam::Point3 &step, const gtsam::SharedNoiseModel &cost_model,
      size_t k_start, ContactGoals *cp_goals) const;
};
}  // namespace gtdynamics
