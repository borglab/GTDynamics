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
#include <gtdynamics/utils/FootContactConstraintSpec.h>

#include <map>
#include <string>
#include <vector>

namespace gtdynamics {

/**
 * @class WalkCycle class stores the sequence of phases
 * in a walk cycle. A WalkCycle is built from FootContactConstraintSpecs
 * and phase lengths, and can spawn phases.
 */
class WalkCycle {
 protected:
  std::vector<Phase> phases_;    ///< Phases in walk cycle
  PointOnLinks contact_points_;  ///< All unique contact points in the walk cycle

 /// Gets the intersection between two PointOnLinks objects
  static PointOnLinks getIntersection(const PointOnLinks &cps1,
                                      const PointOnLinks &cps2) {
    PointOnLinks intersection;
    for (auto &&cp1 : cps1) {
      for (auto &&cp2 : cps2) {
        if (cp1 == cp2) {
          intersection.push_back(cp1);
        }
      }
    }
    return intersection;
  }

 public:
  /// Default Constructor
  WalkCycle() {}

  /**
   * @fn Construct a new Walk Cycle object from a vector of phases.
   * 
   * @param states ........... a vector of phases
   */
  WalkCycle(const std::vector<Phase> &phase_vector) {
    for (auto&& phase : phase_vector)  {
      phases_.push_back(phase);
    }

    // After all phases in the walk cycle are in phases_, 
    // add unique contact from each phase in the walk cycle to contact_points_
    for (auto&& phase : phases_) {
      addPhaseContactPoints(phase);
    }
  }

  /**
   * @fn Construct a new Walk Cycle object from a vector of 
   * FootContactConstraintSpec pointers and phase lengths. 
   * for each element in the vectors, a Phase will be generated with a 
   * corresponding FootContactConstraintSpec and phase length.
   * 
   * @param states ........... a vector of FootContactConstraintSpec shared pointers.
   * @param phase_lengths .... a vector of phase lengths corresponding to the states vector.
   */
  WalkCycle(const FootContactVector &states,
            const std::vector<size_t> &phase_lengths) {
    if (states.size() != phase_lengths.size()) {
      throw std::runtime_error(
          "states vector and phase_lengths vector have different sizes");
    }
    size_t k = 0;
    for (size_t i = 0; i < states.size(); i++) {
      Phase ph = Phase(k, k + phase_lengths[i], states[i]);
      phases_.push_back(ph);
      k += phase_lengths[i];
    }

    // After all phases in the trajectory are in phases_,
    // add unique contact from each phase in the trajectory to contact_points_
    for (auto &&phase : phases_) {
      addPhaseContactPoints(phase);
    }
  }

  /**
   * @fn adds unique contact points from phase to contact_points_ of trajectory 
   * 
   * @param[in] phase.... phase from which to add contact points  
   */
  void addPhaseContactPoints(const Phase &phase);

  /// Return all unique contact points.
  const PointOnLinks& contactPoints() const { return contact_points_; }

   /// Return all contact points from all phases, not only unique
  std::vector<PointOnLinks> allPhasesContactPoints() const;

  /**
   * @fn Returns a vector of PointOnLinks objects for all transitions between
   * phases after applying repetition on the original sequence.
   * @return Transition CPs.
   */
  std::vector<PointOnLinks> transitionContactPoints() const;

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

  /**
   * @fn Returns the initial contact point goal for every contact link.
   * @param[in] robot Robot specification from URDF/SDF.
   * @param[in] ground_height z-coordinate of ground plane in world frame.
   * @return Map from link name to goal points.
   */
  ContactPointGoals initContactPointGoal(const Robot &robot,
                                         double ground_height) const;

  /**
   * @fn Create desired stance and swing trajectories for all contact links.
   * @param[in] robot Robot specification from URDF/SDF.
   * @param[in] cost_model Noise model
   * @param[in] step The 3D vector the foot moves in a step.
   * @param[in] ground_height z-coordinate of ground plane in world frame.
   * @return All objective factors as a NonlinearFactorGraph
   */
  gtsam::NonlinearFactorGraph contactPointObjectives(
      const gtsam::Point3 &step, const gtsam::SharedNoiseModel &cost_model,
      size_t k_start, ContactPointGoals *cp_goals) const;

  /**
   * @fn Returns the swing links for a given phase.
   * @param[in] p    Phase number.
   * @return Vector of swing links.
   */
  std::vector<std::string> getPhaseSwingLinks(size_t p) const; 

  /**
  * @fn Returns the contact points for a given phase.
  * @param[in] p    Phase number.
  * @return Vector of contact links.
  */
  const PointOnLinks getPhaseContactPoints(size_t p) const;

  /// Print to stream.
  friend std::ostream& operator<<(std::ostream& os,
                                  const WalkCycle& walk_cycle);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string& s = "") const;
  
};
}  // namespace gtdynamics
