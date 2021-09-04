/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Phase.h
 * @brief Utility methods for generating Phase objects.
 * @author: Disha Das, Frank Dellaert
 */

#pragma once

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>

#include <iosfwd>

namespace gtdynamics {
/**
 * @class Phase class stores information about a robot stance
 * and its duration.
 */
class Phase {
 public:
  using ContactPointGoals = std::map<std::string, gtsam::Point3>;

 protected:
  ContactPoints contact_points_;  ///< Contact Points
  size_t num_time_steps_;         ///< Number of time steps in this phase

 public:
  /// Constructor
  Phase(size_t num_time_steps) : num_time_steps_(num_time_steps) {}

  /**
   * @fbrief Constructor with all contact points.
   *
   * @param[in] link_names       List of link_names.
   * @param[in] point            Point of contact on link.
   */
  Phase(size_t num_time_steps, const std::vector<std::string> &link_names,
        const gtsam::Point3 &point)
      : num_time_steps_(num_time_steps) {
    addContactPoints(link_names, point);
  }

  /** @fn Adds a contact point in the phase.
   *
   * @param[in] link             Name of link in the robot_configuration.
   * @param[in] point            Point of contact on link.
   */
  void addContactPoint(const std::string &link_name,
                       const gtsam::Point3 &point) {
    // Check if link exists in the robot
    auto ret = contact_points_.emplace(link_name, ContactPoint{point, 0});
    if (!ret.second) {
      throw std::runtime_error("Multiple contact points for link " + link_name);
    }
  }

  /**
   * @fn Add multiple contact points.
   *
   * @param[in] link_names       List of link_names.
   * @param[in] point            Point of contact on link.
   */
  void addContactPoints(const std::vector<std::string> &link_names,
                        const gtsam::Point3 &point) {
    for (auto &&link_name : link_names) {
      addContactPoint(link_name, point);
    }
  }

  /// Returns all the contact points in the stance
  const ContactPoints &contactPoints() const { return contact_points_; }

  /// Check if phase has a contact for given link.
  bool hasContact(const std::string &link_name) const {
    return contact_points_.count(link_name) > 0;
  }

  /// Returns the contact point object of link.
  const ContactPoint &contactPoint(const std::string &link_name) const {
    if (!hasContact(link_name)) {
      throw std::runtime_error("Link " + link_name + " has no contact point!");
    }
    return contact_points_.at(link_name);
  }

  /// Returns the number of time steps in this phase
  int numTimeSteps() const { return num_time_steps_; }

  /// Print to stream.
  friend std::ostream &operator<<(std::ostream &os, const Phase &phase);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string &s) const;

  /**
   * Add PointGoalFactors for all feet as given in cp_goals.
   * @param[in] all_contact_points stance *and* swing feet.
   * @param[in] step 3D vector to move by
   * @param[in] cost_model noise model
   * @param[in] robot needed to get link id and create key
   * @param[in] k_start Factors are added at this time step
   * @param[in] cp_goals either stance goal or start of swing
   */
  gtsam::NonlinearFactorGraph contactPointObjectives(
      const ContactPoints &all_contact_points, const gtsam::Point3 &step,
      const gtsam::SharedNoiseModel &cost_model, const Robot &robot,
      size_t k_start, const ContactPointGoals &cp_goals) const;

  /**
   * Update goal points by `step` for all swing legs.
   * @param[in] step 3D vector to move by
   * @param[in] cp_goals either stance goal or start of swing
   */
  ContactPointGoals updateContactPointGoals(
      const ContactPoints &all_contact_points, const gtsam::Point3 &step,
      const ContactPointGoals &cp_goals) const;

  /// Parse results into a matrix, in order: qs, qdots, qddots, taus, dt
  gtsam::Matrix jointMatrix(const Robot &robot, const gtsam::Values &results,
                            size_t k = 0,
                            boost::optional<double> dt = boost::none) const;
};
}  // namespace gtdynamics
