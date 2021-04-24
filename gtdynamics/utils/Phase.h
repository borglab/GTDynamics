/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Phase.h
 * @brief Utility methods for generating Phase objects.
 * @author: Disha Das
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
protected:
  ContactPoints contact_points_; ///< Contact Points
  int num_time_steps_;           ///< Number of time steps in this phase

public:
  /// Constructor
  Phase(const int &num_time_steps) : num_time_steps_(num_time_steps) {}

  /**
   * @fbrief Constructor with all contact points.
   *
   * @param[in] link_names       List of link_names.
   * @param[in] point            Point of contact on link.
   */
  Phase(const int &num_time_steps, const std::vector<std::string> &link_names,
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
   * Add PointGoalFactors for all stance feet as given in cp_goals.
   * Factors are added at time step k, default 0.
   */
  gtsam::NonlinearFactorGraph stanceObjectives(
      const Robot &robot, std::map<std::string, gtsam::Point3> cp_goals,
      const gtsam::SharedNoiseModel &cost_model, size_t k = 0) const;

  /// Parse results into a matrix, in order: qs, qdots, qddots, taus, dt
  gtsam::Matrix jointValues(const Robot &robot, const gtsam::Values &results,
                            size_t k = 0,
                            boost::optional<double> dt = boost::none) const;
};
} // namespace gtdynamics
