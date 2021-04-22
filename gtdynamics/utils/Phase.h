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

#include <algorithm>
#include <boost/algorithm/string/join.hpp>
#include <iosfwd>

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/universal_robot/Robot.h>

namespace gtdynamics {
/**
 * @class Phase class stores information about a robot stance
 * and its duration.
 */
class Phase {
 protected:
  Robot robot_;                  ///< Robot configuration of this stance
  ContactPoints contact_points_; ///< Contact Points
  int num_time_steps_;           ///< Number of time steps in this phase

 public:
  /// Constructor
  Phase(const Robot &robot_configuration, const int &num_time_steps)
      : robot_(robot_configuration), num_time_steps_(num_time_steps) {}

  /** @fn Adds a contact point in the phase.
   *
   * @param[in] link             Name of link in the robot_configuration.
   * @param[in] point            Point of contact on link.
   * @param[in] contact_height   Height of contact point from ground.
   */
  void addContactPoint(const std::string &link_name, const gtsam::Point3 &point,
                       double contact_height) {
    // Check if link exists in the robot
    robot_.link(link_name);
    auto ret = contact_points_.emplace(link_name,
                                       ContactPoint{point, 0, contact_height});
    if (!ret.second) {
      throw std::runtime_error("Multiple contact points for link " + link_name);
    }
  }

  /**
   * @fn Add multiple contact points.
   *
   * @param[in] link_names            List of link_names.
   * @param[in] point            Point of contact on link.
   * @param[in] contact_height   Height of contact point from ground.
   */
  void addContactPoints(const std::vector<std::string> &link_names,
                        const gtsam::Point3 &point, double contact_height) {
    for (auto &&link_name : link_names) {
      addContactPoint(link_name, point, contact_height);
    }
  }

  /// Returns the robot configuration of the stance
  const Robot &robot() const { return robot_; }

  /// Returns all the contact points in the stance
  const ContactPoints &contactPoints() const { return contact_points_; }

  /// Returns the contact point object of link.
  const ContactPoint &
  getContactPointAtLink(const std::string &link_name) const {
    if (contact_points_.find(link_name) == contact_points_.end()) {
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
};
} // namespace gtdynamics
