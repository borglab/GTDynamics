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

#include "gtdynamics/dynamics/DynamicsGraph.h"
#include "gtdynamics/universal_robot/Robot.h"

namespace gtdynamics {
/**
 * @class Phase class stores information about a robot stance
 * and its duration.
 */
class Phase {
 protected:
  Robot robot_configuration_;     ///< Robot configuration of this stance
  ContactPoints contact_points_;  ///< Contact Points
  int num_time_steps_;            ///< Number of time steps in this phase

 public:
  /// Constructor
  Phase(const Robot& robot_configuration, const int& num_time_steps)
      : robot_configuration_(robot_configuration),
        num_time_steps_(num_time_steps) {}

  /** @fn Adds a contact point in the phase.
   *
   * @param[in] link             Name of link in the robot_configuration.
   * @param[in] point            Point of contact on link.
   * @param[in] contact_height   Height of contact point from ground.
   */
  void addContactPoint(const std::string& link, const gtsam::Point3& point,
                       double contact_height) {
    // Check if link exists in the robot
    robot_configuration_.link(link);

    auto ret =
        contact_points_.emplace(link, ContactPoint{point, 0, contact_height});
    if (!ret.second)
      throw std::runtime_error("Multiple contact points for link " + link +
                               " !");
  }

  /**
   * @fn Add multiple contact points.
   *
   * @param[in] links            List of links.
   * @param[in] point            Point of contact on link.
   * @param[in] contact_height   Height of contact point from ground.
   */
  void addContactPoints(const std::vector<std::string>& links, const gtsam::Point3& point,
                        double contact_height) {
    for (auto&& link : links) {
      addContactPoint(link, point, contact_height);
    }
  }

  /// Returns the robot configuration of the stance
  Robot getRobotConfiguration() const { return robot_configuration_; }

  /// Returns all the contact points in the stance
  ContactPoints getAllContactPoints() const { return contact_points_; }

  /// Returns the contact point object of link.
  const ContactPoint getContactPointAtLink(const std::string& link) {
    if (contact_points_.find(link) == contact_points_.end()) {
      throw std::runtime_error("Link " + link + " has no contact point!");
    }
    return contact_points_[link];
  }

  /// Returns the number of time steps in this phase
  const int numTimeSteps() const { return num_time_steps_; }
};
}  // namespace gtdynamics
