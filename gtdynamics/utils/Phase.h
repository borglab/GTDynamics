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
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/util/ConstraintSpec.h>
#include <gtdynamics/util/Interval.h>

#include <iosfwd>

namespace gtdynamics {
/**
 * @class Phase class stores information about a robot stance
 * and its duration.
 */
using ContactPointGoals = std::map<std::string, gtsam::Point3>;

class Phase : Interval {
 public:
 protected:
  boost::shared_ptr<ConstraintSpec> constraints;

 public:
  /// Constructor
  Phase(size_t k_start, size_t k_end,
        const boost::shared_ptr<ConstraintSpec> &constraints)
      : Interval(k_start, k_end), constraints(constraints) {}

  /**
   * @fbrief Constructor with all contact points, takes list of PointOnLinks.
   *
   * @param[in] num_time_steps  Number of time steps in phase.
   * @param[in] points_on_links List of link PointOnLinks.
   */
  Phase(size_t num_time_steps, const std::vector<PointOnLink> &points_on_links);

  /**
   * @fbrief Constructor with all contact points, takes a number of links and
   * creates same contact points on all links.
   *
   * @param[in] num_time_steps  Number of time steps in phase.
   * @param[in] links           List of link pointers.
   * @param[in] contact_in_com  Point of contact on link.
   */
  Phase(size_t num_time_steps, const std::vector<LinkSharedPtr> &links,
        const gtsam::Point3 &contact_in_com);

  /// Returns all the contact points in the stance
  const PointOnLinks &contactPoints() const { return contact_points_; }

  /// Check if phase has a contact for given link.
  bool hasContact(const LinkSharedPtr &link) const {
    int link_count =
        std::count_if(contact_points_.begin(), contact_points_.end(),
                      [&](const PointOnLink &contact_point) {
                        return contact_point.link->name() == link->name();
                      });
    return link_count > 0;
  }

  // NOTE DISHA: Can modify this function to return multiple contact points on a
  // single link
  /// Returns the contact point object of link.
  const gtsam::Point3 &contactPoint(const std::string &link_name) const {
    auto it = std::find_if(contact_points_.begin(), contact_points_.end(),
                           [&](const PointOnLink &contact_point) {
                             return contact_point.link->name() == link_name;
                           });
    if (it == contact_points_.end())
      throw std::runtime_error("Link " + link_name + " has no contact point!");
    else
      return (*it).point;
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
   * @param[in] k_start Factors are added at this time step
   * @param[in] cp_goals either stance goal or start of swing
   */
  gtsam::NonlinearFactorGraph contactPointObjectives(
      const PointOnLinks &all_contact_points, const gtsam::Point3 &step,
      const gtsam::SharedNoiseModel &cost_model, size_t k_start,
      const ContactPointGoals &cp_goals) const;

  /**
   * Update goal points by `step` for all swing legs.
   * @param[in] step 3D vector to move by
   * @param[in] cp_goals either stance goal or start of swing
   */
  ContactPointGoals updateContactPointGoals(
      const PointOnLinks &all_contact_points, const gtsam::Point3 &step,
      const ContactPointGoals &cp_goals) const;

  /// Parse results into a matrix, in order: qs, qdots, qddots, taus, dt
  gtsam::Matrix jointMatrix(const Robot &robot, const gtsam::Values &results,
                            size_t k = 0,
                            boost::optional<double> dt = boost::none) const;
};
}  // namespace gtdynamics
