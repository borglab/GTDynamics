/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  FootContactConstraintSpec.h
 * @brief Utility methods for generating FootContactConstraintSpec objects.
 * @author: Disha Das, Frank Dellaert
 */

#pragma once

#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/universal_robot/Robot.h>
#include <gtdynamics/utils/ConstraintSpec.h>

#include <iosfwd>

namespace gtdynamics {
/**
 * @class FootContactConstraintSpec class stores information about a robot stance
 * and its duration.
 */

using ContactPointGoals = std::map<std::string, gtsam::Point3>;

class FootContactConstraintSpec : public ConstraintSpec {
 protected:
  PointOnLinks contact_points_;  ///< Contact Points

 public:
  /// Constructor
  FootContactConstraintSpec() {};

  /**
   * @fbrief Constructor with all contact points, takes list of PointOnLinks.
   *
   * @param[in] points_on_links List of link PointOnLinks.
   */
  FootContactConstraintSpec(const std::vector<PointOnLink> &points_on_links);

  /**
   * @fbrief Constructor with all contact points, takes a number of links and
   * creates same contact points on all links.
   *
   * @param[in] links           List of link pointers.
   * @param[in] contact_in_com  Point of contact on link.
   */
  FootContactConstraintSpec(const std::vector<LinkSharedPtr> &links,
                   const gtsam::Point3 &contact_in_com);

  /// Returns all the contact points in the stance
  const PointOnLinks &contactPoints() const { return contact_points_; }

  /// Check if phase has a contact for given link.
  bool hasContact(const LinkSharedPtr &link) const;

  /// Returns the contact point object of link.
  const gtsam::Point3 &contactPoint(const std::string &link_name) const;

  /// Print to stream.
  friend std::ostream &operator<<(std::ostream &os,
                                  const FootContactConstraintSpec &phase);

  /// GTSAM-style print, works with wrapper.
  void print(const std::string &s) const override;

  /**
   * Return PointGoalFactors for all feet as given in cp_goals.
   * @param[in] all_contact_points stance *and* swing feet.
   * @param[in] step 3D vector to move by
   * @param[in] cost_model noise model
   * @param[in] k_start Factors are added at this time step
   * @param[inout] cp_goals either stance goal or start of swing (updated)
   */
  gtsam::NonlinearFactorGraph contactPointObjectives(
      const PointOnLinks &all_contact_points, const gtsam::Point3 &step,
      const gtsam::SharedNoiseModel &cost_model, size_t k_start,
      const ContactPointGoals &cp_goals, const size_t ts) const;

  /**
   * @fn Returns the swing links during this FootContact
   * @return Vector of swing links.
   */
  std::vector<std::string> swingLinks() const;

  /**
   * Update goal points by `step` for all swing legs.
   * @param[in] step 3D vector to move by
   * @param[in] cp_goals either stance goal or start of swing
   */
  ContactPointGoals updateContactPointGoals(
      const PointOnLinks &all_contact_points, const gtsam::Point3 &step,
      const ContactPointGoals &cp_goals) const;
};
}  // namespace gtdynamics
