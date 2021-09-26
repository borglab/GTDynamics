/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  FootContactConstraintSpec.cpp
 * @brief Utility methods for generating FootContactConstraintSpec objects.
 * @author: Disha Das, Frank Dellaert
 */

#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/utils/FootContactConstraintSpec.h>

#include <iostream>

namespace gtdynamics {

using gtsam::Matrix;
using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using std::string;

FootContactConstraintSpec::FootContactConstraintSpec( const std::vector<PointOnLink> &points_on_links) {
  for (auto &&point_on_link : points_on_links) {
    contact_points_.push_back(point_on_link);
  }
}

FootContactConstraintSpec::FootContactConstraintSpec(const std::vector<LinkSharedPtr> &links,
                                   const gtsam::Point3 &contact_in_com) {
  for (auto &&link : links) {
    contact_points_.emplace_back(link, contact_in_com);
  }
}

std::ostream &operator<<(std::ostream &os, const FootContactConstraintSpec &phase) {
  os << "[";
  for (auto &&cp : phase.contactPoints()) {
    os << cp.link->name() << ": [" << cp.point.transpose() << "], ";
  }
  os << "]";
  return os;
}

void FootContactConstraintSpec::print(const string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

// Searches a contact_link from ContactGoals object and returns the
// corresponding goal_point
gtsam::Point3 &pointGoal(ContactGoals *cp_goals,
                         const PointOnLink contact_point) {
  for (auto it = cp_goals->begin(); it != cp_goals->end(); ++it) {
    if ((*it).point_on_link.link->name() == contact_point.link->name() &&
        (*it).point_on_link.point == contact_point.point)
      return (*it).goal_point;
  }
  throw std::runtime_error("Contact Point was not found.");
}

NonlinearFactorGraph FootContactConstraintSpec::contactPointObjectives(
    const PointOnLinks &all_contact_points, const Point3 &step,
    const gtsam::SharedNoiseModel &cost_model, size_t k_start,
    const ContactPointGoals &cp_goals, const size_t ts) const {
  NonlinearFactorGraph factors;

  for (auto &&cp : all_contact_points) {
    const string &name = cp.link->name();
    const Point3 &cp_goal = cp_goals.at(name);
    const bool stance = hasContact(cp.link);
    auto goal_trajectory =
        stance ? StanceTrajectory(cp_goal, ts)
               : SimpleSwingTrajectory(cp_goal, step, ts);

    factors.push_back(PointGoalFactors(cost_model, cp.point, goal_trajectory,
                                       cp.link->id(), k_start));
  }
  return factors;
}

std::vector<string> FootContactConstraintSpec::swingLinks() const {
  std::vector<string> phase_swing_links;
  for (auto &&kv : contactPoints()) {
    if (!hasContact(kv.link)) {
      phase_swing_links.push_back(kv.link->name());
    }
  }
  return phase_swing_links;
}

ContactPointGoals FootContactConstraintSpec::updateContactPointGoals(
    const PointOnLinks &all_contact_points, const Point3 &step,
    const ContactPointGoals &cp_goals) const {
  ContactPointGoals new_goals;

  // For all "feet", update the goal point with step iff in swing.
  for (auto &&cp : all_contact_points) {
    const string &name = cp.link->name();
    const Point3 &cp_goal = cp_goals.at(name);
    const bool stance = hasContact(cp.link);
    // If a contact is not on a stance leg, it is on a swing leg and we advance
    // the contact goal by adding the 3-vector `step`.
    new_goals.emplace(name, stance ? cp_goal : cp_goal + step);
  }
  return new_goals;
}

}  // namespace gtdynamics
