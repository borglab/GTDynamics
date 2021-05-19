/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  Phase.cpp
 * @brief Utility methods for generating Phase objects.
 * @author: Disha Das, Frank Dellaert
 */

#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/utils/Phase.h>

#include <iostream>

namespace gtdynamics {

using gtsam::Matrix;
using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using std::string;

std::ostream &operator<<(std::ostream &os, const Phase &phase) {
  os << "[";
  for (auto &&cp : phase.contactPoints()) {
    os << cp.link->name() << ": " << cp.point << ", ";
  }
  os << "]";
  return os;
}

void Phase::print(const string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

//Searches a contact_link from ContactGoals object and returns the corresponding goal_point
gtsam::Point3 &pointGoal(ContactGoals *cp_goals,
                         const PointOnLink contact_point) {
  for (auto it = cp_goals->begin(); it != cp_goals->end(); ++it) {
    if ((*it).point_on_link.link->name() == contact_point.link->name() &&
        (*it).point_on_link.point == contact_point.point)
      return (*it).goal_point;
  }
  throw std::runtime_error("Contact Point was not found.");
}

NonlinearFactorGraph Phase::contactPointObjectives(
    const PointOnLinks &all_contact_points, const Point3 &step,
    const gtsam::SharedNoiseModel &cost_model, size_t k_start, ContactGoals *cp_goals) const {
  NonlinearFactorGraph factors;

  for (auto &&kv : all_contact_points) {
    Point3 &cp_goal = pointGoal(cp_goals, kv);
    const bool stance = hasContact(kv.link);
    auto goal_trajectory =
        stance ? StanceTrajectory(cp_goal, num_time_steps_)
               : SimpleSwingTrajectory(cp_goal, step, num_time_steps_);
    if (!stance) cp_goal += step;  // Update the goal if swing

    factors.push_back(PointGoalFactors(cost_model, kv.point,
                                       goal_trajectory, kv.link->id(),
                                       k_start));
  }
  return factors;
}

Matrix Phase::jointMatrix(const gtsam::Values &results,
                          size_t k, boost::optional<double> dt) const {
  const auto &joints = robot_.joints();
  const size_t J = joints.size();
  const int m = numTimeSteps(), n = 4 * J + (dt ? 1 : 0);
  Matrix table(m, n);
  for (int i = 0; i < m; i++) {
    size_t j = 0;
    for (auto &&joint : joints) {
      const auto id = joint->id();
      table(i, j + 0 * J) = JointAngle(results, id, k);
      table(i, j + 1 * J) = JointVel(results, id, k);
      table(i, j + 2 * J) = JointAccel(results, id, k);
      table(i, j + 3 * J) = Torque(results, id, k);
      ++j;
    }
    if (dt) {
      table(i, n - 1) = *dt;
    }
    ++k;
  }
  return table;
}

}  // namespace gtdynamics
