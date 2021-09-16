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
    os << cp.first << ": " << cp.second << ", ";
  }
  os << "]";
  return os;
}

void Phase::print(const string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

NonlinearFactorGraph Phase::contactPointObjectives(
    const ContactPoints &all_contact_points, const Point3 &step,
    const gtsam::SharedNoiseModel &cost_model, const Robot &robot,
    size_t k_start, const ContactPointGoals &cp_goals) const {
  NonlinearFactorGraph factors;

  for (auto &&kv : all_contact_points) {
    const string &name = kv.first;
    const Point3 &cp_goal = cp_goals.at(name);
    const bool stance = hasContact(name);
    auto goal_trajectory =
        stance ? StanceTrajectory(cp_goal, num_time_steps_)
               : SimpleSwingTrajectory(cp_goal, step, num_time_steps_);

    factors.push_back(PointGoalFactors(cost_model, kv.second.point,
                                       goal_trajectory, robot.link(name)->id(),
                                       k_start));
  }
  return factors;
}

Phase::ContactPointGoals Phase::updateContactPointGoals(
    const ContactPoints &all_contact_points, const Point3 &step,
    const ContactPointGoals &cp_goals) const {
  ContactPointGoals new_goals;

  // For all "feet", update the goal point with step iff in swing.
  for (auto &&kv : all_contact_points) {
    const string &name = kv.first;
    const Point3 &cp_goal = cp_goals.at(name);
    const bool stance = hasContact(name);
    // If a contact is not on a stance leg, it is on a swing leg and we advance
    // the contact goal by adding the 3-vector `step`.
    new_goals.emplace(name, stance ? cp_goal : cp_goal + step);
  }
  return new_goals;
}

Matrix Phase::jointMatrix(const Robot &robot, const gtsam::Values &results,
                          size_t k, boost::optional<double> dt) const {
  const auto &joints = robot.joints();
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
