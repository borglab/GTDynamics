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

NonlinearFactorGraph Phase::contactLinkObjectives(
    const ContactPoints &all_contact_points, const Point3 &step,
    const gtsam::SharedNoiseModel &cost_model, const Robot &robot,
    size_t k_start, std::map<string, Point3> *cp_goals) const {
  NonlinearFactorGraph factors;

  for (auto &&kv : all_contact_points) {
    const string &name = kv.first;
    Point3 &cp_goal = cp_goals->at(name);
    const bool stance = hasContact(name);
    auto goal_trajectory =
        stance ? StanceTrajectory(cp_goal, num_time_steps_)
               : SimpleSwingTrajectory(cp_goal, step, num_time_steps_);
    if (!stance) cp_goal += step;  // Update the goal if swing

    AddPointGoalFactors(&factors, cost_model, kv.second.point, goal_trajectory,
                        robot.link(name)->id(), k_start);
  }
  return factors;
}

}  // namespace gtdynamics
