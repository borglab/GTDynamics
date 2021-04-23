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

// /// Add PointGoalFactors for a stance foot, starting at (k_start, cp_goal).
// void stanceObjectives(gtsam::NonlinearFactorGraph *factors,
//                     const Point3 &cp_goal,
//                     const gtsam::SharedNoiseModel &cost_model,
//                     const double ground_height,  &name,
//                     size_t k_start = 0) const {
//   for (int k = k_start; k <= k_start + num_time_steps_; k++) {
//     Point3 goal_point(cp_goal.x(), cp_goal.y(), ground_height -
//     0.05); factors->add(pointGoalFactor(name, k, cost_model, goal_point));
//   }
// }

// /**
//  * Add PointGoalFactors for swing foot, starting at (k_start, cp_goal).
//  * Swing feet is moved according to a pre-determined height trajectory, and
//  * moved by the 3D vector step. Returns cp_goal + step * num_time_steps_;
//  */
// Point3 addSwingGoals(gtsam::NonlinearFactorGraph *factors,
//                             Point3 cp_goal,  // by value
//                             const Point3 &step,
//                             const gtsam::SharedNoiseModel &cost_model,
//                             const double ground_height,  &name,
//                             size_t k_start = 0) const {
//   for (int k = k_start; k <= k_start + num_time_steps_; k++) {
//     double t = (double)(k - k_start) / (double)num_time_steps_;
//     double h = ground_height + pow(t, 1.1) * pow(1 - t, 0.7);
//     Point3 goal_point(cp_goal.x(), cp_goal.y(), h);
//     factors->add(pointGoalFactor(name, k, cost_model, goal_point));
//     cp_goal = cp_goal + step;  // imperative
//   }
//   return cp_goal;
// }

gtsam::NonlinearFactorGraph Phase::stanceObjectives(
    const Robot &robot, std::map<std::string, Point3> cp_goals,
    const gtsam::SharedNoiseModel &cost_model, size_t k) const {
  gtsam::NonlinearFactorGraph factors;
  for (auto &&kv : contact_points_) {
    const std::string &name = kv.first;
    const Point3 &cp_goal = cp_goals.at(name);
    const ContactPoint &cp = kv.second;
    AddPointGoalFactors(&factors, cost_model, cp.point,
                        StanceTrajectory(cp_goal, numTimeSteps()),
                        robot.link(name)->id(), k);
  }
  return factors;
}
}  // namespace gtdynamics
