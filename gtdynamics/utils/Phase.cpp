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
std::ostream &operator<<(std::ostream &os, const Phase &phase) {
  os << "[";
  for (auto &&cp : phase.contactPoints()) {
    os << cp.first << ": " << cp.second << ", ";
  }
  os << "]";
  return os;
}

void Phase::print(const std::string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

// /// Add PointGoalFactors for a stance foot, starting at (k_start, cp_goal).
// void addStanceGoals(gtsam::NonlinearFactorGraph *factors,
//                     const gtsam::Point3 &cp_goal,
//                     const gtsam::SharedNoiseModel &cost_model,
//                     const double ground_height, std::string &name,
//                     size_t k_start = 0) const {
//   for (int k = k_start; k <= k_start + num_time_steps_; k++) {
//     gtsam::Point3 goal_point(cp_goal.x(), cp_goal.y(), ground_height -
//     0.05); factors->add(pointGoalFactor(name, k, cost_model, goal_point));
//   }
// }

// /**
//  * Add PointGoalFactors for swing foot, starting at (k_start, cp_goal).
//  * Swing feet is moved according to a pre-determined height trajectory, and
//  * moved by the 3D vector step. Returns cp_goal + step * num_time_steps_;
//  */
// gtsam::Point3 addSwingGoals(gtsam::NonlinearFactorGraph *factors,
//                             gtsam::Point3 cp_goal,  // by value
//                             const gtsam::Point3 &step,
//                             const gtsam::SharedNoiseModel &cost_model,
//                             const double ground_height, std::string &name,
//                             size_t k_start = 0) const {
//   for (int k = k_start; k <= k_start + num_time_steps_; k++) {
//     double t = (double)(k - k_start) / (double)num_time_steps_;
//     double h = ground_height + pow(t, 1.1) * pow(1 - t, 0.7);
//     gtsam::Point3 goal_point(cp_goal.x(), cp_goal.y(), h);
//     factors->add(pointGoalFactor(name, k, cost_model, goal_point));
//     cp_goal = cp_goal + step;  // imperative
//   }
//   return cp_goal;
// }

void Phase::addpointGoalFactors(gtsam::NonlinearFactorGraph *factors,
                                std::map<std::string, gtsam::Point3> *cp_goals,
                                const std::set<std::string> &all_feet,
                                const gtsam::Point3 &step,
                                const gtsam::SharedNoiseModel &cost_model,
                                const double ground_height, size_t num_steps,
                                size_t k) const {
  // For all feet
  for (auto &&name : all_feet) {
    auto &cp_goal = cp_goals->at(name);
    if (contact_points_.count(name)) {
      auto cp = contact_points_.at(name);
      AddPointGoalFactors(factors, cost_model, cp.point,
                          StanceTrajectory(cp_goal, num_steps), id, k);
    } else {
      AddPointGoalFactors(factors, cost_model, point_com,
                          SimpleSwingTrajectory(cp_goal, step, num_steps), id);
    }
  }
}
}  // namespace gtdynamics
