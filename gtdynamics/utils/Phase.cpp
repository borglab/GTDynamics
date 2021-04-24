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

gtsam::NonlinearFactorGraph
Phase::stanceObjectives(const Robot &robot, std::map<string, Point3> cp_goals,
                        const gtsam::SharedNoiseModel &cost_model,
                        size_t k) const {
  gtsam::NonlinearFactorGraph factors;
  for (auto &&kv : contact_points_) {
    const string &name = kv.first;
    const Point3 &cp_goal = cp_goals.at(name);
    const ContactPoint &cp = kv.second;
    AddPointGoalFactors(&factors, cost_model, cp.point,
                        StanceTrajectory(cp_goal, numTimeSteps()),
                        robot.link(name)->id(), k);
  }
  return factors;
}

Matrix Phase::jointValues(const Robot &robot, const gtsam::Values &results,
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
} // namespace gtdynamics
