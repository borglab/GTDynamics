/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  WalkCycle.cpp
 * @brief Class to store walk cycle.
 * @author: Disha Das, Varun Agrawal, Frank Dellaert
 */

#include <gtdynamics/factors/ObjectiveFactors.h>
#include <gtdynamics/utils/WalkCycle.h>

namespace gtdynamics {

using gtsam::Point3;
using std::string;

std::ostream &operator<<(std::ostream &os, const WalkCycle &walk_cycle) {
  os << "[\n";
  for (auto &&phase : walk_cycle.phases()) {
    os << phase << ",\n";
  }
  os << "]";
  return os;
}

void WalkCycle::print(const string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

std::map<std::string, Point3> WalkCycle::initContactPointGoal(
    const Robot &robot) const {
  std::map<std::string, Point3> cp_goals;

  // Go over all phases, and all contact points
  for (auto &&phase : phases_) {
    for (auto &&kv : phase.contactPoints()) {
      auto link_name = kv.first;
      // If no goal set yet, add it here
      if (cp_goals.count(link_name) == 0) {
        LinkSharedPtr link = robot.link(link_name);
        auto foot_w = link->wTcom().transformFrom(kv.second.point);
        cp_goals.emplace(link_name, foot_w);
      }
    }
  }

  return cp_goals;
}

std::vector<std::string> WalkCycle::swingLinks(size_t p) const {
  std::vector<std::string> phase_swing_links;
  const Phase &phase = this->phase(p);
  for (auto &&kv : contact_points_) {
    const std::string &name = kv.first;
    if (!phase.hasContact(name)) {
      phase_swing_links.push_back(name);
    }
  }
  return phase_swing_links;
}

gtsam::NonlinearFactorGraph WalkCycle::swingObjectives(
    const Robot &robot, size_t p, std::map<std::string, Point3> cp_goals,
    const Point3 &step, const gtsam::SharedNoiseModel &cost_model,
    double ground_height, size_t k) const {
  const Phase &phase = phases_[p];
  gtsam::NonlinearFactorGraph factors;
  for (auto &&kv : contact_points_) {
    const std::string &name = kv.first;
    const Point3 &cp_goal = cp_goals.at(name);
    if (!phase.hasContact(name)) {
      AddPointGoalFactors(
          &factors, cost_model, kv.second.point,
          SimpleSwingTrajectory(cp_goal, step, phase.numTimeSteps()),
          robot.link(name)->id());
    }
  }
  return factors;
}
}  // namespace gtdynamics
