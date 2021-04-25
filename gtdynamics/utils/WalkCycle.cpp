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

using gtsam::NonlinearFactorGraph;
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

std::map<string, Point3> WalkCycle::initContactPointGoal(
    const Robot &robot) const {
  std::map<string, Point3> cp_goals;

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

std::vector<string> WalkCycle::swingLinks(size_t p) const {
  std::vector<string> phase_swing_links;
  const Phase &phase = this->phase(p);
  for (auto &&kv : contact_points_) {
    const string &name = kv.first;
    if (!phase.hasContact(name)) {
      phase_swing_links.push_back(name);
    }
  }
  return phase_swing_links;
}

NonlinearFactorGraph WalkCycle::contactPointObjectives(
    const Point3 &step, const gtsam::SharedNoiseModel &cost_model,
    const Robot &robot, size_t k_start,
    std::map<string, Point3> *cp_goals) const {
  NonlinearFactorGraph factors;

  for (const Phase &phase : phases_) {
    // Ask the Phase instance to anchor the stance legs
    factors.add(phase.contactPointObjectives(contact_points_, step, cost_model,
                                            robot, k_start, cp_goals));

    // update the start time step for the next phase
    k_start += phase.numTimeSteps();
  }

  return factors;
}

}  // namespace gtdynamics
