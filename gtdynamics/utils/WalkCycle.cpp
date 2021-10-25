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


void WalkCycle::addPhaseContactPoints(const Phase &phase) {
  // Add unique PointOnLink objects to contact_points_
  for (auto &&kv : phase.footContactConstraintSpec()->contactPoints()) {
    int link_count =
        std::count_if(contact_points_.begin(), contact_points_.end(),
                      [&](const PointOnLink &contact_point) {
                        return contact_point.point == kv.point &&
                               contact_point.link == kv.link;
                      });
    if (link_count == 0) contact_points_.push_back(kv);
  }
}

const Phase &WalkCycle::phase(size_t p) const {
  if (p >= numPhases()) {
    throw std::invalid_argument("Trajectory:phase: no such phase");
  }
  return phases_.at(p);
}

size_t WalkCycle::numTimeSteps() const {
  size_t num_time_steps = 0;
  for (const Phase &p : phases_) num_time_steps += p.numTimeSteps();
  return num_time_steps;
}

ContactPointGoals WalkCycle::initContactPointGoal(const Robot &robot,
                                                   double ground_height) const {
  ContactPointGoals cp_goals;
  const Point3 adjust(0, 0, -ground_height);

  // Go over all phases, and all contact points
  for (auto &&phase : phases_) {
    for (auto &&cp : phase.footContactConstraintSpec()->contactPoints()) {
      auto link_name = cp.link->name();
      // If no goal set yet, add it here
      if (cp_goals.count(link_name) == 0) {
        LinkSharedPtr link = robot.link(link_name);
        const Point3 foot_w = link->bMcom() * cp.point + adjust;
        cp_goals.emplace(link_name, foot_w);
      }
    }
  }

  return cp_goals;
}

NonlinearFactorGraph WalkCycle::contactPointObjectives(const Point3 &step, const SharedNoiseModel &cost_model,
                                                       size_t k_start, ContactPointGoals *cp_goals) const {
  NonlinearFactorGraph factors;

  for (const Phase &phase : phases_) {
    // Ask the Phase instance to anchor the stance legs
    factors.add(phase.footContactConstraintSpec()->
                contactPointObjectives(contact_points_, step, cost_model,
                                             k_start, *cp_goals, phase.numTimeSteps()));
    // Update goals for swing legs
    *cp_goals = phase.footContactConstraintSpec()->
                            updateContactPointGoals(contact_points_, step, *cp_goals);

    // update the start time step for the next phase
    k_start += phase.numTimeSteps();
  }
  
  return factors;
}

std::ostream &operator<<(std::ostream &os, const WalkCycle &walk_cycle) {
  os << "[\n";
  for (auto &&phase : walk_cycle.phases()) {
    os << phase << ",\n";
  }
  os << "]";
  return os;
}

std::vector<std::string> WalkCycle::getPhaseSwingLinks(size_t p) const {
    std::vector<std::string> phase_swing_links;
    const Phase &phase = phases_[p];
    for (auto &&kv : contact_points_) {
      if (!phase.footContactConstraintSpec()->hasContact(kv.link)) {
        phase_swing_links.push_back(kv.link->name());
    }
  }
  return phase_swing_links;
}

void WalkCycle::print(const string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

}  // namespace gtdynamics
