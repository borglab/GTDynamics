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

void WalkCycle::addPhase(const Phase &phase) {
  // Add unique PointOnLink objects to contact_points_
  for (auto &&kv : phase.contactPoints()) {
    int link_count =
        std::count_if(contact_points_.begin(), contact_points_.end(),
                      [&](const PointOnLink &contact_point) {
                        return contact_point.point == kv.point &&
                                contact_point.link == kv.link;
                      });
    if (link_count == 0)
      contact_points_.push_back(kv);
  }
  phases_.push_back(phase);
}

const Phase& WalkCycle::phase(size_t p) const {
  if (p >= numPhases()) {
    throw std::invalid_argument("Trajectory:phase: no such phase");
  }
  return phases_.at(p);
}
  
size_t WalkCycle::numTimeSteps() const {
  size_t num_time_steps = 0;
  for (const Phase& p : phases_) num_time_steps += p.numTimeSteps();
  return num_time_steps;
}
  
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

bool hasPoint(const ContactGoals& cp_goals, const PointOnLink &contact_point) {
    auto it = std::find_if(
        cp_goals.begin(), cp_goals.end(),
        [&](const ContactGoal &cp_goal){
          return cp_goal.point_on_link.link->name() == contact_point.link->name();
        });
    return !(it == cp_goals.end());
}

ContactGoals WalkCycle::initContactPointGoal(
  const Robot &robot, const ContactAdjustments &contact_adjustments) const {
  ContactGoals cp_goals;

  // Go over all phases, and all contact points
  for (auto &&phase : phases_) {
    for (auto &&kv : phase.contactPoints()) {
      // If no goal set yet, add it here
      if (!hasPoint(cp_goals, kv)) {
        auto foot_w = kv.link->wTcom().transformFrom(kv.point);

        for (auto &&contact_adjustment : contact_adjustments) {
          foot_w += robot.link(contact_adjustment.link_name)
                        ->wTcom()
                        .transformFrom(contact_adjustment.adjustment);
        }
        cp_goals.push_back(ContactGoal(kv, foot_w));
      }
    }
  }

  return cp_goals;
}

std::vector<string> WalkCycle::swingLinks(size_t p) const {
  std::vector<string> phase_swing_links;
  const Phase &phase = this->phase(p);
  for (auto &&kv : contact_points_) {
    if (!phase.hasContact(kv.link)) {
      phase_swing_links.push_back(kv.link->name());
    }
  }
  return phase_swing_links;
}

NonlinearFactorGraph WalkCycle::contactPointObjectives(
    const Point3 &step, const gtsam::SharedNoiseModel &cost_model,
    size_t k_start, ContactGoals *cp_goals) const {
  NonlinearFactorGraph factors;

  for (const Phase &phase : phases_) {
    // Ask the Phase instance to anchor the stance legs
    factors.add(phase.contactPointObjectives(contact_points_, step, cost_model,
                                             k_start, cp_goals));

    // update the start time step for the next phase
    k_start += phase.numTimeSteps();
  }

  return factors;
}

}  // namespace gtdynamics
