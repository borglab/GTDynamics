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

#include <algorithm>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::SharedNoiseModel;

void WalkCycle::addPhaseContactPoints(const Phase &phase) {
  // Add unique contact points independently of the specification subtype.
  if (const auto spec = phase.constraintSpec()) {
    for (const auto& point : spec->contactPoints()) {
      if (std::find(contact_points_.begin(), contact_points_.end(), point) ==
          contact_points_.end()) {
        contact_points_.push_back(point);
      }
    }
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

  // Go over all phases, and all contact points
  for (auto &&phase : phases_) {
    if (const auto spec = phase.constraintSpec()) {
      for (const auto& cp : spec->contactPoints()) {
        auto link_name = cp.link->name();
        // If no goal set yet, add it here
        if (cp_goals.count(link_name) == 0) {
          LinkSharedPtr link = robot.link(link_name);
          const Point3 rest_w = link->bMcom() * cp.point;
          // Only ground contacts receive the requested ground height.
          const Point3 goal(rest_w.x(), rest_w.y(),
                            cp.isGround ? ground_height : rest_w.z());
          cp_goals.emplace(link_name, goal);
        }
      }
    }
  }

  return cp_goals;
}

NonlinearFactorGraph WalkCycle::contactPointObjectives(
    const Point3 &step, const SharedNoiseModel &cost_model, size_t k_start,
    ContactPointGoals *cp_goals) const {
  NonlinearFactorGraph factors;

  for (const Phase &phase : phases_) {
    // These stance/swing trajectory generators belong to the walking spec.
    const auto foot_contact_spec =
        std::dynamic_pointer_cast<const FootContactConstraintSpec>(
            phase.constraintSpec());
    if (foot_contact_spec) {
      // Ask the Phase instance to anchor the stance legs
      factors.add(foot_contact_spec->contactPointObjectives(
          contact_points_, step, cost_model, k_start, *cp_goals,
          phase.numTimeSteps()));

      // Update goals for swing legs
      *cp_goals = foot_contact_spec->updateContactPointGoals(contact_points_,
                                                             step, *cp_goals);

      // update the start time step for the next phase
      k_start += phase.numTimeSteps();
    }
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
  if (const auto spec = phases_[p].constraintSpec()) {
    const auto& active = spec->contactPoints();
    for (const auto& point : contact_points_) {
      const bool has_contact = std::any_of(
          active.begin(), active.end(), [&](const PointOnLink& contact) {
            return contact.link->name() == point.link->name();
          });
      if (!has_contact) phase_swing_links.push_back(point.link->name());
    }
  }
  return phase_swing_links;
}

const PointOnLinks WalkCycle::getPhaseContactPoints(size_t p) const {
  if (const auto spec = phases_[p].constraintSpec()) {
    return spec->contactPoints();
  }
  return {};
}

std::vector<PointOnLinks> WalkCycle::allPhasesContactPoints() const {
  std::vector<PointOnLinks> phase_cps;
  for (size_t p = 0; p < phases_.size(); ++p) {
    phase_cps.push_back(getPhaseContactPoints(p));
  }
  return phase_cps;
}

std::vector<PointOnLinks> WalkCycle::transitionContactPoints() const {
  std::vector<PointOnLinks> transitions;
  for (size_t p = 1; p < phases_.size(); ++p) {
    transitions.push_back(getIntersection(getPhaseContactPoints(p - 1),
                                          getPhaseContactPoints(p)));
  }
  return transitions;
}

void WalkCycle::print(const std::string &s) const {
  std::cout << (s.empty() ? s : s + " ") << *this << std::endl;
}

}  // namespace gtdynamics
