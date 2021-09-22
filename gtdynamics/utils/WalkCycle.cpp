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

}  // namespace gtdynamics
