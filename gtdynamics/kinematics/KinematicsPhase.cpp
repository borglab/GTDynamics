/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsPhase.cpp
 * @brief Kinematics for a phase with fixed contacts.
 * @author: Frank Dellaert
 */

#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Phase.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using std::string;
using std::vector;

// TODO(frank): store slices in Phase?
vector<Slice> slices(const Phase& phase) {
  vector<Slice> slices;
  // TODO(frank): make k_start part of the phase
  constexpr size_t k_start = 0;
  for (size_t k = k_start; k < k_start + phase.numTimeSteps(); k++) {
    slices.push_back(Slice(k));
  }
  return slices;
}

template <>
NonlinearFactorGraph Kinematics::graph<Phase>(const Phase& phase) const {
  NonlinearFactorGraph graph;
  for (const Slice& slice : slices(phase)) {
    graph.add(this->graph(slice));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::pointGoalObjectives<Phase>(
    const Phase& phase, const ContactGoals& contact_goals) const {
  NonlinearFactorGraph graph;
  for (const Slice& slice : slices(phase)) {
    graph.add(pointGoalObjectives(slice, contact_goals));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Phase>(
    const Phase& phase) const {
  NonlinearFactorGraph graph;
  for (const Slice& slice : slices(phase)) {
    graph.add(jointAngleObjectives(slice));
  }
  return graph;
}

template <>
Values Kinematics::initialValues<Phase>(const Phase& phase,
                                        double gaussian_noise) const {
  Values values;
  for (const Slice& slice : slices(phase)) {
    values.insert(initialValues(slice, gaussian_noise));
  }
  return values;
}

template <>
Values Kinematics::inverse<Phase>(const Phase& phase,
                                  const ContactGoals& contact_goals) const {
  Values results;
  for (const Slice& slice : slices(phase)) {
    results.insert(inverse(slice, contact_goals));
  }
  return results;
}
}  // namespace gtdynamics