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
// TODO(frank): can a templated class store context-dependent data?
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
NonlinearFactorGraph Kinematics<Phase>::graph(const Phase& phase) {
  NonlinearFactorGraph graph;

  // TODO(frank): shared parameter for robot/parameters instead?
  Kinematics<Slice> kinematics_slice(robot_, p_);
  for (const Slice& slice : slices(phase)) {
    graph.add(kinematics_slice.graph(slice));
  }

  return graph;
}

template <>
NonlinearFactorGraph Kinematics<Phase>::pointGoalObjectives(
    const Phase& phase, const ContactGoals& contact_goals) {
  NonlinearFactorGraph graph;

  Kinematics<Slice> kinematics_slice(robot_, p_);
  for (const Slice& slice : slices(phase)) {
    graph.add(kinematics_slice.pointGoalObjectives(slice, contact_goals));
  }

  return graph;
}

template <>
NonlinearFactorGraph Kinematics<Phase>::jointAngleObjectives(
    const Phase& phase) {
  NonlinearFactorGraph graph;

  Kinematics<Slice> kinematics_slice(robot_, p_);
  for (const Slice& slice : slices(phase)) {
    graph.add(kinematics_slice.jointAngleObjectives(slice));
  }

  return graph;
}

template <>
Values Kinematics<Phase>::initialValues(const Phase& phase,
                                        double gaussian_noise) {
  Values values;

  Kinematics<Slice> kinematics_slice(robot_, p_);
  for (const Slice& slice : slices(phase)) {
    values.insert(kinematics_slice.initialValues(slice, gaussian_noise));
  }

  return values;
}

template <>
Values Kinematics<Phase>::inverse(const Phase& phase,
                                  const ContactGoals& contact_goals) {
  auto graph = this->graph(phase);

  // Add objectives.
  graph.add(pointGoalObjectives(phase, contact_goals));
  graph.add(jointAngleObjectives(phase));

  // TODO(frank): allo pose prior as well.
  // graph.addPrior<gtsam::Pose3>(internal::PoseKey(0, phase.k()),
  // gtsam::Pose3(), nullptr);

  auto values = initialValues(phase);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, p_.lm_parameters);
  Values results = optimizer.optimize();
  return results;
}
}  // namespace gtdynamics