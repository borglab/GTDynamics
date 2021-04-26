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
  constexpr size_t k_start = 777;
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

// template <>
// NonlinearFactorGraph Kinematics<Phase>::pointGoalObjectives(
//     const ContactGoals& contact_goals) {
//   NonlinearFactorGraph graph;

//   // Add objectives.
//   for (const ContactGoal& goal : contact_goals) {
//     const gtsam::Key pose_key =
//         internal::PoseKey(goal.link()->id(), phase.k());
//     graph.emplace_shared<PointGoalFactor>(
//         pose_key, p_.g_cost_model, goal.contact_in_com(), goal.goal_point);
//   }

//   return graph;
// }

// template <>
// NonlinearFactorGraph Kinematics<Phase>::jointAngleObjectives() {
//   NonlinearFactorGraph graph;

//   // Minimize the joint angles.
//   for (auto&& joint : robot_.joints()) {
//     const gtsam::Key key = internal::JointAngleKey(joint->id(),
//     phase.k()); graph.addPrior<double>(key, 0.0, p_.prior_q_cost_model);
//   }

//   return graph;
// }

// template <>
// Values Kinematics<Phase>::initialValues(double gaussian_noise) {
//   Values values;

//   auto sampler_noise_model =
//       gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
//   gtsam::Sampler sampler(sampler_noise_model);

//   // Initialize all joint angles.
//   for (auto&& joint : robot_.joints()) {
//     InsertJointAngle(&values, joint->id(), phase.k(),
//     sampler.sample()[0]);
//   }

//   // Initialize all poses.
//   for (auto&& link : robot_.links()) {
//     InsertPose(&values, link->id(), phase.k(), link->wTcom());
//   }

//   return values;
// }

// template <>
// Values Kinematics<Phase>::inverse(const ContactGoals& contact_goals) {
//   auto graph = this->graph();

//   // Add objectives.
//   graph.add(pointGoalObjectives(contact_goals));
//   graph.add(jointAngleObjectives());

//   // TODO(frank): allo pose prior as well.
//   // graph.addPrior<gtsam::Pose3>(internal::PoseKey(0, phase.k()),
//   // gtsam::Pose3(), nullptr);

//   auto values = initialValues();

//   gtsam::LevenbergMarquardtOptimizer optimizer(graph, values,
//   p_.lm_parameters); Values results = optimizer.optimize(); return results;
// }
}  // namespace gtdynamics