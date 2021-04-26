/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsSlice.cpp
 * @brief Kinematics in single time slice.
 * @author: Frank Dellaert
 */

#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::Values;
using std::map;
using std::string;

std::ostream& operator<<(std::ostream& os, const ContactGoal& cg) {
  os << "{" << cg.point_on_link << ", [" << cg.goal_point.transpose() << "]}";
  return os;
}

void ContactGoal::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this;
}

template <>
NonlinearFactorGraph Kinematics<Slice>::graph() {
  NonlinearFactorGraph graph;

  // Constrain kinematics at joints.
  for (auto&& joint : robot_.joints()) {
    const auto j = joint->id();
    graph.emplace_shared<PoseFactor>(
        internal::PoseKey(joint->parent()->id(), context_.k()),
        internal::PoseKey(joint->child()->id(), context_.k()),
        internal::JointAngleKey(j, context_.k()), p_.p_cost_model, joint);
  }

  return graph;
}

template <>
NonlinearFactorGraph Kinematics<Slice>::pointGoalObjectives(
    const ContactGoals& contact_goals) {
  NonlinearFactorGraph graph;

  // Add objectives.
  for (const ContactGoal& goal : contact_goals) {
    const gtsam::Key pose_key =
        internal::PoseKey(goal.link()->id(), context_.k());
    graph.emplace_shared<PointGoalFactor>(
        pose_key, p_.g_cost_model, goal.contact_in_com(), goal.goal_point);
  }

  return graph;
}

template <>
NonlinearFactorGraph Kinematics<Slice>::jointAngleObjectives() {
  NonlinearFactorGraph graph;

  // Minimize the joint angles.
  for (auto&& joint : robot_.joints()) {
    const gtsam::Key key = internal::JointAngleKey(joint->id(), context_.k());
    graph.addPrior<double>(key, 0.0, p_.prior_q_cost_model);
  }

  return graph;
}

template <>
Values Kinematics<Slice>::initialValues(double gaussian_noise) {
  Values values;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize all joint angles.
  for (auto&& joint : robot_.joints()) {
    InsertJointAngle(&values, joint->id(), context_.k(), sampler.sample()[0]);
  }

  // Initialize all poses.
  for (auto&& link : robot_.links()) {
    InsertPose(&values, link->id(), context_.k(), link->wTcom());
  }

  return values;
}

template <>
Values Kinematics<Slice>::inverse(const ContactGoals& contact_goals) {
  auto graph = this->graph();

  // Add objectives.
  graph.add(pointGoalObjectives(contact_goals));
  graph.add(jointAngleObjectives());

  // TODO(frank): allo pose prior as well.
  // graph.addPrior<gtsam::Pose3>(internal::PoseKey(0, context_.k()),
  // gtsam::Pose3(), nullptr);

  auto values = initialValues();

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, p_.lm_parameters);
  Values results = optimizer.optimize();
  return results;
}
}  // namespace gtdynamics