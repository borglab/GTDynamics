/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsSlice.cpp
 * @brief Kinematics in a single time slice.
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
using gtsam::Values;
using std::string;

std::ostream& operator<<(std::ostream& os, const ContactGoal& cg) {
  os << "{" << cg.point_on_link << ", [" << cg.goal_point.transpose() << "]}";
  return os;
}

void ContactGoal::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this;
}

template <>
NonlinearFactorGraph Kinematics::graph<Slice>(const Slice& slice,
                                              const Robot& robot) const {
  NonlinearFactorGraph graph;

  // Constrain kinematics at joints.
  for (auto&& joint : robot.joints()) {
    const auto j = joint->id();
    graph.emplace_shared<PoseFactor>(p_->p_cost_model, joint, slice.k);
  }

  return graph;
}

template <>
NonlinearFactorGraph Kinematics::pointGoalObjectives<Slice>(
    const Slice& slice, const ContactGoals& contact_goals) const {
  NonlinearFactorGraph graph;

  // Add objectives.
  for (const ContactGoal& goal : contact_goals) {
    const gtsam::Key pose_key = internal::PoseKey(goal.link()->id(), slice.k);
    graph.emplace_shared<PointGoalFactor>(pose_key, p_->g_cost_model,
                                          goal.contactInCoM(), goal.goal_point);
  }

  return graph;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Slice>(
    const Slice& slice, const Robot& robot) const {
  NonlinearFactorGraph graph;

  // Minimize the joint angles.
  for (auto&& joint : robot.joints()) {
    const gtsam::Key key = internal::JointAngleKey(joint->id(), slice.k);
    graph.addPrior<double>(key, 0.0, p_->prior_q_cost_model);
  }

  return graph;
}

template <>
Values Kinematics::initialValues<Slice>(const Slice& slice, const Robot& robot,
                                        double gaussian_noise) const {
  Values values;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize all joint angles.
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&values, joint->id(), slice.k, sampler.sample()[0]);
  }

  // Initialize all poses.
  for (auto&& link : robot.links()) {
    const gtsam::Vector6 xi = sampler.sample();
    InsertPose(&values, link->id(), slice.k, link->bMcom().expmap(xi));
  }

  return values;
}

template <>
Values Kinematics::inverse<Slice>(const Slice& slice, const Robot& robot,
                                  const ContactGoals& contact_goals) const {
  auto graph = this->graph(slice, robot);

  // Add objectives.
  graph.add(pointGoalObjectives(slice, contact_goals));
  graph.add(jointAngleObjectives(slice, robot));

  // TODO(frank): allo pose prior as well.
  // graph.addPrior<gtsam::Pose3>(internal::PoseKey(0, slice.k),
  // gtsam::Pose3(), nullptr);

  auto initial_values = initialValues(slice, robot);

  return optimize(graph, initial_values);
}
}  // namespace gtdynamics
