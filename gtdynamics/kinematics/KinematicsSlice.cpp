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
#include <gtdynamics/kinematics/KinematicsSlice.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Point3;
using gtsam::Values;
using std::map;
using std::string;

NonlinearFactorGraph KinematicsSlice(const Robot& robot,
                                     const KinematicsSettings& opt, size_t k) {
  NonlinearFactorGraph graph;

  // Constrain kinematics at joints.
  for (auto&& joint : robot.joints()) {
    const auto j = joint->id();
    graph.emplace_shared<PoseFactor>(
        internal::PoseKey(joint->parent()->id(), k),
        internal::PoseKey(joint->child()->id(), k),
        internal::JointAngleKey(j, k), opt.p_cost_model, joint);
  }

  return graph;
}

NonlinearFactorGraph PointGoalObjectives(const Robot& robot,
                                         const ContactGoals& contact_goals,
                                         const KinematicsSettings& opt,
                                         size_t k) {
  NonlinearFactorGraph graph;

  // Add objectives.
  for (const ContactGoal& goal : contact_goals) {
    const gtsam::Key pose_key = internal::PoseKey(goal.link()->id(), k);
    graph.emplace_shared<PointGoalFactor>(
        pose_key, opt.g_cost_model, goal.contact_in_com(), goal.goal_point);
  }

  return graph;
}

NonlinearFactorGraph MinimumJointAngleSlice(const Robot& robot,
                                            const KinematicsSettings& opt,
                                            size_t k) {
  NonlinearFactorGraph graph;

  // Minimize the joint angles.
  for (auto&& joint : robot.joints()) {
    const gtsam::Key key = internal::JointAngleKey(joint->id(), k);
    graph.addPrior<double>(key, 0.0, opt.prior_q_cost_model);
  }

  return graph;
}

Values KinematicsSliceInitialValues(const Robot& robot, size_t k,
                                    double gaussian_noise) {
  Values values;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize all joint angles.
  for (auto&& joint : robot.joints()) {
    InsertJointAngle(&values, joint->id(), k, sampler.sample()[0]);
  }

  // Initialize all poses.
  for (auto&& link : robot.links()) {
    InsertPose(&values, link->id(), k, link->wTcom());
  }

  return values;
}

Values InverseKinematics(const Robot& robot, const ContactGoals& contact_goals,
                         const KinematicsSettings& opt, size_t k) {
  auto graph = KinematicsSlice(robot, opt, k);

  // Add objectives.
  graph.add(PointGoalObjectives(robot, contact_goals, opt, k));
  graph.add(MinimumJointAngleSlice(robot, opt, k));

  // TODO(frank): allo pose prior as well.
  // graph.addPrior<gtsam::Pose3>(internal::PoseKey(0, k), gtsam::Pose3(),
  // nullptr);

  auto values = KinematicsSliceInitialValues(robot, k);

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values,
                                               opt.lm_parameters);
  Values results = optimizer.optimize();
  return results;
}
}  // namespace gtdynamics