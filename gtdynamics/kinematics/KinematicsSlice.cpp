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

#include <gtdynamics/factors/JointLimitFactor.h>
#include <gtdynamics/factors/PointGoalFactor.h>
#include <gtdynamics/factors/PoseFactor.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/expressions.h>

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

std::ostream& operator<<(std::ostream& os, const PoseGoal& pg) {
  os << "{" << pg.link()->name() << ", [" << pg.comTee << "], [" << pg.goal_pose << "]}";
  return os;
}

void PoseGoal::print(const std::string& s) const {
  std::cout << (s.empty() ? s : s + " ") << *this;
}

bool PoseGoal::satisfied(const gtsam::Values& values, size_t k,
                         double tol) const {
  return gtsam::equal<gtsam::Pose3>(
      Pose(values, link()->id(), k).compose(poseInCoM()), goal_pose, tol);
}

template <>
NonlinearFactorGraph Kinematics::graph<Slice>(const Slice& slice,
                                              const Robot& robot) const {
  NonlinearFactorGraph graph;

  // Constrain kinematics at joints.
  for (auto&& joint : robot.joints()) {
    const auto j = joint->id();
    graph.add(PoseFactor(PoseKey(joint->parent()->id(), slice.k),
                         PoseKey(joint->child()->id(), slice.k),
                         JointAngleKey(j, slice.k), p_.p_cost_model, joint));
  }

  return graph;
}

template <>
EqualityConstraints Kinematics::constraints<Slice>(const Slice& slice,
                                                   const Robot& robot) const {
  EqualityConstraints constraints;

  // Constrain kinematics at joints.
  gtsam::Vector6 tolerance = p_.p_cost_model->sigmas();
  for (auto&& joint : robot.joints()) {
    auto constraint_expr = joint->poseConstraint(slice.k);
    constraints.emplace_shared<VectorExpressionEquality<6>>(constraint_expr,
                                                            tolerance);
  }

  // Constrain fixed links
  for (auto&& link : robot.links()) {
    if (link->isFixed()) {
      using gtsam::Pose3_;

      // Get an expression for the unknown link pose.
      Pose3_ bTcom(PoseKey(link->id(), slice.k));

      // Just make sure it does not move from its original rest pose
      Pose3_ bMcom(link->bMcom());

      // Create expression to calculate the error in tangent space
      auto constraint_expr = gtsam::logmap(bTcom, bMcom);

      // Add constraint
      constraints.emplace_shared<VectorExpressionEquality<6>>(constraint_expr,
                                                              tolerance);
    }
  }

  return constraints;
}

template <>
NonlinearFactorGraph Kinematics::pointGoalObjectives<Slice>(
    const Slice& slice, const ContactGoals& contact_goals) const {
  NonlinearFactorGraph graph;

  // Add objectives.
  for (const ContactGoal& goal : contact_goals) {
    const gtsam::Key pose_key = PoseKey(goal.link()->id(), slice.k);
    graph.emplace_shared<PointGoalFactor>(pose_key, p_.g_cost_model,
                                          goal.contactInCoM(), goal.goal_point);
  }

  return graph;
}

template <>
EqualityConstraints Kinematics::pointGoalConstraints<Slice>(
    const Slice& slice, const ContactGoals& contact_goals) const {
  EqualityConstraints constraints;

  // Add objectives.
  gtsam::Vector3 tolerance = p_.g_cost_model->sigmas();
  for (const ContactGoal& goal : contact_goals) {
    const gtsam::Key pose_key = PoseKey(goal.link()->id(), slice.k);
    auto constraint_expr =
        PointGoalConstraint(pose_key, goal.contactInCoM(), goal.goal_point);
    constraints.emplace_shared<VectorExpressionEquality<3>>(constraint_expr,
                                                            tolerance);
  }
  return constraints;
}

template <>
NonlinearFactorGraph Kinematics::poseGoalObjectives<Slice>(
    const Slice& slice, const PoseGoals& pose_goals) const {
  gtsam::NonlinearFactorGraph graph;

  if (pose_goals.find(slice.k) != pose_goals.end()) {
    auto pose_key = PoseKey(pose_goals.at(slice.k).link()->id(), slice.k);
    const gtsam::Pose3& desired_pose = pose_goals.at(slice.k).sTcom();
    // TODO(toni): use poseprior from unstable gtsam slam or create new
    // factors, to add pose from link7
    graph.addPrior<gtsam::Pose3>(pose_key, desired_pose, p_.p_cost_model);
  }

  return graph;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Slice>(
    const Slice& slice, const Robot& robot, const Values& mean) const {
  NonlinearFactorGraph graph;

  // Minimize the joint angles.
  for (auto&& joint : robot.joints()) {
    const gtsam::Key key = JointAngleKey(joint->id(), slice.k);
    graph.addPrior<double>(key, (mean.exists(key) ? mean.at<double>(key) : 0.0),
                           p_.prior_q_cost_model);
  }

  return graph;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleLimits<Slice>(
    const Slice& slice, const Robot& robot) const {
  NonlinearFactorGraph graph;
  for (auto&& joint : robot.joints()) {
    graph.add(JointLimitFactor(
        JointAngleKey(joint->id(), slice.k), p_.prior_q_cost_model,
        joint->parameters().scalar_limits.value_lower_limit,
        joint->parameters().scalar_limits.value_upper_limit,
        joint->parameters().scalar_limits.value_limit_threshold));
  }
  return graph;
}

template <>
Values Kinematics::initialValues<Slice>(const Slice& slice, const Robot& robot,
                                        double gaussian_noise,
                                        const gtsam::Values& initial_joints,
                                        bool use_fk) const {
  Values values;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize all joint angles.
  for (auto&& joint : robot.joints()) {
    auto key = JointAngleKey(joint->id(), slice.k);
    double angle = initial_joints.exists(key) ? initial_joints.at<double>(key)
                                              : sampler.sample()[0];
    InsertJointAngle(&values, joint->id(), slice.k, angle);
  }

  // Maybe fk takes a long time, so only compute it if there was a given initial
  // joint value in this slice
  if (use_fk) {
    // Initialize poses with fk of initialized values
    auto fk = robot.forwardKinematics(values, slice.k);
    for (auto&& link : robot.links()) {
      InsertPose(&values, link->id(), slice.k,
                 fk.at<gtsam::Pose3>(PoseKey(link->id(), slice.k)));
    }
  } else {
    // Initialize all poses.
    for (auto&& link : robot.links()) {
      const gtsam::Vector6 xi = sampler.sample();
      InsertPose(&values, link->id(), slice.k, link->bMcom().expmap(xi));
    }
  }

  return values;
}

template <>
Values Kinematics::inverse<Slice>(const Slice& slice, const Robot& robot,
                                  const ContactGoals& contact_goals,
                                  bool contact_goals_as_constraints) const {
  // Robot kinematics constraints
  auto constraints = this->constraints(slice, robot);
  NonlinearFactorGraph graph;

  // Contact goals
  if (contact_goals_as_constraints) {
    constraints.add(this->pointGoalConstraints(slice, contact_goals));
  } else {
    graph.add(pointGoalObjectives(slice, contact_goals));
  }

  // Target joint angles.
  graph.add(jointAngleObjectives(slice, robot));

  // TODO(frank): allo pose prior as well.
  // graph.addPrior<gtsam::Pose3>(PoseKey(0, slice.k),
  // gtsam::Pose3(), nullptr);

  auto initial_values = initialValues(slice, robot);

  return optimize(graph, constraints, initial_values);
}

template <>
gtsam::Values Kinematics::inverse<Slice>(
    const Slice& slice, const Robot& robot, const PoseGoals& pose_goals,
    const gtsam::Values& joint_priors) const {
  auto graph = this->graph(slice, robot);

  // Add prior on joint angles to prefer solution close to our initial estimates
  graph.add(this->jointAngleObjectives(slice, robot, joint_priors));

  // Add priors on link poses with desired poses from argument
  graph.add(this->poseGoalObjectives(slice, pose_goals));

  // Add joint angle limits factors
  graph.add(this->jointAngleLimits(slice, robot));

  // Robot kinematics constraints
  auto constraints = this->constraints(slice, robot);

  auto initial_values =
      this->initialValues(slice, robot, 0.1, joint_priors, true);

  return this->optimize(graph, constraints, initial_values);
}
}  // namespace gtdynamics
