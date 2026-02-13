/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsInterval.cpp
 * @brief Kinematics for an interval with fixed contacts.
 * @author: Frank Dellaert
 */

#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/ContextUtils.h>
#include <gtdynamics/utils/Interval.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using std::string;
using std::vector;

template <>
NonlinearFactorGraph Kinematics::graph<Interval>(const Interval& interval,
                                                 const Robot& robot) const {
  return collectFactors(
      interval, [&](const Slice& slice) { return this->graph(slice, robot); });
}

template <>
NonlinearFactorGraph Kinematics::qFactors<Interval>(
    const Interval& interval, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points,
    const gtsam::Vector3& gravity) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return qFactors(slice, robot, contact_points, gravity);
  });
}

template <>
NonlinearFactorGraph Kinematics::vFactors<Interval>(
    const Interval& interval, const Robot& robot,
    const std::optional<PointOnLinks>& contact_points) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return vFactors(slice, robot, contact_points);
  });
}

template <>
gtsam::NonlinearEqualityConstraints Kinematics::constraints<Interval>(
    const Interval& interval, const Robot& robot) const {
  gtsam::NonlinearEqualityConstraints constraints;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    auto slice_constraints = this->constraints(Slice(k), robot);
    for (const auto& constraint : slice_constraints) {
      constraints.push_back(constraint);
    }
  }
  return constraints;
}

template <>
NonlinearFactorGraph Kinematics::poseGoalObjectives<Interval>(
    const Interval& interval, const PoseGoals& pose_goals) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return poseGoalObjectives(slice, pose_goals);
  });
}

template <>
NonlinearFactorGraph Kinematics::pointGoalObjectives<Interval>(
    const Interval& interval, const ContactGoals& contact_goals) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return pointGoalObjectives(slice, contact_goals);
  });
}

template <>
gtsam::NonlinearEqualityConstraints Kinematics::pointGoalConstraints<Interval>(
    const Interval& interval, const ContactGoals& contact_goals) const {
  gtsam::NonlinearEqualityConstraints constraints;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    auto slice_constraints = pointGoalConstraints(Slice(k), contact_goals);
    for (const auto& constraint : slice_constraints) {
      constraints.push_back(constraint);
    }
  }
  return constraints;
}

template <>
gtsam::NonlinearEqualityConstraints Kinematics::jointAngleConstraints<Interval>(
    const Interval& interval, const Robot& robot,
    const gtsam::Values& joint_targets) const {
  gtsam::NonlinearEqualityConstraints constraints;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    auto slice_constraints = jointAngleConstraints(Slice(k), robot, joint_targets);
    for (const auto& constraint : slice_constraints) {
      constraints.push_back(constraint);
    }
  }
  return constraints;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Interval>(
    const Interval& interval, const Robot& robot, const Values& mean) const {
  return collectFactors(interval, [&](const Slice& slice) {
    return jointAngleObjectives(slice, robot, mean);
  });
}

template <>
NonlinearFactorGraph Kinematics::jointAngleLimits<Interval>(
    const Interval& interval, const Robot& robot) const {
  return collectFactors(
      interval, [&](const Slice& slice) { return jointAngleLimits(slice, robot); });
}

template <>
Values Kinematics::initialValues<Interval>(const Interval& interval,
                                           const Robot& robot,
                                           double gaussian_noise,
                                           const gtsam::Values& joint_priors,
                                           bool use_fk) const {
  return collectValues(interval, [&](const Slice& slice) {
    return initialValues(slice, robot, gaussian_noise, joint_priors, use_fk);
  });
}

template <>
Values Kinematics::inverse<Interval>(const Interval& interval,
                                     const Robot& robot,
                                     const ContactGoals& contact_goals,
                                     bool contact_goals_as_constraints) const {
  return collectValues(interval, [&](const Slice& slice) {
    return inverse(slice, robot, contact_goals, contact_goals_as_constraints);
  });
}

template <>
Values Kinematics::interpolate<Interval>(
    const Interval& interval, const Robot& robot,
    const ContactGoals& contact_goals1,
    const ContactGoals& contact_goals2) const {
  Values result;
  const double dt = 1.0 / (interval.k_start - interval.k_end);  // 5 6 7 8 9 [10
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    const double t = dt * (k - interval.k_start);
    ContactGoals goals;
    transform(contact_goals1.begin(), contact_goals1.end(),
              contact_goals2.begin(), std::back_inserter(goals),
              [t](const ContactGoal& goal1, const ContactGoal& goal2) {
                return ContactGoal{
                    goal1.point_on_link,
                    (1.0 - t) * goal1.goal_point + t * goal2.goal_point};
              });
    result.insert(inverse(Slice(k), robot, goals));
  }
  return result;
}

}  // namespace gtdynamics
