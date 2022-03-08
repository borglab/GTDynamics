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
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(this->graph(Slice(k), robot));
  }
  return graph;
}

template <>
EqualityConstraints Kinematics::constraints<Interval>(
    const Interval& interval, const Robot& robot) const {
  EqualityConstraints constraints;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    constraints.add(this->constraints(Slice(k), robot));
  }
  return constraints;
}

template <>
NonlinearFactorGraph Kinematics::poseGoalObjectives<Interval>(
    const Interval& interval, const Robot& robot,
    const gtsam::Values& goal_poses) const {
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(poseGoalObjectives(Slice(k), robot, goal_poses));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::pointGoalObjectives<Interval>(
    const Interval& interval, const ContactGoals& contact_goals) const {
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(pointGoalObjectives(Slice(k), contact_goals));
  }
  return graph;
}

template <>
EqualityConstraints Kinematics::pointGoalConstraints<Interval>(
    const Interval& interval, const ContactGoals& contact_goals) const {
  EqualityConstraints constraints;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    constraints.add(pointGoalConstraints(Slice(k), contact_goals));
  }
  return constraints;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Interval>(
    const Interval& interval, const Robot& robot, const Values& mean) const {
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(jointAngleObjectives(Slice(k), robot, mean));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleLimits<Interval>(
    const Interval& interval, const Robot& robot) const {
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(jointAngleLimits(Slice(k), robot));
  }
  return graph;
}

template <>
Values Kinematics::initialValues<Interval>(
    const Interval& interval, const Robot& robot, double gaussian_noise,
    const gtsam::Values& joint_priors, bool use_fk) const {
  Values values;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    values.insert(initialValues(Slice(k), robot, gaussian_noise, joint_priors, use_fk));
  }
  return values;
}

template <>
Values Kinematics::inverse<Interval>(const Interval& interval,
                                     const Robot& robot,
                                     const ContactGoals& contact_goals,
                                     bool contact_goals_as_constraints) const {
  Values results;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    results.insert(
        inverse(Slice(k), robot, contact_goals, contact_goals_as_constraints));
  }
  return results;
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
