/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsInterval.cpp
 * @brief Kinematics for an trajectory with fixed contacts.
 * @author: Frank Dellaert
 */

#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Slice.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using std::string;
using std::vector;

template <>
NonlinearFactorGraph Kinematics::graph<Trajectory>(const Trajectory& trajectory,
                                                   const Robot& robot) const {
  NonlinearFactorGraph graph;
  for (auto&& phase : trajectory.phases()) {
    graph.add(this->graph<Interval>(phase, robot));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::pointGoalObjectives<Trajectory>(
    const Trajectory& trajectory, const ContactGoals& contact_goals) const {
  NonlinearFactorGraph graph;
  for (auto&& phase : trajectory.phases()) {
    graph.add(pointGoalObjectives<Interval>(phase, contact_goals));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Trajectory>(
    const Trajectory& trajectory, const Robot& robot) const {
  NonlinearFactorGraph graph;
  for (auto&& phase : trajectory.phases()) {
    graph.add(jointAngleObjectives<Interval>(phase, robot));
  }
  return graph;
}

template <>
Values Kinematics::initialValues<Trajectory>(const Trajectory& trajectory,
                                             const Robot& robot,
                                             double gaussian_noise) const {
  Values values;
  for (auto&& phase : trajectory.phases()) {
    values.insert(initialValues<Interval>(phase, robot, gaussian_noise));
  }
  return values;
}

template <>
Values Kinematics::inverse<Trajectory>(
    const Trajectory& trajectory, const Robot& robot,
    const ContactGoals& contact_goals) const {
  Values results;
  for (auto&& phase : trajectory.phases()) {
    results.insert(inverse<Interval>(phase, robot, contact_goals));
  }
  return results;
}

template<>
Values Kinematics::interpolate<Trajectory>(const Trajectory& trajectory, const Robot& robot,
                               const ContactGoals& contact_goals1,
                               const ContactGoals& contact_goals2) const {
  Values results;
  for (auto&& phase : trajectory.phases()) {
    results.insert(interpolate<Interval>(phase, robot, contact_goals1, contact_goals2));
  }
  return results;
}

}  // namespace gtdynamics
