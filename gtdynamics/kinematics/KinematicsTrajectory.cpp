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
    Interval interval = static_cast<Interval>(phase);
    graph.add(this->graph(interval, robot));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::pointGoalObjectives<Trajectory>(
    const Trajectory& trajectory, const ContactGoals& contact_goals) const {
  NonlinearFactorGraph graph;
  for (size_t k = trajectory.getStartTimeStep(0); k <= trajectory.getEndTimeStep(trajectory.phases().size()-1); k++) {
    graph.add(pointGoalObjectives(Slice(k), contact_goals));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Trajectory>(
    const Trajectory& trajectory, const Robot& robot) const {
  NonlinearFactorGraph graph;
  for (size_t k = trajectory.getStartTimeStep(0); k <= trajectory.getEndTimeStep(trajectory.phases().size()-1); k++) {
    graph.add(jointAngleObjectives(Slice(k), robot));
  }
  return graph;
}

template <>
Values Kinematics::initialValues<Trajectory>(const Trajectory& trajectory,
                                             const Robot& robot,
                                             double gaussian_noise) const {
  Values values;
  for (size_t k = trajectory.getStartTimeStep(0); k <= trajectory.getEndTimeStep(trajectory.phases().size()-1); k++) {
    values.insert(initialValues(Slice(k), robot, gaussian_noise));
  }
  return values;
}

template <>
Values Kinematics::inverse<Trajectory>(
    const Trajectory& trajectory, const Robot& robot,
    const ContactGoals& contact_goals) const {
  Values results;
  for (size_t k = trajectory.getStartTimeStep(0); k <= trajectory.getEndTimeStep(trajectory.phases().size()-1); k++) {
    results.insert(inverse(Slice(k), robot, contact_goals));
  }
  return results;
}

template<>
Values Kinematics::interpolate<Trajectory>(const Trajectory& trajectory, const Robot& robot,
                               const ContactGoals& contact_goals1,
                               const ContactGoals& contact_goals2) const {
  Values result;
  const double dt =
      1.0 / (trajectory.getStartTimeStep(0) - trajectory.getEndTimeStep(trajectory.phases().size()-1));  // 5 6 7 8 9 [10
  for (size_t k = trajectory.getStartTimeStep(0); k <= trajectory.getEndTimeStep(trajectory.phases().size()-1); k++) {
    const double t = dt * (k - trajectory.getStartTimeStep(0));
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
