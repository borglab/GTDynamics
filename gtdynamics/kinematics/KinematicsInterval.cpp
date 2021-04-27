/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsInterval.cpp
 * @brief Kinematics for a interval with fixed contacts.
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
NonlinearFactorGraph Kinematics::graph<Interval>(
    const Interval& interval) const {
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(this->graph(Slice(k)));
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
NonlinearFactorGraph Kinematics::jointAngleObjectives<Interval>(
    const Interval& interval) const {
  NonlinearFactorGraph graph;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    graph.add(jointAngleObjectives(Slice(k)));
  }
  return graph;
}

template <>
Values Kinematics::initialValues<Interval>(const Interval& interval,
                                           double gaussian_noise) const {
  Values values;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    values.insert(initialValues(Slice(k), gaussian_noise));
  }
  return values;
}

template <>
Values Kinematics::inverse<Interval>(const Interval& interval,
                                     const ContactGoals& contact_goals) const {
  Values results;
  for (size_t k = interval.k_start; k <= interval.k_end; k++) {
    results.insert(inverse(Slice(k), contact_goals));
  }
  return results;
}

Values Kinematics::interpolate(const Interval& interval,
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
    result.insert(inverse(Slice(k), goals));
  }
  return result;
}

}  // namespace gtdynamics
