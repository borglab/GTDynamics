/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  KinematicsPhase.cpp
 * @brief Kinematics for a Phase with fixed contacts.
 * @author: Dan Barladeanu, Frank Dellaert
 */

#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Phase.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using std::string;
using std::vector;

template <>
NonlinearFactorGraph Kinematics::graph<Phase>(const Phase& phase,
                                                 const Robot& robot) const {
  NonlinearFactorGraph graph;
  Interval interval = static_cast<Interval>(phase);
  graph.add(this->graph(interval, robot));
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::pointGoalObjectives<Phase>(
    const Phase& phase, const ContactGoals& contact_goals) const {
  NonlinearFactorGraph graph;
  Interval interval = static_cast<Interval>(phase);
  graph.add(pointGoalObjectives(interval, contact_goals));
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Phase>(
    const Phase& phase, const Robot& robot) const {
  NonlinearFactorGraph graph;
  Interval interval = static_cast<Interval>(phase);
  graph.add(jointAngleObjectives(interval, robot));
  return graph;
}

template <>
Values Kinematics::initialValues<Phase>(const Phase& phase,
                                           const Robot& robot,
                                           double gaussian_noise) const {
  Values values;
  Interval interval = static_cast<Interval>(phase);
  values.insert(initialValues(interval, robot, gaussian_noise));
  return values;
}

template <>
Values Kinematics::inverse<Phase>(const Phase& phase,
                                     const Robot& robot,
                                     const ContactGoals& contact_goals) const {
  Values results;
  Interval interval = static_cast<Interval>(phase);
  results.insert(inverse(interval, robot, contact_goals));
  return results;
}

template <>
Values Kinematics::interpolate<Phase>(const Phase& phase, const Robot& robot,
                                         const ContactGoals& contact_goals1,
                                         const ContactGoals& contact_goals2) const {
  Values result;
  Interval interval = static_cast<Interval>(phase);
  result = interpolate(interval, robot, contact_goals1, contact_goals2);
  return result;
}

}  // namespace gtdynamics
