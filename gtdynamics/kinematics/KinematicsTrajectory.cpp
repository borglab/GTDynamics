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
#include <gtdynamics/utils/ContextUtils.h>
#include <gtdynamics/utils/Slice.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <algorithm>
#include <map>
#include <stdexcept>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using std::string;
using std::vector;

NonlinearFactorGraph Kinematics::contactHeightObjectives(
    const Trajectory& trajectory, const gtsam::Vector3& gravity,
    double ground_plane_height) const {
  // Union the contacts at shared slices so a persistent contact is not weighted
  // twice, while distinct points on the same link retain their own objectives.
  std::map<size_t, PointOnLinks> contacts_by_slice;
  for (const auto& phase : trajectory.phases()) {
    const auto contacts =
        std::dynamic_pointer_cast<const FootContactConstraintSpec>(
            phase.constraintSpec());
    if (!contacts) {
      throw std::invalid_argument(
          "contactHeightObjectives requires a FootContactConstraintSpec");
    }
    forEachSlice(phase, [&](const Slice& slice) {
      auto& points = contacts_by_slice[slice.k];
      for (const auto& point : contacts->contactPoints()) {
        if (std::find(points.begin(), points.end(), point) == points.end()) {
          points.push_back(point);
        }
      }
    });
  }
  NonlinearFactorGraph graph;
  for (const auto& [k, points] : contacts_by_slice) {
    graph.add(contactHeightObjectives(Slice(k), points, gravity,
                                     ground_plane_height));
  }
  return graph;
}

template <>
NonlinearFactorGraph Kinematics::graph<Trajectory>(const Trajectory& trajectory,
                                                   const Robot& robot) const {
  NonlinearFactorGraph graph;
  for (auto&& phase : trajectory.phases()) {
    graph.add(this->graph<Phase>(phase, robot));
  }
  return graph;
}

template <>
gtsam::NonlinearEqualityConstraints Kinematics::constraints<Trajectory>(
    const Trajectory& trajectory, const Robot& robot) const {
  gtsam::NonlinearEqualityConstraints constraints;
  for (auto&& phase : trajectory.phases()) {
    auto phase_constraints = this->constraints<Interval>(phase, robot);
    for (const auto& constraint : phase_constraints) {
      constraints.push_back(constraint);
    }
  }
  return constraints;
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
gtsam::NonlinearEqualityConstraints Kinematics::pointGoalConstraints<Trajectory>(
    const Trajectory& trajectory, const ContactGoals& contact_goals) const {
  gtsam::NonlinearEqualityConstraints constraints;
  for (auto&& phase : trajectory.phases()) {
    auto phase_constraints =
        pointGoalConstraints<Interval>(phase, contact_goals);
    for (const auto& constraint : phase_constraints) {
      constraints.push_back(constraint);
    }
  }
  return constraints;
}

template <>
gtsam::NonlinearEqualityConstraints Kinematics::jointAngleConstraints<Trajectory>(
    const Trajectory& trajectory, const Robot& robot,
    const Values& joint_targets) const {
  gtsam::NonlinearEqualityConstraints constraints;
  for (auto&& phase : trajectory.phases()) {
    auto phase_constraints =
        jointAngleConstraints<Interval>(phase, robot, joint_targets);
    for (const auto& constraint : phase_constraints) {
      constraints.push_back(constraint);
    }
  }
  return constraints;
}

template <>
NonlinearFactorGraph Kinematics::jointAngleObjectives<Trajectory>(
    const Trajectory& trajectory, const Robot& robot,
    const Values& mean) const {
  NonlinearFactorGraph graph;
  for (auto&& phase : trajectory.phases()) {
    graph.add(jointAngleObjectives<Interval>(phase, robot, mean));
  }
  return graph;
}

template <>
Values Kinematics::initialValues<Trajectory>(
    const Trajectory& trajectory, const Robot& robot, double gaussian_noise,
    const gtsam::Values& initial_joints, bool use_fk) const {
  Values values;
  for (auto&& phase : trajectory.phases()) {
    values.insert(
        initialValues<Interval>(phase, robot, gaussian_noise, initial_joints, use_fk));
  }
  return values;
}

template <>
Values Kinematics::inverse<Trajectory>(
    const Trajectory& trajectory, const Robot& robot,
    const ContactGoals& contact_goals,
    bool contact_goals_as_constraints) const {
  Values results;
  for (auto&& phase : trajectory.phases()) {
    results.insert(inverse<Interval>(phase, robot, contact_goals,
                                     contact_goals_as_constraints));
  }
  return results;
}

template <>
Values Kinematics::interpolate<Trajectory>(
    const Trajectory& trajectory, const Robot& robot,
    const ContactGoals& contact_goals1,
    const ContactGoals& contact_goals2) const {
  Values results;
  for (auto&& phase : trajectory.phases()) {
    results.insert(
        interpolate<Interval>(phase, robot, contact_goals1, contact_goals2));
  }
  return results;
}

}  // namespace gtdynamics
