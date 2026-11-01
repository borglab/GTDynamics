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

#include <gtdynamics/factors/ContactEqualityFactor.h>
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/utils/Slice.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace gtdynamics {

using gtsam::NonlinearFactorGraph;
using gtsam::Values;
using std::string;
using std::vector;

NonlinearFactorGraph Kinematics::imuFactors(
    const Trajectory& trajectory, int link_id,
    const std::vector<gtsam::PreintegratedImuMeasurementsG>& pims,
    gtsam::Key bias_key) const {
  std::set<size_t> indices;
  const auto& phases = trajectory.phases();
  for (size_t i = 0; i < phases.size(); ++i) {
    const auto& phase = phases[i];
    if (phase.k_start > phase.k_end ||
        phase.k_end > static_cast<size_t>(std::numeric_limits<int>::max()) ||
        (i && phase.k_start < phases[i - 1].k_end)) {
      throw std::invalid_argument(
          "imuFactors requires ordered phases with at most one shared "
          "endpoint");
    }
    for (size_t k = phase.k_start; k <= phase.k_end; ++k) indices.insert(k);
  }
  const size_t count = indices.empty() ? 0 : indices.size() - 1;
  if (pims.size() != count) {
    throw std::invalid_argument(
        "imuFactors requires one PIM per consecutive pair of unique slices");
  }
  NonlinearFactorGraph graph;
  auto first = indices.begin();
  for (size_t i = 0; i < count; ++i) {
    const auto second = std::next(first);
    if (!std::isfinite(pims[i].deltaTij()) || pims[i].deltaTij() <= 0) {
      throw std::invalid_argument(
          "imuFactors requires positive finite preintegration durations");
    }
    if (bias_key == PoseKey(link_id, *first) ||
        bias_key == PoseKey(link_id, *second) ||
        bias_key == LinearVelocityKey(link_id, *first) ||
        bias_key == LinearVelocityKey(link_id, *second)) {
      throw std::invalid_argument(
          "imuFactors bias key collides with a pose or velocity key");
    }
    graph.emplace_shared<gtsam::GalileanImuFactor>(
        PoseKey(link_id, *first), LinearVelocityKey(link_id, *first),
        PoseKey(link_id, *second), LinearVelocityKey(link_id, *second),
        bias_key, pims[i]);
    first = second;
  }
  return graph;
}

NonlinearFactorGraph Kinematics::contactEqualityFactors(
    const Trajectory& trajectory, const gtsam::SharedNoiseModel& model) const {
  if (!model || model->dim() != 3) {
    throw std::invalid_argument(
        "contactEqualityFactors requires a 3D noise model");
  }
  // Union contacts at shared phase endpoints before constructing temporal
  // edges.
  std::map<size_t, PointOnLinks> contacts;
  const auto& phases = trajectory.phases();
  for (size_t i = 0; i < phases.size(); ++i) {
    const auto& phase = phases[i];
    if (phase.k_start > phase.k_end ||
        phase.k_end > static_cast<size_t>(std::numeric_limits<int>::max()) ||
        (i && phase.k_start < phases[i - 1].k_end)) {
      throw std::invalid_argument(
          "contactEqualityFactors requires ordered phases "
          "with at most one shared endpoint");
    }
    const auto spec = phase.constraintSpec();
    if (!spec) {
      throw std::invalid_argument(
          "contactEqualityFactors requires contact specifications");
    }
    const auto points = spec->groundContactPoints();
    for (const auto& point : points) {
      if (!point.link || !point.point.allFinite()) {
        throw std::invalid_argument(
            "contactEqualityFactors requires valid contact points");
      }
    }
    for (size_t k = phase.k_start; k <= phase.k_end; ++k) {
      auto& current = contacts[k];
      for (const auto& point : points) {
        if (std::find(current.begin(), current.end(), point) == current.end()) {
          current.push_back(point);
        }
      }
    }
  }
  NonlinearFactorGraph graph;
  for (auto current = contacts.begin(); current != contacts.end(); ++current) {
    if (current == contacts.begin()) continue;
    const auto previous = std::prev(current);
    if (current->first != previous->first + 1) continue;
    for (const auto& point : current->second) {
      if (std::find(previous->second.begin(), previous->second.end(), point) !=
          previous->second.end()) {
        graph.emplace_shared<ContactEqualityFactor>(
            point, model, previous->first, current->first);
      }
    }
  }
  return graph;
}

NonlinearFactorGraph Kinematics::contactHeightObjectives(
    const Trajectory& trajectory, const gtsam::Vector3& gravity,
    double ground_plane_height) const {
  NonlinearFactorGraph graph;
  for (const auto& phase : trajectory.phases()) {
    graph.add(contactHeightObjectives(phase, gravity, ground_plane_height));
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
