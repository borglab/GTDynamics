/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  StaticsSlice.cpp
 * @brief Statics in single time slice.
 * @author: Frank Dellaert
 */

#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>

#include "gtdynamics/factors/TorqueFactor.h"             // TODO: move
#include "gtdynamics/factors/WrenchEquivalenceFactor.h"  // TODO: move
#include "gtdynamics/factors/WrenchPlanarFactor.h"       // TODO: move
#include "gtdynamics/statics/StaticWrenchFactor.h"
#include "gtdynamics/statics/Statics.h"

namespace gtdynamics {
using gtsam::assert_equal;
using gtsam::Point3;
using gtsam::Pose3;
using std::map;
using std::string;

gtsam::NonlinearFactorGraph Statics::wrenchEquivalenceFactors(
    const Slice& slice, const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot.joints()) {
    graph.add(WrenchEquivalenceFactor(p_.f_cost_model, joint, slice.k));
  }
  return graph;
}

gtsam::NonlinearFactorGraph Statics::torqueFactors(const Slice& slice,
                                                   const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot.joints()) {
    graph.add(TorqueFactor(p_.t_cost_model, joint, slice.k));
  }
  return graph;
}

gtsam::NonlinearFactorGraph Statics::wrenchPlanarFactors(
    const Slice& slice, const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  if (p_.planar_axis)
    for (auto&& joint : robot.joints()) {
      graph.add(WrenchPlanarFactor(p_.planar_cost_model, *p_.planar_axis, joint,
                                   slice.k));
    }
  return graph;
}

gtsam::NonlinearFactorGraph Statics::graph(const Slice& slice,
                                           const Robot& robot) const {
  gtsam::NonlinearFactorGraph graph;
  const auto k = slice.k;

  // Add static wrench factors for all links
  for (auto&& link : robot.links()) {
    int i = link->id();
    if (link->isFixed()) continue;
    const auto& connected_joints = link->joints();
    std::vector<DynamicsSymbol> wrench_keys;

    // Add wrench keys for joints.
    for (auto&& joint : connected_joints)
      wrench_keys.push_back(internal::WrenchKey(i, joint->id(), k));

    // Add static wrench factor for link.
    graph.emplace_shared<StaticWrenchFactor>(
        wrench_keys, internal::PoseKey(link->id(), k), p_.fs_cost_model,
        link->mass(), p_.gravity);
  }

  /// Add a WrenchEquivalenceFactor for each joint.
  graph.add(wrenchEquivalenceFactors(slice, robot));

  /// Add a TorqueFactor for each joint.
  graph.add(torqueFactors(slice, robot));

  /// Add a WrenchPlanarFactor for each joint.
  graph.add(wrenchPlanarFactors(slice, robot));

  return graph;
}

gtsam::Values Statics::initialValues(const Slice& slice, const Robot& robot,
                                     double gaussian_noise) const {
  gtsam::Values values;
  const auto k = slice.k;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize wrenches and torques to 0.
  for (auto&& joint : robot.joints()) {
    int j = joint->id();
    InsertWrench(&values, joint->parent()->id(), j, k, gtsam::Z_6x1);
    InsertWrench(&values, joint->child()->id(), j, k, gtsam::Z_6x1);
    InsertTorque(&values, j, k, 0.0);
  }

  return values;
}

gtsam::Values Statics::solve(const Slice& slice, const Robot& robot,
                             const gtsam::Values& configuration) const {
  auto graph = this->graph(slice, robot);
  gtsam::Values initial_values;
  initial_values.insert(initialValues(slice, robot));

  // In this function we assume the kinematics is given, and we add priors to
  // the graph to enforce this. Would be much nicer with constant expressions.

  // Add constraints for poses and initialize them.
  for (auto&& link : robot.links()) {
    auto key = internal::PoseKey(link->id(), slice.k);
    auto pose = configuration.at<Pose3>(key);
    graph.emplace_shared<gtsam::NonlinearEquality1<Pose3>>(pose, key);
    initial_values.insert(key, pose);
  }

  // Add constraints for joint angles and initialize them.
  for (auto&& joint : robot.joints()) {
    auto key = internal::JointAngleKey(joint->id(), slice.k);
    auto q = configuration.at<double>(key);
    graph.emplace_shared<gtsam::NonlinearEquality1<double>>(q, key);
    initial_values.insert(key, q);
  }

  return optimize(graph, initial_values);
}

gtsam::Values Statics::minimizeTorques(const Slice& slice,
                                       const Robot& robot) const {
  auto graph = this->graph(slice, robot);

  auto values = Kinematics::initialValues(slice, robot);
  values.insert(initialValues(slice, robot));

  // TODO(frank): make IPOPT optimizer base class.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, p_.lm_parameters);
  return optimizer.optimize();
}
}  // namespace gtdynamics