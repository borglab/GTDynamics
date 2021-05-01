/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testStaticsSlice.cpp
 * @brief Test Statics in single time slice.
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
    const Slice& slice) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot_.joints()) {
    graph.emplace_shared<WrenchEquivalenceFactor>(
        p_.f_cost_model, boost::static_pointer_cast<const JointTyped>(joint),
        slice.k);
  }
  return graph;
}

gtsam::NonlinearFactorGraph Statics::torqueFactors(const Slice& slice) const {
  gtsam::NonlinearFactorGraph graph;
  for (auto&& joint : robot_.joints()) {
    graph.emplace_shared<TorqueFactor>(
        p_.t_cost_model, boost::static_pointer_cast<const JointTyped>(joint),
        slice.k);
  }
  return graph;
}

gtsam::NonlinearFactorGraph Statics::wrenchPlanarFactors(
    const Slice& slice) const {
  gtsam::NonlinearFactorGraph graph;
  if (p_.planar_axis)
    for (auto&& joint : robot_.joints()) {
      graph.emplace_shared<WrenchPlanarFactor>(
          p_.planar_cost_model, *p_.planar_axis,
          boost::static_pointer_cast<const JointTyped>(joint), slice.k);
    }
  return graph;
}

gtsam::NonlinearFactorGraph Statics::graph(const Slice& slice) const {
  gtsam::NonlinearFactorGraph graph;
  const auto k = slice.k;

  // Add static wrench factors for all links
  for (auto&& link : robot_.links()) {
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
  graph.add(wrenchEquivalenceFactors(slice));

  /// Add a TorqueFactor for each joint.
  graph.add(torqueFactors(slice));

  /// Add a WrenchPlanarFactor for each joint.
  graph.add(wrenchPlanarFactors(slice));

  return graph;
}

gtsam::Values Statics::initialValues(const Slice& slice,
                                     double gaussian_noise) const {
  gtsam::Values values;
  const auto k = slice.k;

  auto sampler_noise_model =
      gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
  gtsam::Sampler sampler(sampler_noise_model);

  // Initialize wrenches and torques to 0.
  for (auto&& joint : robot_.joints()) {
    int j = joint->id();
    InsertWrench(&values, joint->parent()->id(), j, k, gtsam::Z_6x1);
    InsertWrench(&values, joint->child()->id(), j, k, gtsam::Z_6x1);
    InsertTorque(&values, j, k, 0.0);
  }

  return values;
}

gtsam::Values Statics::solve(const Slice& slice,
                             const gtsam::Values& configuration) const {
  auto graph = this->graph(slice);
  gtsam::Values values;
  values.insert(initialValues(slice));

  // In this function we assume the kinematics is given, and we add priors to
  // the graph to enforce this. Would be much nicer with constant expressions.

  // Add constraints for poses and initialize them.
  for (auto&& link : robot_.links()) {
    auto key = internal::PoseKey(link->id(), slice.k);
    auto pose = configuration.at<Pose3>(key);
    graph.emplace_shared<gtsam::NonlinearEquality1<Pose3>>(pose, key);
    values.insert(key, pose);
  }

  // Add constriants for joint angles and initialize them.
  for (auto&& joint : robot_.joints()) {
    auto key = internal::JointAngleKey(joint->id(), slice.k);
    auto q = configuration.at<double>(key);
    graph.emplace_shared<gtsam::NonlinearEquality1<double>>(q, key);
    values.insert(key, q);
  }

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, p_.lm_parameters);
  return optimizer.optimize();
}

gtsam::Values Statics::minimizeTorques(const Slice& slice) const {
  auto graph = this->graph(slice);

  auto values = Kinematics::initialValues(slice);
  values.insert(initialValues(slice));

  // TODO(frank): make IPOPT optimizer base class.
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, values, p_.lm_parameters);
  return optimizer.optimize();
}
}  // namespace gtdynamics