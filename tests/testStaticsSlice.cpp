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

#include <CppUnitLite/TestHarness.h>
// #include <gtdynamics/statics/Statics.h>
#include <gtdynamics/factors/TorqueFactor.h>             // TODO: move
#include <gtdynamics/factors/WrenchEquivalenceFactor.h>  // TODO: move
#include <gtdynamics/factors/WrenchPlanarFactor.h>       // TODO: move
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/statics/StaticWrenchFactor.h>
#include <gtdynamics/utils/Slice.h>
#include <gtsam/linear/Sampler.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

/// Noise models etc specific to Statics class
struct StaticsParameters : public KinematicsParameters {
  boost::optional<gtsam::Vector3> gravity, planar_axis;

  using Isotropic = gtsam::noiseModel::Isotropic;
  const gtsam::SharedNoiseModel fs_cost_model,  // statics cost model
      f_cost_model,                             // wrench equivalence factor
      t_cost_model,                             // torque factor
      planar_cost_model;                        // planar factor

  /// Constructor with default arguments
  StaticsParameters(
      double sigma_dynamics = 1e-5,
      const boost::optional<gtsam::Vector3>& gravity = boost::none,
      const boost::optional<gtsam::Vector3>& planar_axis = boost::none)
      : gravity(gravity),
        planar_axis(planar_axis),
        fs_cost_model(Isotropic::Sigma(6, 1e-4)),
        f_cost_model(gtsam::noiseModel::Isotropic::Sigma(6, sigma_dynamics)),
        t_cost_model(gtsam::noiseModel::Isotropic::Sigma(1, sigma_dynamics)) {}
};

/// Algorithms for Statics, i.e. kinematics + wrenches at rest
class Statics : public Kinematics {
 protected:
  StaticsParameters p_;  // TODO(frank): stored twice w different type?

 public:
  /**
   * @fn Constructor.
   */
  Statics(const Robot& robot,
          const StaticsParameters& parameters = StaticsParameters())
      : Kinematics(robot, parameters), p_(parameters) {}

  /**
   * Create keys for unkowns and initial values.
   * TODO(frank): if we inherit, should we have *everything below us?
   * @param slice Slice instance.
   */
  gtsam::Values initialValues(const Slice& slice,
                              double gaussian_noise = 1e-6) const {
    gtsam::Values values;
    const auto k = slice.k;

    auto sampler_noise_model =
        gtsam::noiseModel::Isotropic::Sigma(6, gaussian_noise);
    gtsam::Sampler sampler(sampler_noise_model);

    // Initialize link poses.
    for (auto&& link : robot_.links()) {
      int i = link->id();
      const gtsam::Vector6 xi = sampler.sample();
      InsertPose(&values, i, k, link->wTcom().expmap(xi));
    }

    // Initialize wrenches and torques to 0.
    for (auto&& joint : robot_.joints()) {
      int j = joint->id();
      InsertWrench(&values, joint->parent()->id(), j, k, gtsam::Z_6x1);
      InsertWrench(&values, joint->child()->id(), j, k, gtsam::Z_6x1);
      InsertTorque(&values, j, k, 0.0);
    }

    return values;
  }

  /// Graph with a WrenchEquivalenceFactor for each joint
  gtsam::NonlinearFactorGraph wrenchEquivalenceFactors(const Slice& slice) {
    gtsam::NonlinearFactorGraph graph;
    for (auto&& joint : robot_.joints()) {
      graph.emplace_shared<WrenchEquivalenceFactor>(
          p_.f_cost_model, boost::static_pointer_cast<const JointTyped>(joint),
          slice.k);
    }
    return graph;
  }

  /// Graph with a TorqueFactor for each joint
  gtsam::NonlinearFactorGraph torqueFactors(const Slice& slice) {
    gtsam::NonlinearFactorGraph graph;
    for (auto&& joint : robot_.joints()) {
      graph.emplace_shared<TorqueFactor>(
          p_.t_cost_model, boost::static_pointer_cast<const JointTyped>(joint),
          slice.k);
    }
    return graph;
  }

  /// Graph with a WrenchPlanarFactor for each joint
  gtsam::NonlinearFactorGraph wrenchPlanarFactors(const Slice& slice) {
    gtsam::NonlinearFactorGraph graph;
    if (p_.planar_axis)
      for (auto&& joint : robot_.joints()) {
        graph.emplace_shared<WrenchPlanarFactor>(
            p_.planar_cost_model, *p_.planar_axis,
            boost::static_pointer_cast<const JointTyped>(joint), slice.k);
      }
    return graph;
  }

  /**
   * Create graph with only static balance factors.
   * TODO(frank): if we inherit, should we have *everything below us?
   * @param slice Slice instance.
   */
  gtsam::NonlinearFactorGraph graph(const Slice& slice) {
    gtsam::NonlinearFactorGraph graph;
    const auto k = slice.k;

    std::map<uint8_t, LinkSharedPtr> sorted;
    for (auto&& link : robot_.links()) sorted.emplace(link->id(), link);

    for (auto&& kv : sorted) {
      int i = kv.first;
      auto link = kv.second;
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

    return graph;
  }

  /**
   * Solve for wrenches given kinematics configuration.
   * @param slice Slice instance.
   * @param configuration A known kinematics configuration.
   */
  gtsam::Values solve(const Slice& slice, const gtsam::Values& configuration) {
    auto graph = this->graph(slice);
    GTD_PRINT(graph);

    auto values = initialValues(slice);
    GTD_PRINT(values);

    // gtsam::LevenbergMarquardtOptimizer optimizer(graph, values,
    //                                              p_.lm_parameters);
    gtsam::GaussNewtonOptimizer optimizer(graph, values);
    return optimizer.optimize();
  }
};

TEST(Phase, Statics) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;
  robot.print();

  // Create a slice.
  const size_t k = 1;
  const Slice slice(k);

  // Instantiate statics algorithms
  StaticsParameters parameters;
  Statics statics(robot, parameters);

  // Get an inverse kinematics solution
  auto ik_solution = statics.Kinematics::inverse(slice, contact_goals);

  // Now, solve for wrenches
  auto result = statics.solve(slice, ik_solution);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
