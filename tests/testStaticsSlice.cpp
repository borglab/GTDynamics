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
#include <gtdynamics/kinematics/Kinematics.h>
#include <gtdynamics/statics/StaticWrenchFactor.h>
#include <gtdynamics/utils/Slice.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

/// Noise models etc specific to Statics class
struct StaticsParameters : public KinematicsParameters {
  boost::optional<gtsam::Vector3> gravity;

  using Isotropic = gtsam::noiseModel::Isotropic;
  const gtsam::SharedNoiseModel fs_cost_model;  // statics cost model

  /// Constructor with default arguments
  StaticsParameters(
      const boost::optional<gtsam::Vector3>& gravity = boost::none)
      : gravity(gravity), fs_cost_model(Isotropic::Sigma(6, 1e-4)) {}
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
   * Create graph with only static balance factors.
   * TODO(frank): if we inherit, should we have *everything below us?
   * @param slice Slice instance.
   */
  gtsam::NonlinearFactorGraph graph(const Slice& slice) {
    gtsam::NonlinearFactorGraph graph;
    const auto k = slice.k;

    for (auto&& link : robot_.links()) {
      int i = link->id();
      if (!link->isFixed()) {
        const auto& connected_joints = link->joints();
        std::vector<DynamicsSymbol> wrench_keys;

        // Add wrench keys for joints.
        for (auto&& joint : connected_joints)
          wrench_keys.push_back(internal::WrenchKey(i, joint->id(), k));

        // add wrench factor for link
        graph.emplace_shared<StaticWrenchFactor>(
            wrench_keys, internal::PoseKey(link->id(), k), p_.fs_cost_model,
            link->mass(), p_.gravity);
      }
    }

    return graph;
  }

  /**
   * Solve for wrenches given kinematics configuration.
   * @param slice Slice instance.
   * @param configuration A known kinematics configuration.
   */
  gtsam::Values solve(const Slice& slice, const gtsam::Values& configuration) {
    gtsam::Values result;

    return result;
  }
};

TEST(Phase, Statics) {
  // Load robot and establish contact/goal pairs
  using namespace contact_goals_example;

  // Create a slice.
  const size_t k = 777;
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
