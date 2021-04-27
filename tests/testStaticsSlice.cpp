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
#include <gtdynamics/utils/Slice.h>

#include "contactGoalsExample.h"

using namespace gtdynamics;
using gtsam::assert_equal;
using gtsam::Point3;
using std::map;
using std::string;

/// Noise models etc specific to Statics class
struct StaticsParameters : public KinematicsParameters {
  using Isotropic = gtsam::noiseModel::Isotropic;
  StaticsParameters() {}
};

/// Algorithms for Statics, i.e. kinematics + wrenches at rest
class Statics : public Kinematics {
  Robot robot_;
  StaticsParameters p_;

 public:
  /**
   * @fn Constructor.
   */
  Statics(const Robot& robot,
          const StaticsParameters& parameters = StaticsParameters())
      : Kinematics(robot, parameters) {}

  /**
   * Solve for wrenches given kinematics configuration.
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
