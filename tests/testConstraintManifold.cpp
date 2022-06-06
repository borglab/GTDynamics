/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testConstraintManifold.cpp
 * @brief Test constraint manifold.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/optimizer/ConstraintManifold.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/slam/BetweenFactor.h>

#include <boost/format.hpp>

using namespace gtsam;
using namespace gtdynamics;

/** Simple example Pose3 with between constraints. */
TEST(ConstraintManifold, connected_poses) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;

  // Constraints.
  gtdynamics::EqualityConstraints constraints;
  auto noise = noiseModel::Unit::Create(6);
  auto factor12 = boost::make_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), noise);
  auto factor23 = boost::make_shared<BetweenFactor<Pose3>>(
      x2_key, x3_key, Pose3(Rot3(), Point3(0, 1, 0)), noise);
  constraints.emplace_shared<gtdynamics::ZeroErrorFactorEquality>(factor12);
  constraints.emplace_shared<gtdynamics::ZeroErrorFactorEquality>(factor23);

  // Create manifold values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  cm_base_values.insert(x2_key, Pose3(Rot3(), Point3(0, 0, 1)));
  cm_base_values.insert(x3_key, Pose3(Rot3(), Point3(0, 1, 1)));

  // Construct manifold.
  auto component = boost::make_shared<ConnectedComponent>(constraints);
  ConstraintManifold manifold(component, cm_base_values);

  // Check recover
  Values values;
  Matrix H_recover_x2, H_recover_x3;
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 0, 1)),
                      manifold.recover<Pose3>(x2_key, H_recover_x2)));
  EXPECT(assert_equal(Pose3(Rot3(), Point3(0, 1, 1)),
                      manifold.recover<Pose3>(x3_key, H_recover_x3)));

  // Check recover jacobian
  std::function<Pose3(const ConstraintManifold&)> x2_recover_func =
      std::bind(&ConstraintManifold::recover<Pose3>, std::placeholders::_1,
                x2_key, boost::none);
  EXPECT(assert_equal(numericalDerivative11<Pose3, ConstraintManifold, 6>(
                          x2_recover_func, manifold),
                      H_recover_x2));

  std::function<Pose3(const ConstraintManifold&)> x3_recover_func =
      std::bind(&ConstraintManifold::recover<Pose3>, std::placeholders::_1,
                x3_key, boost::none);
  EXPECT(assert_equal(numericalDerivative11<Pose3, ConstraintManifold, 6>(
                          x3_recover_func, manifold),
                      H_recover_x3));
}

// TODO: unit test with a kinematics/dynamics example.

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
