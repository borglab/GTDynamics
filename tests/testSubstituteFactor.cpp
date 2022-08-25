/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testSubstituteFactor.cpp
 * @brief test substitute factor.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/manifold/SubstituteFactor.h>

using namespace gtsam;

TEST(SubstituteFactor, pose) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;
  Key x4_key = 4;
  Key cm_key = 2;

  // Constraints.
  gtdynamics::EqualityConstraints constraints;
  auto noise = noiseModel::Unit::Create(3);
  auto factor12 = boost::make_shared<BetweenFactor<Point3>>(
      x1_key, x2_key, Point3(0, 0, 1), noise);
  auto factor23 = boost::make_shared<BetweenFactor<Point3>>(
      x2_key, x3_key, Point3(0, 1, 0), noise);
  constraints.emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor12);
  constraints.emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor23);

  // Create manifold values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Point3(0, 0, 0));
  cm_base_values.insert(x2_key, Point3(0, 0, 1));
  cm_base_values.insert(x3_key, Point3(0, 1, 1));

  // Construct manifold.
  auto component = boost::make_shared<ConnectedComponent>(constraints);
  ConstraintManifold manifold(component, cm_base_values);

  // Construct cost factor.
  auto cost_factor1 = boost::make_shared<BetweenFactor<Point3>>(
      x3_key, x4_key, Point3(1, 0, 0), noise);
  auto cost_factor2 = boost::make_shared<BetweenFactor<Point3>>(
      x1_key, x3_key, Point3(1, 0, 0), noise);

  // Replacement map.
  std::map<Key, Key> replacement_map;
  replacement_map[x1_key] = cm_key;
  replacement_map[x2_key] = cm_key;
  replacement_map[x3_key] = cm_key;

  // Test constructor.
  SubstituteFactor subs_factor1(cost_factor1, replacement_map);
  SubstituteFactor subs_factor2(cost_factor2, replacement_map);

  // Test keys.
  EXPECT(subs_factor1.isReplaced(x3_key));
  EXPECT(!subs_factor1.isReplaced(x4_key));
  EXPECT(subs_factor2.isReplaced(x1_key));
  EXPECT(subs_factor2.isReplaced(x3_key));

  // Construct values for testing.
  Values values;
  values.insert(cm_key, manifold);
  values.insert(x4_key, Point3(2, 2, 2));

  // Check error.
  Vector expected_error1 = (Vector(3) << 1, 1, 1).finished();
  EXPECT(assert_equal(expected_error1, subs_factor1.unwhitenedError(values)));

  Vector expected_error2 = (Vector(3) << -1, 1, 1).finished();
  EXPECT(assert_equal(expected_error2, subs_factor2.unwhitenedError(values)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(subs_factor1, values, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(subs_factor2, values, 1e-7, 1e-5);
}

TEST(SubstituteFactor, fully_constrained_manifold) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;
  Key x4_key = 4;
  Key cm_key = 2;

  // Constraints.
  gtdynamics::EqualityConstraints constraints;
  auto noise = noiseModel::Unit::Create(3);
  auto factor1 =
      boost::make_shared<PriorFactor<Point3>>(x1_key, Point3(0, 0, 0), noise);
  auto factor12 = boost::make_shared<BetweenFactor<Point3>>(
      x1_key, x2_key, Point3(0, 0, 1), noise);
  auto factor23 = boost::make_shared<BetweenFactor<Point3>>(
      x2_key, x3_key, Point3(0, 1, 0), noise);
  constraints.emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor1);
  constraints.emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor12);
  constraints.emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor23);

  // Create manifold values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Point3(0, 0, 0));
  cm_base_values.insert(x2_key, Point3(0, 0, 1));
  cm_base_values.insert(x3_key, Point3(0, 1, 1));

  // Construct manifold.
  auto component = boost::make_shared<ConnectedComponent>(constraints);
  ConstraintManifold manifold(component, cm_base_values);

  // Construct cost factor.
  auto cost_factor1 = boost::make_shared<BetweenFactor<Point3>>(
      x3_key, x4_key, Point3(1, 0, 0), noise);
  auto cost_factor2 = boost::make_shared<BetweenFactor<Point3>>(
      x1_key, x3_key, Point3(1, 0, 0), noise);

  // Replacement map.
  std::map<Key, Key> replacement_map;
  replacement_map[x1_key] = cm_key;
  replacement_map[x2_key] = cm_key;
  replacement_map[x3_key] = cm_key;
  Values fc_manifolds;
  fc_manifolds.insert(cm_key, manifold);

  // Test constructor.
  SubstituteFactor subs_factor1(cost_factor1, replacement_map, fc_manifolds);
  SubstituteFactor subs_factor2(cost_factor2, replacement_map, fc_manifolds);
  EXPECT(!subs_factor2.checkActive());

  // Construct values for testing.
  Values values;
  values.insert(x4_key, Point3(2, 2, 2));

  // Check error.
  Vector expected_error1 = (Vector(3) << 1, 1, 1).finished();
  EXPECT(assert_equal(expected_error1, subs_factor1.unwhitenedError(values)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(subs_factor1, values, 1e-7, 1e-5);
}


  int main() {
    TestResult tr;
    return TestRegistry::runAllTests(tr);
  }
