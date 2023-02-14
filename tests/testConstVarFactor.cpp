/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testConstVarFactor.cpp
 * @brief test const variable factor.
 * @author Yetong Zhang
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/factors/ConstVarFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace gtsam;

TEST(ConstVarFactor, errorAndJacobian) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;

  // Construct original factor.
  auto noise = noiseModel::Unit::Create(6);
  auto base_factor = std::make_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(1, 0, 0)), noise);

  // Fixed variables.
  KeySet fixed_keys;
  fixed_keys.insert(x1_key);
  Values fixed_values;
  fixed_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));

  // Test constructor.
  ConstVarFactor const_var_factor(base_factor, fixed_keys);
  const_var_factor.setFixedValues(fixed_values);
  EXPECT(const_var_factor.checkActive());

  // Construct values for testing.
  Values values;
  values.insert(x2_key, Pose3(Rot3(), Point3(1, 0, 0)));

  // Check error.
  Vector expected_error1 = Vector::Zero(6);
  EXPECT(
      assert_equal(expected_error1, const_var_factor.unwhitenedError(values)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(const_var_factor, values, 1e-7, 1e-5);

  // A factor with all variables fixed should not be active.
  fixed_keys.insert(x2_key);
  ConstVarFactor const_var_factor_all_fixed(base_factor, fixed_keys);
  EXPECT(!const_var_factor_all_fixed.checkActive());
}

TEST(ConstVarGraph, error) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;

  // Construct original factor graph.
  auto noise = noiseModel::Unit::Create(6);
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Pose3>>(x1_key, x2_key, Pose3(Rot3(), Point3(1, 0, 0)), noise);
  graph.addPrior<Pose3>(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  graph.addPrior<Pose3>(x2_key, Pose3(Rot3(), Point3(0, 0, 0)));

  // fixed keys
  KeySet fixed_keys;
  fixed_keys.insert(x1_key);
  Values fixed_values;
  fixed_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));

  // Construct const var graph with fixed keys.
  NonlinearFactorGraph new_graph;
  ConstVarFactors const_var_factors;
  std::tie(new_graph, const_var_factors) = ConstVarGraph(graph, fixed_keys);
  EXPECT_LONGS_EQUAL(2, new_graph.size());
  EXPECT_LONGS_EQUAL(1, const_var_factors.size());

  // Construct const var graph with fixed values.
  new_graph = ConstVarGraph(graph, fixed_values);
  EXPECT_LONGS_EQUAL(2, new_graph.size());
  Values values;
  values.insert(x2_key, Pose3(Rot3(), Point3(0.5, 0, 0)));
  EXPECT_DOUBLES_EQUAL(0.25, new_graph.error(values), 1e-5);
}


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
