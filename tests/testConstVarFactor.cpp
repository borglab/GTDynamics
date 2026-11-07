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

// Substitution preserves the base residual and free-variable Jacobian.
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
  gtdynamics::ConstVarFactor const_var_factor(base_factor, fixed_keys);
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
  gtdynamics::ConstVarFactor const_var_factor_all_fixed(base_factor,
                                                        fixed_keys);
  EXPECT(!const_var_factor_all_fixed.checkActive());
}

// Graph substitution removes fully fixed factors and preserves remaining costs.
TEST(ConstVarGraph, error) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;

  // Construct original factor graph.
  auto noise = noiseModel::Unit::Create(6);
  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(1, 0, 0)), noise);
  graph.addPrior<Pose3>(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  graph.addPrior<Pose3>(x2_key, Pose3(Rot3(), Point3(0, 0, 0)));

  // fixed keys
  KeySet fixed_keys;
  fixed_keys.insert(x1_key);
  Values fixed_values;
  fixed_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));

  // Construct const var graph with fixed keys.
  NonlinearFactorGraph new_graph;
  gtdynamics::ConstVarFactors const_var_factors;
  std::tie(new_graph, const_var_factors) =
      gtdynamics::ConstVarGraph(graph, fixed_keys);
  EXPECT_LONGS_EQUAL(2, new_graph.size());
  EXPECT_LONGS_EQUAL(1, const_var_factors.size());

  // Construct const var graph with fixed values.
  new_graph = gtdynamics::ConstVarGraph(graph, fixed_values);
  EXPECT_LONGS_EQUAL(2, new_graph.size());
  Values values;
  values.insert(x2_key, Pose3(Rot3(), Point3(0.5, 0, 0)));
  EXPECT_DOUBLES_EQUAL(0.25, new_graph.error(values), 1e-5);
}

/* ************************************************************************* */
namespace fixed_value_updates {

// Only this factor's fixed keys are copied; partial updates retain other keys.
TEST(ConstVarFactor, LocalKeysAndPartialUpdates) {
  const auto base = std::make_shared<BetweenFactor<Pose3>>(
      1, 2, Pose3(), noiseModel::Unit::Create(6));
  gtdynamics::ConstVarFactor factor(base, KeySet{1, 2, 9999});
  Values trajectory;
  for (Key key = 1; key <= 10000; ++key)
    trajectory.insert(key, Pose3(Rot3(), Point3(key, 0, 0)));
  factor.setFixedValues(trajectory);
  EXPECT_LONGS_EQUAL(2, factor.fixedValues().size());

  Values update;
  const Pose3 replacement(Rot3::Rz(0.2), Point3(3, 4, 5));
  update.insert(1, replacement);
  update.insert(9999, Pose3());
  factor.setFixedValues(update);
  EXPECT(assert_equal(replacement, factor.fixedValues().at<Pose3>(1)));
  EXPECT(
      assert_equal(trajectory.at<Pose3>(2), factor.fixedValues().at<Pose3>(2)));
  EXPECT_LONGS_EQUAL(2, factor.fixedValues().size());
  factor.setFixedValues(Values());
  Values expected;
  expected.insert(1, replacement);
  expected.insert(2, trajectory.at<Pose3>(2));
  EXPECT(assert_equal(base->unwhitenedError(expected),
                      factor.unwhitenedError(Values()), 1e-12));
}

}  // namespace fixed_value_updates
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
