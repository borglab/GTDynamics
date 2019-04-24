/**
 * @file  testToolPoseFactor.cpp
 * @brief test tool goal factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <ToolPoseFactor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

namespace example {
// nosie model
noiseModel::Gaussian::shared_ptr cost_model =
    noiseModel::Gaussian::Covariance(I_6x6);
Key pose_key = Symbol('p', 1);
}  // namespace example

/**
 * Test tool pose factor
 */
TEST(ToolPoseFactor, error) {
  // Create all factors
  Pose3 tTn = Pose3(Rot3(), Point3(-1, 0, 0));
  Pose3 tool_pose = Pose3(Rot3(), Point3(2, 0, 0));

  ToolPoseFactor factor(example::pose_key, example::cost_model, tTn, tool_pose);
  Pose3 pose(Rot3(), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(pose);
  expected_errors << 0, 0, 0, 0, 0, 0;
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));

  // Make sure linearization is correct
  Values values;
  values.insert(example::pose_key, pose);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
