/**
 * @file  testBasePoseFactor.cpp
 * @brief test forward kinematics factor
 * @Author: Frank Dellaert and Mandy Xie
 */
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <BasePoseFactor.h>
#include <DHLink.h>

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
 * Test twist factor for stationary case
 */
TEST(BasePoseFactor, error) {
  // Create factor
  Pose3 base_pose = Pose3(Rot3::Rz(M_PI / 2), Point3(1, 0, 0));
  BasePoseFactor factor(example::pose_key, example::cost_model, base_pose);
  Pose3 pose = Pose3(Rot3::Rz(M_PI / 2), Point3(1, 0, 0));
  Vector6 actual_errors, expected_errors;
  Matrix actual_H, expected_H;

  actual_errors = factor.evaluateError(pose, actual_H);
  expected_errors << 0, 0, 0, 0, 0, 0;
  expected_H = numericalDerivative11(
      boost::function<Vector(const Pose3 &)>(
          boost::bind(&BasePoseFactor::evaluateError, factor, _1, boost::none)),
      pose, 1e-6);
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  EXPECT(assert_equal(expected_H, actual_H, 1e-6));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
