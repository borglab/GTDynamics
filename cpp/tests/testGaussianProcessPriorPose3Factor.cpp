/**
 *  @file testGaussianProcessPriorPose3Factor.cpp
 *  @test for Gaussian process prior linear factor for joint angle, joint
 *   velocity, and joint acceleration
 *  @author Mandy Xie
 **/

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>

#include <GaussianProcessPriorPose3Factor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

TEST(GaussianProcessPriorPose3Factor, Factor) {
  const double delta_t = 0.1;
  Matrix Qc = I_6x6;
  noiseModel::Gaussian::shared_ptr Qc_model =
      noiseModel::Gaussian::Covariance(Qc);
  Key p1_key = Symbol('p', 1), p2_key = Symbol('p', 2);
  Key v1_key = Symbol('v', 1), v2_key = Symbol('v', 2);
  Key v1dot_key = Symbol('a', 1), v2dot_key = Symbol('a', 2);
  GaussianProcessPriorPose3Factor factor(p1_key, v1_key, v1dot_key, p2_key,
                                         v2_key, v2dot_key, Qc_model, delta_t);
  Pose3 p1, p2;
  Vector6 v1, v1dot, v2, v2dot;
  v1.setZero(), v1dot.setZero(), v2.setZero(), v2dot.setZero();
  Vector actual_errors, expected_errors;

  actual_errors = factor.evaluateError(p1, v1, v1dot, p2, v2, v2dot);
  expected_errors = Vector(18).setZero();
  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(p1_key, p1);
  values.insert(v1_key, v1);
  values.insert(v1dot_key, v1dot);
  values.insert(p2_key, p2);
  values.insert(v2_key, v2);
  values.insert(v2dot_key, v2dot);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
