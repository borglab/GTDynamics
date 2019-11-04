/**
 *  @file testGaussianProcessPriorFactor.cpp
 *  @test for Gaussian process prior linear factor for joint angle, joint
 *   velocity, and joint acceleration
 *  @author Mandy Xie
 **/
#include <GaussianProcessPriorFactor.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>

#include <CppUnitLite/TestHarness.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace manipulator;

TEST(GaussianProcessPriorFactor, Factor) {
  const double delta_t = 0.1;
  Matrix Qc = I_1x1;
  noiseModel::Gaussian::shared_ptr Qc_model =
      noiseModel::Gaussian::Covariance(Qc);
  Key q1_key = Symbol('x', 1), q2_key = Symbol('x', 2);
  Key qVel1_key = Symbol('v', 1), qVel2_key = Symbol('v', 2);
  Key qAccel1_key = Symbol('a', 1), qAccel2_key = Symbol('a', 2);
  GaussianProcessPriorFactor factor(q1_key, qVel1_key, qAccel1_key, q2_key,
                                    qVel2_key, qAccel2_key, Qc_model, delta_t);
  double q1 = 0, qVel1 = 0, qAccel1 = 0, q2 = 0, qVel2 = 0, qAccel2 = 0;
  Vector3 actual_errors, expected_errors;

  actual_errors = factor.evaluateError(q1, qVel1, qAccel1, q2, qVel2, qAccel2);
  expected_errors.setZero();

  EXPECT(assert_equal(expected_errors, actual_errors, 1e-6));
  // Make sure linearization is correct
  Values values;
  values.insert(q1_key, q1);
  values.insert(qVel1_key, qVel1);
  values.insert(qAccel1_key, qAccel1);
  values.insert(q2_key, q2);
  values.insert(qVel2_key, qVel2);
  values.insert(qAccel2_key, qAccel2);
  double diffDelta = 1e-7;
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, diffDelta, 1e-3);
}

/* main function */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
