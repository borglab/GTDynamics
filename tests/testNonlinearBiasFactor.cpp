/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testNonlinearBiasFactor.cpp
 * @brief Test Nonlinear Bias Factor.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "constrainedExample.h"
#include "gtdynamics/optimizer/NonlinearBiasFactor.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;
using std::map;
using std::string;

TEST(NonlinearBiasFactor, derivative) {
  using namespace constrained_example;
  Values values;
  values.insert(x1_key, -0.2);
  values.insert(x2_key, -0.2);

  auto cost_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  NoiseModelFactor::shared_ptr original_factor = NoiseModelFactor::shared_ptr(new ExpressionFactor<double>(cost_noise, 0., cost1_expr));
  Vector bias = Vector::Ones(1);
  auto bias_factor = NonlinearBiasFactor(original_factor, bias);


  EXPECT_CORRECT_FACTOR_JACOBIANS(bias_factor, values, 1e-7, 1e-5);

}















// // new
// TEST(EqualityConstraint, constructor) {
//   using namespace constrained_example;


//   // simplest constructor with just expression g(x)
//   // with default tolerance and mu = 1, bias=0
//   auto constraint = EqualityConstraint(cost1_expr);

//   // constructor that also specifies tolerance
//   // when used as a factor, implements the soft constraint with noisemodel theta*theta^T
//   double tolerance = 0.1;
//   auto constraint = EqualityConstraint(cost1_expr, tolerance);


//   // constructor that also allows annealing coefficient mu
//   // implements mu||g(x)||_Theta
//   double mu = 2;
//   auto constraint = EqualityConstraint(cost1_expr, tolerance, mu);

//   // final constructor
//   Vector bias = Vector::Ones(1);
//   auto constraint = EqualityConstraint(cost1_expr, tolerance, mu, bias);

//   constraint.feasible(value)

// }

// nonlinear factor
// noisemodel factor
// expression factor
// equality constraint factor


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}





