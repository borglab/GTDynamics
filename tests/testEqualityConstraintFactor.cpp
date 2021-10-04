/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testEqualityConstraintFactor.cpp
 * @brief Test Equality Constraint Factor.
 * @author: Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/factorTesting.h>

#include "constrainedExample.h"
#include "gtdynamics/optimizer/EqualityConstraintFactor.h"

using namespace gtdynamics;
using namespace gtsam;
using gtsam::assert_equal;
using std::map;
using std::string;


TEST(EqualityConstraint, constructor) {
  using namespace constrained_example;


  // simplest constructor with just expression g(x)
  // with default tolerance and mu = 1, bias=0
  auto constraint1 = EqualityConstraintFactor<double>(cost1_expr, 0.);

  // constructor that also specifies tolerance
  // when used as a factor, implements the soft constraint with noisemodel Diag::tolerance
  Vector tolerance = Vector::Ones(1) * 0.5;
  auto constraint2 = EqualityConstraintFactor<double>(cost1_expr, 0., tolerance);

  // constructor that also allows annealing coefficient mu
  // implements mu||g(x)||_Theta
  double mu = 2;
  auto constraint3 = EqualityConstraintFactor<double>(cost1_expr, 0., tolerance, mu);

  // final constructor
  Vector bias = Vector::Ones(1);
  auto constraint4 = EqualityConstraintFactor<double>(cost1_expr, 0., tolerance, mu, bias);

  Values values;
  values.insert(x1_key, 0.0);
  values.insert(x2_key, 0.0);
  EXPECT(constraint4.feasible(values));
  EXPECT_CORRECT_FACTOR_JACOBIANS(constraint4, values, 1e-7, 1e-5);

  Values values1;
  values1.insert(x1_key, 1.0);
  values1.insert(x2_key, 1.0);
  EXPECT(!constraint4.feasible(values1));
  EXPECT_CORRECT_FACTOR_JACOBIANS(constraint4, values1, 1e-7, 1e-5);
}

// nonlinear factor
// noisemodel factor
// expression factor
// equality constraint factor


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}





