/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTangentCone.cpp
 * @brief test tangent cone.
 * @author Yetong Zhang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/optimizer/ConvexIQPSolver.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtsam;
using namespace gtdynamics;

/** Test solving
 * argmin ||x-0||^2 + ||y-1||^2
 * s.t.   x-y > 0
 *        x+y > 0
 */
TEST(SolveConvexIQP, example_2D) {

  Key x_key = 1;
  Key y_key = 2;

  GaussianFactorGraph graph;
  auto model = noiseModel::Unit::Create(1);
  graph.add(JacobianFactor(x_key, I_1x1, Vector::Zero(1), model));
  graph.add(JacobianFactor(y_key, I_1x1, Vector1(1), model));

  LinearInequalityConstraints constraints;
  auto factor1 = std::make_shared<JacobianFactor>(x_key, I_1x1, y_key, -I_1x1, Vector::Zero(1), model);
  auto factor2 = std::make_shared<JacobianFactor>(x_key, I_1x1, y_key, I_1x1, Vector::Zero(1), model);
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(factor1);
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(factor2);

  IndexSet init_active_indices;
  VectorValues init_values;
  init_values.insert(x_key, Vector1(0.0));
  init_values.insert(y_key, Vector1(0.0));

  auto [solution, active_indices, num_solves, solve_successful] = SolveConvexIQP(graph, constraints, init_active_indices, init_values);
  VectorValues expected_solution;
  expected_solution.insert(x_key, Vector1(0.5));
  expected_solution.insert(y_key, Vector1(0.5));
  EXPECT(assert_equal(expected_solution, solution));
  IndexSet expected_active_indices;
  expected_active_indices.insert(0);
  EXPECT(assert_container_equality(expected_active_indices, active_indices));
  
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
