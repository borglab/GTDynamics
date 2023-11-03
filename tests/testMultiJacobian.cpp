/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testMultiJacobian.cpp
 * @brief Test jacobian w.r.t. multiple variables.
 * @author Yetong Zhang
 */


#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/manifold/MultiJacobian.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianFactorGraph.h>

using namespace gtsam;

// Test add and mult operators for MultiJacobian
TEST(MultiJacobian, Add_Mult) {
  Key x1 = 1;
  Key x2 = 2;
  Key x3 = 3;
  MultiJacobian jac1, jac2;
  Matrix H1_x1 = (Matrix(2, 2) << 1, 2, 3, 4).finished();
  Matrix H1_x2 = (Matrix(2, 1) << 3, 2).finished();
  Matrix H2_x1 = (Matrix(2, 2) << 2, 2, 3, 3).finished();
  Matrix H2_x3 = (Matrix(2, 2) << 4, 3, 2, 1).finished();
  jac1.insert({x1, H1_x1});
  jac1.insert({x2, H1_x2});
  jac2.insert({x1, H2_x1});
  jac2.insert({x3, H2_x3});

  MultiJacobian expected_sum;
  expected_sum.insert({x1, (Matrix(2, 2) << 3, 4, 6, 7).finished()});
  expected_sum.insert({x2, H1_x2});
  expected_sum.insert({x3, H2_x3});
  EXPECT(expected_sum.equals(jac1 + jac2));

  MultiJacobian jac12_sum = jac1;
  jac12_sum += jac2;
  EXPECT(expected_sum.equals(jac12_sum));

  Matrix m = I_2x2 * 2;
  MultiJacobian expected_mult1;
  expected_mult1.insert({x1, (Matrix(2, 2) << 2, 4, 6, 8).finished()});
  expected_mult1.insert({x2, (Matrix(2, 1) << 6, 4).finished()});
  EXPECT(expected_mult1.equals(m * jac1));
}


// Test add and mult operators for MultiJacobian
TEST(MultiJacobians, Mult) {
  Key x1 = 1;
  Key x2 = 2;
  Key x3 = 3;
  Key x4 = 4;

  MultiJacobian jac1_x1, jac1_x2, jac1_x3, jac1_x4;
  jac1_x1.insert({x1, I_2x2});
  jac1_x2.insert({x2, I_2x2});
  jac1_x3.insert({x3, I_2x2});
  jac1_x4.insert({x1, (Matrix(1,2)<<1,2).finished()});
  jac1_x4.insert({x2, (Matrix(1,2)<<2,1).finished()});
  jac1_x4.insert({x3, (Matrix(1,2)<<1,-1).finished()});
  MultiJacobians jacs1{{x1, jac1_x1}, {x2, jac1_x2}, {x3, jac1_x3}, {x4, jac1_x4}};

  MultiJacobian jac2_x1, jac2_x2, jac2_x3;
  jac2_x1.insert({x1, I_2x2});
  jac2_x2.insert({x2, I_2x2});
  jac2_x3.insert({x1, 2*I_2x2});
  jac2_x3.insert({x2, -1*I_2x2});
  MultiJacobians jacs2{{x1, jac2_x1}, {x2, jac2_x2}, {x3, jac2_x3}};

  MultiJacobian expected_jac_x1, expected_jac_x2, expected_jac_x3, expected_jac_x4;
  expected_jac_x1.insert({x1, I_2x2});
  expected_jac_x2.insert({x2, I_2x2});
  expected_jac_x3.insert({x1, 2*I_2x2});
  expected_jac_x3.insert({x2, -1*I_2x2});
  expected_jac_x4.insert({x1, (Matrix(1,2)<<1+2,2-2).finished()});
  expected_jac_x4.insert({x2, (Matrix(1,2)<<2-1,1+1).finished()});

  MultiJacobians jacs_mult = JacobiansMultiply(jacs1, jacs2);
  EXPECT(expected_jac_x1.equals(jacs_mult.at(x1)));
  EXPECT(expected_jac_x2.equals(jacs_mult.at(x2)));
  EXPECT(expected_jac_x3.equals(jacs_mult.at(x3)));
  EXPECT(expected_jac_x4.equals(jacs_mult.at(x4)));
}

/// Test computing jacobians from a bayes net.
TEST(MultiJacobian, ComputeBayesNetJacobian) {
  /// Construct a bayes net
  Key x1 = 1;
  Key x2 = 2;
  Key x3 = 3;
  Key x4 = 4;
  Key x5 = 5;
  GaussianFactorGraph graph;
  auto model1 = noiseModel::Isotropic::Sigma(1, 1.0);
  auto model2 = noiseModel::Isotropic::Sigma(2, 1.0);
  graph.add(JacobianFactor(x1, I_1x1, x2, I_1x1, x3, -I_1x1, Vector1(0), model1));
  Matrix21 H_3, H_4;
  H_3 << 1, 0;
  H_4 << 0, 1;
  graph.add(JacobianFactor(x3, H_3, x4, H_4, x5, -I_2x2, Vector2(0, 0), model2));

  Ordering ordering;
  ordering.push_back(x5);
  ordering.push_back(x3);
  auto elim_result = graph.eliminatePartialSequential(ordering);
  auto bayes_net = elim_result.first;


  MultiJacobians jacobians;
  KeyVector basis_keys{x1, x2, x4};
  std::map<Key, size_t> var_dim;
  var_dim.emplace(x1, 1);
  var_dim.emplace(x2, 1);
  var_dim.emplace(x4, 1);
  ComputeBayesNetJacobian(*bayes_net, basis_keys, var_dim, jacobians);

  MultiJacobian jacobian_x3, jacobian_x5;
  jacobian_x3.addJacobian(x1, I_1x1);
  jacobian_x3.addJacobian(x2, I_1x1);
  jacobian_x5.addJacobian(x1, H_3);
  jacobian_x5.addJacobian(x2, H_3);
  jacobian_x5.addJacobian(x4, H_4);
  EXPECT(jacobians.at(x1).equals(MultiJacobian::Identity(x1, 1)));
  EXPECT(jacobians.at(x2).equals(MultiJacobian::Identity(x2, 1)));
  EXPECT(jacobians.at(x4).equals(MultiJacobian::Identity(x4, 1)));
  EXPECT(jacobians.at(x3).equals(jacobian_x3));
  EXPECT(jacobians.at(x5).equals(jacobian_x5));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
