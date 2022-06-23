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
#include <gtdynamics/optimizer/MultiJacobian.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/GaussianFactorGraph.h>

using namespace gtsam;

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
