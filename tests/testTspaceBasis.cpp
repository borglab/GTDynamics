/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testTspaceBasis.cpp
 * @brief Test tagent space basis for constraint manifold.
 * @author Yetong Zhang
 */

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/optimizer/EqualityConstraint.h>
#include <gtdynamics/manifold/ConstraintManifold.h>
#include <gtdynamics/manifold/TspaceBasis.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <memory>

using namespace gtsam;
using namespace gtdynamics;

/** Simple example Pose3 with between constraints. */
TEST(TspaceBasis, connected_poses) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;

  // Constraints.
  auto constraints = std::make_shared<gtdynamics::EqualityConstraints>();
  auto noise = noiseModel::Unit::Create(6);
  auto factor12 = std::make_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), noise);
  auto factor23 = std::make_shared<BetweenFactor<Pose3>>(
      x2_key, x3_key, Pose3(Rot3(), Point3(0, 1, 0)), noise);
  constraints->emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor12);
  constraints->emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor23);

  // Create manifold values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  cm_base_values.insert(x2_key, Pose3(Rot3(), Point3(0, 0, 1)));
  cm_base_values.insert(x3_key, Pose3(Rot3(), Point3(0, 1, 1)));

  // Construct basis.
  KeyVector basis_keys{x3_key};
  auto basis_params = std::make_shared<TspaceBasisParams>();
  auto basis_m = std::make_shared<OrthonormalBasis>(constraints, cm_base_values,
                                                    basis_params);
  auto basis_e = std::make_shared<EliminationBasis>(constraints, cm_base_values,
                                                    basis_params, basis_keys);
  auto sparse_creator = OrthonormalBasisCreator::CreateSparse();
  auto basis_sm = sparse_creator->create(constraints, cm_base_values);

  auto linear_graph = constraints->meritGraph().linearize(cm_base_values);
  std::vector<TspaceBasis::shared_ptr> basis_vec{basis_m, basis_e, basis_sm};

  // Check dimension.
  for (const auto& basis : basis_vec) {
    EXPECT_LONGS_EQUAL(6, basis->dim());
  }

  // Check tagent vector properties
  for (int i = 0; i < 6; i++) {
    Vector xi1 = Vector::Zero(6);
    xi1(i) = 1;
    Vector xi10 = xi1 * 10;
    for (const auto& basis : basis_vec) {
      // check null space property
      auto delta1 = basis->computeTangentVector(xi1);
      EXPECT_DOUBLES_EQUAL(0.0, linear_graph->error(delta1), 1e-9);
      // check linear space property
      auto delta10 = basis->computeTangentVector(xi10);
      EXPECT(assert_equal(delta10, 10 * delta1));
      // check construct xi from tangent vector
      auto recvoer_xi1 = basis->computeXi(delta1);
      EXPECT(assert_equal(xi1, recvoer_xi1));
    }
  }

  // Check linear space property
  Vector xi1 = Vector::Zero(6);
  Vector xi2 = Vector::Zero(6);
  xi1(1) = 1;
  xi2(2) = 1;
  for (const auto& basis : basis_vec) {
    // check null space property
    auto delta1 = basis->computeTangentVector(xi1);
    auto delta2 = basis->computeTangentVector(xi2);
    auto delta12 = basis->computeTangentVector(xi1 + xi2);
    EXPECT(assert_equal(delta12, delta1 + delta2));
  }

  // recoverJacobian will be checked in ConstraintManifold tests.

  // Create new values for testing.
  Values new_cm_base_values;
  new_cm_base_values.insert(x1_key, Pose3(Rot3(), Point3(1, 1, 1)));
  new_cm_base_values.insert(x2_key, Pose3(Rot3(), Point3(1, 1, 2)));
  new_cm_base_values.insert(x3_key, Pose3(Rot3(), Point3(1, 2, 2)));

  // Check createWithNewValues.
  for (const auto& basis : basis_vec) {
    auto new_basis = basis->createWithNewValues(new_cm_base_values);
    for (int i = 0; i < 6; i++) {
      Vector xi1 = Vector::Zero(6);
      xi1(i) = 1;
      // check null space property
      auto delta1 = basis->computeTangentVector(xi1);
      EXPECT_DOUBLES_EQUAL(0.0, linear_graph->error(delta1), 1e-9);
    }
  }
}

/** Simple example Pose3 with between constraints. */
TEST(TspaceBasis, linear_system) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;
  Key x4_key = 4;
  Double_ x1(x1_key);
  Double_ x2(x2_key);
  Double_ x3(x3_key);
  Double_ x4(x4_key);
  double tol = 1.0;
  auto constraints = std::make_shared<EqualityConstraints>();
  constraints->emplace_shared<DoubleExpressionEquality>(x1 + x2 + x3, tol);
  constraints->emplace_shared<DoubleExpressionEquality>(x1 + x2 + 2 * x3 + x4,
                                                        tol);

  Values values;
  values.insertDouble(x1_key, 0.0);
  values.insertDouble(x2_key, 0.0);
  values.insertDouble(x3_key, 0.0);
  values.insertDouble(x4_key, 0.0);
  KeyVector basis_keys{x1_key, x2_key};
  auto basis_params = std::make_shared<TspaceBasisParams>();
  // auto basis_m = std::make_shared<MatrixBasis>(basis_params, cc, values);
  auto basis_e =
      std::make_shared<EliminationBasis>(constraints, values, basis_params, basis_keys);
  auto basis_m = std::make_shared<OrthonormalBasis>(constraints, values, basis_params);
  auto sparse_creator = OrthonormalBasisCreator::CreateSparse();
  auto basis_sm = sparse_creator->create(constraints, values);

  // Construct new basis by adding addtional constraints
  EqualityConstraints new_constraints;
  new_constraints.emplace_shared<DoubleExpressionEquality>(x2, tol);
  auto new_basis_e = basis_e->createWithAdditionalConstraints(new_constraints, values);
  auto new_basis_m = basis_m->createWithAdditionalConstraints(new_constraints, values);
  auto new_basis_sm = basis_sm->createWithAdditionalConstraints(new_constraints, values);
  NonlinearFactorGraph merit_graph = constraints->meritGraph();
  merit_graph.add(new_constraints.meritGraph());
  auto linear_graph = merit_graph.linearize(values);
  for (const auto& vector: new_basis_e->basisVectors()) {
    EXPECT(assert_equal(0.0, linear_graph->error(vector)));
  }
  for (const auto& vector: new_basis_m->basisVectors()) {
    EXPECT(assert_equal(0.0, linear_graph->error(vector)));
  }
  for (const auto& vector: new_basis_sm->basisVectors()) {
    EXPECT(assert_equal(0.0, linear_graph->error(vector)));
  }

  Matrix expected_H_x1 = I_1x1;
  Matrix expected_H_x2 = I_1x1*0;
  Matrix expected_H_x3 = I_1x1 * -1;
  Matrix expected_H_x4 = I_1x1;
  EXPECT(assert_equal(expected_H_x1, new_basis_e->recoverJacobian(x1_key)));
  EXPECT(assert_equal(expected_H_x2, new_basis_e->recoverJacobian(x2_key)));
  EXPECT(assert_equal(expected_H_x3, new_basis_e->recoverJacobian(x3_key)));
  EXPECT(assert_equal(expected_H_x4, new_basis_e->recoverJacobian(x4_key)));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
