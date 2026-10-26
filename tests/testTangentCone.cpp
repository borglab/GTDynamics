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
#include <gtdynamics/cmcopt/TangentCone.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtsam;
using namespace gtdynamics;

template <typename Map> bool ContainerEqual(Map const &lhs, Map const &rhs) {
  return lhs.size() == rhs.size() &&
         std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

/** Test projecting into the cone defined by y<=2x; 2y>=x. */
TEST(project, example_2d) {
  LinearInequalityConstraints constraints;
  auto factor1 = std::make_shared<JacobianFactor>(
      1, (Matrix(1, 2) << 1, -2).finished(), Vector1(0), noiseModel::Unit::Create(1));
  auto factor2 = std::make_shared<JacobianFactor>(
      1, (Matrix(1, 2) << -2, 1).finished(), Vector1(0), noiseModel::Unit::Create(1));
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(factor1);
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(factor2);
  TangentCone cone(constraints);

  {
    Vector xi = Vector2(1, 0);
    IndexSet active_indices;
    Vector projected_xi;
    std::tie(active_indices, projected_xi) = cone.project(xi);
    IndexSet expected_active_indices;
    expected_active_indices.insert(0);
    EXPECT(ContainerEqual(expected_active_indices, active_indices));
    Vector expected_projected_xi = Vector2(0.8, 0.4);
    EXPECT(assert_equal(expected_projected_xi, projected_xi));
  }
  {
    Vector xi = Vector2(0, 2);
    IndexSet active_indices;
    Vector projected_xi;
    std::tie(active_indices, projected_xi) = cone.project(xi);
    IndexSet expected_active_indices;
    expected_active_indices.insert(1);
    EXPECT(ContainerEqual(expected_active_indices, active_indices));
    Vector expected_projected_xi = Vector2(0.8, 1.6);
    EXPECT(assert_equal(expected_projected_xi, projected_xi));
  }
  {
    Vector xi = Vector2(-1, -2);
    IndexSet active_indices;
    Vector projected_xi;
    std::tie(active_indices, projected_xi) = cone.project(xi);
    IndexSet expected_active_indices;
    expected_active_indices.insert(0);
    expected_active_indices.insert(1);
    EXPECT(ContainerEqual(expected_active_indices, active_indices));
    Vector expected_projected_xi = Vector2(0.0, 0.0);
    EXPECT(assert_equal(expected_projected_xi, projected_xi));
  }
}

// Preserve the affine clipping regression through the production projection
// caller, including the non-unit noise on the inequality residual.
TEST(project, AffineUpperBound) {
  LinearInequalityConstraints constraints;
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, I_1x1, Vector1(2),
                                       noiseModel::Isotropic::Sigma(1, 3)));
  const auto [active, projected] = TangentCone(constraints).project(Vector1(3));
  EXPECT_DOUBLES_EQUAL(2.0, projected(0), 1e-8);
  EXPECT(active.exists(0));
}

TEST(project, UnilateralContactCanIncrease) {
  LinearInequalityConstraints constraints;
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, -I_1x1, Vector1(0)));
  const auto [active, projected] = TangentCone(constraints).project(Vector1(4));
  EXPECT_DOUBLES_EQUAL(4.0, projected(0), 1e-8);
  EXPECT(active.empty());
}

TEST(project, MultirowWorkingSet) {
  LinearInequalityConstraints constraints;
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, Matrix2::Identity(), Vector2::Zero()));
  TangentCone cone(constraints);
  {
    const auto [active, projected] = cone.project(Vector2(1, -1));
    EXPECT(assert_equal(Vector2(0, -1), projected, 1e-8));
    // A partially active block cannot become a whole-block equality.
    EXPECT(active.empty());
  }
  {
    const auto [active, projected] = cone.project(Vector2(1, 2));
    EXPECT(assert_equal(Vector2::Zero(), projected, 1e-8));
    EXPECT_LONGS_EQUAL(1, active.size());
    EXPECT(active.exists(0));
  }
}

TEST(project, EmptyCone) {
  LinearInequalityConstraints constraints;
  const Vector direction = Vector2(1, -2);
  const auto [active, projected] = TangentCone(constraints).project(direction);
  EXPECT(assert_equal(direction, projected));
  EXPECT(active.empty());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
