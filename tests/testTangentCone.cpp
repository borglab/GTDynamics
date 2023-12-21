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
#include <gtdynamics/imanifold/TangentCone.h>
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
  Matrix A = (Matrix(2, 2) << -1, 2, 2, -1).finished();

  LinearInequalityConstraints constraints;
  auto factor1 = std::make_shared<JacobianFactor>(
      1, (Matrix(1, 2) << -1, 2).finished(), Vector1(0), noiseModel::Unit::Create(1));
  auto factor2 = std::make_shared<JacobianFactor>(
      1, (Matrix(1, 2) << 2, -1).finished(), Vector1(0), noiseModel::Unit::Create(1));
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

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
