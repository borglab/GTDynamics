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

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/DynamicsGraph.h>
#include <gtdynamics/optimizer/ConstraintManifold.h>
#include <gtdynamics/optimizer/TspaceBasis.h>
#include <gtdynamics/universal_robot/RobotModels.h>
#include <gtdynamics/utils/Initializer.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/slam/BetweenFactor.h>

#include <boost/format.hpp>

using namespace gtsam;
using namespace gtdynamics;

/** Simple example Pose3 with between constraints. */
TEST(TspaceBasis, connected_poses) {
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;

  // Constraints.
  gtdynamics::EqualityConstraints constraints;
  auto noise = noiseModel::Unit::Create(6);
  auto factor12 = boost::make_shared<BetweenFactor<Pose3>>(
      x1_key, x2_key, Pose3(Rot3(), Point3(0, 0, 1)), noise);
  auto factor23 = boost::make_shared<BetweenFactor<Pose3>>(
      x2_key, x3_key, Pose3(Rot3(), Point3(0, 1, 0)), noise);
  constraints.emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor12);
  constraints.emplace_shared<gtdynamics::FactorZeroErrorConstraint>(factor23);

  // Create manifold values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Pose3(Rot3(), Point3(0, 0, 0)));
  cm_base_values.insert(x2_key, Pose3(Rot3(), Point3(0, 0, 1)));
  cm_base_values.insert(x3_key, Pose3(Rot3(), Point3(0, 1, 1)));

  // Construct manifold.
  auto component = boost::make_shared<ConnectedComponent>(constraints);
  
  // Construct basis.
  MatrixBasis basis_m(component, cm_base_values);
  KeyVector basis_keys {x3_key};
  EliminationBasis basis_e(component, cm_base_values, basis_keys);

  // Check tagent vector.
  Vector xi = (Vector(6) << 1, 1, 1, 1, 1, 1).finished();
  auto delta_m = basis_m.computeTangentVector(xi);
  auto delta_e = basis_e.computeTangentVector(xi);
  EXPECT(assert_equal(delta_m, delta_e));

  // Check recoverJacobian.
  auto H_x1_m = basis_m.recoverJacobian(x1_key);
  auto H_x2_m = basis_m.recoverJacobian(x2_key);
  auto H_x3_m = basis_m.recoverJacobian(x3_key);
  auto H_x1_e = basis_e.recoverJacobian(x1_key);
  auto H_x2_e = basis_e.recoverJacobian(x2_key);
  auto H_x3_e = basis_e.recoverJacobian(x3_key);
  EXPECT(assert_equal(H_x1_m, H_x1_e));
  EXPECT(assert_equal(H_x2_m, H_x2_e));
  EXPECT(assert_equal(H_x3_m, H_x3_e));
}


int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
