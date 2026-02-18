/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testPenaltyOptimizr.cpp
 * @brief Test penalty method optimzier for constrained optimization.
 * @author: Yetong Zhang
 */

#include "constrainedExample.h"
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/constrained_optimizer/IPOptOptimizer.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/numericalDerivative.h>

using namespace gtsam;
using namespace gtdynamics;

using std::placeholders::_1;

TEST(IFOptTranslator, VecToPose) {
  Vector6 vec;
  vec << 0.2, 0.4, -1.3, 4, 1, 6;
  auto pose = IFOptTranslator::VecToPose(vec);
  auto vec_back = IFOptTranslator::PoseToVec(pose);
  EXPECT(assert_equal(vec, vec_back));

  Matrix66 H;
  IFOptTranslator::VecToPose(vec, H);
  auto f = std::bind(IFOptTranslator::VecToPose, _1, nullptr);
  auto numericalH0 = numericalDerivative11<Pose3, Vector6>(f, vec);
  EXPECT(assert_equal(numericalH0, H));
}

/* ************************************************************************* */
// An optimiztion with Pose3 variables.
TEST(IPOptimizer, Pose3) {
  Key pose1_key = PoseKey(1, 0);
  Key pose2_key = PoseKey(2, 0);

  Pose3 init_pose1(Rot3::RzRyRx(Vector3(0.4, 1.2, -0.7)), Vector3(-1, 4, 2));
  Pose3 init_pose2(Rot3::RzRyRx(Vector3(3.4, -2.2, 1.1)), Vector3(4, -2, 8));

  Pose3 gt_pose1(Rot3::RzRyRx(Vector3(2.1, 1.4, 2.7)), Vector3(5, 1, 3));
  Pose3 rel_pose(Rot3::RzRyRx(Vector3(0.4, 1.2, -0.7)), Vector3(-1, 4, 2));
  Pose3 gt_pose2 = gt_pose1.compose(rel_pose);

  /// Create initial values.
  Values init_values;
  init_values.insert(pose1_key, init_pose1);
  init_values.insert(pose2_key, init_pose2);

  /// Constraints
  EqualityConstraints e_constraints;
  auto factor = std::make_shared<BetweenFactor<Pose3>>(
      pose1_key, pose2_key, rel_pose, noiseModel::Isotropic::Sigma(6, 1.0));
  e_constraints.emplace_shared<FactorZeroErrorConstraint>(factor,
                                                          Vector::Ones(6));
  InequalityConstraints i_constraints;

  NonlinearFactorGraph cost;
  cost.addPrior(pose1_key, gt_pose1, noiseModel::Isotropic::Sigma(6, 1.0));

  /// Solve the constraint problem with IPOPT.
  IPOptimizer optimizer;
  Values results =
      optimizer.optimize(cost, e_constraints, i_constraints, init_values);

  EXPECT(assert_equal(gt_pose1, results.at<Pose3>(pose1_key), 1e-2));
  EXPECT(assert_equal(gt_pose2, results.at<Pose3>(pose2_key), 1e-2));
}

/* ************************************************************************* */
TEST(IPOptimizer, EConstrainedExample) {
  using namespace e_constrained_example;

  /// Create initial values.
  Values init_values;
  init_values.insert(x1_key, -0.2);
  init_values.insert(x2_key, -0.2);

  /// Solve the constraint problem with Augmented Lagrangian optimizer.
  IPOptimizer optimizer;
  Values results = optimizer.optimize(cost, constraints, init_values);

  /// Check the result is correct within tolerance.
  Values gt_results;
  gt_results.insert(x1_key, 0.0);
  gt_results.insert(x2_key, 0.0);
  double tol = 1e-4;
  EXPECT(assert_equal(gt_results, results, tol));
}

/* ************************************************************************* */
TEST(PenaltyOptimizer, IEConstrainedExample) {
  using namespace i_constrained_example;

  /// Create initial values.
  Values init_values;
  init_values.insert(x1_key, 0.0);
  init_values.insert(x2_key, 0.0);

  /// Solve the constraint problem with Augmented Lagrangian optimizer.
  IPOptimizer optimizer;
  Values results =
      optimizer.optimize(cost, e_constraints, i_constraints, init_values);

  /// Check the result is correct within tolerance.
  Values gt_results;
  gt_results.insert(x1_key, sqrt(2) / 2);
  gt_results.insert(x2_key, sqrt(2) / 2);
  double tol = 1e-4;
  EXPECT(
      assert_equal(gt_results.atDouble(x1_key), results.atDouble(x1_key), tol));
  EXPECT(
      assert_equal(gt_results.atDouble(x2_key), results.atDouble(x2_key), tol));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
