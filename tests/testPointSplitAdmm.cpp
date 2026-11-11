/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file testPointSplitAdmm.cpp
 * @brief Check the experimental point-splitting QP on exact CMC cone models.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/PointSplitAdmmSolver.h>
#include <gtdynamics/cmcopt/ProxQpSolver.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/LinearConstraint.h>

using namespace gtsam;
using namespace gtdynamics;

TEST(PointSplitAdmm, OnePointActiveFace) {
  QpProblem problem;
  problem.addCost(JacobianFactor(7, Matrix2::Identity(), Vector2(2., -3.)));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(7, -Matrix2::Identity(), Vector2::Zero())));
  Values prototype;
  prototype.insert(7, Vector(Vector2::Zero()));
  const auto result = internal::solvePointSplitAdmm(
      problem, prototype, std::nullopt, 2000);
  EXPECT(result.converged);
  if (!result.converged) return;
  EXPECT(assert_equal(Vector2(2., 0.), result.values.at<Vector>(7), 1e-7));
  EXPECT_LONGS_EQUAL(2, result.activeRows.size());
  EXPECT(!result.activeRows[0]);
  EXPECT(result.activeRows[1]);
  EXPECT(result.primalResidual <= 1e-7);
  EXPECT(result.dualResidual <= 1e-7);
  EXPECT(result.stationarityResidual <= 1e-7);
}

TEST(PointSplitAdmm, CoupledPointConesVersusProxQp) {
  // Optimal solution: x1=0 on its boundary and x2=1 in the cone's interior.
  QpProblem problem;
  problem.addCost(JacobianFactor(1, Matrix1::Identity(), Vector1(-3.)));
  problem.addCost(JacobianFactor(2, Matrix1::Identity(), Vector1(2.)));
  problem.addCost(JacobianFactor(
      1, Matrix1::Identity(), 2, -Matrix1::Identity(), Vector1::Zero()));
  for (Key key : {Key(1), Key(2)})
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(key, -Matrix1::Identity(), Vector1::Zero())));
  Values prototype;
  prototype.insert(1, Vector(Vector1::Zero()));
  prototype.insert(2, Vector(Vector1::Zero()));
  internal::PointSplitAdmmParams params;
  params.projectionThreads = 2;
  const auto result = internal::solvePointSplitAdmm(
      problem, prototype, Ordering{2, 1}, 3000, params);
  EXPECT(result.converged);
  if (!result.converged) return;
  EXPECT_DOUBLES_EQUAL(0., result.values.at<Vector>(1)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(1., result.values.at<Vector>(2)(0), 1e-7);
  EXPECT(result.activeRows[0]);
  EXPECT(!result.activeRows[1]);
#ifdef GTDYNAMICS_WITH_PROXQP
  const auto prox = internal::solveProxQp(
      problem, prototype, Ordering{2, 1}, 3000);
  EXPECT(prox.converged);
  if (prox.converged) {
    EXPECT(assert_equal(prox.values, result.values, 1e-7));
    EXPECT(prox.activeRows == result.activeRows);
  }
#endif
}

TEST(PointSplitAdmm, DependentAndOpposingRows) {
  for (bool opposing : {false, true}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(7, Matrix1::Identity(), Vector1(2.)));
    Matrix A(2, 1);
    A << -1., opposing ? 1. : -2.;
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(7, A, Vector2::Zero())));
    Values prototype;
    prototype.insert(7, Vector(Vector1::Zero()));
    const auto result = internal::solvePointSplitAdmm(
        problem, prototype, std::nullopt, 5000);
    EXPECT(result.converged);
    if (result.converged)
      EXPECT_DOUBLES_EQUAL(opposing ? 0. : 2.,
                           result.values.at<Vector>(7)(0), 1e-7);
  }
}

TEST(PointSplitAdmm, PointParallelMatchesSerial) {
  QpProblem problem;
  Values prototype;
  for (Key key = 1; key <= 32; ++key) {
    prototype.insert(key, Vector(Vector2::Zero()));
    problem.addCost(JacobianFactor(
        key, Matrix2::Identity(), Vector2(2., -3.)));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(key, -Matrix2::Identity(), Vector2::Zero())));
    if (key > 1)
      problem.addCost(JacobianFactor(
          key - 1, 0.25 * Matrix2::Identity(), key,
          -0.25 * Matrix2::Identity(), Vector2::Zero()));
  }
  internal::PointSplitAdmmParams serial;
  const auto a = internal::solvePointSplitAdmm(
      problem, prototype, std::nullopt, 5000, serial);
  serial.projectionThreads = 4;
  const auto b = internal::solvePointSplitAdmm(
      problem, prototype, std::nullopt, 5000, serial);
  EXPECT(a.converged);
  EXPECT(b.converged);
  if (a.converged && b.converged)
    EXPECT(assert_equal(a.values, b.values, 1e-7));
}

TEST(PointSplitAdmm, RejectUnsupportedFormulationsAndFailures) {
  QpProblem problem;
  problem.addCost(JacobianFactor(1, Matrix1::Identity(), Vector1(2.)));
  Values prototype;
  prototype.insert(1, Vector(Vector1::Zero()));
  internal::PointSplitAdmmParams params;
  params.rho = 0.;
  CHECK_EXCEPTION(internal::solvePointSplitAdmm(
      problem, prototype, std::nullopt, 100, params), std::invalid_argument);
  params.rho = 1.;
  const auto exhausted = internal::solvePointSplitAdmm(
      problem, prototype, std::nullopt, 0, params);
  EXPECT(!exhausted.converged);
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(1, Matrix1::Identity(), Vector1(1.))));
  CHECK_EXCEPTION(internal::solvePointSplitAdmm(
      problem, prototype, std::nullopt, 100, params), std::invalid_argument);
}
