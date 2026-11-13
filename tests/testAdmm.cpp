/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file testAdmm.cpp
 * @brief Check both ADMM splitting strategies on exact CMC cone models.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/AdmmSolver.h>
#include <gtdynamics/cmcopt/ProxQpSolver.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/linear/HessianFactor.h>

#include <cmath>

using namespace gtsam;
using gtdynamics::AdmmSplitting;
namespace cmc = gtdynamics::internal;

TEST(Admm, OnePointActiveFace) {
  QpProblem problem;
  problem.addCost(JacobianFactor(7, Matrix2::Identity(), Vector2(2., -3.)));
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(7, -Matrix2::Identity(), Vector2::Zero())));
  Values prototype;
  prototype.insert(7, Vector(Vector2::Zero()));
  const auto result = cmc::solveAdmm(problem, prototype, std::nullopt, 2000);
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

TEST(Admm, CoupledPointConesVersusProxQp) {
  // Optimal solution: x1=0 on its boundary and x2=1 in the cone's interior.
  QpProblem problem;
  problem.addCost(JacobianFactor(1, Matrix1::Identity(), Vector1(-3.)));
  problem.addCost(JacobianFactor(2, Matrix1::Identity(), Vector1(2.)));
  problem.addCost(JacobianFactor(1, Matrix1::Identity(), 2,
                                 -Matrix1::Identity(), Vector1::Zero()));
  for (Key key : {Key(1), Key(2)})
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(key, -Matrix1::Identity(), Vector1::Zero())));
  Values prototype;
  prototype.insert(1, Vector(Vector1::Zero()));
  prototype.insert(2, Vector(Vector1::Zero()));
  cmc::AdmmParams params;
  params.projectionThreads = 2;
  const auto result =
      cmc::solveAdmm(problem, prototype, Ordering{2, 1}, 3000, params);
  EXPECT(result.converged);
  if (!result.converged) return;
  EXPECT_DOUBLES_EQUAL(0., result.values.at<Vector>(1)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(1., result.values.at<Vector>(2)(0), 1e-7);
  EXPECT(result.activeRows[0]);
  EXPECT(!result.activeRows[1]);
#ifdef GTDYNAMICS_WITH_PROXQP
  const auto prox = cmc::solveProxQp(problem, prototype, Ordering{2, 1}, 3000);
  EXPECT(prox.converged);
  if (prox.converged) {
    EXPECT(assert_equal(prox.values, result.values, 1e-7));
    EXPECT(prox.activeRows == result.activeRows);
  }
#endif
}

TEST(Admm, DependentAndOpposingRows) {
  for (bool opposing : {false, true}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(7, Matrix1::Identity(), Vector1(2.)));
    Matrix A(2, 1);
    A << -1., opposing ? 1. : -2.;
    problem.addConstraint(
        LinearConstraint::LessEqual(JacobianFactor(7, A, Vector2::Zero())));
    Values prototype;
    prototype.insert(7, Vector(Vector1::Zero()));
    const auto result = cmc::solveAdmm(problem, prototype, std::nullopt, 5000);
    EXPECT(result.converged);
    if (result.converged)
      EXPECT_DOUBLES_EQUAL(opposing ? 0. : 2., result.values.at<Vector>(7)(0),
                           1e-7);
  }
}

TEST(Admm, PointParallelMatchesSerial) {
  QpProblem problem;
  Values prototype;
  for (Key key = 1; key <= 32; ++key) {
    prototype.insert(key, Vector(Vector2::Zero()));
    problem.addCost(JacobianFactor(key, Matrix2::Identity(), Vector2(2., -3.)));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(key, -Matrix2::Identity(), Vector2::Zero())));
    if (key > 1)
      problem.addCost(JacobianFactor(key - 1, 0.25 * Matrix2::Identity(), key,
                                     -0.25 * Matrix2::Identity(),
                                     Vector2::Zero()));
  }
  cmc::AdmmParams serial;
  const auto a = cmc::solveAdmm(problem, prototype, std::nullopt, 5000, serial);
  serial.projectionThreads = 4;
  const auto b = cmc::solveAdmm(problem, prototype, std::nullopt, 5000, serial);
  EXPECT(a.converged);
  EXPECT(b.converged);
  if (a.converged && b.converged)
    EXPECT(assert_equal(a.values, b.values, 1e-7));
}

TEST(Admm, RejectUnsupportedFormulationsAndFailures) {
  QpProblem problem;
  problem.addCost(JacobianFactor(1, Matrix1::Identity(), Vector1(2.)));
  Values prototype;
  prototype.insert(1, Vector(Vector1::Zero()));
  cmc::AdmmParams params;
  params.rho = 0.;
  CHECK_EXCEPTION(cmc::solveAdmm(problem, prototype, std::nullopt, 100, params),
                  std::invalid_argument);
  params.rho = 1.;
  const auto exhausted =
      cmc::solveAdmm(problem, prototype, std::nullopt, 0, params);
  EXPECT(!exhausted.converged);
  problem.addConstraint(LinearConstraint::LessEqual(
      JacobianFactor(1, Matrix1::Identity(), Vector1(1.))));
  CHECK_EXCEPTION(cmc::solveAdmm(problem, prototype, std::nullopt, 100, params),
                  std::invalid_argument);
}

// Checks that damping retries refresh the numerical factors of a shared tree.
TEST(Admm, ReuseMultifrontalStructureAcrossDampingTrials) {
  Values prototype;
  prototype.insert(1, Vector(Vector1::Zero()));
  cmc::AdmmWorkspace workspace;
  for (double lambda : {0.25, 3.0, 0.25}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(1, Matrix1::Identity(), Vector1(2.)));
    problem.addCost(JacobianFactor(1, std::sqrt(lambda) * Matrix1::Identity(),
                                   Vector1::Zero()));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(1, -Matrix1::Identity(), Vector1::Zero())));
    const auto result = workspace.solve(problem, prototype, std::nullopt, 2000);
    EXPECT(result.converged);
    if (result.converged)
      EXPECT_DOUBLES_EQUAL(2. / (1. + lambda), result.values.at<Vector>(1)(0),
                           1e-7);
  }
}

// Checks penalty sensitivity against an analytic, badly scaled unconstrained
// QP.
TEST(Admm, LowCurvatureNeedsAppropriatePenalty) {
  QpProblem problem;
  Matrix2 jacobian = Matrix2::Zero();
  jacobian(0, 0) = 0.01;
  jacobian(1, 1) = 100.;
  problem.addCost(JacobianFactor(1, jacobian, Vector2(0.01, 200.)));
  Values prototype;
  prototype.insert(1, Vector(Vector2::Zero()));
  auto slow = cmc::solveAdmm(problem, prototype, std::nullopt, 1000);
  EXPECT(!slow.converged);
  EXPECT(std::isnan(slow.stationarityResidual));
  EXPECT(slow.message.find("not evaluated") != std::string::npos);
  cmc::AdmmParams params;
  params.rho = 1e-4;
  params.absoluteTolerance = 1e-12;
  params.relativeTolerance = 1e-12;
  const auto solved =
      cmc::solveAdmm(problem, prototype, std::nullopt, 1000, params);
  EXPECT(solved.converged);
  if (solved.converged)
    EXPECT(assert_equal(Vector2(1., 2.), solved.values.at<Vector>(1), 1e-7));
  EXPECT(solved.iterations < 50);
}

// Verifies that changed objective right-hand sides do not reuse stale
// gradients.
TEST(Admm, ChangedRightHandSideWithSameStructure) {
  Values prototype;
  prototype.insert(1, Vector(Vector1::Zero()));
  cmc::AdmmWorkspace workspace;
  for (double target : {2., -3., 4.}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(1, Matrix1::Identity(), Vector1(target)));
    problem.addConstraint(LinearConstraint::LessEqual(
        JacobianFactor(1, -Matrix1::Identity(), Vector1::Zero())));
    const auto result = workspace.solve(problem, prototype, std::nullopt, 1000);
    EXPECT(result.converged);
    if (result.converged)
      EXPECT_DOUBLES_EQUAL(std::max(0., target), result.values.at<Vector>(1)(0),
                           1e-7);
  }
}

// An oblique face requires the same anisotropic metric in both ADMM steps.
TEST(Admm, DiagonalMetricObliqueFace) {
  for (double rho : {0.1, 1., 10.}) {
    QpProblem problem;
    Matrix2 jacobian = Matrix2::Identity();
    jacobian(1, 1) = 10.;
    problem.addCost(JacobianFactor(1, jacobian, Vector2(2., 30.)));
    Matrix A(1, 2);
    A << 1., 1.;
    problem.addConstraint(
        LinearConstraint::LessEqual(JacobianFactor(1, A, Vector1::Zero())));
    Values prototype;
    prototype.insert(1, Vector(Vector2::Zero()));
    cmc::AdmmParams params;
    params.rho = rho;
    params.useDiagonalMetric = true;
    const auto result =
        cmc::solveAdmm(problem, prototype, std::nullopt, 1000, params);
    EXPECT(result.converged);
    if (result.converged) {
      EXPECT(assert_equal(Vector2(-298. / 101., 298. / 101.),
                          result.values.at<Vector>(1), 1e-7));
      EXPECT(result.activeRows[0]);
      EXPECT(result.stationarityResidual <= 1e-7);
    }
  }
}

// Diagonal scaling removes the penalty imbalance across eight curvature orders.
TEST(Admm, DiagonalMetricLowCurvatureAndDampingReuse) {
  Values prototype;
  prototype.insert(1, Vector(Vector2::Zero()));
  cmc::AdmmWorkspace workspace;
  cmc::AdmmParams params;
  params.useDiagonalMetric = true;
  params.absoluteTolerance = 1e-12;
  params.relativeTolerance = 1e-12;
  Matrix2 jacobian = Matrix2::Zero();
  jacobian.diagonal() << 0.01, 100.;
  for (double damping : {0., 3., 0.}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(1, jacobian, Vector2(0.01, 200.)));
    problem.addCost(JacobianFactor(1, std::sqrt(damping) * Matrix2::Identity(),
                                   Vector2::Zero()));
    const auto result =
        workspace.solve(problem, prototype, std::nullopt, 100, params);
    EXPECT(result.converged);
    EXPECT(result.iterations < 60);
    if (result.converged)
      EXPECT(
          assert_equal(Vector2(1e-4 / (1e-4 + damping), 2e4 / (1e4 + damping)),
                       result.values.at<Vector>(1), 1e-7));
  }
}

// Scaling must preserve the physical optimum and original constraint row order.
TEST(Admm, ScaledConstraintSplitObliqueFace) {
  for (double rowScale : {1., 1000.}) {
    QpProblem problem;
    Matrix2 jacobian = Matrix2::Zero();
    jacobian.diagonal() << 0.01, 100.;
    problem.addCost(JacobianFactor(1, jacobian, Vector2(0.02, 300.)));
    problem.addCost(JacobianFactor(2, Matrix1::Identity(), Vector1(-2.)));
    problem.addConstraint(LinearConstraint::GreaterEqual(
        JacobianFactor(2, Matrix1::Identity(), Vector1::Zero())));
    Matrix rows(2, 2);
    rows << rowScale, rowScale, 0., 0.;
    problem.addConstraint(
        LinearConstraint::LessEqual(JacobianFactor(1, rows, Vector2::Zero())));
    Values prototype;
    prototype.insert(1, Vector(Vector2::Zero()));
    prototype.insert(2, Vector(Vector1::Zero()));
    cmc::AdmmParams params;
    params.splitting = AdmmSplitting::ConstraintOutputs;
    params.useDiagonalMetric = true;
    const auto result =
        cmc::solveAdmm(problem, prototype, Ordering{2, 1}, 2000, params);
    EXPECT(result.converged);
    if (!result.converged) continue;
    const double optimum = (3e4 - 2e-4) / (1e4 + 1e-4);
    EXPECT(assert_equal(Vector2(-optimum, optimum), result.values.at<Vector>(1),
                        1e-7));
    EXPECT_DOUBLES_EQUAL(0., result.values.at<Vector>(2)(0), 1e-7);
    EXPECT_LONGS_EQUAL(3, result.activeRows.size());
    EXPECT(result.activeRows[0]);
    EXPECT(result.activeRows[1]);
    EXPECT(!result.activeRows[2]);
    EXPECT(result.stationarityResidual <= 1e-7);
  }
}

// Constraint splitting must preserve coupled costs and opposing/duplicate rows.
TEST(Admm, ConstraintSplitCoupledAndDependentRows) {
  for (bool scaled : {false, true}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(1, Matrix1::Identity(), Vector1(-3.)));
    problem.addCost(JacobianFactor(2, Matrix1::Identity(), Vector1(2.)));
    problem.addCost(JacobianFactor(1, Matrix1::Identity(), 2,
                                   -Matrix1::Identity(), Vector1::Zero()));
    Matrix rows(3, 1);
    rows << 1., -1., -2.;
    problem.addConstraint(
        LinearConstraint::LessEqual(JacobianFactor(1, rows, Vector3::Zero())));
    Values prototype;
    prototype.insert(1, Vector(Vector1::Zero()));
    prototype.insert(2, Vector(Vector1::Zero()));
    cmc::AdmmParams params;
    params.splitting = AdmmSplitting::ConstraintOutputs;
    params.useDiagonalMetric = scaled;
    const auto result =
        cmc::solveAdmm(problem, prototype, std::nullopt, 2000, params);
    EXPECT(result.converged);
    if (result.converged) {
      EXPECT_DOUBLES_EQUAL(0., result.values.at<Vector>(1)(0), 1e-7);
      EXPECT_DOUBLES_EQUAL(1., result.values.at<Vector>(2)(0), 1e-7);
    }
  }
}

// The no-inequality case solves once, including Hessian inputs and damping
// reuse.
TEST(Admm, ConstraintSplitUnconstrainedHessianAndReuse) {
  Values prototype;
  prototype.insert(1, Vector(Vector2::Zero()));
  cmc::AdmmParams params;
  params.splitting = AdmmSplitting::ConstraintOutputs;
  params.useDiagonalMetric = true;
  cmc::AdmmWorkspace workspace;
  for (double damping : {0.25, 3., 0.25}) {
    QpProblem problem;
    Matrix2 information = Matrix2::Zero();
    information.diagonal() << 1e-4, 1e4;
    problem.addCost(
        HessianFactor(1, information, Vector2(1e-4, 2e4), 40000.0001));
    problem.addCost(JacobianFactor(1, std::sqrt(damping) * Matrix2::Identity(),
                                   Vector2::Zero()));
    const auto result =
        workspace.solve(problem, prototype, std::nullopt, 10, params);
    EXPECT(result.converged);
    EXPECT_LONGS_EQUAL(1, result.iterations);
    if (result.converged)
      EXPECT(
          assert_equal(Vector2(1e-4 / (1e-4 + damping), 2e4 / (1e4 + damping)),
                       result.values.at<Vector>(1), 1e-7));
  }
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
