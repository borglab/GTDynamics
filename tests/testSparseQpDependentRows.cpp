/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/** @file testSparseQpDependentRows.cpp
 *  @brief GTSAM-only reproducer for the sparse QP replacement's release gate.
 *  @author GTDynamics contributors
 *  No CMC geometry, retractor, or working-set implementation is involved.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/ActiveSetSolver.h>

namespace sparse_qp_dependency {
using namespace gtsam;
double solve(double secondRow) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix1::Identity(), Vector1(2.)));
  problem.addConstraint(LinearConstraint::LessEqual(JacobianFactor(
      0, (Matrix(2, 1) << -1., secondRow).finished(), Vector2::Zero())));
  Values zero;
  zero.insert(0, Vector(Vector1::Zero()));
  auto params = std::make_shared<ActiveSetSolverParams>();
  params->qpSubproblemSolver =
      ActiveSetSolverParams::QpSubproblemSolver::Sparse;
  params->regularization = 0.;
  return ActiveSetSolver(problem, params).optimize(zero).at<Vector>(0)(0);
}

// Duplicate halfspaces must not prevent reaching the unique primal optimum.
TEST(SparseQpDependentRows, DuplicateHalfspaces) {
  // min (x-2)^2/2 subject to x>=0, 2*x>=0. Unique minimizer: x=2.
  EXPECT_DOUBLES_EQUAL(2., solve(-2.), 1e-9);
}

// Opposing halfspaces must identify the unique feasible point.
TEST(SparseQpDependentRows, OpposingHalfspaces) {
  // min (x-2)^2/2 subject to x>=0, x<=0. Unique feasible point: x=0.
  EXPECT_DOUBLES_EQUAL(0., solve(1.), 1e-9);
}

}  // namespace sparse_qp_dependency

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
