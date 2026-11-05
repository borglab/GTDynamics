/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testWorkingFace.cpp
 * @brief Compare local active-face elimination with constrained Gaussian QR.
 * @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/WorkingFace.h>
#include <gtdynamics/utils/GraphUtils.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/HessianFactor.h>

/* ************************************************************************* */
namespace local_constraint_solver {
using namespace gtsam;
using gtdynamics::internal::compactGaussianCosts;
using namespace gtdynamics::internal;

/** Convert oracle face factors to production component-local matrices. */
VectorValues solveFaces(const GaussianFactorGraph& cost,
                        const GaussianFactorGraph& constraints,
                        const NonlinearOptimizerParams& params) {
  WorkingFaces faces;
  for (const auto& factor : constraints) {
    const auto& jacobian = dynamic_cast<const JacobianFactor&>(*factor);
    if (jacobian.size() != 1 || !jacobian.getb().isZero(0.0))
      throw std::invalid_argument("test face is not local and homogeneous");
    faces.emplace(jacobian.front(), Matrix(jacobian.getA(jacobian.begin())));
  }
  return solveWorkingFaces(cost, faces, params);
}

/** Couple three vector variables with nonunit noise and explicit damping. */
GaussianFactorGraph dampedCost() {
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(
      1, Matrix3::Identity(), Vector3(1, -2, 3),
      noiseModel::Diagonal::Sigmas(Vector3(0.5, 1.2, 0.9)));
  graph.emplace_shared<JacobianFactor>(2, Matrix2::Identity(), Vector2(-1, 4));
  graph.emplace_shared<JacobianFactor>(3, Matrix1::Identity(), Vector1(2));
  const Matrix23 left = (Matrix23() << 1, 2, 0, 0, 1, -1).finished();
  const Matrix2 right = (Matrix2() << 0.4, -1, 0.8, 0.6).finished();
  const Matrix21 other = (Matrix21() << 0.4, -0.7).finished();
  graph.emplace_shared<JacobianFactor>(1, left, 2, right, 3, other,
                                       Vector2(0.3, -0.2));
  graph.emplace_shared<JacobianFactor>(1, 0.3 * Matrix3::Identity(),
                                       Vector3::Zero());
  graph.emplace_shared<JacobianFactor>(2, 0.7 * Matrix2::Identity(),
                                       Vector2::Zero());
  graph.emplace_shared<JacobianFactor>(3, 0.2 * Matrix1::Identity(),
                                       Vector1::Zero());
  return graph;
}

/** Create an exactly homogeneous active-face block. */
void constrain(GaussianFactorGraph& graph, Key key, const Matrix& rows) {
  graph.emplace_shared<JacobianFactor>(
      key, rows, Vector::Zero(rows.rows()),
      noiseModel::Constrained::All(rows.rows()));
}

/** Keep the original key order, including keys that may become fixed to zero.
 */
NonlinearOptimizerParams parameters() {
  NonlinearOptimizerParams params;
  params.linearSolverType = NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  params.ordering = Ordering{3, 1, 2};
  return params;
}

/** Solve the same objective and faces with the previous constrained QR path. */
VectorValues referenceSolution(const GaussianFactorGraph& cost,
                               const GaussianFactorGraph& constraints) {
  GaussianFactorGraph graph = cost;
  graph.push_back(constraints.begin(), constraints.end());
  auto params = parameters();
  params.linearSolverType = NonlinearOptimizerParams::SEQUENTIAL_QR;
  return gtdynamics::SolveLinear(graph, params);
}

// Verifies the damped cross-slice optimum and cost match constrained QR.
TEST(WorkingFace, CoupledVectors) {
  const auto cost = dampedCost();
  GaussianFactorGraph constraints;
  constrain(constraints, 1, (Matrix23() << 1, 1, 0, 0, 1, -1).finished());
  constrain(constraints, 2, (Matrix12() << 1, -2).finished());
  const auto actual = solveFaces(cost, constraints, parameters());
  const auto expected = referenceSolution(cost, constraints);
  EXPECT(assert_equal(expected, actual, 1e-8));
  EXPECT_DOUBLES_EQUAL(cost.error(expected), cost.error(actual), 1e-9);
}

// Verifies anisotropic LM damping is transformed with the nonorthonormal local
// basis, rather than replaced by identity or recomputed after reduction.
TEST(WorkingFace, AnisotropicDamping) {
  auto cost = dampedCost();
  cost.emplace_shared<JacobianFactor>(1, Vector3(0.05, 3.0, 0.7).asDiagonal(),
                                      Vector3::Zero());
  cost.emplace_shared<JacobianFactor>(2, Vector2(1.5, 0.2).asDiagonal(),
                                      Vector2::Zero());
  GaussianFactorGraph constraints;
  constrain(constraints, 1, (Matrix13() << 1, 2, -3).finished());
  constrain(constraints, 2, (Matrix12() << 2, -1).finished());
  const auto actual = solveFaces(cost, constraints, parameters());
  const auto expected = referenceSolution(cost, constraints);
  EXPECT(assert_equal(expected, actual, 1e-8));
  EXPECT_DOUBLES_EQUAL(cost.error(expected), cost.error(actual), 1e-9);
}

// Verifies redundant faces and a zero row retain the same feasible optimum.
TEST(WorkingFace, DependentFaces) {
  const auto cost = dampedCost();
  GaussianFactorGraph constraints;
  const Matrix rows =
      (Matrix(4, 3) << 1, 1, 0, 2, 2, 0, 0, 1, -1, 0, 0, 0).finished();
  constrain(constraints, 1, rows);
  const auto actual = solveFaces(cost, constraints, parameters());
  const auto expected = referenceSolution(cost, constraints);
  EXPECT(assert_equal(expected, actual, 1e-8));
  EXPECT_DOUBLES_EQUAL(cost.error(expected), cost.error(actual), 1e-9);
}

// Verifies zero-DOF blocks disappear from elimination and recover exact zeros.
TEST(WorkingFace, FullyConstrainedBlocks) {
  const auto cost = dampedCost();
  GaussianFactorGraph constraints;
  constrain(constraints, 1, Matrix3::Identity());
  auto actual = solveFaces(cost, constraints, parameters());
  EXPECT(assert_equal(referenceSolution(cost, constraints), actual, 1e-8));
  EXPECT(assert_equal(Vector3::Zero(), actual.at(1), 0.0));
  constrain(constraints, 2, Matrix2::Identity());
  constrain(constraints, 3, Matrix1::Identity());
  actual = solveFaces(cost, constraints, parameters());
  EXPECT(assert_equal(referenceSolution(cost, constraints), actual, 1e-8));
  EXPECT_DOUBLES_EQUAL(0.0, actual.norm(), 0.0);
}

// Verifies sparse recovery retains nonzero kernel entries below ordinary
// sparsification thresholds, preserving the exact original face.
TEST(WorkingFace, TinyKernelCoefficients) {
  const auto cost = dampedCost();
  GaussianFactorGraph constraints;
  const double coefficient = 1e-13;
  constrain(constraints, 1, (Matrix13() << 1.0, coefficient, 0.0).finished());
  const auto actual = solveFaces(cost, constraints, parameters());
  EXPECT(std::abs(actual.at(1)(0) + coefficient * actual.at(1)(1)) < 1e-25);
  EXPECT(std::abs(actual.at(1)(0)) > 1e-14);
}

// Verifies unused face scaling does not change rank or the recovered solution.
TEST(WorkingFace, FaceScalingAndEmptyRows) {
  const auto cost = dampedCost();
  GaussianFactorGraph constraints;
  constrain(constraints, 1,
            (Matrix23() << 1e-12, 1e-12, 0, 0, 1e12, -1e12).finished());
  constrain(constraints, 2, Matrix::Zero(0, 2));
  const auto actual = solveFaces(cost, constraints, parameters());
  GaussianFactorGraph reference;
  constrain(reference, 1, (Matrix23() << 1, 1, 0, 0, 1, -1).finished());
  EXPECT(assert_equal(referenceSolution(cost, reference), actual, 1e-8));
}

// Recovery must ignore arbitrary row scaling without accepting off-face steps.
TEST(WorkingFace, ScaledRecoveryChecks) {
  // The second row is numerically dependent at the rank threshold. Ordinary
  // increments violate it only by roundoff; very large increments expose a
  // real residual that recovery must reject even when rows are scaled down.
  for (double scale : {1e-12, 1.0, 1e12}) {
    const Matrix22 rows{{1.0, 0.0}, {1.0, 1e-16}};
    const WorkingFace face(scale * rows);
    VectorValues reduced;
    reduced.insert(1, Vector1(1.0));
    const auto actual = face.recover(reduced.at(1));
    EXPECT((rows * actual).norm() < 1e-7);
    reduced.at(1) = Vector1(1e12);
    CHECK_EXCEPTION(face.recover(reduced.at(1)), std::runtime_error);
  }
}

// Verifies unconstrained and empty problems preserve their original semantics.
TEST(WorkingFace, NoConstraints) {
  const auto cost = dampedCost();
  const auto actual = solveFaces(cost, GaussianFactorGraph(), parameters());
  EXPECT(assert_equal(gtdynamics::SolveLinear(cost, parameters()), actual));
  const auto empty =
      solveFaces(GaussianFactorGraph(), GaussianFactorGraph(), parameters());
  EXPECT_LONGS_EQUAL(0, empty.size());
}

// Verifies missing keys, mismatched dimensions, and hard objective rows fail
// explicitly rather than triggering a global constrained fallback.
TEST(WorkingFace, InvalidInputs) {
  const auto cost = dampedCost();
  CHECK_EXCEPTION(
      solveWorkingFaces(cost, {{4, WorkingFace(Matrix1::Identity())}},
                        parameters()),
      std::invalid_argument);
  CHECK_EXCEPTION(
      solveWorkingFaces(cost, {{1, WorkingFace(Matrix2::Identity())}},
                        parameters()),
      std::invalid_argument);
  GaussianFactorGraph exactCost;
  exactCost.emplace_shared<JacobianFactor>(1, Matrix3::Identity(),
                                           Vector3(1, 2, 3),
                                           noiseModel::Constrained::All(3));
  CHECK_EXCEPTION(solveWorkingFaces(exactCost, {}, parameters()),
                  std::invalid_argument);
}

// Verifies native Hessian costs can use the same unconstrained solve path.
TEST(WorkingFace, HessianCosts) {
  GaussianFactorGraph cost;
  cost.emplace_shared<HessianFactor>(1, Matrix2::Identity(), Vector2(1, 2),
                                     5.0);
  const WorkingFaces faces{{1, WorkingFace((Matrix12() << 1, -1).finished())}};
  const auto actual = solveWorkingFaces(cost, faces, parameters());
  EXPECT(assert_equal(Vector2(1.5, 1.5), actual.at(1), 1e-10));
}

// Verifies the actual reduced graph contains only ordinary cost factors and
// preserves the damped quadratic under recovery, including nonunit noise.
TEST(WorkingFace, UnconstrainedGraphContract) {
  const auto cost = dampedCost();
  const WorkingFaces faces{
      {1, WorkingFace((Matrix23() << 1, 1, 0, 0, 1, -1).finished())},
      {2, WorkingFace((Matrix12() << 1, -2).finished())}};
  const auto reduced = reduceWorkingFaces(cost, faces);
  VectorValues eta;
  for (const auto& [key, dim] : reduced.getKeyDimMap())
    eta.insert(key, Vector::Constant(dim, 0.7));
  for (const auto& factor : reduced) {
    const auto jacobian = std::dynamic_pointer_cast<JacobianFactor>(factor);
    EXPECT(jacobian && !jacobian->isConstrained());
  }
  EXPECT_DOUBLES_EQUAL(
      cost.error(recoverWorkingFaces(eta, cost.getKeyDimMap(), faces)),
      reduced.error(eta), 1e-9);
}

// Verifies mixed-noise stacking preserves cost and gradient at nonzero values.
TEST(WorkingFace, CompactedCostEquivalence) {
  const auto original = dampedCost();
  const auto compact = compactGaussianCosts(original);
  EXPECT_LONGS_EQUAL(7, original.size());
  EXPECT_LONGS_EQUAL(4, compact.size());
  VectorValues values;
  values.insert(1, Vector3(0.4, -1.2, 2.7));
  values.insert(2, Vector2(0.8, 1.9));
  values.insert(3, Vector1(-0.6));
  EXPECT_DOUBLES_EQUAL(original.error(values), compact.error(values), 1e-10);
  EXPECT(
      assert_equal(original.gradientAtZero(), compact.gradientAtZero(), 1e-10));
  EXPECT(assert_equal(original.hessianDiagonal(), compact.hessianDiagonal(),
                      1e-10));
  GaussianFactorGraph constraints;
  constrain(constraints, 1, (Matrix23() << 1, 1, 0, 0, 1, -1).finished());
  constrain(constraints, 2, (Matrix12() << 1, -2).finished());
  const auto actual = solveFaces(compact, constraints, parameters());
  EXPECT(assert_equal(referenceSolution(original, constraints), actual, 1e-8));
}

// Verifies key order is immaterial and unsupported factors retain their
// identity.
TEST(WorkingFace, CompactionScopesAndFallback) {
  GaussianFactorGraph original;
  original.emplace_shared<JacobianFactor>(
      1, Matrix1::Identity(), 2, 2 * Matrix1::Identity(), Vector1(0.5));
  original.emplace_shared<JacobianFactor>(
      2, 3 * Matrix1::Identity(), 1, 4 * Matrix1::Identity(), Vector1(-1.5),
      noiseModel::Isotropic::Sigma(1, 0.7));
  original.emplace_shared<HessianFactor>(3, Matrix1::Identity(), Vector1(0.2),
                                         0.04);
  original.emplace_shared<JacobianFactor>(
      4, Matrix1::Identity(), Vector1::Zero(), noiseModel::Constrained::All(1));
  original.push_back(GaussianFactor::shared_ptr{});
  const auto compact = compactGaussianCosts(original);
  EXPECT_LONGS_EQUAL(4, compact.size());
  EXPECT(compact[1] == original[2]);
  EXPECT(compact[2] == original[3]);
  EXPECT(compact[3] == nullptr);
  VectorValues values;
  values.insert(1, Vector1(0.8));
  values.insert(2, Vector1(-1.2));
  values.insert(3, Vector1(0.5));
  values.insert(4, Vector1::Zero());
  EXPECT_DOUBLES_EQUAL(original.error(values), compact.error(values), 1e-10);
}

}  // namespace local_constraint_solver
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
