/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testIELMWorkingSet.cpp
 * @brief Check local working sets against exhaustive constrained QR oracles.
 * @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IELMWorkingSet.h>
#include <gtsam/base/TestableAssertions.h>

#include <limits>
#include <random>

/* ************************************************************************* */
namespace component_working_set {
using namespace gtsam;
using namespace gtdynamics;
using namespace gtdynamics::internal;
/** Assemble local geometry for synthetic test components, preserving original
 * indices. */
ComponentLinearizations localGeometry(
    const LinearInequalityConstraints& constraints) {
  std::map<Key, LinearIConstraintMap> grouped;
  for (size_t index = 0; index < constraints.size(); ++index)
    grouped[constraints[index]->createL2Factor()->front()].emplace(
        index, constraints[index]);
  ComponentLinearizations result;
  for (const auto& [key, rows] : grouped)
    result.emplace(key, std::make_shared<IEManifoldLinearization>(rows));
  return result;
}

/** Add a component-local homogeneous inequality block. */
void addBlock(LinearInequalityConstraints& constraints, Key key,
              const Matrix& rows) {
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(key, rows, Vector::Zero(rows.rows())));
}

/** Exhaustively check all 16 working faces for two 2D nonnegative variables.
 * This oracle uses the old constrained Gaussian QR solver, without depending
 * on the new face-selection or null-space code.
 */
VectorValues exhaustiveOptimum(const GaussianFactorGraph& cost) {
  NonlinearOptimizerParams params;
  params.linearSolverType = NonlinearOptimizerParams::SEQUENTIAL_QR;
  double bestError = std::numeric_limits<double>::infinity();
  VectorValues best;
  for (unsigned mask = 0; mask < 16; ++mask) {
    auto graph = cost;
    for (unsigned row = 0; row < 4; ++row) {
      if (!(mask & (1u << row))) continue;
      const Matrix12 normal = -Matrix2::Identity().row(row % 2);
      graph.emplace_shared<JacobianFactor>(row / 2 + 1, normal, Vector1::Zero(),
                                           noiseModel::Constrained::All(1));
    }
    const auto values = SolveLinear(graph, params);
    if (values.at(1).minCoeff() < -1e-8 || values.at(2).minCoeff() < -1e-8)
      continue;
    const double error = cost.error(values);
    if (error < bestError) {
      best = values;
      bestError = error;
    }
  }
  return best;
}

// Verifies coupled optima, row addition/release, and reversed elimination order
// against an exhaustive oracle for deterministic positive-definite problems.
TEST(IELMWorkingSet, CoupledWorkingSets) {
  std::mt19937 engine(1986);
  std::normal_distribution<double> normal;
  LinearInequalityConstraints constraints;
  addBlock(constraints, 1, -Matrix2::Identity());
  addBlock(constraints, 2, -Matrix2::Identity());
  const auto components = localGeometry(constraints);
  VectorValues initial;
  initial.insert(1, Vector2::Zero());
  initial.insert(2, Vector2::Zero());
  NonlinearOptimizerParams params;
  params.linearSolverType = NonlinearOptimizerParams::SEQUENTIAL_CHOLESKY;
  params.ordering = Ordering{2, 1};
  for (size_t sample = 0; sample < 32; ++sample) {
    Matrix4 matrix;
    Vector4 target;
    for (Eigen::Index i = 0; i < matrix.size(); ++i)
      matrix.data()[i] = normal(engine);
    matrix += 4 * Matrix4::Identity();
    for (Eigen::Index i = 0; i < target.size(); ++i) target(i) = normal(engine);
    GaussianFactorGraph cost;
    cost.emplace_shared<JacobianFactor>(1, matrix.leftCols<2>(), 2,
                                        matrix.rightCols<2>(), target);
    const auto expected = exhaustiveOptimum(cost);
    for (const bool startOnFaces : {false, true}) {
      IndexSetMap active;
      if (startOnFaces) {
        active[1].insert(0);
        active[2].insert(1);
      }
      const auto result =
          refineWorkingSet(cost, components, initial, active, params, 100);
      EXPECT(result.converged);
      EXPECT(assert_equal(expected, result.delta, 1e-7));
      EXPECT_DOUBLES_EQUAL(cost.error(expected), cost.error(result.delta),
                           1e-9);
    }
  }
}

// Verifies separate inequality blocks on one component are combined, while
// positive noise scaling preserves the selected face and cached raw checks.
TEST(IELMWorkingSet, MultipleBlocksOnOneComponent) {
  LinearInequalityConstraints constraints;
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, (Matrix13() << 1, 2, 0).finished(),
                                       Vector1::Zero(),
                                       noiseModel::Isotropic::Sigma(1, 0.1)));
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, (Matrix13() << 0, 1, -1).finished(),
                                       Vector1::Zero(),
                                       noiseModel::Isotropic::Sigma(1, 3.0)));
  GaussianFactorGraph cost;
  cost.emplace_shared<JacobianFactor>(1, Matrix3::Identity(), Vector3(1, 2, 3));
  const auto components = localGeometry(constraints);
  IndexSet selected;
  selected.insert(0);
  selected.insert(1);
  const auto actual = solveWorkingFaces(
      cost, {{1, components.at(1)->workingFace(selected)}}, {});
  auto original = cost;
  const auto faces = constraints.constraintGraph(selected);
  original.push_back(faces.begin(), faces.end());
  NonlinearOptimizerParams params;
  params.linearSolverType = NonlinearOptimizerParams::SEQUENTIAL_QR;
  EXPECT(assert_equal(SolveLinear(original, params), actual, 1e-9));
  EXPECT(components.at(1)->active(actual.at(1), selected));
  VectorValues probe;
  probe.insert(1, Vector3(2e-5, 0, 1));
  const auto violated = components.at(1)->violated(probe.at(1));
  EXPECT_LONGS_EQUAL(1, violated.size());
  EXPECT(violated.exists(0));
}

// Verifies redundant and opposing faces retain feasibility while a valid
// inward direction is released; zero rows never create artificial variables.
TEST(IELMWorkingSet, DependentAndOpposingRows) {
  LinearInequalityConstraints constraints;
  const Matrix rows =
      (Matrix(5, 2) << 1, 0, 2, 0, -1, 0, 0, -1, 0, 0).finished();
  addBlock(constraints, 1, rows);
  const auto components = localGeometry(constraints);
  GaussianFactorGraph cost;
  cost.emplace_shared<JacobianFactor>(1, Matrix2::Identity(), Vector2(1, 2));
  VectorValues initial;
  initial.insert(1, Vector2::Zero());
  IndexSetMap active;
  active[1].insert(0);
  const auto result =
      refineWorkingSet(cost, components, initial, active, {}, 100);
  EXPECT(result.converged);
  EXPECT(assert_equal(Vector2(0, 2), result.delta.at(1), 1e-8));
  EXPECT(result.blockingIndices.empty());
}

// Verifies exhaustion rejects an uncertified step and partial blocks are not
// incorrectly promoted to whole-block equalities during nonlinear retraction.
TEST(IELMWorkingSet, BudgetAndPartialBlock) {
  LinearInequalityConstraints constraints;
  addBlock(constraints, 1, -Matrix2::Identity());
  const auto components = localGeometry(constraints);
  GaussianFactorGraph cost;
  cost.emplace_shared<JacobianFactor>(1, Matrix2::Identity(), Vector2(2, -3));
  VectorValues initial;
  initial.insert(1, Vector2::Zero());
  IndexSetMap active;
  active[1].insert(0);
  EXPECT(!refineWorkingSet(cost, components, initial, active, {}, 1).converged);
  const auto result =
      refineWorkingSet(cost, components, initial, active, {}, 100);
  EXPECT(result.converged);
  EXPECT(assert_equal(Vector2(2, 0), result.delta.at(1), 1e-9));
  EXPECT(result.blockingIndices.empty());
}

// Verifies invalid assembly fails rather than recreating a global constrained
// problem; every nonlinear constraint must already belong to one component.
TEST(IELMWorkingSet, RejectNonlocalAndAffineRows) {
  LinearInequalityConstraints constraints;
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, Matrix1::Identity(), 2,
                                       Matrix1::Identity(), Vector1::Zero()));
  CHECK_EXCEPTION(localGeometry(constraints), std::invalid_argument);
  constraints.clear();
  constraints.emplace_shared<JacobianLinearInequalityConstraint>(
      std::make_shared<JacobianFactor>(1, Matrix1::Identity(), Vector1(0.1)));
  CHECK_EXCEPTION(localGeometry(constraints), std::invalid_argument);
}
}  // namespace component_working_set
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
