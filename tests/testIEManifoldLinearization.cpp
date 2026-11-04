/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testIEManifoldLinearization.cpp
 * @brief Verify state-owned geometry, index mapping and cache invalidation.
 * @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <gtdynamics/cmcopt/IEConstraintManifold.h>
#include <gtdynamics/cmcopt/WorkingFace.h>
#include <gtsam/base/TestableAssertions.h>

/* ************************************************************************* */
namespace component_linearization {
using namespace gtsam;
using namespace gtdynamics;
using namespace gtdynamics::internal;
size_t derivativeCalls = 0;

/** Count derivative evaluations independently of residual-only queries. */
double square(const double& x, OptionalJacobian<1, 1> H) {
  if (H) {
    ++derivativeCalls;
    (*H)(0, 0) = 2 * x;
  }
  return x * x;
}

/** Leave two inactive blocks before the nonlinear active block. */
IEConstraintManifold makeManifold() {
  auto inequalities = std::make_shared<NonlinearInequalityConstraints>();
  inequalities->push_back(ScalarExpressionInequalityConstraint::LeqZero(
      Double_(Key(7)) - Double_(10.), 1.));
  inequalities->push_back(ScalarExpressionInequalityConstraint::LeqZero(
      Double_(0.) - Double_(Key(7)) - Double_(10.), 1.));
  inequalities->push_back(ScalarExpressionInequalityConstraint::LeqZero(
      Double_(square, Double_(Key(7))) - Double_(1.), 0.25));
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  params->retractorCreator = std::make_shared<BarrierRetractorCreator>();
  params->affineEqualities = true;
  Values values;
  values.insert(Key(7), 1.);
  return IEConstraintManifold(params,
                              std::make_shared<NonlinearEqualityConstraints>(),
                              inequalities, values);
}

// Projection and working-face charts share one derivative evaluation.
TEST(IEManifoldLinearization, SharedAcrossConsumers) {
  derivativeCalls = 0;
  const auto manifold = makeManifold();
  const auto local = manifold.linearization();
  EXPECT_LONGS_EQUAL(1, derivativeCalls);
  EXPECT(local->indices() == IndexSet{2});
  EXPECT(assert_equal(Matrix1::Constant(2.), local->rows(2)));
  EXPECT(assert_equal(Matrix1::Constant(8.), local->rows(2, true)));
  const auto copy = manifold;
  EXPECT(copy.linearization() == local);

  VectorValues ambient;
  ambient.insert(7, Vector1(1.));
  EXPECT(manifold.blockingIndices(ambient) == IndexSet{2});
  const auto projected = manifold.projectTangentCone(Vector(Vector1(1.)));
  EXPECT(projected.first == IndexSet{2});
  EXPECT(assert_equal(Vector1::Zero(), projected.second, 1e-9));
  EXPECT(assert_equal(ambient,
                      manifold.toAmbient(manifold.toCoordinates(ambient))));
  EXPECT(local->violated(Vector1(1.)) == IndexSet{2});
  EXPECT(!local->active(Vector1(1.), IndexSet{2}));
  GaussianFactorGraph cost;
  cost.emplace_shared<JacobianFactor>(41, Matrix1::Identity(), Vector1(1.));
  const auto step =
      solveWorkingFaces(cost, {{41, local->workingFace(IndexSet{2})}}, {});
  EXPECT(assert_equal(Vector1::Zero(), step.at(41), 0.));
  EXPECT_LONGS_EQUAL(1, derivativeCalls);
}

// Even affine equality bases cannot reuse nonlinear inequality geometry at a
// new state. Explicit modes and independently surviving snapshots remain valid.
TEST(IEManifoldLinearization, FreshGeometryAtNewState) {
  derivativeCalls = 0;
  auto manifold = makeManifold();
  const auto oldGeometry = manifold.linearization();
  Values values;
  values.insert(Key(7), 2.);
  const auto next = manifold.createWithNewValues(values, IndexSet{2});
  EXPECT(next.linearization() != oldGeometry);
  EXPECT(next.eBasis() == manifold.eBasis());
  EXPECT_LONGS_EQUAL(2, derivativeCalls);
  EXPECT(assert_equal(Matrix1::Constant(4.), next.linearization()->rows(2)));
  EXPECT(assert_equal(Matrix1::Constant(2.), oldGeometry->rows(2)));
  EXPECT_DOUBLES_EQUAL(3., next.inequalityResidual(2)(0), 0.);
  values.update(Key(7), 0.5);
  manifold = manifold.createWithNewValues(values);
  EXPECT(manifold.linearization()->indices().empty());
  EXPECT_LONGS_EQUAL(2, derivativeCalls);
  EXPECT(assert_equal(Matrix1::Constant(2.), oldGeometry->rows(2)));
}
// Local boundary and dual queries retain original indices and deterministic
// ties.
TEST(IEManifoldLinearization, OriginalRowsAndBoundaryOrder) {
  LinearIConstraintMap constraints;
  constraints.emplace(
      2, std::make_shared<JacobianLinearInequalityConstraint>(
             std::make_shared<JacobianFactor>(
                 7, (Matrix12() << 1., 0.).finished(), Vector1::Zero())));
  constraints.emplace(
      8, std::make_shared<JacobianLinearInequalityConstraint>(
             std::make_shared<JacobianFactor>(
                 7, (Matrix12() << 0., 1.).finished(), Vector1::Zero())));
  const IEManifoldLinearization local(constraints);
  using Row = IEManifoldLinearization::Row;
  using RowSet = IEManifoldLinearization::RowSet;
  const Vector2 current(-2., -1.), direction(4., 2.);
  const auto first = local.boundaryStep(current, direction, {});
  EXPECT_DOUBLES_EQUAL(0.5, first.first, 0.);
  EXPECT(first.second == Row(2, 0));
  const auto bounded = local.boundaryStep(current, direction, {}, 0.25);
  EXPECT_DOUBLES_EQUAL(0.25, bounded.first, 0.);
  EXPECT(!bounded.second);
  const auto second = local.boundaryStep(current, direction, RowSet{{2, 0}});
  EXPECT(second.second == Row(8, 0));
  const auto active = local.initialRows(IndexSet{8}, Vector2(-1., 0.));
  EXPECT(active == RowSet({{8, 0}}));
  EXPECT(local.completeBlocks(active) == IndexSet{8});
  const auto leaving =
      local.leavingRow(Vector2(-3., 2.), RowSet{{2, 0}, {8, 0}});
  EXPECT_DOUBLES_EQUAL(-2., leaving.first, 0.);
  EXPECT(leaving.second == Row(8, 0));
}

}  // namespace component_linearization
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
