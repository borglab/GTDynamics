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
#include <gtsam/base/TestableAssertions.h>

/* ************************************************************************* */
namespace component_linearization {
using namespace gtsam;
using namespace gtdynamics;
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

// Projection and QP row access share one derivative evaluation.
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
}  // namespace component_linearization
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
