/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testCMTangentSpace.cpp
 * @brief Verify state-owned geometry, index mapping and cache invalidation.
 * @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/BarrierRetractor.h>
#include <gtdynamics/cmcopt/CMPoint.h>
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
CMPoint makePoint() {
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
  const auto type = std::make_shared<IEConstraintManifold>(
      NonlinearEqualityConstraints{}, *inequalities, values, *params);
  return type->point(values);
}

// Projection and QP row access share one derivative evaluation.
TEST(CMTangentSpace, SharedAcrossConsumers) {
  derivativeCalls = 0;
  const auto point = makePoint();
  const auto local = point.tangentSpace();
  EXPECT_LONGS_EQUAL(1, derivativeCalls);
  EXPECT(local->indices() == IndexSet{2});
  EXPECT(assert_equal(Matrix1::Constant(2.), local->rows(2)));
  EXPECT(assert_equal(Matrix1::Constant(8.), local->rows(2, true)));
  const auto copy = point;
  EXPECT(copy.tangentSpace() == local);

  VectorValues ambient;
  ambient.insert(7, Vector1(1.));
  EXPECT(point.blockingIndices(ambient) == IndexSet{2});
  const auto projected = point.projectTangentCone(Vector(Vector1(1.)));
  EXPECT(projected.first == IndexSet{2});
  EXPECT(assert_equal(Vector1::Zero(), projected.second, 1e-9));
  EXPECT(assert_equal(ambient,
                      point.toAmbient(point.toCoordinates(ambient))));
  EXPECT(local->violated(Vector1(1.)) == IndexSet{2});
  EXPECT(!local->active(Vector1(1.), IndexSet{2}));
  EXPECT_LONGS_EQUAL(1, derivativeCalls);
}

// Even affine equality bases cannot reuse nonlinear inequality geometry at a
// new state. Explicit modes and independently surviving snapshots remain valid.
TEST(CMTangentSpace, FreshGeometryAtNewState) {
  derivativeCalls = 0;
  auto point = makePoint();
  const auto oldGeometry = point.tangentSpace();
  Values values;
  values.insert(Key(7), 2.);
  const auto next = point.manifold()->point(point.bindings(), values, IndexSet{2});
  EXPECT(next.tangentSpace() != oldGeometry);
  EXPECT(assert_equal(point.toAmbient(Vector::Ones(point.dim())),
                        next.toAmbient(Vector::Ones(next.dim()))));
  EXPECT_LONGS_EQUAL(2, derivativeCalls);
  EXPECT(assert_equal(Matrix1::Constant(4.), next.tangentSpace()->rows(2)));
  EXPECT(assert_equal(Matrix1::Constant(2.), oldGeometry->rows(2)));
  EXPECT_DOUBLES_EQUAL(3., next.inequalityResidual(2)(0), 0.);
  values.update(Key(7), 0.5);
  point = point.manifold()->point(point.bindings(), values);
  EXPECT(point.tangentSpace()->indices().empty());
  EXPECT_LONGS_EQUAL(2, derivativeCalls);
  EXPECT(assert_equal(Matrix1::Constant(2.), oldGeometry->rows(2)));
}
}  // namespace component_linearization
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
