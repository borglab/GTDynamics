

#include "gtdynamics/cmcopt/IERetractor.h"
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/cmcopt/IEConstraintManifold.h>

using namespace gtdynamics;
using namespace gtsam;

template <typename Container>
bool container_equal(const Container &c1, const Container &c2) {
  if (c1.size() != c2.size()) {
    return false;
  }
  return std::equal(c1.begin(), c1.end(), c2.begin());
}

TEST(IEConstraintManifold, HalfSphere) {
  Key point_key = Symbol('p', 0);

  Expression<Point3> point_expr(point_key);
  Expression<double> sphere_expr(&norm3, point_expr);
  Expression<double> z_expr(&point3_z, point_expr);

  auto e_constraints = std::make_shared<NonlinearEqualityConstraints>();
  auto i_constraints = std::make_shared<NonlinearInequalityConstraints>();
  e_constraints->emplace_shared<ExpressionEqualityConstraint<double>>(
      sphere_expr, 1.0, Vector1(1.0));
  i_constraints->push_back(
      ScalarExpressionInequalityConstraint::GeqZero(z_expr, 1.0));

  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->equalityManifoldParams = std::make_shared<ConstraintManifold::Params>();
  params->retractorCreator = std::make_shared<BarrierRetractorCreator>();
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();

  {
    Values values;
    values.insert(point_key, Point3(0.6, 0.8, 0));

    // Test constructor.
    IEConstraintManifold manifold(params, e_constraints, i_constraints, values);

    // Test active constraints.
    auto active_indices = manifold.activeIndices();
    EXPECT(active_indices.size() == 1);

    // Test project tangent cone.
    // manifold.eBasis()->print();
    Vector2 xi1(1, 1);
    Vector2 xi2(1, 0);
    Vector2 xi3(1, -1);

    IndexSet blocking_indices;
    Vector projected_xi;
    VectorValues projected_tv;
    std::tie(blocking_indices, projected_xi) = manifold.projectTangentCone(xi1);
    Vector2 expected_proj_xi1(1, 1);
    EXPECT(assert_equal(expected_proj_xi1, projected_xi));
    EXPECT(blocking_indices.size() == 0);
    VectorValues tv1;
    tv1.insert(point_key, Vector3(1, -0.75, 1));
    std::tie(blocking_indices, projected_tv) = manifold.projectTangentCone(tv1);
    VectorValues expected_proj_tv1;
    expected_proj_tv1.insert(point_key, Vector3(1, -0.75, 1));
    EXPECT(assert_equal(expected_proj_tv1, projected_tv));

    std::tie(blocking_indices, projected_xi) = manifold.projectTangentCone(xi2);
    Vector2 expected_proj_xi2(1, 0);
    EXPECT(assert_equal(expected_proj_xi2, projected_xi));

    std::tie(blocking_indices, projected_xi) = manifold.projectTangentCone(xi3);
    Vector2 expected_proj_xi3(1, 0);
    EXPECT(assert_equal(expected_proj_xi3, projected_xi));
    EXPECT(blocking_indices.size() == 1);
    VectorValues tv3;
    tv3.insert(point_key, Vector3(1, -0.75, -1));
    std::tie(blocking_indices, projected_tv) = manifold.projectTangentCone(tv3);
    VectorValues expected_proj_tv3;
    expected_proj_tv3.insert(point_key, Vector3(1, -0.75, 0));
    EXPECT(assert_equal(expected_proj_tv3, projected_tv));

    // Test blocking constraints
    VectorValues delta1, delta2, delta3;
    delta1.insert(point_key, Vector3(8, -6, 1));
    delta2.insert(point_key, Vector3(8, -6, 0));
    delta3.insert(point_key, Vector3(8, -6, -1));
    EXPECT(manifold.blockingIndices(delta1).size() == 0);
    EXPECT(manifold.blockingIndices(delta2).size() == 0);
    EXPECT(manifold.blockingIndices(delta3).size() == 1);

    // Test construct emanifold
    auto e_manifold = manifold.eConstraintManifold();
    EXPECT(e_manifold.dim() == 2);
    IndexSet a_indices;
    a_indices.insert(0);
    auto e_manifold1 = manifold.eConstraintManifold(a_indices);
    EXPECT(e_manifold1.dim() == 1);

    // Test linear i-constraints
    Key manifold_key = 3;
    VectorValues tangentVector;
    tangentVector.insert(point_key, Vector3(8, -6, 2));
    Vector xi = manifold.eBasis()->computeXi(tangentVector);
    VectorValues delta;
    delta.insert(manifold_key, xi);
    auto linearBaseInequalityConstraints = manifold.linearActiveBaseInequalityConstraints();
    auto linearManifoldInequalityConstraints = manifold.linearActiveManifoldInequalityConstraints(manifold_key);

    for (const auto&[idx, base_constraint]: linearBaseInequalityConstraints) {
      auto manifold_constraint = linearManifoldInequalityConstraints.at(idx);
      auto base_constraint_eval = (*base_constraint)(tangentVector);
      auto manifold_constraint_eval = (*manifold_constraint)(delta);
      EXPECT(assert_equal(Vector1(-2.0), base_constraint_eval));
      EXPECT(assert_equal(Vector1(-2.0), manifold_constraint_eval));
    }
  }
}

TEST(IEConstraintManifold, PenaltyRetractionRetainsNewBoundary) {
  const Key key = 7;
  auto inequalities = std::make_shared<NonlinearInequalityConstraints>();
  inequalities->push_back(ScalarExpressionInequalityConstraint::GeqZero(
      Expression<double>(key), 1.0));
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  auto retractionParams = std::make_shared<IERetractorParams>();
  retractionParams->ensureFeasible = true;
  retractionParams->feasibleThreshold = 1e-8;
  params->retractorCreator =
      std::make_shared<BarrierRetractorCreator>(retractionParams);
  Values values;
  values.insert(key, 1.0);
  const IEConstraintManifold manifold(
      params, std::make_shared<NonlinearEqualityConstraints>(), inequalities,
      values);
  EXPECT(manifold.activeIndices().empty());
  VectorValues step;
  step.insert(key, Vector1(-2.0));
  const auto boundary = manifold.retract(step);
  EXPECT_DOUBLES_EQUAL(0.0, boundary.values().at<double>(key), 1e-8);
  // A feasible boundary point still needs its face in the next tangent QP.
  EXPECT(boundary.activeIndices().exists(0));
  const auto [blocking, projected] = boundary.projectTangentCone(step);
  EXPECT(blocking.exists(0));
  EXPECT_DOUBLES_EQUAL(0.0, projected.at(key)(0), 1e-8);
  step.at(key) = Vector1(-0.5);
  const auto interior = manifold.retract(step);
  EXPECT_DOUBLES_EQUAL(0.5, interior.values().at<double>(key), 1e-8);
  EXPECT(interior.activeIndices().empty());
}

// Constraint sigma must be applied once, and must not change the feasible side.
TEST(IEConstraintManifold, ScaledInequalityLinearization) {
  const Key key = 7;
  const Key manifoldKey = 9;
  for (const double sigma : {0.01, 3.0}) {
    for (const bool lowerBound : {false, true}) {
      auto inequalities = std::make_shared<NonlinearInequalityConstraints>();
      const Expression<double> expression(key);
      inequalities->push_back(lowerBound
          ? ScalarExpressionInequalityConstraint::GeqZero(expression, sigma)
          : ScalarExpressionInequalityConstraint::LeqZero(expression, sigma));
      auto params = std::make_shared<IEConstraintManifold::Params>();
      params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
      params->retractorCreator = std::make_shared<BarrierRetractorCreator>();
      Values values;
      values.insert(key, 0.0);
      const IEConstraintManifold manifold(
          params, std::make_shared<NonlinearEqualityConstraints>(), inequalities,
          values);
      VectorValues outward, inward;
      outward.insert(key, Vector1(lowerBound ? -2.0 : 2.0));
      inward.insert(key, -outward.at(key));
      VectorValues coordinates;
      coordinates.insert(manifoldKey, manifold.eBasis()->computeXi(outward));
      const auto base = manifold.linearActiveBaseInequalityConstraints().at(0);
      const auto reduced =
          manifold.linearActiveManifoldInequalityConstraints(manifoldKey).at(0);
      EXPECT_DOUBLES_EQUAL(2.0 / sigma, (*base)(outward)(0), 1e-8);
      EXPECT_DOUBLES_EQUAL(2.0 / sigma, (*reduced)(coordinates)(0), 1e-8);
      EXPECT(!base->feasible(outward));
      EXPECT(base->feasible(inward));
      EXPECT(manifold.blockingIndices(outward).exists(0));
      EXPECT(manifold.blockingIndices(inward).empty());
      const auto [blocking, projected] = manifold.projectTangentCone(outward);
      EXPECT(blocking.exists(0));
      EXPECT_DOUBLES_EQUAL(0.0, projected.at(key)(0), 1e-8);
      const auto [released, unchanged] = manifold.projectTangentCone(inward);
      EXPECT(released.empty());
      EXPECT(assert_equal(inward, unchanged, 1e-8));
    }
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
