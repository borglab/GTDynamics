

#include "gtdynamics/cmcopt/IERetractor.h"
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/constraints/InequalityConstraint.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>
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
  Key point_key = gtsam::Symbol('p', 0);

  gtsam::Expression<Point3> point_expr(point_key);
  gtsam::Expression<double> sphere_expr(&norm3, point_expr);
  gtsam::Expression<double> z_expr(&point3_z, point_expr);

  auto e_constraints = std::make_shared<EqualityConstraints>();
  auto i_constraints = std::make_shared<InequalityConstraints>();
  e_constraints->emplace_shared<DoubleExpressionEquality>(sphere_expr, 1.0);
  i_constraints->emplace_shared<DoubleExpressionInequality>(z_expr, 1.0);

  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->ecm_params = std::make_shared<ConstraintManifold::Params>();
  params->retractor_creator = std::make_shared<BarrierRetractorCreator>();
  params->e_basis_creator = std::make_shared<OrthonormalBasisCreator>();

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
    VectorValues tangent_vector;
    tangent_vector.insert(point_key, Vector3(8, -6, 2));
    Vector xi = manifold.eBasis()->computeXi(tangent_vector);
    VectorValues delta;
    delta.insert(manifold_key, xi);
    auto linear_base_i_constraints = manifold.linearActiveBaseIConstraints();
    auto linear_manifold_i_constraints = manifold.linearActiveManIConstraints(manifold_key);

    for (const auto&[idx, base_constraint]: linear_base_i_constraints) {
      auto manifold_constraint = linear_manifold_i_constraints.at(idx);
      auto base_constraint_eval = (*base_constraint)(tangent_vector);
      auto manifold_constraint_eval = (*manifold_constraint)(delta);
      EXPECT(assert_equal(Vector1(2.0), base_constraint_eval));
      EXPECT(assert_equal(Vector1(2.0), manifold_constraint_eval));
    }
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
