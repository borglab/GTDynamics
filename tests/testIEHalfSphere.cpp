

#include "gtdynamics/imanifold/IERetractor.h"
#include "gtdynamics/manifold/ConnectedComponent.h"
#include "gtdynamics/optimizer/EqualityConstraint.h"
#include "gtdynamics/optimizer/InequalityConstraint.h"
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtdynamics/imanifold/IEHalfSphere.h>

using namespace gtdynamics;
using namespace gtsam;

TEST(HalfSphere, constraints) {
  double r = 3.0;
  IEHalfSphere half_sphere(r);
  half_sphere.sphere_tol = 1e-5;
  half_sphere.z_tol = 1e-5;

  auto e_constraints = half_sphere.eConstraints(0);
  auto i_constraints = half_sphere.iConstraints(0);

  Values values1;
  values1.insert(PointKey(0), Point3(3, 2, 1));

  Values values2;
  values2.insert(PointKey(0), Point3(2.4, 1.8, 0));

  Values values3;
  values3.insert(PointKey(0), Point3(2.4, 0, -1.8));

  EXPECT(!e_constraints.at(0)->feasible(values1));
  EXPECT(e_constraints.at(0)->feasible(values2));
  EXPECT(e_constraints.at(0)->feasible(values3));

  EXPECT(i_constraints.at(0)->feasible(values1));
  EXPECT(i_constraints.at(0)->feasible(values2));
  EXPECT(!i_constraints.at(0)->feasible(values3));
}

TEST(HalfSphereRetractor, retract) {
  // Create retractor.
  double r = 3.0;
  IEHalfSphere half_sphere(r);
  auto retractor = std::make_shared<HalfSphereRetractor>(half_sphere);

  // Create manifold.
  auto e_constraints = half_sphere.eConstraints(0);
  auto i_constraints =
      std::make_shared<InequalityConstraints>(half_sphere.iConstraints(0));
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(retractor);
  params->e_basis_creator =
      std::make_shared<TspaceBasisCreator>(params->ecm_params->basis_params);
  auto e_cc = std::make_shared<ConnectedComponent>(e_constraints);
  Values values1;
  values1.insert(PointKey(0), Point3(3, 0, 0));
  IEConstraintManifold manifold1(params, e_cc, i_constraints, values1);

  // retract
  VectorValues delta1;
  delta1.insert(PointKey(0), Vector3(0, 3, 0));
  auto new_manifold = retractor->retract(&manifold1, delta1);

  // check new manifold
  Values expected_values1;
  expected_values1.insert(PointKey(0), Point3(3 / sqrt(2), 3 / sqrt(2), 0));
  EXPECT(assert_equal(expected_values1, new_manifold.values()));

  IndexSet expected_active_indices;
  expected_active_indices.insert(0);
  EXPECT(assert_container_equality(expected_active_indices,
                                   new_manifold.activeIndices()));
}

TEST(HalfSphere, Dome) {
  Key point_key = PointKey(0);
  double r = 1;
  IEHalfSphere half_sphere(r);
  auto retractor = std::make_shared<DomeRetractor>(half_sphere);

  Values values;
  values.insert(point_key, Point3(1, 0, 0));

  EqualityConstraints e_constraints;
  auto i_constraints =
      std::make_shared<InequalityConstraints>(half_sphere.iDomeConstraints(0));
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->retractor_creator =
      std::make_shared<UniversalIERetractorCreator>(retractor);
  params->e_basis_creator =
      std::make_shared<TspaceBasisCreator>(params->ecm_params->basis_params);
  auto e_cc = std::make_shared<ConnectedComponent>(e_constraints,
                                                   i_constraints->keys());

  IEConstraintManifold manifold(params, e_cc, i_constraints, values);

  {
    VectorValues delta;
    delta.insert(point_key, Vector3(0, 1, 0));

    // check projected direction
    IndexSet tight_indices;
    VectorValues proj_delta;
    std::tie(tight_indices, proj_delta) = manifold.projectTangentCone(delta);
    EXPECT(tight_indices.size() == 0);
    EXPECT(assert_equal(delta, proj_delta));

    // check retraction
    Values expected_values;
    expected_values.insert(point_key, Point3(1 / sqrt(2), 1 / sqrt(2), 0));
    auto new_manifold = manifold.retract(delta, tight_indices);
    EXPECT(assert_equal(expected_values, new_manifold.values()));
  }

  {
    VectorValues delta;
    delta.insert(point_key, Vector3(1, 1, 0));

    // check projected direction
    IndexSet tight_indices;
    VectorValues proj_delta;
    std::tie(tight_indices, proj_delta) = manifold.projectTangentCone(delta);
    EXPECT(tight_indices.size() == 1);
    EXPECT(tight_indices.find(0) != tight_indices.end());
    VectorValues expected_proj_delta;
    expected_proj_delta.insert(point_key, Vector3(0, 1, 0));
    EXPECT(assert_equal(expected_proj_delta, proj_delta));

    // check retraction
    Values expected_values;
    expected_values.insert(point_key, Point3(1 / sqrt(2), 1 / sqrt(2), 0));
    auto new_manifold = manifold.retract(proj_delta, tight_indices);
    EXPECT(assert_equal(expected_values, new_manifold.values()));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
