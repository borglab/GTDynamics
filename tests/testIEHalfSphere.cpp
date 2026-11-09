/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testIEHalfSphere.cpp
 * @brief Unit tests for the inequality-constrained half-sphere examples.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <gtdynamics/scenarios/DomeRetractor.h>
#include <gtdynamics/scenarios/HalfSphereRetractor.h>
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IERetractor.h>
#include <gtdynamics/cmopt/TangentSpaceBasis.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>

using namespace gtdynamics;
using namespace gtsam;

TEST(HalfSphere, constraints) {
  double r = 3.0;
  IEHalfSphere half_sphere(r);
  half_sphere.sphere_tol_ = 1e-5;
  half_sphere.z_tol_ = 1e-5;

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

  // Create point.
  auto e_constraints = std::make_shared<NonlinearEqualityConstraints>(
      half_sphere.eConstraints(0));
  auto i_constraints = std::make_shared<NonlinearInequalityConstraints>(
      half_sphere.iConstraints(0));
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(retractor);
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();
  Values values1;
  values1.insert(PointKey(0), Point3(3, 0, 0));
  const auto point1Type = std::make_shared<IEConstraintManifold>(
      *e_constraints, *i_constraints, values1, *params);
  CMPoint point1 = point1Type->point(values1);

  // retract
  VectorValues delta1;
  delta1.insert(PointKey(0), Vector3(0, 3, 0));
  auto newPoint = retractor->retract(&point1, delta1);

  // check new manifold
  Values expected_values1;
  expected_values1.insert(PointKey(0), Point3(3 / sqrt(2), 3 / sqrt(2), 0));
  EXPECT(assert_equal(expected_values1, newPoint.values()));

  IndexSet expected_active_indices;
  expected_active_indices.insert(0);
  EXPECT(assert_container_equality(expected_active_indices,
                                   newPoint.activeIndices()));
}

TEST(HalfSphere, Dome) {
  Key point_key = PointKey(0);
  double r = 1;
  IEHalfSphere half_sphere(r);
  auto retractor = std::make_shared<DomeRetractor>(half_sphere);

  Values values;
  values.insert(point_key, Point3(1, 0, 0));

  auto e_constraints = std::make_shared<NonlinearEqualityConstraints>();
  auto i_constraints = std::make_shared<NonlinearInequalityConstraints>(
      half_sphere.iDomeConstraints(0));
  auto params = std::make_shared<IEConstraintManifold::Params>();
  params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(retractor);
  params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();

  const auto pointType = std::make_shared<IEConstraintManifold>(
      *e_constraints, *i_constraints, values, *params);
  CMPoint point = pointType->point(values);

  {
    VectorValues delta;
    delta.insert(point_key, Vector3(0, 1, 0));

    // check projected direction
    IndexSet tight_indices;
    VectorValues proj_delta;
    std::tie(tight_indices, proj_delta) = point.projectTangentCone(delta);
    EXPECT(tight_indices.size() == 0);
    EXPECT(assert_equal(delta, proj_delta));

    // check retraction
    Values expected_values;
    expected_values.insert(point_key, Point3(1 / sqrt(2), 1 / sqrt(2), 0));
    auto newPoint = point.retract(delta, tight_indices);
    EXPECT(assert_equal(expected_values, newPoint.values()));
  }

  {
    VectorValues delta;
    delta.insert(point_key, Vector3(1, 1, 0));

    // check projected direction
    IndexSet tight_indices;
    VectorValues proj_delta;
    std::tie(tight_indices, proj_delta) = point.projectTangentCone(delta);
    // At (1,0,0), the solid dome requires dx<=0. Remove the outward
    // radial component while retaining motion along the boundary.
    EXPECT(tight_indices.size() == 1);
    EXPECT(tight_indices.exists(0));
    VectorValues expected_proj_delta;
    expected_proj_delta.insert(point_key, Vector3(0, 1, 0));
    EXPECT(assert_equal(expected_proj_delta, proj_delta));

    // check retraction
    Values expected_values;
    expected_values.insert(point_key, Point3(1 / sqrt(2), 1 / sqrt(2), 0));
    auto newPoint = point.retract(proj_delta, tight_indices);
    EXPECT(assert_equal(expected_values, newPoint.values()));
  }
  {
    VectorValues inward;
    inward.insert(point_key, Vector3(-1, 1, 0));
    auto [active, projected] = point.projectTangentCone(inward);
    EXPECT(active.empty());
    EXPECT(assert_equal(inward, projected));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
