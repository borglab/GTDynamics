
#include "gtdynamics/cmcopt/IERetractor.h"
#include "gtdynamics/cmopt/TangentSpaceBasis.h"
#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/NonlinearInequalityConstraint.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtdynamics/scenarios/IECartPoleWithFriction.h>
#include <gtdynamics/scenarios/IEHalfSphere.h>
#include <gtdynamics/cmcopt/IEOptimizer.h>

using namespace gtdynamics;
using namespace gtsam;

TEST(identifyManifolds, HalfSphere) {
  IEHalfSphere half_sphere;
  size_t num_steps = 2;

  NonlinearEqualityConstraints e_constraints;
  NonlinearInequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    e_constraints.add(half_sphere.eConstraints(k));
    i_constraints.add(half_sphere.iConstraints(k));
  }

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    values.insert(PointKey(k), Point3(0, 1, 0));
  }

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<HalfSphereRetractor>(half_sphere));
  iecm_params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();

  auto manifolds = IEOptimizer::identifyManifolds(e_constraints, i_constraints,
                                                  values, iecm_params);
  for (const auto &it : manifolds) {
    const auto &manifold = it.second;
    EXPECT_LONGS_EQUAL(1, manifold.iConstraints()->size());
    EXPECT_LONGS_EQUAL(1, manifold.eConstraints()->size());
    EXPECT_LONGS_EQUAL(1, manifold.values().size());
  }
  EXPECT_LONGS_EQUAL(num_steps + 1, manifolds.size());
}

TEST(identifyManifolds, CartPoleWithFriction) {
  IECartPoleWithFriction cp;
  size_t num_steps = 2;

  NonlinearEqualityConstraints e_constraints;
  NonlinearInequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    e_constraints.add(cp.eConstraints(k));
    i_constraints.add(cp.iConstraints(k));
  }

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    values.insert(cp.defaultValues(k));
  }

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<CartPoleWithFrictionRetractor>(cp));
  iecm_params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();

  auto manifolds = IEOptimizer::identifyManifolds(e_constraints, i_constraints,
                                                  values, iecm_params);
  for (const auto &it : manifolds) {
    const auto &manifold = it.second;
    EXPECT_LONGS_EQUAL(2, manifold.iConstraints()->size());
    EXPECT_LONGS_EQUAL(3, manifold.eConstraints()->size());
    EXPECT_LONGS_EQUAL(6, manifold.values().size());
  }
  EXPECT_LONGS_EQUAL(num_steps + 1, manifolds.size());
}

TEST(identifyManifolds, Dome) {
  IEHalfSphere half_sphere;
  size_t num_steps = 2;

  NonlinearEqualityConstraints e_constraints;
  NonlinearInequalityConstraints i_constraints;
  for (size_t k = 0; k <= num_steps; k++) {
    i_constraints.add(half_sphere.iDomeConstraints(k));
  }

  auto iecm_params = std::make_shared<IEConstraintManifold::Params>();
  iecm_params->retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<DomeRetractor>(half_sphere));
  iecm_params->equalityBasisCreator = std::make_shared<OrthonormalBasisCreator>();

  Values values;
  for (size_t k = 0; k <= num_steps; k++) {
    values.insert(PointKey(k), Point3(0, 1, 0));
  }

  auto manifolds = IEOptimizer::identifyManifolds(e_constraints, i_constraints,
                                                  values, iecm_params);
  for (const auto &it : manifolds) {
    const auto &manifold = it.second;
    EXPECT_LONGS_EQUAL(2, manifold.iConstraints()->size());
    EXPECT_LONGS_EQUAL(0, manifold.eConstraints()->size());
    EXPECT_LONGS_EQUAL(1, manifold.values().size());
  }
  EXPECT_LONGS_EQUAL(num_steps + 1, manifolds.size());
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
