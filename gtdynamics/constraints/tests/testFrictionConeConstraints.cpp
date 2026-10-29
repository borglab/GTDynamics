/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testFrictionConeConstraints.cpp
 * @brief Direct constraints checked against expression references and AL
 * solves.
 * @author Frank Dellaert
 */

#include "FrictionConeConstraintExpressions.h"

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/constraints/FrictionConeConstraints.h>
#include <gtsam/base/Testable.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace gtsam;
using namespace gtdynamics;

namespace {
const Key key = Symbol('f', 0);
const Vector2 mu(0.5, 0.25);

Values atForce(const Vector3 &force) {
  Values values;
  values.insert(key, force);
  return values;
}

bool sameFactor(const NoiseModelFactor &actual,
                const NoiseModelFactor &expected, const Values &values) {
  std::vector<Matrix> actualH(actual.size()), expectedH(expected.size());
  if (!assert_equal(expected.unwhitenedError(values, &expectedH),
                    actual.unwhitenedError(values, &actualH), 1e-10) ||
      !assert_equal(expected.whitenedError(values),
                    actual.whitenedError(values), 1e-10))
    return false;
  for (size_t i = 0; i < actualH.size(); ++i)
    if (!assert_equal(expectedH[i], actualH[i], 1e-10))
      return false;
  return true;
}

bool equivalent(const NonlinearInequalityConstraint &actual,
                const NonlinearInequalityConstraint &expected,
                const Values &values) {
  std::vector<Matrix> actualH(actual.size()), expectedH(expected.size());
  if (!assert_equal(expected.unwhitenedExpr(values, &expectedH),
                    actual.unwhitenedExpr(values, &actualH), 1e-10) ||
      !assert_equal(expected.whitenedExpr(values), actual.whitenedExpr(values),
                    1e-10))
    return false;
  for (size_t i = 0; i < actualH.size(); ++i)
    if (!assert_equal(expectedH[i], actualH[i], 1e-10))
      return false;
  if (!sameFactor(actual, expected, values) ||
      !sameFactor(*actual.createEqualityConstraint(),
                  *expected.createEqualityConstraint(), values))
    return false;
  for (double weight : {0.25, 4.0}) {
    if (!sameFactor(*actual.penaltyFactor(weight),
                    *expected.penaltyFactor(weight), values) ||
        !sameFactor(*actual.penaltyFactorEquality(weight),
                    *expected.penaltyFactorEquality(weight), values) ||
        !sameFactor(*actual.penaltyFactorCustom(nullptr, weight),
                    *expected.penaltyFactorCustom(nullptr, weight), values))
      return false;
    for (const auto &function :
         std::vector<InequalityPenaltyFunction::shared_ptr>{
             std::make_shared<SmoothRampPoly2>(0.5),
             std::make_shared<SoftPlusFunction>(2.0)}) {
      const auto actualPenalty = actual.penaltyFactorCustom(function, weight);
      const auto expectedPenalty =
          expected.penaltyFactorCustom(function, weight);
      if (!sameFactor(*actualPenalty, *expectedPenalty, values) ||
          !actualPenalty->active(values))
        return false;
    }
  }
  return true;
}

AugmentedLagrangianParams::shared_ptr alParams() {
  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->maxIterations = 100;
  params->storeOptProgress = true;
  params->bclPenaltyIncreaseRate = 10.0;
  // Tighten the inner solve gradually while requiring the same final KKT
  // tolerance.
  params->bclBetaOmega = 0.5;
  params->absoluteViolationTolerance = 1e-8;
  params->absoluteStationarityTolerance = 1e-7;
  params->lmParams.maxIterations = 200;
  params->lmParams.absoluteErrorTol = params->lmParams.relativeErrorTol = 1e-12;
  return params;
}
} // namespace

TEST(FrictionConeConstraints, PyramidEdgesAndForceMap) {
  const auto constraints = PyramidalFrictionConeConstraint::Create(key, 0.5);
  LONGS_EQUAL(4, constraints.size());
  const Matrix34 map = PyramidalFrictionConeConstraint::ForceMatrix(mu);
  for (size_t i = 0; i < 4; ++i) {
    Values values;
    Vector4 edges = Vector4::Zero();
    edges(i) = 2;
    values.insert(key, edges);
    EXPECT(constraints.at(i)->feasible(values));
    DOUBLES_EQUAL(-2, constraints.at(i)->unwhitenedExpr(values)(0), 1e-12);
    DOUBLES_EQUAL(-4, constraints.at(i)->whitenedExpr(values)(0), 1e-12);
    const Vector3 f = map * edges;
    DOUBLES_EQUAL(f(2), std::abs(f(0)) / mu(0) + std::abs(f(1)) / mu(1), 1e-12);
    const auto boundary = constraints.at(i)->createEqualityConstraint();
    EXPECT_CORRECT_FACTOR_JACOBIANS(*boundary->penaltyFactor(1.0), values, 1e-6,
                                    1e-7);
    edges(i) = -2;
    values.update(key, edges);
    EXPECT(!constraints.at(i)->feasible(values));
    EXPECT_CORRECT_FACTOR_JACOBIANS(*boundary->penaltyFactor(1.0), values, 1e-6,
                                    1e-7);
  }
  const Vector4 edges(1, 2, 3, 4);
  EXPECT(assert_equal(Vector3(-0.5, -0.25, 10), Vector3(map * edges)));
}

TEST(FrictionConeConstraints, EllipticAndSignedSquareFeasibleSets) {
  EllipticFrictionConeConstraint elliptic(key, mu);
  SignedSquareFrictionConeConstraint square(key, mu);
  const auto withNormal = EllipticFrictionConeConstraint::Create(key, mu);
  LONGS_EQUAL(2, withNormal.size());
  for (double n : {-2., -0.5, 0., 0.5, 2.}) {
    for (double tx : {-1., -0.25, 0., 0.25, 1.}) {
      for (double ty : {-0.5, -0.125, 0., 0.125, 0.5}) {
        const auto values = atForce(Vector3(tx, ty, n));
        const bool expected = std::hypot(tx / mu(0), ty / mu(1)) <= n;
        EXPECT(expected == elliptic.feasible(values));
        EXPECT(expected == square.feasible(values));
        EXPECT(expected == (withNormal.violationNorm(values) == 0));
      }
    }
  }
  // Unlike an unsigned square, a purely pulling force must be rejected.
  EXPECT(!square.feasible(atForce(Vector3(0, 0, -1))));
  // The diamond pyramid is strictly smaller than the ellipse between its rays.
  const Vector3 f(0.3, 0.15, 1);
  EXPECT(elliptic.feasible(atForce(f)));
  EXPECT(std::abs(f(0)) / mu(0) + std::abs(f(1)) / mu(1) > f(2));
}

TEST(FrictionConeConstraints, BoundaryJacobiansAndScaling) {
  EllipticFrictionConeConstraint elliptic(key, mu, 0.5);
  SignedSquareFrictionConeConstraint square(key, mu, 0.5);
  for (const Vector3 &force : {Vector3(0.1, 0.05, 1), Vector3(0.5, 0, 1),
                               Vector3(1, 0.5, 1), Vector3(0.2, 0.3, -1)}) {
    const auto values = atForce(force);
    for (const auto *c : std::vector<const NonlinearInequalityConstraint *>{
             &elliptic, &square}) {
      const auto boundary = c->createEqualityConstraint();
      EXPECT_CORRECT_FACTOR_JACOBIANS(*boundary->penaltyFactor(1.0), values,
                                      1e-6, 1e-7);
      EXPECT(assert_equal(c->unwhitenedExpr(values),
                          boundary->unwhitenedError(values)));
      EXPECT(assert_equal(Vector(2 * c->unwhitenedExpr(values)),
                          c->whitenedExpr(values)));
      EXPECT(assert_equal(
          c->unwhitenedExpr(values),
          std::dynamic_pointer_cast<NonlinearInequalityConstraint>(c->clone())
              ->unwhitenedExpr(values)));
    }
  }
  // Inside the cone the signed boundary must retain a nonzero derivative,
  // even though the clamped inequality penalty and its derivative are zero.
  const auto values = atForce(Vector3(0.1, 0.05, 1));
  std::vector<Matrix> H(1), clamped(1);
  EXPECT(elliptic.unwhitenedExpr(values, &H)(0) < 0);
  DOUBLES_EQUAL(-1, H[0](0, 2), 1e-12);
  DOUBLES_EQUAL(0, elliptic.unwhitenedError(values, &clamped)(0), 1e-12);
  DOUBLES_EQUAL(0, clamped[0].norm(), 1e-12);
}

TEST(FrictionConeConstraints, ApexConventions) {
  EllipticFrictionConeConstraint elliptic(key, mu);
  SignedSquareFrictionConeConstraint square(key, mu);
  const auto zero = atForce(Vector3::Zero());
  std::vector<Matrix> H(1);
  DOUBLES_EQUAL(0, elliptic.unwhitenedExpr(zero, &H)(0), 1e-12);
  EXPECT(assert_equal(Matrix((Matrix13() << 0, 0, -1).finished()), H[0]));
  DOUBLES_EQUAL(0, square.unwhitenedExpr(zero, &H)(0), 1e-12);
  EXPECT(assert_equal(Matrix(Matrix13::Zero()), H[0]));
  // A tiny nonzero tangential force still has the directional norm derivative.
  elliptic.unwhitenedExpr(atForce(Vector3(1e-16, 0, 1)), &H);
  DOUBLES_EQUAL(1 / mu(0), H[0](0, 0), 1e-12);
  // The normal derivative has the same negative sign on both sides of zero.
  square.unwhitenedExpr(atForce(Vector3(0, 0, -2)), &H);
  DOUBLES_EQUAL(-1, H[0](0, 2), 1e-12);
}

TEST(FrictionConeConstraints, ExpressionReferences) {
  // Include feasible, tight, violated, negative-normal, and zero-force cases.
  for (double sigma : {0.2, 1.0, 3.0}) {
    const auto normal =
        EllipticFrictionConeConstraint::Create(key, mu, sigma).at(0);
    EllipticFrictionConeConstraint elliptic(key, mu, sigma);
    SignedSquareFrictionConeConstraint square(key, mu, sigma);
    const std::vector<std::pair<const NonlinearInequalityConstraint *,
                                NonlinearInequalityConstraint::shared_ptr>>
        pairs{{normal.get(), friction_test::Normal(key, sigma)},
              {&elliptic, friction_test::Elliptic(key, mu, sigma)},
              {&square, friction_test::SignedSquare(key, mu, sigma)}};
    for (const Vector3 &f :
         {Vector3(0.1, 0.05, 1), Vector3(0.5, 0, 1), Vector3(1, 0.5, 1),
          Vector3(0.2, 0.3, -1), Vector3(0, 0, -1), Vector3(0, 0, 0),
          Vector3(0.2, 0.1, 0)}) {
      const Values values = atForce(f);
      for (const auto &[direct, reference] : pairs) {
        EXPECT(equivalent(*direct, *reference, values));
      }
    }
    for (size_t edge = 0; edge < 4; ++edge) {
      PyramidalFrictionConeConstraint direct(key, edge, sigma);
      const auto reference = friction_test::Pyramid(key, edge, sigma);
      for (double coefficient : {-1.0, 0.0, 1.0}) {
        Vector4 e = Vector4::Constant(0.5);
        e(edge) = coefficient;
        Values values;
        values.insert(key, e);
        EXPECT(equivalent(direct, *reference, values));
      }
    }
  }
}

TEST(FrictionConeConstraints, BoundaryLifetimeAndRekeying) {
  // Boundary and penalty objects own their data; they must not borrow a
  // reference to a temporary constraint or depend on its original key.
  const auto boundary = SignedSquareFrictionConeConstraint(key, mu, 0.2)
                            .createEqualityConstraint();
  const auto penalty =
      EllipticFrictionConeConstraint(key, mu, 0.2)
          .penaltyFactorCustom(std::make_shared<SoftPlusFunction>(), 4.0);
  const Values values = atForce(Vector3(0.2, 0.1, 1));
  EXPECT(assert_equal(
      friction_test::SignedSquare(key, mu, 0.2)->whitenedExpr(values),
      boundary->whitenedError(values)));
  EXPECT(assert_equal(
      friction_test::Elliptic(key, mu, 0.2)
          ->penaltyFactorCustom(std::make_shared<SoftPlusFunction>(), 4.0)
          ->whitenedError(values),
      penalty->whitenedError(values)));
  const Key newKey = Symbol('f', 1);
  Values rekeyed;
  rekeyed.insert(newKey, values.at<Vector3>(key));
  for (const auto &factor :
       std::vector<NoiseModelFactor::shared_ptr>{boundary, penalty}) {
    const auto byKeys = std::dynamic_pointer_cast<NoiseModelFactor>(
        factor->rekey(KeyVector{newKey}));
    const auto byMap = std::dynamic_pointer_cast<NoiseModelFactor>(
        factor->rekey(std::map<Key, Key>{{key, newKey}}));
    EXPECT(assert_equal(factor->whitenedError(values),
                        byKeys->whitenedError(rekeyed)));
    EXPECT(assert_equal(factor->whitenedError(values),
                        byMap->whitenedError(rekeyed)));
  }
  EllipticFrictionConeConstraint direct(key, mu);
  EXPECT(assert_equal(direct.unwhitenedExpr(values),
                      std::dynamic_pointer_cast<NonlinearInequalityConstraint>(
                          direct.rekey(KeyVector{newKey}))
                          ->unwhitenedExpr(rekeyed)));
}

TEST(FrictionConeConstraints, InvalidParameters) {
  for (double invalid : {0., -1., std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::quiet_NaN()}) {
    CHECK_EXCEPTION(EllipticFrictionConeConstraint(key, Vector2(invalid, 1)),
                    std::invalid_argument);
    CHECK_EXCEPTION(
        SignedSquareFrictionConeConstraint(key, Vector2(1, invalid)),
        std::invalid_argument);
    CHECK_EXCEPTION(
        PyramidalFrictionConeConstraint::ForceMatrix(Vector2(invalid, 1)),
        std::invalid_argument);
    CHECK_EXCEPTION(PyramidalFrictionConeConstraint(key, 0, invalid),
                    std::invalid_argument);
    CHECK_EXCEPTION(EllipticFrictionConeConstraint(key, mu, invalid),
                    std::invalid_argument);
    CHECK_EXCEPTION(SignedSquareFrictionConeConstraint(key, mu, invalid),
                    std::invalid_argument);
  }
  CHECK_EXCEPTION(PyramidalFrictionConeConstraint(key, 4),
                  std::invalid_argument);
}

TEST(FrictionConeConstraints, AugmentedLagrangianProjection) {
  // Orthogonal projection of (2,0,0) onto the unit circular cone is (1,0,1).
  NonlinearFactorGraph cost;
  cost.addPrior<Vector3>(key, Vector3(2, 0, 0), noiseModel::Unit::Create(3));
  const Values initial = atForce(Vector3(0.2, 0, 1));
  for (bool signedSquare : {false, true}) {
    auto inequalities =
        EllipticFrictionConeConstraint::Create(key, Vector2::Ones());
    if (signedSquare) {
      inequalities.resize(0);
      inequalities.emplace_shared<SignedSquareFrictionConeConstraint>(
          key, Vector2::Ones());
    }
    const ConstrainedOptProblem problem(cost, {}, inequalities);
    AugmentedLagrangianOptimizer optimizer(problem, initial, alParams());
    const Values result = optimizer.optimize();
    if (!optimizer.progress().empty() &&
        !optimizer.progress().back().converged) {
      const auto &state = optimizer.progress().back();
      std::cerr << "signedSquare=" << signedSquare
                << " iterations=" << state.iteration
                << " stationarity=" << state.augmentedLagrangianStationarity
                << " violation=" << state.generalizedConstraintViolation
                << " inner=" << state.innerConverged << "\n";
    }
    EXPECT(!optimizer.progress().empty() &&
           optimizer.progress().back().converged);
    EXPECT(assert_equal(Vector3(1, 0, 1), result.at<Vector3>(key), 1e-5));
    EXPECT(inequalities.violationNorm(result) < 1e-7);
  }
  // Projection of edge coefficients onto the positive orthant clips negatives.
  NonlinearFactorGraph edgeCost;
  edgeCost.addPrior<Vector4>(key, Vector4(-1, 2, -3, 4),
                             noiseModel::Unit::Create(4));
  Values edgeInitial;
  edgeInitial.insert<Vector4>(key, Vector4::Ones());
  const auto inequalities = PyramidalFrictionConeConstraint::Create(key);
  const ConstrainedOptProblem problem(edgeCost, {}, inequalities);
  const Values result =
      AugmentedLagrangianOptimizer(problem, edgeInitial, alParams()).optimize();
  EXPECT(assert_equal(Vector4(0, 2, 0, 4), result.at<Vector4>(key), 1e-6));
  EXPECT(inequalities.violationNorm(result) < 1e-7);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
