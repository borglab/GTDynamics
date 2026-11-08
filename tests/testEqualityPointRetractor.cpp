/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file testEqualityPointRetractor.cpp
 * @brief Existing equality retractions through the shared point interfaces.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmopt/ECMPoint.h>
#include <gtdynamics/cmopt/EConstraintManifold.h>
#include <gtdynamics/cmopt/BasisRetractor.h>
#include <gtdynamics/cmopt/DynamicsRetractor.h>
#include <gtdynamics/cmopt/ProjectionRetractor.h>
#include <gtdynamics/cmopt/UnconstrainedOptimizationRetractor.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/PriorFactor.h>

#include <atomic>
#include <future>

using namespace gtsam;
using namespace gtdynamics;

namespace {
// A nonlinear equality whose tangent update needs a genuine retraction.
class Parabola : public NoiseModelFactor2<double, double> {
 public:
  inline static std::atomic<size_t> clones{0};

  Parabola(Key x, Key y)
      : NoiseModelFactor2<double, double>(noiseModel::Unit::Create(1), x, y) {}

  NonlinearFactor::shared_ptr clone() const override {
    ++clones;
    return std::make_shared<Parabola>(*this);
  }

  Vector evaluateError(const double &x, const double &y,
                       OptionalMatrixType Hx = nullptr,
                       OptionalMatrixType Hy = nullptr) const override {
    if (Hx) *Hx = Matrix1::Constant(-2 * x);
    if (Hy) *Hy = Matrix1::Identity();
    return Vector1(y - x * x);
  }
};

BasisKeyFunction selectX() {
  return [](const KeyVector &) { return KeyVector{1}; };
}

EConstraintManifold::shared_ptr parabolaType(
    const RetractorCreator::shared_ptr &policy) {
  Values prototype;
  prototype.insert(1, 1.0);
  prototype.insert(2, 1.0);
  NonlinearEqualityConstraints constraints;
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(1, 2));
  EConstraintManifold::Params params;
  params.equalityBasisCreator =
      std::make_shared<EliminationBasisCreator>(selectX());
  params.retractorCreator = policy;
  return std::make_shared<EConstraintManifold>(constraints, prototype, params);
}

RetractionParams::shared_ptr preciseParams() {
  auto params = std::make_shared<RetractionParams>();
  params->feasibleThreshold = 1e-16;
  params->lmParams.setAbsoluteErrorTol(1e-14);
  params->lmParams.setRelativeErrorTol(1e-14);
  params->lmParams.setMaxIterations(40);
  return params;
}
}  // namespace

TEST(EqualityPointRetractor, ExistingPoliciesAndConfigurationSnapshots) {
  for (size_t variant = 0; variant < 3; ++variant) {
    auto settings = preciseParams();
    RetractorCreator::shared_ptr policy;
    if (variant == 0)
      policy = std::make_shared<BasisRetractorCreator>(selectX(), settings);
    else if (variant == 1)
      policy = std::make_shared<ProjectionRetractorCreator>(settings);
    else
      policy =
          std::make_shared<UnconstrainedOptimizationRetractorCreator>(settings);
    const auto type = parabolaType(policy);

    // Mutating caller-owned options cannot change an existing runtime type.
    settings->feasibleThreshold = 1e10;
    settings->lmParams.setMaxIterations(0);
    Values values;
    values.insert(101, 1.0);
    values.insert(102, 1.0);
    const auto point = type->point({{1, 101}, {2, 102}}, values);
    const auto next = point.retract(Vector(Vector1(.3)));
    EXPECT(next.evaluateEqualityViolation() < 1e-7);
    EXPECT(next.values().atDouble(101) > 1.0);
    EXPECT(point.values().equals(values));
    EXPECT(next.manifold().get() == type.get());
    EXPECT(next.bindings() == point.bindings());
    if (variant == 0) {
      EXPECT_DOUBLES_EQUAL(1.3, next.values().atDouble(101), 1e-10);
      EXPECT_DOUBLES_EQUAL(1.69, next.values().atDouble(102), 1e-8);
    }
  }
}

TEST(EqualityPointRetractor, CapturedSelectorsAreResolvedAtTypeConstruction) {
  auto selected = std::make_shared<KeyVector>(KeyVector{1});
  auto calls = std::make_shared<size_t>(0);
  BasisKeyFunction selector = [selected, calls](const KeyVector &) {
    ++*calls;
    return *selected;
  };
  const auto type = parabolaType(
      std::make_shared<BasisRetractorCreator>(selector, preciseParams()));
  EXPECT_LONGS_EQUAL(1, *calls);
  *selected = KeyVector{2};
  Values values;
  values.insert(101, 1.0);
  values.insert(102, 1.0);
  const auto point = type->point({{1, 101}, {2, 102}}, values);
  const auto next = point.retract(Vector(Vector1(.3)));
  EXPECT_LONGS_EQUAL(1, *calls);
  EXPECT_DOUBLES_EQUAL(1.3, next.values().atDouble(101), 1e-10);
  EXPECT_DOUBLES_EQUAL(1.69, next.values().atDouble(102), 1e-8);

  const Key q = JointAngleKey(0), v = JointVelKey(0), a = JointAccelKey(0);
  Values dynamicsValues;
  dynamicsValues.insert(q, 1.0);
  dynamicsValues.insert(v, 1.0);
  dynamicsValues.insert(a, 1.0);
  NonlinearEqualityConstraints constraints;
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<PriorFactor<double>>(q, 1.0,
                                            noiseModel::Unit::Create(1)));
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(q, v));
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(v, a));
  *selected = KeyVector{q, v, a};
  *calls = 0;
  auto settings = preciseParams();
  settings->useBasisKeys = true;
  EConstraintManifold::Params params;
  params.retractorCreator =
      std::make_shared<DynamicsRetractorCreator>(settings, selector);
  const auto dynamicsType = std::make_shared<EConstraintManifold>(
      constraints, dynamicsValues, params);
  EXPECT_LONGS_EQUAL(1, *calls);
  *selected = KeyVector{999};
  const auto dynamicsPoint = dynamicsType->point(dynamicsValues);
  VectorValues delta = dynamicsValues.zeroVectors();
  for (auto &[key, value] : delta) value(0) = .2;
  const auto restored = dynamicsPoint.retract(delta);
  EXPECT_LONGS_EQUAL(1, *calls);
  EXPECT(restored.values().equals(dynamicsValues, 1e-6));
}

TEST(EqualityPointRetractor, EqualityPoliciesRejectGeneralIETypes) {
  Values values;
  values.insert(1, 1.0);
  values.insert(2, 1.0);
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(1, 2));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      ScalarExpressionInequalityConstraint::GeqZero(Double_(Key(1)), 1.0));
  std::vector<RetractorCreator::shared_ptr> policies{
      std::make_shared<BasisRetractorCreator>(selectX(), preciseParams()),
      std::make_shared<ProjectionRetractorCreator>(preciseParams()),
      std::make_shared<UnconstrainedOptimizationRetractorCreator>(
          preciseParams()),
      std::make_shared<DynamicsRetractorCreator>(preciseParams())};
  for (const auto &policy : policies) {
    IEConstraintManifold::Params params;
    params.retractorCreator = policy;
    auto type = std::make_shared<IEConstraintManifold>(
        equalities, inequalities, values, params);
    CHECK_EXCEPTION(type->point(values), std::invalid_argument);
  }
}

TEST(EqualityPointRetractor, ExplicitOrderingBindsAndConditionsEveryPolicy) {
  for (size_t variant = 0; variant < 3; ++variant) {
    auto settings = preciseParams();
    settings->lmParams.ordering = Ordering{2, 1, 999};
    RetractorCreator::shared_ptr policy;
    if (variant == 0)
      policy = std::make_shared<BasisRetractorCreator>(selectX(), settings);
    else if (variant == 1)
      policy = std::make_shared<ProjectionRetractorCreator>(settings);
    else
      policy =
          std::make_shared<UnconstrainedOptimizationRetractorCreator>(settings);
    const auto type = parabolaType(policy);
    for (Key offset : {100, 300}) {
      Values values;
      values.insert(offset + 1, 1.0);
      values.insert(offset + 2, 1.0);
      const auto point =
          type->point({{1, offset + 1}, {2, offset + 2}}, values);
      const auto next = point.retract(Vector(Vector1(.3)));
      EXPECT(next.evaluateEqualityViolation() < 1e-7);
      EXPECT(point.values().equals(values));
    }

    Values fixed, remaining;
    fixed.insert(1, 1.0);
    remaining.insert(200, 1.0);
    const auto conditioned = type->condition(fixed);
    const auto fixedPoint = conditioned->point({{2, 200}}, remaining);
    VectorValues delta;
    delta.insert(200, Vector1(.4));
    const auto restored = fixedPoint.retract(delta);
    EXPECT_LONGS_EQUAL(0, restored.dim());
    EXPECT_DOUBLES_EQUAL(1., restored.values().atDouble(200), 1e-7);
  }
}

TEST(EqualityPointRetractor, ProjectionUsesPreparedBoundCoordinateKeys) {
  auto settings = preciseParams();
  settings->useBasisKeys = true;
  settings->lmParams.ordering = Ordering{2, 1};
  const auto type =
      parabolaType(std::make_shared<ProjectionRetractorCreator>(settings));
  Values values;
  values.insert(101, 1.0);
  values.insert(102, 1.0);
  const auto point = type->point({{1, 101}, {2, 102}}, values);
  EXPECT(point.eBasis()->basisKeys() ==
         std::optional<KeyVector>(KeyVector{101}));
  const auto next = point.retract(Vector(Vector1(.3)));
  EXPECT(next.evaluateEqualityViolation() < 1e-7);

  auto constraints = std::make_shared<NonlinearEqualityConstraints>();
  constraints->emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(1, 2));
  CHECK_EXCEPTION(ProjectionRetractor(constraints, settings),
                  std::invalid_argument);
  Values prototype;
  prototype.insert(1, 1.0);
  prototype.insert(2, 1.0);
  EConstraintManifold::Params params;
  params.retractorCreator =
      std::make_shared<ProjectionRetractorCreator>(settings);
  const auto orthonormal =
      std::make_shared<EConstraintManifold>(*constraints, prototype, params);
  CHECK_EXCEPTION(orthonormal->point(prototype), std::invalid_argument);

  Values fixed, remaining;
  fixed.insert(1, 1.0);
  remaining.insert(200, 1.0);
  const auto conditioned = type->condition(fixed);
  const auto fixedPoint = conditioned->point({{2, 200}}, remaining);
  EXPECT(fixedPoint.eBasis()->basisKeys() ==
         std::optional<KeyVector>(KeyVector()));
  EXPECT_LONGS_EQUAL(0, fixedPoint.retract(Vector()).dim());
}

TEST(EqualityPointRetractor, DynamicsOrdersOnlyEachBoundInnerGraph) {
  const Key q = JointAngleKey(0), v = JointVelKey(0), a = JointAccelKey(0);
  const Key boundQ = JointAngleKey(0, 10), boundV = JointVelKey(0, 10),
            boundA = JointAccelKey(0, 10);
  Values prototype, values;
  prototype.insert(q, 1.0);
  prototype.insert(v, 1.0);
  prototype.insert(a, 1.0);
  values.insert(boundQ, 1.0);
  values.insert(boundV, 1.0);
  values.insert(boundA, 1.0);
  NonlinearEqualityConstraints constraints;
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<PriorFactor<double>>(q, 1.0,
                                            noiseModel::Unit::Create(1)));
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(q, v));
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(v, a));
  auto settings = preciseParams();
  settings->lmParams.ordering = Ordering{a, v, q};
  EConstraintManifold::Params params;
  params.retractorCreator =
      std::make_shared<DynamicsRetractorCreator>(settings);
  const auto type =
      std::make_shared<EConstraintManifold>(constraints, prototype, params);
  const auto point =
      type->point({{q, boundQ}, {v, boundV}, {a, boundA}}, values);
  VectorValues delta = values.zeroVectors();
  for (auto &[key, value] : delta) value(0) = .2;
  EXPECT(point.retract(delta).values().equals(values, 1e-6));

  Values fixed, remaining;
  fixed.insert(q, 1.0);
  remaining.insert(boundV, 1.0);
  remaining.insert(boundA, 1.0);
  const auto conditioned = type->condition(fixed);
  const auto fixedPoint =
      conditioned->point({{v, boundV}, {a, boundA}}, remaining);
  VectorValues freeDelta = remaining.zeroVectors();
  for (auto &[key, value] : freeDelta) value(0) = .2;
  EXPECT(fixedPoint.retract(freeDelta).values().equals(remaining, 1e-6));
}

TEST(EqualityPointRetractor, ApproximateBudgetIsPreserved) {
  auto settings = preciseParams();
  settings->lmParams.setMaxIterations(0);
  const auto type = parabolaType(
      std::make_shared<UnconstrainedOptimizationRetractorCreator>(settings));
  Values values;
  values.insert(1, 1.0);
  values.insert(2, 1.0);
  const auto point = type->point(values);
  const auto next = point.retract(Vector(Vector1(.5)));
  EXPECT_DOUBLES_EQUAL(1.5, next.values().atDouble(1), 1e-10);
  EXPECT_DOUBLES_EQUAL(2.0, next.values().atDouble(2), 1e-10);
  EXPECT_DOUBLES_EQUAL(.25, next.evaluateEqualityViolation(), 1e-10);
  EXPECT(point.values().equals(values));
}

TEST(EqualityPointRetractor, SharedTypeAndConcurrentFixedFactorWorkspaces) {
  const auto type = parabolaType(
      std::make_shared<BasisRetractorCreator>(selectX(), preciseParams()));
  std::vector<ECMPoint> points;
  for (size_t i = 0; i < 3; ++i) {
    Values values;
    const Key x = 100 + 2 * i, y = x + 1;
    values.insert(x, 1.0);
    values.insert(y, 1.0);
    points.push_back(type->point({{1, x}, {2, y}}, values));
  }
  // Workspace creation shares immutable graph factors rather than cloning them.
  const size_t clonesBefore = Parabola::clones.load();
  const auto workspace1 = points.front().retractor();
  const auto workspace2 = points.front().retractor();
  EXPECT(workspace1.get() != workspace2.get());
  EXPECT_LONGS_EQUAL(clonesBefore, Parabola::clones.load());

  std::vector<std::future<ECMPoint>> results;
  for (size_t i = 0; i < 12; ++i) {
    const auto point = points[i % points.size()];
    results.push_back(std::async(std::launch::async, [point, i]() {
      return point.retract(Vector(Vector1(.01 * (i + 1))));
    }));
  }
  for (size_t i = 0; i < results.size(); ++i) {
    const auto result = results[i].get();
    const double x = 1 + .01 * (i + 1);
    const Key xKey = 100 + 2 * (i % points.size());
    EXPECT_DOUBLES_EQUAL(x, result.values().atDouble(xKey), 1e-10);
    EXPECT_DOUBLES_EQUAL(x * x, result.values().atDouble(xKey + 1), 1e-8);
    EXPECT(result.manifold().get() == type.get());
    EXPECT_DOUBLES_EQUAL(1, points[i % points.size()].values().atDouble(xKey),
                         1e-10);
  }
}

TEST(EqualityPointRetractor, ConditioningRemovesCapturedBasisSlots) {
  const auto type = parabolaType(
      std::make_shared<BasisRetractorCreator>(selectX(), preciseParams()));
  Values fixed;
  fixed.insert(1, 1.0);
  const auto conditioned = type->condition(fixed);
  Values remaining;
  remaining.insert(200, 1.0);
  const auto point = conditioned->point({{2, 200}}, remaining);
  EXPECT_LONGS_EQUAL(0, point.dim());
  VectorValues delta;
  delta.insert(200, Vector1(.5));
  const auto next = point.retract(delta);
  EXPECT_DOUBLES_EQUAL(1, next.values().atDouble(200), 1e-7);
  EXPECT(next.manifold().get() == conditioned.get());
  EXPECT(point.values().equals(remaining));
}

TEST(EqualityPointRetractor, DynamicsWorkspacesKeepTheirFixedFactorsTogether) {
  const Key q = JointAngleKey(0), v = JointVelKey(0), a = JointAccelKey(0);
  Values values;
  values.insert(q, 1.0);
  values.insert(v, 1.0);
  values.insert(a, 1.0);
  NonlinearEqualityConstraints constraints;
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<PriorFactor<double>>(q, 1.0,
                                            noiseModel::Unit::Create(1)));
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(q, v));
  constraints.emplace_shared<ZeroCostConstraint>(
      std::make_shared<Parabola>(v, a));
  EConstraintManifold::Params params;
  params.retractorCreator =
      std::make_shared<DynamicsRetractorCreator>(preciseParams());
  auto type =
      std::make_shared<EConstraintManifold>(constraints, values, params);
  const auto point = type->point(values);
  std::vector<std::future<IECMPoint>> results;
  for (size_t i = 0; i < 8; ++i) {
    results.push_back(std::async(std::launch::async, [point, i]() {
      auto workspace = point.retractor();
      VectorValues delta = point.values().zeroVectors();
      for (auto &[key, vector] : delta) vector(0) = .1 * (i + 1);
      return workspace->retract(&point, delta);
    }));
  }
  for (auto &future : results) {
    const auto result = future.get();
    EXPECT(result.evaluateEqualityViolation() < 1e-6);
    EXPECT(result.values().equals(values, 1e-6));
  }
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
