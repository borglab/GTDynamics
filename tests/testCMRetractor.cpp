/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file testCMRetractor.cpp
 * @brief Common policies, bindings, metrics, and typed IE interoperability.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmopt/ECMPoint.h>
#include <gtdynamics/constrained/CMRetractor.h>
#include <gtsam/base/TestableAssertions.h>

#include <future>

#include "examples/example_constraint_manifold/ManualIEConstraintManifold.h"

using namespace gtsam;
using namespace gtdynamics;
using namespace manual_ieconstraint_manifold;

namespace {
// A straight affine chart needs no inequality or factor-specific interface.
class AmbientRetractor : public CMRetractor {
  CMRetractorParams::shared_ptr params_;

 public:
  explicit AmbientRetractor(const CMRetractorParams::shared_ptr& params)
      : params_(CMRetractorParams::snapshot(params)) {}

  shared_ptr cloneWorkspace() const override {
    return std::make_shared<AmbientRetractor>(params_);
  }

  Values retractValues(const CMPoint& point, const VectorValues& delta,
                       CMRetractionInfo* = nullptr) const override {
    return point.values().retract(delta);
  }

  const CMRetractorParams& commonParams() const override { return *params_; }

 protected:
  CMRetractorParams& mutableCommonParams() override { return *params_; }
};

// Runtime types snapshot settings before preparing any bound point.
class AmbientPolicy : public CMRetractorCreator {
  CMRetractorParams::shared_ptr params_;

 public:
  explicit AmbientPolicy(const CMRetractorParams::shared_ptr& params)
      : params_(CMRetractorParams::snapshot(params)) {}

  shared_ptr clonePolicy() const override {
    return std::make_shared<AmbientPolicy>(params_);
  }

  shared_ptr cloneForKeys(const KeySet& keys) const override {
    return std::make_shared<AmbientPolicy>(
        CMRetractorParams::snapshot(params_, &keys));
  }

  CMRetractor::shared_ptr prepare(const CMPoint& point) const override {
    return AmbientRetractor(params_).bindPoint(point);
  }

  const CMRetractorParams& commonParams() const override { return *params_; }
};

EConstraintManifold::Params settings(
    const CMRetractorCreator::shared_ptr& policy) {
  EConstraintManifold::Params params;
  params.affineEqualities = true;
  params.equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector&) { return KeyVector{0}; });
  params.retractorCreator = policy;
  return params;
}
}  // namespace

TEST(CMRetractor, EqualityTypeAcceptsACommonPolicy) {
  const auto [equalities, inequalities] = lineConstraints(0, 1);
  const auto policy =
      std::make_shared<AmbientPolicy>(std::make_shared<CMRetractorParams>());
  const auto type = std::make_shared<EConstraintManifold>(
      equalities, lineValues(0, 1, 1), settings(policy));
  const auto point = type->point({{0, 10}, {1, 11}}, lineValues(10, 11, 1));
  const CMPoint& commonPoint = point;
  const auto workspace = commonPoint.retractionWorkspace();
  EXPECT(!std::dynamic_pointer_cast<IERetractor>(workspace));

  const Vector step = Vector1(.25);
  const Values values = workspace->retractValues(point, point.toAmbient(step));
  const auto next = point.retract(step);
  EXPECT(assert_equal(values, next.values()));
  EXPECT(assert_equal(lineValues(10, 11, 1.25), next.values()));
  EXPECT_DOUBLES_EQUAL(0, next.evaluateEqualityViolation(), 1e-12);
  EXPECT(point.values().equals(lineValues(10, 11, 1)));
  EXPECT(next.manifold() == type);
  EXPECT(next.bindings() == point.bindings());
  EXPECT(assert_equal(point.values(), point.moveToBoundary({}).values()));
  CHECK_EXCEPTION(point.moveToBoundary({0}), std::invalid_argument);
}

TEST(CMRetractor, SharedPolicyBindsAndIsolatesTrialMetrics) {
  auto metric = std::make_shared<CMRetractorParams>();
  metric->useVaryingSigma = true;
  metric->metricSigmas = std::make_shared<VectorValues>();
  metric->metricSigmas->insert(0, Vector1(1));
  metric->metricSigmas->insert(1, Vector1(2));
  metric->metricSigmas->insert(99, Vector1(99));
  const auto policy = std::make_shared<AmbientPolicy>(metric);
  const auto [equalities, inequalities] = lineConstraints(0, 1);
  const auto type = std::make_shared<EConstraintManifold>(
      equalities, lineValues(0, 1, 1), settings(policy));
  const auto first = type->point({{0, 10}, {1, 11}}, lineValues(10, 11, 1));
  const auto second = type->point({{0, 20}, {1, 21}}, lineValues(20, 21, 2));

  // Caller changes and one trial's metric cannot change another workspace.
  metric->metricSigmas->at(0)(0) = 100;
  VectorValues trialMetric;
  trialMetric.insert(10, Vector1(.25));
  trialMetric.insert(20, Vector1(7));
  auto trial = first.retractionWorkspace(&trialMetric);
  auto independent = first.retractionWorkspace();
  auto otherPoint = second.retractionWorkspace();
  EXPECT_LONGS_EQUAL(1, trial->commonParams().metricSigmas->size());
  EXPECT_DOUBLES_EQUAL(.25, trial->commonParams().metricSigmas->at(10)(0),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(1, independent->commonParams().metricSigmas->at(10)(0),
                       1e-12);
  EXPECT_DOUBLES_EQUAL(2, otherPoint->commonParams().metricSigmas->at(21)(0),
                       1e-12);

  independent->scopeMetric(KeySet{11});
  EXPECT_LONGS_EQUAL(1, independent->commonParams().metricSigmas->size());
  EXPECT_LONGS_EQUAL(
      2, first.retractionWorkspace()->commonParams().metricSigmas->size());
  EXPECT(type->usesVaryingMetric());

  auto work = [&first] {
    auto local = first.retractionWorkspace();
    VectorValues sigmas;
    sigmas.insert(10, Vector1(4));
    local->setMetricSigmas(sigmas);
    return local->retractValues(first, first.toAmbient(Vector1(.5)));
  };
  auto parallel = std::async(std::launch::async, work);
  EXPECT(assert_equal(work(), parallel.get()));
  EXPECT_DOUBLES_EQUAL(
      1, first.retractionWorkspace()->commonParams().metricSigmas->at(10)(0),
      1e-12);
}

TEST(CMRetractor, GeneralIETypeRequiresBoundaryOperations) {
  const auto [equalities, inequalities] = lineConstraints(0, 1);
  const auto policy =
      std::make_shared<AmbientPolicy>(std::make_shared<CMRetractorParams>());
  const auto type = std::make_shared<IEConstraintManifold>(
      equalities, inequalities, lineValues(0, 1, 1), settings(policy));
  CHECK_EXCEPTION(type->point(lineValues(0, 1, 1)), std::invalid_argument);
}

TEST(CMRetractor, IEPoliciesRetainTheirBehaviorThroughTheCommonInterface) {
  const auto type = lineType();
  const auto point = type->point({{0, 10}, {1, 11}}, lineValues(10, 11, .5));
  const CMPoint& commonPoint = point;
  const auto workspace = commonPoint.retractionWorkspace();
  EXPECT(std::dynamic_pointer_cast<IERetractor>(workspace));

  // A crossing step still invokes the original corner-aware retraction.
  const Vector step = Vector1(-1);
  CMRetractionInfo commonInfo;
  const auto values =
      workspace->retractValues(commonPoint, point.toAmbient(step), &commonInfo);
  IERetractionInfo typedInfo;
  const auto next = point.retract(step, {}, &typedInfo);
  EXPECT(assert_equal(values, next.values(), 1e-8));
  EXPECT_DOUBLES_EQUAL(0, next.values().at<Vector>(10)(0), 1e-7);
  EXPECT_LONGS_EQUAL(typedInfo.numALSolveCalls, commonInfo.numALSolveCalls);
  EXPECT_LONGS_EQUAL(typedInfo.numLMIterations, commonInfo.numLMIterations);
  EXPECT(point.values().equals(lineValues(10, 11, .5)));

  // Generic binding must retain the IE policy's local prior-key conversion.
  const CMRetractor::shared_ptr localPolicy =
      std::make_shared<ReducedALRetractor>(
          std::make_shared<IERetractorParams>(), ReducedALRetractor::Settings(),
          KeyVector{0});
  const auto bound = localPolicy->bindPoint(commonPoint);
  EXPECT(assert_equal(next.values(),
                      bound->retractValues(commonPoint, point.toAmbient(step)),
                      1e-8));
}

TEST(CMRetractor, CommonMetricPreservesPriorWeightsAndSnapshots) {
  auto params = std::make_shared<CMRetractorParams>();
  params->useVaryingSigma = true;
  params->metricSigmas = std::make_shared<VectorValues>();
  params->metricSigmas->insert(0, Vector1(2));
  params->metricSigmas->insert(1, Vector1(4));
  const auto snapshot = CMRetractorParams::snapshot(params);
  params->metricSigmas->at(0)(0) = 20;
  const auto weights =
      snapshot->priorPrecisions(lineValues(0, 1, 0), KeyVector{0, 0, 1});
  EXPECT_DOUBLES_EQUAL(.5, weights.at(0)(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(.0625, weights.at(1)(0), 1e-12);
  snapshot->metricSigmas->at(1)(0) = 0;
  CHECK_EXCEPTION(snapshot->priorPrecisions(lineValues(0, 1, 0), {1}),
                  std::invalid_argument);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
