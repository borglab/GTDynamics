/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved. See LICENSE.
 * -------------------------------------------------------------------------- */
/**
 * @file testTypedManifoldHierarchy.cpp
 * @brief Runtime contracts, concrete GTSAM values, and compiled equality
 * charts.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmopt/ECMPoint.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/internal/CompiledIEConstraintManifold.h>
#include <gtdynamics/cmopt/internal/CompiledEConstraintManifold.h>
#include <gtdynamics/cmopt/LMManifoldOptimizer.h>
#include <gtdynamics/cmopt/NonlinearManifoldOptimizer.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/slam/PriorFactor.h>

#include <atomic>
#include <type_traits>

using namespace gtsam;
using namespace gtdynamics;

static_assert(std::is_base_of_v<ConstraintManifold, IEConstraintManifold>);
static_assert(std::is_base_of_v<IEConstraintManifold, EConstraintManifold>);
static_assert(std::is_base_of_v<CMPoint, IECMPoint>);
static_assert(std::is_base_of_v<IECMPoint, ECMPoint>);
static_assert(
    std::is_same_v<decltype(std::declval<IECMPoint>().retract(Vector())),
                   IECMPoint>);
static_assert(std::is_same_v<
              decltype(std::declval<ECMPoint>().retract(Vector())), ECMPoint>);

static_assert(!std::is_copy_assignable_v<IEConstraintManifold>);
static_assert(!std::is_move_assignable_v<IEConstraintManifold>);
static_assert(!std::is_copy_assignable_v<EConstraintManifold>);

namespace {
Values lineValues(Key x, Key y, double coordinate) {
  Values values;
  values.insert(x, Vector(Vector1(coordinate)));
  values.insert(y, Vector(Vector1(2 * coordinate)));
  return values;
}

NonlinearEqualityConstraints lineEqualities() {
  NonlinearEqualityConstraints constraints;
  constraints.push_back(
      LinearConstraint::Equal(JacobianFactor(0, Matrix1::Constant(-2), 1,
                                             Matrix1::Identity(),
                                             Vector1::Zero()))
          .createEqualityFactor());
  return constraints;
}

EConstraintManifold::Params affineSettings() {
  EConstraintManifold::Params params;
  params.affineEqualities = true;
  params.equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector&) { return KeyVector{0}; });
  return params;
}

// This Recovery deliberately has no inequality or boundary interface.
struct LineRecovery {
  double fromAmbient(const Values& values) const {
    return values.at<Vector>(0)(0);
  }
  Values ambient(double coordinate) const {
    return lineValues(0, 1, coordinate);
  }
  std::map<Key, Matrix> jacobians(double) const {
    return {{0, Matrix1::Identity()}, {1, Matrix1::Constant(2)}};
  }
};

// The IE adapter adds a boundary contract to the same ambient embedding.
struct BoundedLineRecovery : LineRecovery {
  std::vector<gtdynamics::internal::NativeBoundary> boundaries(double x) const {
    return {{Vector1(-x), Vector1::Ones(), {{0, Matrix1::Constant(-1)}}}};
  }
  double boundary(double value, const IndexSet& active) const {
    for (size_t index : active)
      if (index != 0)
        throw std::invalid_argument("Invalid native line boundary");
    return active.empty() ? value : 0.;
  }
};

// Cloning a constraint retains the counter while conditioning owns its
// geometry.
class CountedOffset final : public NonlinearEqualityConstraint {
  std::shared_ptr<std::atomic<size_t>> constructions_;

 public:
  explicit CountedOffset(std::shared_ptr<std::atomic<size_t>> constructions)
      : NonlinearEqualityConstraint(noiseModel::Constrained::All(1),
                                    KeyVector{0, 1, 2}),
        constructions_(std::move(constructions)) {}

  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    if (H) {
      ++*constructions_;
      (*H)[0] = Matrix1::Constant(-2);
      (*H)[1] = Matrix1::Identity();
      (*H)[2] = Matrix1::Constant(-1);
    }
    return Vector1(values.at<Vector>(1)(0) - 2 * values.at<Vector>(0)(0) -
                   values.at<Vector>(2)(0));
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<CountedOffset>(*this);
  }
};
}  // namespace

TEST(TypedManifoldHierarchy, ConcreteValuesAndWidening) {
  const auto type = std::make_shared<EConstraintManifold>(
      lineEqualities(), lineValues(0, 1, 0), affineSettings());
  const auto first = type->point({{0, 10}, {1, 11}}, lineValues(10, 11, 1));
  const auto second = type->point({{0, 20}, {1, 21}}, lineValues(20, 21, 3));
  EXPECT(first.manifold() == second.manifold());
  EXPECT(first.activeIndices().empty());
  EXPECT_LONGS_EQUAL(0, first.manifold()->inequalityCount(first));
  CHECK_EXCEPTION(first.localCoordinates(second), std::invalid_argument);

  // A widened value keeps the exact same runtime type and ambient state.
  const ECMPoints equalities{{100, first}, {101, second}};
  const auto widened = widenPoints(equalities);
  EXPECT(widened.at(100).manifold().get() == type.get());
  EXPECT(assert_equal(first.values(), widened.at(100).values()));
  const auto recovered = ECMPoint::fromIEPoint(widened.at(100));
  EXPECT(recovered.equals(first));

  Values stored;
  stored.insert(100, first);
  stored.insert(101, widened.at(101));
  EXPECT(dynamic_cast<const GenericValue<ECMPoint>*>(&stored.at(100)));
  EXPECT(dynamic_cast<const GenericValue<IECMPoint>*>(&stored.at(101)));
  CHECK_EXCEPTION(stored.at<IECMPoint>(100), ValuesIncorrectType);
  CHECK_EXCEPTION(stored.at<ECMPoint>(101), ValuesIncorrectType);
  VectorValues increments;
  increments.insert(100, Vector1(.25));
  increments.insert(101, Vector1(-.5));
  const auto moved = stored.retract(increments);
  EXPECT(dynamic_cast<const GenericValue<ECMPoint>*>(&moved.at(100)));
  EXPECT(dynamic_cast<const GenericValue<IECMPoint>*>(&moved.at(101)));
  EXPECT(assert_equal(first.values(), stored.at<ECMPoint>(100).values()));
}

TEST(TypedManifoldHierarchy, EqualityAssignmentThroughBaseIsChecked) {
  const auto equalityType = std::make_shared<EConstraintManifold>(
      lineEqualities(), lineValues(0, 1, 0), affineSettings());
  const auto generalType = std::make_shared<IEConstraintManifold>(
      lineEqualities(), NonlinearInequalityConstraints{}, lineValues(0, 1, 0),
      affineSettings());
  auto equality = equalityType->point(lineValues(0, 1, 1));
  const auto before = equality;
  auto general = generalType->point(lineValues(0, 1, 2));
  IECMPoint& base = equality;
  CHECK_EXCEPTION(base = general, std::invalid_argument);
  EXPECT(equality.equals(before));
  CHECK_EXCEPTION(base = std::move(general), std::invalid_argument);
  EXPECT(equality.equals(before));

  // Widened equality values are valid sources; general IE values remain
  // general.
  const auto next = equalityType->point(lineValues(0, 1, 3));
  const IECMPoint widened = next;
  base = widened;
  EXPECT(equality.equals(next));
  IECMPoint ordinary = next;
  ordinary = general;
  EXPECT(ordinary.manifold().get() == generalType.get());
  ECMPoint empty;
  equality = empty;
  EXPECT(!equality.manifold());
  equality = before;
  EXPECT(equality.equals(before));
}

TEST(TypedManifoldHierarchy, ExplicitEqualityView) {
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(
      LinearConstraint::GreaterEqual(
          JacobianFactor(0, Matrix1::Identity(), Vector1::Zero()))
          .createInequalityFactor());
  const auto source = std::make_shared<IEConstraintManifold>(
      lineEqualities(), inequalities, lineValues(0, 1, 0), affineSettings());
  const auto boundary = source->point(lineValues(0, 1, 0));
  CHECK_EXCEPTION(ECMPoint::fromIEPoint(boundary), std::invalid_argument);
  const auto equality = boundary.equalityView();
  EXPECT(equality.manifold().get() != source.get());
  EXPECT(equality.activeIndices().empty());
  EXPECT_LONGS_EQUAL(0, equality.manifold()->inequalityCount(equality));
  EXPECT_LONGS_EQUAL(1, source->inequalityCount(boundary));
  const auto outside = equality.retract(Vector(Vector1(-1)));
  EXPECT(outside.values().at<Vector>(0)(0) < -.9);
  EXPECT_DOUBLES_EQUAL(0., outside.evaluateEqualityViolation(), 1e-8);
}

TEST(TypedManifoldHierarchy, CompiledInequalityViewRequiresExplicitChart) {
  const auto native =
      std::make_shared<gtdynamics::internal::CompiledIEConstraintManifold<
          double, BoundedLineRecovery>>(0., BoundedLineRecovery());
  const auto point = native->point(lineValues(0, 1, 1));
  EXPECT_LONGS_EQUAL(1, native->inequalityCount(point));
  CHECK_EXCEPTION(point.equalityView(), std::invalid_argument);
}

TEST(TypedManifoldHierarchy, ConditionedAffineInitializationIsIndependent) {
  auto constructions = std::make_shared<std::atomic<size_t>>(0);
  NonlinearEqualityConstraints constraints;
  constraints.emplace_shared<CountedOffset>(constructions);
  auto prototype = lineValues(0, 1, 0);
  prototype.insert(2, Vector(Vector1(0)));
  EConstraintManifold::Params params;
  params.affineEqualities = true;
  const auto family =
      std::make_shared<EConstraintManifold>(constraints, prototype, params);
  Values fixedFirst, fixedSecond;
  fixedFirst.insert(2, Vector(Vector1(1)));
  fixedSecond.insert(2, Vector(Vector1(3)));
  const auto first = family->condition(fixedFirst);
  const auto second = family->condition(fixedSecond);
  EXPECT(dynamic_cast<const EConstraintManifold*>(first.get()));
  EXPECT_LONGS_EQUAL(0, constructions->load());
  auto values = lineValues(10, 11, 0);
  values.update(11, Vector(Vector1(1)));
  const auto a = first->point({{0, 10}, {1, 11}}, values);
  first->point({{0, 10}, {1, 11}}, values);
  EXPECT_LONGS_EQUAL(1, constructions->load());
  values.update(11, Vector(Vector1(3)));
  const auto b = second->point({{0, 10}, {1, 11}}, values);
  EXPECT_LONGS_EQUAL(2, constructions->load());
  EXPECT_DOUBLES_EQUAL(0., a.evaluateEqualityViolation(), 1e-10);
  EXPECT_DOUBLES_EQUAL(0., b.evaluateEqualityViolation(), 1e-10);
  CHECK_EXCEPTION(a.localCoordinates(b), std::invalid_argument);
  CHECK_EXCEPTION(first->point(a.bindings(), a.values(), IndexSet{0}),
                  std::invalid_argument);
}

TEST(TypedManifoldHierarchy, CompiledEqualityChartUsesEveryOptimizer) {
  const auto native = std::make_shared<
      gtdynamics::internal::CompiledEConstraintManifold<double, LineRecovery>>(
      0., LineRecovery());
  const auto runtime = std::make_shared<EConstraintManifold>(
      lineEqualities(), lineValues(0, 1, 0), affineSettings());
  const auto initial = lineValues(10, 11, .5);
  NonlinearFactorGraph costs;
  costs.emplace_shared<PriorFactor<Vector>>(11, Vector(Vector1(4)),
                                            noiseModel::Unit::Create(1));
  ManifoldOptimizerParameters settings;
  const std::vector<NonlinearManifoldOptimizer::NonlinearOptimizerParamsVariant>
      solvers{LevenbergMarquardtParams(), GaussNewtonParams(), DoglegParams()};
  for (const EConstraintManifold::shared_ptr& type :
       {EConstraintManifold::shared_ptr(native),
        EConstraintManifold::shared_ptr(runtime)}) {
    const auto point = type->point({{0, 10}, {1, 11}}, initial);
    const ECMPoints points{{100, point}};
    EXPECT_DOUBLES_EQUAL(2., point.eBasis()->recoverJacobian(11)(0, 0), 1e-10);
    const auto check = [&](const Values& result) {
      EXPECT_DOUBLES_EQUAL(2., result.at<Vector>(10)(0), 1e-5);
      EXPECT_DOUBLES_EQUAL(4., result.at<Vector>(11)(0), 1e-5);
    };
    for (const auto& solver : solvers) {
      NonlinearManifoldOptimizer optimizer(settings, solver);
      check(optimizer.optimize(costs, points));
    }
    LMManifoldOptimizer custom(settings);
    check(custom.optimize(costs, points));
    for (auto solver :
         {IELMParams::LinearSolver::Gaussian, IELMParams::LinearSolver::QP}) {
      IELMParams params;
      params.linearSolver = solver;
      check(IELMOptimizer(params).optimize(costs, points, {}));
    }
    EXPECT(assert_equal(initial, point.values()));
  }
  // The compiled path supplies geometry directly without constraint factors.
  const auto compiledPoint = native->point({{0, 10}, {1, 11}}, initial);
  EXPECT(compiledPoint.eConstraints()->empty());
  EXPECT(compiledPoint.iConstraints()->empty());
  const IECMPoint widened = compiledPoint;
  EXPECT(widened.equalityView().manifold().get() == native.get());
  EXPECT(widened.manifold()->equalityView().get() == native.get());
}

TEST(TypedManifoldHierarchy, RedundantFixedEqualityConditioning) {
  NonlinearEqualityConstraints constraints;
  constraints.push_back(
      LinearConstraint::Equal(
          JacobianFactor(0, Matrix1::Identity(), Vector1::Zero()))
          .createEqualityFactor());
  constraints.push_back(
      LinearConstraint::Equal(
          JacobianFactor(1, Matrix1::Identity(), Vector1::Zero()))
          .createEqualityFactor());
  constraints.push_back(
      LinearConstraint::Equal(JacobianFactor(0, Matrix1::Identity(), 1,
                                             Matrix1::Identity(),
                                             Vector1::Zero()))
          .createEqualityFactor());
  const auto state = lineValues(0, 1, 0);
  auto params = affineSettings();
  params.equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector& keys) { return keys; });
  CHECK_EXCEPTION(
      std::make_shared<IEConstraintManifold>(
          constraints, NonlinearInequalityConstraints{}, state, params),
      std::invalid_argument);
  const auto type =
      std::make_shared<EConstraintManifold>(constraints, state, params);
  const auto point = type->point(state);
  EXPECT_LONGS_EQUAL(0, point.dim());
  EXPECT_LONGS_EQUAL(3, point.eConstraints()->dim());
  EXPECT_LONGS_EQUAL(0, point.eBasis()->recoverJacobian(0).cols());
  Values fixed, remaining;
  fixed.insert(0, Vector(Vector1::Zero()));
  remaining.insert(1, Vector(Vector1::Zero()));
  const auto conditioned = type->condition(fixed);
  const auto conditionedPoint = conditioned->point(remaining);
  EXPECT_LONGS_EQUAL(0, conditionedPoint.dim());
  EXPECT_LONGS_EQUAL(2, conditionedPoint.eConstraints()->dim());
  EXPECT_LONGS_EQUAL(0, conditionedPoint.eBasis()->recoverJacobian(1).cols());
  EXPECT(assert_equal(remaining, conditionedPoint.retract(Vector(0)).values()));
}

TEST(TypedManifoldHierarchy, DirectKernelsRejectOtherRuntimeTypes) {
  const auto first = std::make_shared<EConstraintManifold>(
      lineEqualities(), lineValues(0, 1, 0), affineSettings());
  const auto second = std::make_shared<EConstraintManifold>(
      lineEqualities(), lineValues(0, 1, 0), affineSettings());
  const auto a = first->point(lineValues(0, 1, 1));
  const auto b = second->point(lineValues(0, 1, 1));
  const Vector zero = Vector1::Zero();
  const auto ambientZero = b.values().zeroVectors();
  CHECK_EXCEPTION(first->retract(b, zero), std::invalid_argument);
  CHECK_EXCEPTION(first->retractAmbient(b, ambientZero), std::invalid_argument);
  CHECK_EXCEPTION(first->moveToBoundary(b, {}), std::invalid_argument);
  CHECK_EXCEPTION(first->localCoordinates(a, b), std::invalid_argument);
  CHECK_EXCEPTION(first->localCoordinates(b, a), std::invalid_argument);
  const ConstraintManifold& erased = *first;
  CHECK_EXCEPTION(erased.retractValues(b, zero), std::invalid_argument);

  const auto native = std::make_shared<
      gtdynamics::internal::CompiledEConstraintManifold<double, LineRecovery>>(
      0., LineRecovery());
  const auto compiled = native->point(lineValues(0, 1, 1));
  CHECK_EXCEPTION(native->retract(a, zero), std::invalid_argument);
  CHECK_EXCEPTION(native->retractAmbient(a, ambientZero),
                  std::invalid_argument);
  CHECK_EXCEPTION(native->moveToBoundary(a, {}), std::invalid_argument);
  CHECK_EXCEPTION(native->localCoordinates(compiled, a), std::invalid_argument);
  CHECK_EXCEPTION(native->localCoordinates(a, compiled), std::invalid_argument);
}

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
