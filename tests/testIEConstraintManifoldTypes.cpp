/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testIEConstraintManifoldTypes.cpp
 * @brief Type/value ownership, binding, conditioning, and compiled CMC charts.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmcopt/IEGDOptimizer.h>
#include <gtdynamics/cmcopt/IELMOptimizer.h>
#include <gtdynamics/cmcopt/internal/CompiledIEConstraintManifold.h>
#include <gtdynamics/cmopt/ECMPoint.h>
#include <gtdynamics/constrained/internal/ReadOnlyBasis.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <atomic>
#include <future>
#include <limits>

#include "examples/example_constraint_manifold/ManualIEConstraintManifold.h"

namespace native_line {
using namespace gtsam;
/** An ordinary compiled GTSAM value; its C++ class supplies the manifold type.
 */
struct Point {
  enum { dimension = 1 };
  double x = 0;
  Point() = default;
  explicit Point(double value) : x(value) {}
  Point retract(const Vector1& v) const {
    return Point(std::max(0., x + v(0)));
  }

  Vector1 localCoordinates(const Point& other) const {
    return Vector1(other.x - x);
  }

  bool equals(const Point& other, double tolerance = 1e-8) const {
    return std::abs(x - other.x) <= tolerance;
  }

  void print(const std::string& prefix = "") const {
    std::cout << prefix << x << '\n';
  }
};

}  // namespace native_line
namespace gtsam {
template <>
struct traits<native_line::Point> : internal::Manifold<native_line::Point> {};
}  // namespace gtsam
GTSAM_CONCEPT_MANIFOLD_INST(native_line::Point)

namespace runtime_types {
using namespace manual_ieconstraint_manifold;
using namespace gtdynamics::internal;
/** Native ambient embedding and boundary operations; no constraint factors. */
struct Recovery {
  native_line::Point fromAmbient(const Values& values) const {
    return native_line::Point(values.at<Vector>(0)(0));
  }

  Values ambient(const native_line::Point& value) const {
    return lineValues(0, 1, value.x);
  }

  std::map<Key, Matrix> jacobians(const native_line::Point&) const {
    return {{0, Matrix1::Identity()}, {1, Matrix1::Constant(2)}};
  }

  std::vector<NativeBoundary> boundaries(
      const native_line::Point& value) const {
    return {{Vector1(-value.x), Vector1(2), {{0, Matrix1::Constant(-1)}}}};
  }

  native_line::Point boundary(const native_line::Point& value,
                              const IndexSet& active) const {
    for (size_t index : active)
      if (index != 0) throw std::invalid_argument("Unknown line boundary");
    return active.empty() ? value : native_line::Point(0);
  }
};

IEConstraintManifold::shared_ptr compiledType() {
  return std::make_shared<
      CompiledIEConstraintManifold<native_line::Point, Recovery>>(
      native_line::Point(), Recovery());
}

bool invalid(const std::function<void()>& call) {
  try {
    call();
  } catch (const std::invalid_argument&) {
    return true;
  }

  return false;
}

NonlinearFactorGraph coupledCost() {
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(10, Vector(Vector1(3)),
                                            noiseModel::Unit::Create(1));
  graph.emplace_shared<PriorFactor<Vector>>(20, Vector(Vector1(-3)),
                                            noiseModel::Unit::Create(1));
  graph.emplace_shared<BetweenFactor<Vector>>(10, 20, Vector(Vector1(0)),
                                              noiseModel::Unit::Create(1));
  return graph;
}

IECMPoints twoPoints(const IEConstraintManifold::shared_ptr& type) {
  return {
      {Symbol('c', 0), type->point({{0, 10}, {1, 11}}, lineValues(10, 11, 0))},
      {Symbol('c', 1), type->point({{0, 20}, {1, 21}}, lineValues(20, 21, 0))}};
}

// Discovery exposes definitions separately; points retain their owning types.
TEST(IEConstraintManifold, DiscoveredTypesAndPoints) {
  const auto initial = ambientValues(twoPoints(lineType()));
  auto [equalities, inequalities] = lineConstraints(10, 11);
  const auto second = lineConstraints(20, 21);
  equalities.add(second.first);
  inequalities.add(second.second);
  auto params = std::make_shared<IEConstraintManifold::Params>();
  auto components = IEOptimizer::identifyConnectedComponents(
      equalities, inequalities, initial, params);
  EXPECT_LONGS_EQUAL(2, components.points.size());
  EXPECT(!components.types.empty());
  EXPECT(assert_equal(initial, ambientValues(components.points)));
  for (const auto& [key, point] : components.points) {
    bool found = false;
    for (const auto& type : components.types) found |= type == point.manifold();
    EXPECT(found);
  }

  // A discovered definition can construct additional, differently bound points.
  const auto original = components.points.begin()->second;
  const auto& type = original.manifold();
  const auto slots = original.values().keys();
  const auto extra =
      type->point({{slots[0], 30}, {slots[1], 31}}, lineValues(30, 31, 2));
  EXPECT(extra.manifold() == original.manifold());
  EXPECT(original.activeIndices().exists(0));
  EXPECT(extra.activeIndices().empty());
  components.types.clear();
  EXPECT_DOUBLES_EQUAL(2, extra.values().at<Vector>(30)(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(0, original.values().at<Vector>(slots[0])(0), 1e-12);

  // An entirely unconstrained problem creates no runtime definitions or points.
  const auto empty =
      IEOptimizer::identifyConnectedComponents({}, {}, initial, params);
  EXPECT(empty.types.empty());
  EXPECT(empty.points.empty());
}

// One type supports distinct identities, values, boundaries, and numeric
// geometry.
TEST(IEConstraintManifold, SharedDefinitionAndIndependentPoints) {
  const auto type = lineType();
  const auto first = type->point({{0, 10}, {1, 11}}, lineValues(10, 11, 0));
  const auto second = type->point({{0, 20}, {1, 21}}, lineValues(20, 21, 2));
  EXPECT(first.manifold() == second.manifold());
  EXPECT(first.activeIndices().exists(0));
  EXPECT(second.activeIndices().empty());
  EXPECT(first.tangentSpace() != second.tangentSpace());
  const auto geometry = type->linearize(first);
  EXPECT(assert_equal(first.tangentSpace()->rows(0), geometry->rows(0)));
  EXPECT(
      assert_equal(Matrix1::Constant(2), second.eBasis()->recoverJacobian(21)));
  const auto next = first.retract(Vector(Vector1(1)));
  EXPECT_DOUBLES_EQUAL(0, first.values().at<Vector>(10)(0), 1e-10);
  EXPECT_DOUBLES_EQUAL(1, next.values().at<Vector>(10)(0), 1e-8);
  EXPECT_DOUBLES_EQUAL(2, next.values().at<Vector>(11)(0), 1e-8);
  EXPECT(assert_equal(first.eBasis()->recoverJacobian(11),
                      next.eBasis()->recoverJacobian(11)));
  EXPECT(next.tangentSpace() != first.tangentSpace());
  EXPECT(invalid([&] { first.localCoordinates(second); }));
}

// GTSAM Values and traits consume ordinary vectors and preserve type identity.
TEST(IECMPoint, GtsamValueAndChartOperations) {
  const auto type = lineType();
  const auto point = type->point(lineValues(0, 1, 1));
  Values values;
  values.insert(Symbol('c', 0), point);
  VectorValues delta;
  delta.insert(Symbol('c', 0), Vector1(.25));
  const auto retracted = values.retract(delta);
  const auto& next = retracted.at<IECMPoint>(Symbol('c', 0));
  EXPECT(assert_equal(Vector1(.25), point.localCoordinates(next), 1e-9));
  EXPECT_LONGS_EQUAL(1, traits<IECMPoint>::GetDimension(point));
  EXPECT(traits<IECMPoint>::Equals(
      point, traits<IECMPoint>::Retract(point, Vector(Vector1::Zero()))));
  const auto otherType = lineType()->point(lineValues(0, 1, 1));
  EXPECT(!point.equals(otherType));
  EXPECT(invalid([&] { type->linearize(otherType); }));
  EXPECT(invalid([&] { point.localCoordinates(otherType); }));
  EXPECT(invalid([&] { point.retract(Vector(Vector2::Zero())); }));
}

// Binding validates missing, repeated, extra, and incompatible ambient
// variables.
TEST(IECMPoint, RejectInvalidBindings) {
  const auto type = lineType();
  const auto values = lineValues(10, 11, 1);
  EXPECT(invalid([&] { type->point({{0, 10}}, values); }));
  EXPECT(invalid([&] { type->point({{0, 10}, {1, 10}}, values); }));
  Values wrong;
  wrong.insert(10, 1.);
  wrong.insert(11, 2.);
  EXPECT(invalid([&] { type->point({{0, 10}, {1, 11}}, wrong); }));
  Values oversized = values;
  oversized.insert(12, 0.);
  EXPECT(invalid([&] { type->point({{0, 10}, {1, 11}}, oversized); }));
  Values dimension = values;
  dimension.update(10, Vector(Vector2::Zero()));
  EXPECT(invalid([&] { type->point({{0, 10}, {1, 11}}, dimension); }));
}

// Runtime and native types take identical QP and nonlinear steps through CMC.
TEST(IEConstraintManifold, CompiledAndFactorDefinedOptimization) {
  Values nativeValues;
  nativeValues.insert(100, native_line::Point(1));
  VectorValues nativeDelta;
  nativeDelta.insert(100, Vector1(.25));
  EXPECT_DOUBLES_EQUAL(
      1.25, nativeValues.retract(nativeDelta).at<native_line::Point>(100).x,
      1e-12);
  const auto graph = coupledCost();
  std::optional<Values> reference, retractedReference;
  std::optional<VectorValues> stepReference;
  GaussianFactorGraph::shared_ptr linearReference;
  for (bool native : {false, true}) {
    const auto points = twoPoints(native ? compiledType() : lineType());
    if (native) {
      EXPECT(points.begin()->second.eConstraints()->empty());
      EXPECT(points.begin()->second.iConstraints()->empty());
    }
    // Both representations must expose the same reduced Gaussian objective.
    const auto transformed = IEOptimizer::pointGraph(graph, points);
    const IELMState state(points, {}, graph, transformed, .2, 10);
    IELMParams params;
    const IELMTrial::LinearUpdate step(.2, graph, state, params);
    EXPECT(step.solveSuccessful);
    if (linearReference)
      EXPECT(state.linearPointGraph->equals(*linearReference, 1e-9));
    if (stepReference)
      EXPECT(assert_equal(*stepReference, step.tangentVector, 1e-9));
    linearReference = state.linearPointGraph;
    stepReference = step.tangentVector;
    EXPECT(step.tangentVector.at(10)(0) > 0);
    EXPECT_DOUBLES_EQUAL(0, step.tangentVector.at(20)(0), 1e-8);

    // Compare recovered states after the same QP step and nonlinear retraction.
    const IELMTrial::NonlinearUpdate update(state, step, graph, 2);
    const auto retracted = ambientValues(update.newPoints);
    if (retractedReference)
      EXPECT(assert_equal(*retractedReference, retracted, 1e-8));
    retractedReference = retracted;
    EXPECT(update.newPoints.at(Symbol('c', 1)).activeIndices().exists(0));
    IELMOptimizer optimizer(params,
                            std::make_shared<IEConstraintManifold::Params>());
    const auto result = optimizer.optimize(graph, points, {});
    EXPECT_DOUBLES_EQUAL(1.5, result.at<Vector>(10)(0), 1e-5);
    EXPECT_DOUBLES_EQUAL(0, result.at<Vector>(20)(0), 1e-7);
    if (reference) EXPECT(assert_equal(*reference, result, 1e-7));
    reference = result;
#ifdef GTDYNAMICS_WITH_PROXQP
    params.qpBackend = IELMParams::QpBackend::ProxQP;
    const IELMTrial::LinearUpdate prox(.2, graph, state, params);
    EXPECT(prox.solveSuccessful);
    EXPECT(assert_equal(step.tangentVector, prox.tangentVector, 1e-7));
#endif
  }
}

// Manual type sharing agrees with automatic component discovery for the same
// NLP.
TEST(IEConstraintManifold, ManualAndAutomaticDiscoveryAgree) {
  const auto graph = coupledCost();
  const auto points = twoPoints(lineType());
  NonlinearEqualityConstraints equalities;
  NonlinearInequalityConstraints inequalities;
  for (const auto& [key, point] : points) {
    equalities.add(*point.eConstraints());
    inequalities.add(*point.iConstraints());
  }

  IELMOptimizer optimizer(IELMParams(),
                          std::make_shared<IEConstraintManifold::Params>());
  const auto explicitResult = optimizer.optimize(graph, points, {});
  const auto automatic = optimizer.optimize(graph, equalities, inequalities,
                                            ambientValues(points));
  EXPECT(assert_equal(explicitResult, automatic, 1e-5));
}

// Both optimizers accept manually bound points coupled to free ambient values.
TEST(IEConstraintManifold, GradientDescentUsesPointsAndFreeVariables) {
  NonlinearFactorGraph costs;
  costs.emplace_shared<PriorFactor<Vector>>(10, Vector(Vector1(3)),
                                            noiseModel::Unit::Create(1));
  costs.emplace_shared<PriorFactor<Vector>>(30, Vector(Vector1(2)),
                                            noiseModel::Unit::Create(1));
  costs.emplace_shared<BetweenFactor<Vector>>(10, 30, Vector(Vector1(0)),
                                              noiseModel::Unit::Create(1));
  Values free;
  free.insert(30, Vector(Vector1(0)));
  GradientDescentParams params;
  params.initialLambda = .1;
  params.maxIterations = 200;
  params.absoluteErrorTol = 1e-12;
  params.relativeErrorTol = 1e-12;
  IEGDOptimizer gd(params);
  for (bool native : {false, true}) {
    const auto type = native ? compiledType() : lineType();
    const IECMPoints points{{Symbol('c', 0), type->point({{0, 10}, {1, 11}},
                                                       lineValues(10, 11, 0))}};
    const auto result = gd.optimize(costs, points, free);
    const auto lm = IELMOptimizer().optimize(costs, points, free);
    EXPECT(assert_equal(lm, result, 1e-4));
    EXPECT_DOUBLES_EQUAL(8. / 3., result.at<Vector>(10)(0), 1e-4);
    EXPECT_DOUBLES_EQUAL(7. / 3., result.at<Vector>(30)(0), 1e-4);
  }
}

struct PreparationCounts {
  std::atomic<size_t> structural{0}, numerical{0};
  std::atomic<bool> failNext{false};
};

/** Count real equality linearizations rather than pointer identity. */
class CountedEquality final : public NonlinearEqualityConstraint {
  std::shared_ptr<PreparationCounts> counts_;
  bool nonlinear_;

 public:
  CountedEquality(const std::shared_ptr<PreparationCounts>& counts,
                  bool nonlinear)
      : NonlinearEqualityConstraint(noiseModel::Constrained::All(1),
                                    KeyVector{0, 1}),
        counts_(counts),
        nonlinear_(nonlinear) {}
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    const double x = values.at<Vector>(0)(0), y = values.at<Vector>(1)(0);
    if (H) {
      ++counts_->numerical;
      if (counts_->failNext.exchange(false))
        throw RetractionFailure("Injected equality construction failure");
      (*H)[0] = Matrix1::Constant(nonlinear_ ? -2 * x : -2);
      (*H)[1] = Matrix1::Identity();
    }
    return Vector1(y - (nonlinear_ ? x * x : 2 * x) - 3);
  }
  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<CountedEquality>(*this);
  }
};

class CountedBasisCreator final : public TangentSpaceBasisCreator {
  std::shared_ptr<PreparationCounts> counts_;
  TangentSpaceBasisCreator::shared_ptr inner_;

 public:
  CountedBasisCreator(const std::shared_ptr<PreparationCounts>& counts,
                      bool elimination)
      : TangentSpaceBasisCreator(std::make_shared<TangentSpaceBasisParams>()),
        counts_(counts) {
    if (elimination)
      inner_ = std::make_shared<EliminationBasisCreator>(
          [](const KeyVector&) { return KeyVector{0}; });
    else
      inner_ = std::make_shared<OrthonormalBasisCreator>();
  }
  shared_ptr clone() const override {
    auto copy = std::make_shared<CountedBasisCreator>(*this);
    copy->inner_ = inner_->clone();
    return copy;
  }
  std::shared_ptr<const TangentSpaceBasis> prepare(
      const NonlinearEqualityConstraints::shared_ptr constraints,
      const std::map<Key, size_t>& dimensions) const override {
    ++counts_->structural;
    return inner_->prepare(constraints, dimensions);
  }
  TangentSpaceBasis::shared_ptr create(
      const NonlinearEqualityConstraints::shared_ptr constraints,
      const Values& values) const override {
    return inner_->create(constraints, values);
  }
};

Values countedValues(Key x, Key y, double value, bool nonlinear = false) {
  auto result = lineValues(x, y, value);
  result.update(y,
                Vector(Vector1((nonlinear ? value * value : 2 * value) + 3)));
  return result;
}

IEConstraintManifold::shared_ptr countedType(
    const std::shared_ptr<PreparationCounts>& counts, bool affine,
    bool elimination = true) {
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<CountedEquality>(counts, !affine);
  IEConstraintManifold::Params params;
  params.affineEqualities = affine;
  params.equalityBasisCreator =
      std::make_shared<CountedBasisCreator>(counts, elimination);
  // Preparation must not evaluate sample numerical values, even invalid ones.
  const auto prototype =
      countedValues(0, 1, std::numeric_limits<double>::quiet_NaN());
  return std::make_shared<IEConstraintManifold>(
      equalities, NonlinearInequalityConstraints{}, prototype, params);
}

TEST(IEConstraintManifold, TypeOwnsStructureAndAffineNumerics) {
  for (bool elimination : {false, true}) {
    for (bool affine : {false, true}) {
      auto counts = std::make_shared<PreparationCounts>();
      const auto type = countedType(counts, affine, elimination);
      EXPECT_LONGS_EQUAL(1, counts->structural.load());
      EXPECT_LONGS_EQUAL(0, counts->numerical.load());
      const auto a =
          type->point({{0, 10}, {1, 11}}, countedValues(10, 11, 1, !affine));
      const auto b =
          type->point({{0, 20}, {1, 21}}, countedValues(20, 21, 2, !affine));
      EXPECT_LONGS_EQUAL(1, counts->structural.load());
      EXPECT_LONGS_EQUAL(affine ? 1 : 2, counts->numerical.load());
      EXPECT_DOUBLES_EQUAL(affine ? 2 : 4,
                           b.eBasis()->recoverJacobian(21)(0, 0) /
                               b.eBasis()->recoverJacobian(20)(0, 0),
                           1e-10);
      EXPECT_DOUBLES_EQUAL(0, b.evaluateEqualityViolation(), 1e-10);
      EXPECT(a.eConstraints() != b.eConstraints());
      EXPECT_DOUBLES_EQUAL(1, a.values().at<Vector>(10)(0), 1e-10);
    }
  }
}

TEST(IEConstraintManifold, ConcurrentFirstUseAndFailedInitialization) {
  auto counts = std::make_shared<PreparationCounts>();
  const auto type = countedType(counts, true);
  counts->failNext = true;
  bool failed = false;
  try {
    type->point(countedValues(0, 1, 0));
  } catch (const RetractionFailure&) {
    failed = true;
  }
  EXPECT(failed);
  std::promise<void> start;
  auto ready = start.get_future().share();
  auto build = [&](Key x, Key y) {
    ready.wait();
    return type->point({{0, x}, {1, y}}, countedValues(x, y, 2));
  };
  auto first = std::async(std::launch::async, build, 10, 11);
  auto second = std::async(std::launch::async, build, 20, 21);
  start.set_value();
  const auto a = first.get(), b = second.get();
  EXPECT_DOUBLES_EQUAL(2, a.eBasis()->recoverJacobian(11)(0, 0), 1e-10);
  EXPECT_DOUBLES_EQUAL(2, b.eBasis()->recoverJacobian(21)(0, 0), 1e-10);
  EXPECT_LONGS_EQUAL(1, counts->structural.load());
  EXPECT_LONGS_EQUAL(
      2, counts->numerical.load());  // Failed attempt plus one success.
}

TEST(IEConstraintManifold, WeightsAndDampingDoNotChangeEqualities) {
  auto counts = std::make_shared<PreparationCounts>();
  const auto type = countedType(counts, true);
  const auto point = type->point({{0, 10}, {1, 11}}, countedValues(10, 11, 1));
  const auto basis = point.eBasis()->recoverJacobian(11);
  for (double sigma : {1., 3.}) {
    NonlinearFactorGraph costs;
    costs.emplace_shared<PriorFactor<Vector>>(
        10, Vector(Vector1(4)), noiseModel::Isotropic::Sigma(1, sigma));
    const IECMPoints points{{Symbol('c', 0), point}};
    const auto graph = IEOptimizer::pointGraph(costs, points);
    IELMState state(points, {}, costs, graph, .1, 10.);
    for (double damping : {.1, 1.}) {
      const IELMTrial::LinearUpdate step(damping, costs, state, IELMParams());
      EXPECT(step.solveSuccessful);
      EXPECT(step.delta.at(Symbol('c', 0)).allFinite());
    }
    auto metric = IERetractorParams::createVaryingSigmas();
    metric->metricSigmas->insert(10, Vector1(sigma));
    metric->metricSigmas->insert(11, Vector1(2 * sigma));
    const auto first = point.projectionMetric(*metric);
    EXPECT(first == point.projectionMetric(*metric));
    EXPECT(assert_equal(basis, point.eBasis()->recoverJacobian(11)));
  }
  EXPECT_LONGS_EQUAL(1, counts->structural.load());
  EXPECT_LONGS_EQUAL(1, counts->numerical.load());
}

TEST(IEConstraintManifold, ReadOnlyViewCannotModifySharedAffineBasis) {
  const auto type = lineType();
  const auto a = type->point(lineValues(0, 1, 1));
  ReadOnlyBasis view(a.eBasis());
  bool rejected = false;
  try {
    view.construct(lineValues(0, 1, 2));
  } catch (const std::logic_error&) {
    rejected = true;
  }
  EXPECT(rejected);
  const auto b = type->point(lineValues(0, 1, 2));
  EXPECT(assert_equal(a.eBasis()->recoverJacobian(1),
                      b.eBasis()->recoverJacobian(1)));
  NonlinearEqualityConstraints extra;
  extra.push_back(a.iConstraints()->at(0)->createEqualityConstraint());
  const auto boundary = a.eBasis()->createWithAdditionalConstraints(extra, a.values());
  EXPECT_LONGS_EQUAL(0, boundary->dim());
  EXPECT_LONGS_EQUAL(1, a.dim());
}

// Empty inequalities are a normal type, and conditioning can fix all
// coordinates.
TEST(IEConstraintManifold, EqualityOnlyAndZeroDimensionalPoints) {
  const auto [equalities, ignored] = lineConstraints(0, 1);
  auto type = std::make_shared<IEConstraintManifold>(
      equalities, NonlinearInequalityConstraints(), lineValues(0, 1, 1));
  const auto point = type->point(lineValues(0, 1, 1));
  EXPECT(point.activeIndices().empty());
  EXPECT_LONGS_EQUAL(1, point.dim());
  Values fixed;
  fixed.insert(0, Vector(Vector1::Zero()));
  const auto conditioned = lineType()->condition(fixed);
  Values remaining;
  remaining.insert(1, Vector(Vector1::Zero()));
  const auto fixedPoint = conditioned->point(remaining);
  EXPECT_LONGS_EQUAL(0, fixedPoint.dim());
  EXPECT_LONGS_EQUAL(0, fixedPoint.localCoordinates(fixedPoint).size());
  EXPECT(assert_equal(remaining, fixedPoint.retract(Vector(0)).values(), 1e-8));
}

/** q*x-y=0 demonstrates value-dependent equality geometry after conditioning.
 */
class SlopeConstraint final : public NonlinearEqualityConstraint {
  bool affine_;
  std::shared_ptr<PreparationCounts> counts_;

 public:
  explicit SlopeConstraint(bool affine = false,
                           std::shared_ptr<PreparationCounts> counts = {})
      : NonlinearEqualityConstraint(noiseModel::Constrained::All(1),
                                    KeyVector{2, 0, 1}),
        affine_(affine),
        counts_(std::move(counts)) {}
  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    const double q = values.at<Vector>(2)(0), x = values.at<Vector>(0)(0),
                 y = values.at<Vector>(1)(0);
    if (H) {
      if (counts_) ++counts_->numerical;
      (*H)[0] = Matrix1::Constant(affine_ ? 1 : x);
      (*H)[1] = Matrix1::Constant(affine_ ? 2 : q);
      (*H)[2] = Matrix1::Constant(-1);
    }
    return Vector1(affine_ ? q + 2 * x - y : q * x - y);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<SlopeConstraint>(*this);
  }
};

// Every conditioning has its own constant geometry and affine offset.
TEST(IEConstraintManifold, ConditionedAffineCachesAreIndependent) {
  auto counts = std::make_shared<PreparationCounts>();
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<SlopeConstraint>(true, counts);
  auto prototype = lineValues(0, 1, 0);
  prototype.insert(2, Vector(Vector1(1)));
  prototype.update(1, Vector(Vector1(1)));
  IEConstraintManifold::Params params;
  params.affineEqualities = true;
  const auto family = std::make_shared<IEConstraintManifold>(
      equalities, NonlinearInequalityConstraints{}, prototype, params);
  Values q1, q3;
  q1.insert(2, Vector(Vector1(1)));
  q3.insert(2, Vector(Vector1(3)));
  const auto first = family->condition(q1), third = family->condition(q3);
  EXPECT_LONGS_EQUAL(0, counts->numerical.load());
  auto values = lineValues(0, 1, 0);
  values.update(1, Vector(Vector1(1)));
  const auto a = first->point(values);
  values = lineValues(0, 1, 1);
  values.update(1, Vector(Vector1(3)));
  const auto next = first->point(values);
  EXPECT_LONGS_EQUAL(1, counts->numerical.load());
  EXPECT_DOUBLES_EQUAL(0, a.evaluateEqualityViolation(), 1e-10);
  EXPECT_DOUBLES_EQUAL(0, next.evaluateEqualityViolation(), 1e-10);
  values.update(1, Vector(Vector1(5)));
  const auto b = third->point(values);
  EXPECT_LONGS_EQUAL(2, counts->numerical.load());
  EXPECT_DOUBLES_EQUAL(0, b.evaluateEqualityViolation(), 1e-10);
  family->point(prototype);
  EXPECT_LONGS_EQUAL(3, counts->numerical.load());
}

// Conditioning defines distinct spaces and never reuses q-dependent
// projections.
TEST(IEConstraintManifold, ConditioningChangesTheSpace) {
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<SlopeConstraint>();
  auto prototype = lineValues(0, 1, 0);
  prototype.insert(2, Vector(Vector1(1)));
  IEConstraintManifold::Params params;
  params.equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector&) { return KeyVector{0, 2}; });
  const auto family = std::make_shared<IEConstraintManifold>(
      equalities, NonlinearInequalityConstraints(), prototype, params);
  Values q1, q2;
  q1.insert(2, Vector(Vector1(1)));
  q2.insert(2, Vector(Vector1(3)));
  const auto first = family->condition(q1), second = family->condition(q2);
  auto state = lineValues(0, 1, 0);
  const auto a = first->point(state), b = second->point(state);
  EXPECT(a.manifold() != b.manifold());
  EXPECT_DOUBLES_EQUAL(1,
                       a.eBasis()->recoverJacobian(1)(0, 0) /
                           a.eBasis()->recoverJacobian(0)(0, 0),
                       1e-10);
  EXPECT_DOUBLES_EQUAL(3,
                       b.eBasis()->recoverJacobian(1)(0, 0) /
                           b.eBasis()->recoverJacobian(0)(0, 0),
                       1e-10);
  q1.update(2, Vector(Vector1(9)));
  EXPECT_DOUBLES_EQUAL(
      1,
      first->point(state).eBasis()->recoverJacobian(1)(0, 0) /
          first->point(state).eBasis()->recoverJacobian(0)(0, 0),
      1e-10);
  EXPECT(invalid([&] { a.localCoordinates(b); }));
}

// A shared conditioned type supports independent simultaneous metric
// projections.
TEST(IEConstraintManifold,
     ParallelRetractionDoesNotShareConditioningOrMetrics) {
  auto settings = std::make_shared<IERetractorParams>();
  settings->useVaryingSigma = true;
  settings->metricSigmas = std::make_shared<VectorValues>();
  IEConstraintManifold::Params params;
  params.affineEqualities = true;
  params.equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector&) { return KeyVector{0, 2}; });
  params.retractorCreator = std::make_shared<UniversalIERetractorCreator>(
      std::make_shared<ReducedALRetractor>(settings));
  auto [equalities, inequalities] = lineConstraints(0, 1);
  equalities = NonlinearEqualityConstraints();
  equalities.emplace_shared<SlopeConstraint>(true);
  auto prototype = lineValues(0, 1, 0);
  prototype.insert(2, Vector(Vector1(0)));
  auto family = std::make_shared<IEConstraintManifold>(equalities, inequalities,
                                                       prototype, params);
  Values fixed;
  fixed.insert(2, Vector(Vector1(2)));
  const auto type = family->condition(fixed);
  Values stateA = lineValues(10, 11, 1), stateB = lineValues(20, 21, 3);
  stateA.update(11, Vector(Vector1(4)));
  stateB.update(21, Vector(Vector1(8)));
  const auto a = type->point({{0, 10}, {1, 11}}, stateA),
             b = type->point({{0, 20}, {1, 21}}, stateB);
  VectorValues metricA, metricB;
  metricA.insert(10, Vector1(.1));
  metricA.insert(11, Vector1(2));
  metricB.insert(20, Vector1(3));
  metricB.insert(21, Vector1(.2));
  auto evaluate = [](const IECMPoint& p, const VectorValues& metric,
                     double step) {
    VectorValues delta;
    const Key x = p.bindings().at(0), y = p.bindings().at(1);
    delta.insert(x, Vector1(step));
    delta.insert(y, Vector1(2 * step));
    return p.retract(delta, {}, nullptr, &metric);
  };
  const auto serialA = evaluate(a, metricA, -2),
             serialB = evaluate(b, metricB, 1);
  auto futureA =
      std::async(std::launch::async, [&] { return evaluate(a, metricA, -2); });
  auto futureB =
      std::async(std::launch::async, [&] { return evaluate(b, metricB, 1); });
  EXPECT(assert_equal(serialA.values(), futureA.get().values(), 1e-8));
  EXPECT(assert_equal(serialB.values(), futureB.get().values(), 1e-8));
  EXPECT_DOUBLES_EQUAL(0, serialA.values().at<Vector>(10)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(2, serialA.values().at<Vector>(11)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(4, serialB.values().at<Vector>(20)(0), 1e-7);
  EXPECT(settings->metricSigmas->size() == 0);
  EXPECT(a.retractor()->params()->metricSigmas->size() == 0);
  EXPECT(assert_equal(stateA, a.values()));
  EXPECT(assert_equal(stateB, b.values()));
}

// New component identities skip both occupied ambient and free-variable keys.
TEST(IEConstraintManifold, PointKeysHaveIndependentIdentity) {
  const Key x = Symbol('c', 0), y = Symbol('c', 1);
  auto initial = lineValues(x, y, 1);
  const auto [equalities, inequalities] = lineConstraints(x, y);
  auto params = std::make_shared<IEConstraintManifold::Params>();
  const auto components = IEOptimizer::identifyConnectedComponents(
      equalities, inequalities, initial, params);
  const auto& points = components.points;
  EXPECT_LONGS_EQUAL(Symbol('c', 2), points.begin()->first);
  EXPECT(invalid([&] {
    IEOptimizer::validatePoints(IECMPoints{{x, points.begin()->second}}, {});
  }));
  EXPECT(invalid([&] {
    IEOptimizer::validatePoints(
        IECMPoints{{Symbol('c', 3), points.begin()->second},
                 {Symbol('c', 4), points.begin()->second}},
        {});
  }));
}

// Policy capture keeps only local metrics and binds them to each point once.
TEST(IEConstraintManifold, ComponentLocalMetricSnapshots) {
  auto weights = IERetractorParams::createVaryingSigmas();
  weights->metricSigmas->insert(0, Vector1(1));
  weights->metricSigmas->insert(1, Vector1(2));
  for (Key key = 100; key < 1100; ++key)
    weights->metricSigmas->insert(key, Vector1(3));
  auto [equalities, inequalities] = lineConstraints(0, 1);
  IEConstraintManifold::Params params;
  params.affineEqualities = true;
  params.retractorCreator = std::make_shared<UniversalIERetractorCreator>(
      std::make_shared<ReducedALRetractor>(weights));
  auto type = std::make_shared<IEConstraintManifold>(
      equalities, inequalities, lineValues(0, 1, 0), params);
  const auto point = type->point({{0, 10}, {1, 11}}, lineValues(10, 11, 0));
  const auto workspace = point.retractor();
  EXPECT_LONGS_EQUAL(2, workspace->params()->metricSigmas->size());
  EXPECT(workspace->params()->metricSigmas->exists(10));
  auto trialWeights = *weights->metricSigmas;
  trialWeights.insert(10, Vector1(4));
  const auto trialWorkspace = point.retractor(&trialWeights);
  EXPECT_LONGS_EQUAL(1, trialWorkspace->params()->metricSigmas->size());
  EXPECT(trialWorkspace->params()->metricSigmas->exists(10));
  weights->metricSigmas->at(0)(0) = 100;
  VectorValues step;
  step.insert(10, Vector1(2));
  step.insert(11, Vector1(0));
  const auto projected = point.retract(step);
  EXPECT_DOUBLES_EQUAL(1, projected.values().at<Vector>(10)(0), 1e-9);
  EXPECT_DOUBLES_EQUAL(2, projected.values().at<Vector>(11)(0), 1e-9);
}

/** A fixed projection metric, separate from the outer cost metric. This uses
 * the same policy separation as the articulated retractor creator.
 */
class FixedMetricCreator final : public IERetractorCreator {
 public:
  shared_ptr clone() const override {
    return std::make_shared<FixedMetricCreator>(*this);
  }

  IERetractor::shared_ptr create(const IECMPoint& point) const override {
    auto projection = IERetractorParams::createVaryingSigmas();
    projection->metricSigmas->insert(point.bindings().at(0), Vector1(1));
    projection->metricSigmas->insert(point.bindings().at(1), Vector1(2));
    return std::make_shared<ReducedALRetractor>(projection);
  }
};

// A cost-derived trial metric must not replace another type's fixed metric.
TEST(IEConstraintManifold, MixedRetractionPoliciesKeepTheirOwnMetrics) {
  const auto [equalities, inequalities] = lineConstraints(0, 1);
  IEConstraintManifold::Params fixedParams, varyingParams;
  fixedParams.affineEqualities = varyingParams.affineEqualities = true;
  fixedParams.retractorCreator = std::make_shared<FixedMetricCreator>();
  varyingParams.retractorCreator =
      std::make_shared<UniversalIERetractorCreator>(
          std::make_shared<ReducedALRetractor>(
              IERetractorParams::createVaryingSigmas()));
  const auto fixed = std::make_shared<IEConstraintManifold>(
      equalities, inequalities, lineValues(0, 1, 0), fixedParams);
  const auto varying = std::make_shared<IEConstraintManifold>(
      equalities, inequalities, lineValues(0, 1, 0), varyingParams);
  const IECMPoints points{
      {Symbol('c', 0), fixed->point({{0, 10}, {1, 11}}, lineValues(10, 11, 0))},
      {Symbol('c', 1),
       varying->point({{0, 20}, {1, 21}}, lineValues(20, 21, 0))}};
  NonlinearFactorGraph costs;
  for (Key key : {10, 11, 20, 21})
    costs.emplace_shared<PriorFactor<Vector>>(key, Vector(Vector1(0)),
                                              noiseModel::Unit::Create(1));
  const IELMState state(points, {}, costs,
                        IEOptimizer::pointGraph(costs, points), .1, 10);
  IELMTrial::LinearUpdate trial;
  trial.tangentVector = ambientValues(points).zeroVectors();
  // Exercise ambient feasibility projection with a target outside y=2x.
  trial.tangentVector.at(10)(0) = 2;
  const IELMTrial::NonlinearUpdate update(state, trial, costs, 2);
  EXPECT_DOUBLES_EQUAL(
      1, update.newPoints.at(Symbol('c', 0)).values().at<Vector>(10)(0), 1e-8);
  EXPECT_DOUBLES_EQUAL(0, points.at(Symbol('c', 0)).values().at<Vector>(10)(0),
                       1e-12);
}

// Embedded expressions retain local identities, sigmas, and unclipped faces.
TEST(IEConstraintManifold, BoundExpressionsAndNonlinearGeometry) {
  const Double_ x(Key(0)), y(Key(1));
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<ExpressionEqualityConstraint<double>>(x * x + y, 1.,
                                                                  Vector1(.2));
  NonlinearInequalityConstraints inequalities;
  inequalities.push_back(ScalarExpressionInequalityConstraint::GeqZero(x, .25));
  Values local, values;
  local.insert(0, 0.);
  local.insert(1, 1.);
  values.insert(10, 0.);
  values.insert(11, 1.);
  const auto type =
      std::make_shared<IEConstraintManifold>(equalities, inequalities, local);
  const auto point = type->point({{0, 10}, {1, 11}}, values);
  VectorValues outward;
  outward.insert(10, Vector1(-2));
  outward.insert(11, Vector1(0));
  EXPECT_DOUBLES_EQUAL(
      8,
      (point.tangentSpace()->rows(0, true) * point.toCoordinates(outward))(0),
      1e-9);
  values.update(10, 2.);
  values.update(11, -3.);
  const auto next = point.manifold()->point(point.bindings(), values);
  EXPECT(next.eBasis() != point.eBasis());
  EXPECT_DOUBLES_EQUAL(0,
                       (4 * next.eBasis()->recoverJacobian(10) +
                        next.eBasis()->recoverJacobian(11))
                           .norm(),
                       1e-9);
  const auto boundary = next.iConstraints()->at(0)->createEqualityConstraint();
  EXPECT_DOUBLES_EQUAL(-2, boundary->unwhitenedError(values)(0), 1e-9);
  EXPECT_DOUBLES_EQUAL(.25, boundary->sigmas()(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(0, next.evaluateEqualityViolation(), 1e-9);
}

// Free-only factors retain their original behavior and avoid ownership copies.
TEST(IEConstraintManifold, PointGraphPreservesFreeFactors) {
  auto points = twoPoints(lineType());
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(50, Vector(Vector1(3)),
                                            noiseModel::Unit::Create(1));
  graph.emplace_shared<BetweenFactor<Vector>>(10, 50, Vector(Vector1(0)),
                                              noiseModel::Unit::Create(1));
  const auto transformed = IEOptimizer::pointGraph(graph, points);
  EXPECT(transformed.at(0) == graph.at(0));
  EXPECT(transformed.at(1) != graph.at(1));
  Values values;
  for (const auto& [key, point] : points) values.insert(key, point);
  values.insert(50, Vector(Vector1(1)));
  auto ambient = ambientValues(points);
  ambient.insert(50, Vector(Vector1(1)));
  EXPECT_DOUBLES_EQUAL(graph.error(ambient), transformed.error(values), 1e-12);
}

}  // namespace runtime_types
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
