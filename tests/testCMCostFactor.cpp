/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  testCMCostFactor.cpp
 * @brief Test cost arguments connected to immutable point slots.
 * @author Yetong Zhang
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/cmopt/ECMPoint.h>
#include <gtdynamics/constrained/CMCostFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/constrained/NonlinearEqualityConstraint.h>
#include <gtsam/constrained/LinearConstraint.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <sstream>
#endif

using namespace gtsam;
using namespace gtdynamics;

TEST(CMCostFactor, pose) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;
  Key x4_key = 4;
  Key cm_key = 100;

  // Constraints.
  auto constraints = std::make_shared<gtsam::NonlinearEqualityConstraints>();
  auto noise = noiseModel::Unit::Create(3);
  auto factor12 = std::make_shared<BetweenFactor<Point3>>(
      x1_key, x2_key, Point3(0, 0, 1), noise);
  auto factor23 = std::make_shared<BetweenFactor<Point3>>(
      x2_key, x3_key, Point3(0, 1, 0), noise);
  constraints->emplace_shared<gtsam::ZeroCostConstraint>(factor12);
  constraints->emplace_shared<gtsam::ZeroCostConstraint>(factor23);

  // Create point values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Point3(0, 0, 0));
  cm_base_values.insert(x2_key, Point3(0, 0, 1));
  cm_base_values.insert(x3_key, Point3(0, 1, 1));

  // Construct point.
  auto type = std::make_shared<EConstraintManifold>(*constraints, cm_base_values);
  ECMPoint point = type->point(cm_base_values);

  // Construct cost factor.
  auto cost_factor1 = std::make_shared<BetweenFactor<Point3>>(
      x3_key, x4_key, Point3(1, 0, 0), noise);
  auto cost_factor2 = std::make_shared<BetweenFactor<Point3>>(
      x1_key, x3_key, Point3(1, 0, 0), noise);

  // Connect original arguments to identity-bound local slots.
  CMCostFactor::OwnershipIndex ownership;
  ownership[x1_key] = {cm_key, x1_key};
  ownership[x2_key] = {cm_key, x2_key};
  ownership[x3_key] = {cm_key, x3_key};

  // Test constructor.
  CMCostFactor subs_factor1(cost_factor1, ownership, Values(), CMCostFactor::PointType::Equality);
  CMCostFactor subs_factor2(cost_factor2, ownership, Values(), CMCostFactor::PointType::Equality);

  // Test keys.
  EXPECT(subs_factor1.keys() == KeyVector({cm_key, x4_key}));
  EXPECT(subs_factor2.keys() == KeyVector({cm_key}));

  // Construct values for testing.
  Values values;
  values.insert(cm_key, point);
  values.insert(x4_key, Point3(2, 2, 2));

  // Check error.
  Vector expected_error1 = (Vector(3) << 1, 1, 1).finished();
  EXPECT(assert_equal(expected_error1, subs_factor1.unwhitenedError(values)));

  Vector expected_error2 = (Vector(3) << -1, 1, 1).finished();
  EXPECT(assert_equal(expected_error2, subs_factor2.unwhitenedError(values)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(subs_factor1, values, 1e-7, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(subs_factor2, values, 1e-7, 1e-5);
}

TEST(CMCostFactor, fully_constrained_manifold) {
  // Keys.
  Key x1_key = 1;
  Key x2_key = 2;
  Key x3_key = 3;
  Key x4_key = 4;
  Key cm_key = 100;

  // Constraints.
  auto constraints = std::make_shared<gtsam::NonlinearEqualityConstraints>();
  auto noise = noiseModel::Unit::Create(3);
  auto factor1 =
      std::make_shared<PriorFactor<Point3>>(x1_key, Point3(0, 0, 0), noise);
  auto factor12 = std::make_shared<BetweenFactor<Point3>>(
      x1_key, x2_key, Point3(0, 0, 1), noise);
  auto factor23 = std::make_shared<BetweenFactor<Point3>>(
      x2_key, x3_key, Point3(0, 1, 0), noise);
  constraints->emplace_shared<gtsam::ZeroCostConstraint>(factor1);
  constraints->emplace_shared<gtsam::ZeroCostConstraint>(factor12);
  constraints->emplace_shared<gtsam::ZeroCostConstraint>(factor23);

  // Create point values for testing.
  Values cm_base_values;
  cm_base_values.insert(x1_key, Point3(0, 0, 0));
  cm_base_values.insert(x2_key, Point3(0, 0, 1));
  cm_base_values.insert(x3_key, Point3(0, 1, 1));

  // Construct point.
  auto type = std::make_shared<EConstraintManifold>(*constraints, cm_base_values);
  ECMPoint point = type->point(cm_base_values);

  // Construct cost factor.
  auto cost_factor1 = std::make_shared<BetweenFactor<Point3>>(
      x3_key, x4_key, Point3(1, 0, 0), noise);
  auto cost_factor2 = std::make_shared<BetweenFactor<Point3>>(
      x1_key, x3_key, Point3(1, 0, 0), noise);

  // Connect original arguments to identity-bound local slots.
  CMCostFactor::OwnershipIndex ownership;
  ownership[x1_key] = {cm_key, x1_key};
  ownership[x2_key] = {cm_key, x2_key};
  ownership[x3_key] = {cm_key, x3_key};
  Values fc_manifolds;
  fc_manifolds.insert(cm_key, point);

  // Test constructor.
  CMCostFactor subs_factor1(cost_factor1, ownership, fc_manifolds, CMCostFactor::PointType::Equality);
  CMCostFactor subs_factor2(cost_factor2, ownership, fc_manifolds, CMCostFactor::PointType::Equality);
  EXPECT(!subs_factor2.checkActive());

  // Construct values for testing.
  Values values;
  values.insert(x4_key, Point3(2, 2, 2));

  // Check error.
  Vector expected_error1 = (Vector(3) << 1, 1, 1).finished();
  EXPECT(assert_equal(expected_error1, subs_factor1.unwhitenedError(values)));

  // Check jacobian computation is correct.
  EXPECT_CORRECT_FACTOR_JACOBIANS(subs_factor1, values, 1e-7, 1e-5);
}

namespace {

// Preserve argument order explicitly, including mixed ambient dimensions.
class MixedArgumentCost final : public NoiseModelFactor {
 public:
  explicit MixedArgumentCost(const KeyVector& keys)
      : NoiseModelFactor(noiseModel::Diagonal::Sigmas(Vector3(2, 3, 4)), keys) {}

  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    const std::vector<Matrix> blocks{
        (Matrix(3, 2) << 1, 2, -1, 3, 2, 1).finished(),
        (Matrix(3, 1) << 3, -2, 1).finished(),
        (Matrix(3, 3) << 2, 1, 0, -1, 3, 2, 1, 0, 4).finished(),
        (Matrix(3, 1) << -1, 4, 2).finished(),
        (Matrix(3, 2) << 2, -3, 1, 2, -2, 1).finished()};
    Vector result = Vector3(.5, -1, 2);
    for (size_t index = 0; index < keys().size(); ++index) {
      result += blocks[index] * values.at<Vector>(keys()[index]);
      if (H) (*H)[index] = blocks[index];
    }
    return result;
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<MixedArgumentCost>(*this);
  }
};

Values mixedValues(Key scalar, Key pair, Key unused, double coordinate) {
  Values values;
  values.insert(scalar, Vector(Vector1(coordinate)));
  values.insert(pair, Vector(Vector2(2 * coordinate + 1, 3 * coordinate - 2)));
  values.insert(unused, Vector(Vector3(7, 8, 9)));
  return values;
}

EConstraintManifold::shared_ptr mixedType() {
  NonlinearEqualityConstraints equalities;
  equalities.push_back(
      LinearConstraint::Equal(JacobianFactor(
          3, Matrix2::Identity(), 77,
          (Matrix(2, 1) << -2, -3).finished(), Vector2(1, -2)))
          .createEqualityFactor());
  EConstraintManifold::Params params;
  params.affineEqualities = true;
  return std::make_shared<EConstraintManifold>(
      equalities, mixedValues(77, 3, 900, 1), params);
}

// A nonlinear chart makes stale cached recovery Jacobians observable.
class Parabola final : public NonlinearEqualityConstraint {
 public:
  Parabola()
      : NonlinearEqualityConstraint(noiseModel::Constrained::All(1),
                                    KeyVector{7, 22}) {}

  Vector unwhitenedError(const Values& values,
                         OptionalMatrixVecType H = nullptr) const override {
    const double x = values.at<Vector>(7)(0);
    if (H) {
      (*H)[0] = Matrix1::Constant(-2 * x);
      (*H)[1] = Matrix1::Identity();
    }
    return Vector1(values.at<Vector>(22)(0) - x * x);
  }

  NonlinearFactor::shared_ptr clone() const override {
    return std::make_shared<Parabola>(*this);
  }
};

Values parabolaValues(Key x, Key y, double coordinate) {
  Values values;
  values.insert(x, Vector(Vector1(coordinate)));
  values.insert(y, Vector(Vector1(coordinate * coordinate)));
  return values;
}

}  // namespace

TEST(CMCostFactor, OrderedSlotsCoupledPointsAndFreeVariables) {
  const auto type = mixedType();
  const auto first = type->point({{77, 31}, {3, 11}, {900, 55}},
                                 mixedValues(31, 11, 55, 1));
  const auto second = type->point({{77, 71}, {3, 21}, {900, 99}},
                                  mixedValues(71, 21, 99, 2));
  const auto original = std::make_shared<MixedArgumentCost>(
      KeyVector{11, 31, 500, 71, 21});
  NonlinearFactorGraph graph;
  graph.push_back(original);

  Values ambient = first.values();
  ambient.insert(second.values());
  ambient.insert(500, Vector(Vector3(1, -2, 3)));

  // Exact concrete extraction is exercised on both GTSAM value categories.
  for (const auto category : {CMCostFactor::PointType::Equality,
                              CMCostFactor::PointType::Inequality}) {
    Values points;
    if (category == CMCostFactor::PointType::Equality) {
      points.insert(100, first);
      points.insert(101, second);
    } else {
      points.insert(100, static_cast<const IECMPoint&>(first));
      points.insert(101, static_cast<const IECMPoint&>(second));
    }
    const auto transformed = pointCostGraph(graph, points, category);
    EXPECT_LONGS_EQUAL(1, transformed.size());
    const auto factor = std::dynamic_pointer_cast<CMCostFactor>(transformed[0]);
    EXPECT(factor != nullptr);
    EXPECT(factor->keys() == KeyVector({100, 500, 101}));

    Values values = points;
    values.insert(500, ambient.at(500));
    EXPECT(assert_equal(original->unwhitenedError(ambient),
                        factor->unwhitenedError(values)));
    EXPECT_DOUBLES_EQUAL(original->error(ambient), factor->error(values), 1e-9);
    EXPECT(assert_equal(original->whitenedError(ambient),
                        factor->whitenedError(values)));
    EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values, 1e-7, 1e-5);

    // Clone keeps the compiled connection layout and still uses current values.
    const auto cloned = std::dynamic_pointer_cast<CMCostFactor>(factor->clone());
    EXPECT(cloned->keys() == factor->keys());
    EXPECT(assert_equal(factor->unwhitenedError(values),
                        cloned->unwhitenedError(values)));
    EXPECT_CORRECT_FACTOR_JACOBIANS(*cloned, values, 1e-7, 1e-5);

    // A fresh Values container can contain a point with incompatible bindings.
    const auto rebound = type->point({{77, 12}, {3, 32}, {900, 56}},
                                     mixedValues(12, 32, 56, 1));
    Values wrong;
    if (category == CMCostFactor::PointType::Equality)
      wrong.insert(100, rebound);
    else
      wrong.insert(100, static_cast<const IECMPoint&>(rebound));
    wrong.insert(101, points.at(101));
    wrong.insert(500, ambient.at(500));
    CHECK_EXCEPTION(factor->unwhitenedError(wrong), std::invalid_argument);
    std::vector<Matrix> H(factor->size());
    CHECK_EXCEPTION(factor->unwhitenedError(wrong, &H), std::invalid_argument);
  }
}

TEST(CMCostFactor, UsesCurrentNonlinearRecoveryAfterRetraction) {
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<Parabola>();
  EConstraintManifold::Params params;
  params.equalityBasisCreator = std::make_shared<EliminationBasisCreator>(
      [](const KeyVector&) { return KeyVector{7}; });
  const auto type = std::make_shared<EConstraintManifold>(
      equalities, parabolaValues(7, 22, 1), params);
  const auto first = type->point({{7, 13}, {22, 41}},
                                 parabolaValues(13, 41, 1));
  Values points;
  points.insert(100, first);
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(41, Vector(Vector1(2.1)),
                                            noiseModel::Isotropic::Sigma(1, .4));
  const auto transformed = pointCostGraph(
      graph, points, CMCostFactor::PointType::Equality);
  const auto factor = std::dynamic_pointer_cast<CMCostFactor>(transformed[0]);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, points, 1e-7, 1e-5);

  const auto moved = first.retract(Vector(Vector1(.4)));
  EXPECT(moved.bindings() == first.bindings());
  EXPECT((moved.eBasis()->recoverJacobian(41) -
          first.eBasis()->recoverJacobian(41)).norm() > .1);
  Values next;
  next.insert(100, moved);
  std::vector<Matrix> H(factor->size());
  EXPECT(assert_equal(
      std::dynamic_pointer_cast<NoiseModelFactor>(graph[0])->unwhitenedError(
          moved.values()),
      factor->unwhitenedError(next, &H)));
  EXPECT(assert_equal(moved.eBasis()->recoverJacobian(41), H[0]));
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, next, 1e-7, 1e-5);
}

TEST(CMCostFactor, FixedPointArgumentsAndFreeOnlyReuse) {
  const Key slot = 37, ambientKey = 12, pointKey = 100, freeKey = 200;
  const auto noise = noiseModel::Isotropic::Sigma(3, 2.);
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<ZeroCostConstraint>(
      std::make_shared<PriorFactor<Point3>>(slot, Point3(1, 2, 3), noise));
  Values prototype, ambient;
  prototype.insert(slot, Point3(1, 2, 3));
  ambient.insert(ambientKey, Point3(1, 2, 3));
  const auto type = std::make_shared<EConstraintManifold>(equalities, prototype);
  const auto fixed = type->point({{slot, ambientKey}}, ambient);
  EXPECT_LONGS_EQUAL(0, fixed.dim());
  Values points;
  points.insert(pointKey, fixed);

  NonlinearFactorGraph graph;
  graph.emplace_shared<BetweenFactor<Point3>>(
      ambientKey, freeKey, Point3(-1, 1, 2), noise);
  graph.emplace_shared<PriorFactor<Point3>>(ambientKey, Point3(4, 5, 6), noise);
  const auto freeOnly = std::make_shared<PriorFactor<Point3>>(
      freeKey, Point3(0, 1, 2), noise);
  graph.push_back(freeOnly);
  graph.push_back(NonlinearFactor::shared_ptr());
  const auto transformed = pointCostGraph(
      graph, points, CMCostFactor::PointType::Equality);
  EXPECT_LONGS_EQUAL(2, transformed.size());
  EXPECT(transformed[0]->keys() == KeyVector({freeKey}));
  EXPECT(transformed[1].get() == freeOnly.get());

  Values values;
  values.insert(freeKey, Point3(2, 4, 5));
  ambient.insert(values);
  EXPECT_DOUBLES_EQUAL(graph[0]->error(ambient),
                       transformed[0]->error(values), 1e-9);
  const auto factor = std::dynamic_pointer_cast<CMCostFactor>(transformed[0]);
  EXPECT_CORRECT_FACTOR_JACOBIANS(*factor, values, 1e-7, 1e-5);
}

#ifdef GTDYNAMICS_ENABLE_BOOST_SERIALIZATION
TEST(CMCostFactor, SerializationRetainsSlotConnections) {
  const auto type = mixedType();
  const auto point = type->point({{77, 31}, {3, 11}, {900, 55}},
                                 mixedValues(31, 11, 55, 1));
  Values points;
  points.insert(100, point);
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Vector>>(11, Vector(Vector2(-2, 4)),
                                            noiseModel::Isotropic::Sigma(2, 3));
  const auto transformed = pointCostGraph(
      graph, points, CMCostFactor::PointType::Equality);
  const auto factor = std::dynamic_pointer_cast<CMCostFactor>(transformed[0]);

  // Register the original polymorphic factor; points are not serialized here.
  std::stringstream stream;
  {
    boost::archive::text_oarchive archive(stream);
    archive.register_type<noiseModel::Isotropic>();
    archive.register_type<PriorFactor<Vector>>();
    archive << *factor;
  }
  CMCostFactor restored;
  {
    boost::archive::text_iarchive archive(stream);
    archive.register_type<noiseModel::Isotropic>();
    archive.register_type<PriorFactor<Vector>>();
    archive >> restored;
  }
  EXPECT(restored.keys() == factor->keys());
  EXPECT(assert_equal(factor->unwhitenedError(points),
                      restored.unwhitenedError(points)));
  EXPECT_DOUBLES_EQUAL(factor->error(points), restored.error(points), 1e-9);
  EXPECT_CORRECT_FACTOR_JACOBIANS(restored, points, 1e-7, 1e-5);

  // A second round trip exercises captured ambient values and a free argument.
  Values fixedPrototype, fixedAmbient;
  fixedPrototype.insert(37, Point3(1, 2, 3));
  fixedAmbient.insert(12, Point3(1, 2, 3));
  NonlinearEqualityConstraints fixedEqualities;
  fixedEqualities.emplace_shared<ZeroCostConstraint>(
      std::make_shared<PriorFactor<Point3>>(
          37, Point3(1, 2, 3), noiseModel::Unit::Create(3)));
  const auto fixedType = std::make_shared<EConstraintManifold>(
      fixedEqualities, fixedPrototype);
  Values fixedPoints;
  fixedPoints.insert(100, fixedType->point({{37, 12}}, fixedAmbient));
  const auto original = std::make_shared<BetweenFactor<Point3>>(
      200, 12, Point3(-1, 2, 3), noiseModel::Isotropic::Sigma(3, 2));
  const CMCostFactor mixed(original, {{12, {100, 37}}}, fixedPoints,
                           CMCostFactor::PointType::Equality);
  std::stringstream mixedStream;
  {
    boost::archive::text_oarchive archive(mixedStream);
    archive.register_type<noiseModel::Isotropic>();
    archive.register_type<BetweenFactor<Point3>>();
    archive.register_type<GenericValue<Point3>>();
    archive << mixed;
  }
  CMCostFactor mixedRestored;
  {
    boost::archive::text_iarchive archive(mixedStream);
    archive.register_type<noiseModel::Isotropic>();
    archive.register_type<BetweenFactor<Point3>>();
    archive.register_type<GenericValue<Point3>>();
    archive >> mixedRestored;
  }
  Values freeValues;
  freeValues.insert(200, Point3(2, 4, 6));
  EXPECT(mixedRestored.keys() == KeyVector({200}));
  EXPECT(assert_equal(mixed.unwhitenedError(freeValues),
                      mixedRestored.unwhitenedError(freeValues)));
  EXPECT_DOUBLES_EQUAL(mixed.error(freeValues),
                       mixedRestored.error(freeValues), 1e-9);
  EXPECT_CORRECT_FACTOR_JACOBIANS(mixedRestored, freeValues, 1e-7, 1e-5);
}
#endif

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
