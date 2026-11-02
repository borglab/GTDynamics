#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/utils/ContactMotion.h>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace gtdynamics;
using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

/* ************************************************************************* */
namespace contact_motion {
const std::vector<double> times{0, .05, .1, .15, .2};
const std::vector<Point3> sole{{0, 0, 0}, {.2, 0, 0}, {0, .1, 0}};

// Static observations label only complete windows, independently of world
// frame.
TEST(ContactMotion, StaticAndWorldFrameInvariance) {
  const Pose3 transform(Rot3::RzRyRx(.3, -.2, .8), Point3(4, -2, 3));
  const auto result =
      contactMotionLabels(times, std::vector<Pose3>(5, transform), sole);
  EXPECT_LONGS_EQUAL(-1, result.front().label);
  EXPECT_LONGS_EQUAL(1, result[2].label);
  DOUBLES_EQUAL(1, result[2].score, 1e-12);
  EXPECT_LONGS_EQUAL(-1, result.back().label);
}

// Linear motion uses actual nonuniform timestamps and can produce uncertain
// labels.
TEST(ContactMotion, IrregularTimesAndThresholdMargins) {
  const std::vector<double> irregular{0, .03, .1, .14, .2};
  for (double velocity : {.05, .175, .4}) {
    std::vector<Pose3> poses;
    for (double time : irregular)
      poses.emplace_back(Rot3(), Point3(velocity * time, 0, 0));
    const auto sample = contactMotionLabels(irregular, poses, sole)[2];
    DOUBLES_EQUAL(velocity, sample.maxSpeed, 1e-12);
    EXPECT_LONGS_EQUAL(velocity < .1    ? 1
                       : velocity > .25 ? 0
                                        : -1,
                       sample.label);
    if (velocity == .175) DOUBLES_EQUAL(.5, sample.score, 1e-12);
  }
}

// Rotation about a stationary corner must not label the entire sole stationary.
TEST(ContactMotion, PivotAndConstantWorldTransform) {
  std::vector<Pose3> poses, transformed;
  const Pose3 world(Rot3::RzRyRx(.2, -.5, .6), Point3(5, 4, -2));
  for (double time : times) {
    poses.emplace_back(Rot3::Rz(2 * time), Point3::Zero());
    transformed.push_back(world * poses.back());
  }
  const auto sample = contactMotionLabels(times, poses, sole)[2];
  const auto other = contactMotionLabels(times, transformed, sole)[2];
  DOUBLES_EQUAL(0, sample.pointSpeeds[0], 1e-12);
  EXPECT(sample.maxSpeed > .3);
  EXPECT_LONGS_EQUAL(0, sample.label);
  DOUBLES_EQUAL(sample.maxSpeed, other.maxSpeed, 1e-12);
  DOUBLES_EQUAL(sample.maxExcursion, other.maxExcursion, 1e-12);
}

// A foot that moves and returns has zero endpoint speed but nonzero excursion.
TEST(ContactMotion, ReturningMotionIsNotStationary) {
  std::vector<Pose3> poses(5);
  poses[2] = Pose3(Rot3(), Point3(.05, 0, 0));
  const auto sample = contactMotionLabels(times, poses, sole)[2];
  DOUBLES_EQUAL(0, sample.maxSpeed, 1e-12);
  DOUBLES_EQUAL(.05, sample.maxExcursion, 1e-12);
  EXPECT_LONGS_EQUAL(0, sample.label);
}

// Invalid timestamps, geometry, input sizes, and threshold ordering are
// rejected.
TEST(ContactMotion, InvalidInputs) {
  const std::vector<Pose3> poses(5);
  CHECK_EXCEPTION(contactMotionLabels({0, 0, .1, .15, .2}, poses, sole),
                  std::invalid_argument);
  CHECK_EXCEPTION(contactMotionLabels(times, {}, sole), std::invalid_argument);
  CHECK_EXCEPTION(contactMotionLabels(times, poses, {}), std::invalid_argument);
  ContactMotionParams params;
  params.movingSpeed = params.stationarySpeed;
  CHECK_EXCEPTION(contactMotionLabels(times, poses, sole, params),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      contactMotionLabels(times, poses,
                          {{std::numeric_limits<double>::quiet_NaN(), 0, 0}}),
      std::invalid_argument);
}
// A rotating sole with a stationary origin is moving in the pose model.
TEST(ContactPoseMotion, PurePivotAndFrameInvariance) {
  std::vector<Pose3> poses;
  for (double time : times)
    poses.emplace_back(Rot3::Rz(2 * time), Point3::Zero());
  const auto sample = contactPoseMotionLabels(times, poses)[2];
  DOUBLES_EQUAL(0, sample.maxSpeed, 1e-12);
  DOUBLES_EQUAL(2, sample.angularSpeed, 1e-12);
  EXPECT_LONGS_EQUAL(0, sample.label);
  const Pose3 frame(Rot3::Ry(.4), Point3(2, -3, 1));
  for (auto& pose : poses) pose = frame * pose;
  const auto transformed = contactPoseMotionLabels(times, poses)[2];
  DOUBLES_EQUAL(sample.angularSpeed, transformed.angularSpeed, 1e-10);
  DOUBLES_EQUAL(sample.angularExcursion, transformed.angularExcursion, 1e-10);
  EXPECT_LONGS_EQUAL(sample.label, transformed.label);
}

// Angular excursion catches rocking that returns to its initial orientation.
TEST(ContactPoseMotion, ReturningRotationAndValidation) {
  std::vector<Pose3> poses(5);
  poses[2] = Pose3(Rot3::Ry(.2), Point3::Zero());
  const auto sample = contactPoseMotionLabels(times, poses)[2];
  DOUBLES_EQUAL(0, sample.angularSpeed, 1e-12);
  DOUBLES_EQUAL(.2, sample.angularExcursion, 1e-12);
  EXPECT_LONGS_EQUAL(0, sample.label);
  ContactPoseMotionParams params;
  params.movingAngularSpeed = params.stationaryAngularSpeed;
  CHECK_EXCEPTION(contactPoseMotionLabels(times, poses, params),
                  std::invalid_argument);
}
}  // namespace contact_motion
/* ************************************************************************* */
namespace walking_contact {
ContactMotionSample sample(double speed, double angularSpeed = 0) {
  const std::vector<double> times{0, .05, .1, .15, .2};
  std::vector<Pose3> poses;
  for (double time : times)
    poses.emplace_back(Rot3::Ry(angularSpeed * time),
                       Point3(speed * time, 0, 0));
  return contactPoseMotionLabels(times, poses)[2];
}

// Swing translation promotes uncertain/rocking support, not raw stationarity.
TEST(WalkingContact, SupportAndRocking) {
  const auto swing = sample(.5), uncertain = sample(.15),
             rocking = sample(0, 2);
  EXPECT_LONGS_EQUAL(2, walkingContactLabels({uncertain}, {swing})[0]);
  EXPECT_LONGS_EQUAL(2, walkingContactLabels({rocking}, {swing})[0]);
  EXPECT_LONGS_EQUAL(0, walkingContactLabels({swing}, {rocking})[0]);
  EXPECT_LONGS_EQUAL(1, walkingContactLabels({sample(0)}, {swing})[0]);
  EXPECT_LONGS_EQUAL(0, rocking.label);
  DOUBLES_EQUAL(0, rocking.score, 1e-12);
}

// Neither pivots alone, simultaneous translation nor incomplete windows infer
// support; an excursion can reveal swing even when endpoint speed is zero.
TEST(WalkingContact, AmbiguousMotionAndBoundaries) {
  const auto uncertain = sample(.15), swing = sample(.5),
             rocking = sample(0, 2);
  EXPECT_LONGS_EQUAL(-1, walkingContactLabels({uncertain}, {rocking})[0]);
  EXPECT_LONGS_EQUAL(-1, walkingContactLabels({uncertain}, {uncertain})[0]);
  EXPECT_LONGS_EQUAL(0, walkingContactLabels({swing}, {swing})[0]);
  auto incomplete = swing;
  incomplete.completeWindow = false;
  incomplete.label = -1;
  EXPECT_LONGS_EQUAL(-1, walkingContactLabels({uncertain}, {incomplete})[0]);
  EXPECT_LONGS_EQUAL(-1, walkingContactLabels({incomplete}, {swing})[0]);
  auto returning = swing;
  returning.maxSpeed = 0;
  returning.maxExcursion = .025;
  EXPECT_LONGS_EQUAL(2, walkingContactLabels({uncertain}, {returning})[0]);
  EXPECT_LONGS_EQUAL(0, walkingContactLabels({returning}, {swing})[0]);
  ContactPoseMotionParams params;
  params.movingSpeed = 1;
  params.movingExcursion = .1;
  EXPECT_LONGS_EQUAL(-1, walkingContactLabels({uncertain}, {swing}, params)[0]);
}

// Reject mismatched sequences, recursive inferred labels, and invalid metrics.
TEST(WalkingContact, InvalidInputs) {
  const auto foot = sample(.15);
  CHECK_EXCEPTION(walkingContactLabels({foot}, {}), std::invalid_argument);
  CHECK_EXCEPTION(walkingContactLabels({}, {}), std::invalid_argument);
  auto invalid = foot;
  invalid.label = 2;
  CHECK_EXCEPTION(walkingContactLabels({invalid}, {foot}),
                  std::invalid_argument);
  invalid = foot;
  invalid.maxSpeed = std::numeric_limits<double>::quiet_NaN();
  CHECK_EXCEPTION(walkingContactLabels({foot}, {invalid}),
                  std::invalid_argument);
  ContactPoseMotionParams params;
  params.movingExcursion = -1;
  CHECK_EXCEPTION(walkingContactLabels({foot}, {foot}, params),
                  std::invalid_argument);
}
}  // namespace walking_contact
/* ************************************************************************* */
namespace contact_support {
// Each foot factor conditions on both incoming displacements; all four
// entries and all sixteen transition entries have the documented key order.
TEST(ContactSupport, PairLikelihoodsAndCoupling) {
  ContactSupportParams params;
  // Asymmetric rows detect swapped previous/next keys or left/right order.
  for (size_t row = 0; row < 4; ++row) {
    for (size_t col = 0; col < 4; ++col)
      params.transition(row, col) = 1 + 4 * row + col;
    params.transition.row(row) /= params.transition.row(row).sum();
  }
  const std::vector<double> left{.002, .018}, right{.016, .001};
  const auto graph = contactSupportGraph(left, right, params);
  EXPECT_LONGS_EQUAL(6, graph.size());
  for (size_t previous = 0; previous < 4; ++previous)
    for (size_t next = 0; next < 4; ++next) {
      const gtsam::DiscreteValues values{
          {0, previous / 2}, {1, previous % 2}, {2, next / 2}, {3, next % 2}};
      DOUBLES_EQUAL(.25, (*graph.at(0))(values), 1e-12);
      DOUBLES_EQUAL(previous == 0 ? params.noSupportWeight : 1,
                    (*graph.at(1))(values), 1e-12);
      DOUBLES_EQUAL(params.transition(previous, next), (*graph.at(3))(values),
                    1e-12);
    }
  for (size_t foot = 0; foot < 2; ++foot) {
    const auto& distances = foot == 0 ? left : right;
    std::vector<double> expected;
    for (size_t a = 0; a < 2; ++a)
      for (size_t b = 0; b < 2; ++b) {
        const double sa = a ? params.supportSigma : params.swingSigma;
        const double sb = b ? params.supportSigma : params.swingSigma;
        expected.push_back(
            std::exp(-.5 * (distances[0] * distances[0] / (sa * sa) +
                            distances[1] * distances[1] / (sb * sb))) /
            (sa * sb));
      }
    const double maximum = *std::max_element(expected.begin(), expected.end());
    for (size_t a = 0; a < 2; ++a)
      for (size_t b = 0; b < 2; ++b)
        DOUBLES_EQUAL(expected[2 * a + b] / maximum,
                      (*graph.at(4 + foot))(
                          gtsam::DiscreteValues{{foot, a}, {2 + foot, b}}),
                      1e-12);
  }
}

// Overlapping pair factors contribute each incoming observation exactly once,
// including the first and last slices (verified by a likelihood ratio).
TEST(ContactSupport, OverlapWeights) {
  const std::vector<double> distances{.001, .004, .007};
  const auto graph = contactSupportGraph(distances, distances);
  gtsam::DiscreteValues stance, swing;
  for (size_t k = 0; k < 3; ++k) {
    stance[2 * k] = 1;
    swing[2 * k] = 0;
  }
  double actual = 1, expected = 1;
  for (size_t factor : {size_t(5), size_t(8)})
    actual *= (*graph.at(factor))(stance) / (*graph.at(factor))(swing);
  for (double d : distances)
    expected *= (.020 / .003) *
                std::exp(-.5 * d * d * (1 / (.003 * .003) - 1 / (.020 * .020)));
  DOUBLES_EQUAL(expected, actual, 1e-10);
}

// Exact max-product matches exhaustive MAP over six binary variables.
TEST(ContactSupport, MaxProductMatchesEnumeration) {
  const auto graph = contactSupportGraph({.001, .015, .02}, {.02, .004, .001});
  const auto result = graph.optimize(gtsam::Ordering::NATURAL);
  double best = 0;
  for (size_t mask = 0; mask < 64; ++mask) {
    gtsam::DiscreteValues values;
    for (size_t key = 0; key < 6; ++key) values[key] = (mask >> key) & 1;
    best = std::max(best, graph(values));
  }
  DOUBLES_EQUAL(best, graph(result), 1e-12);
}

// Long chains recover L/R/double support and permit both swing under strong
// evidence. Large displacements on both feet no longer reward alternation.
TEST(ContactSupport, LongChainAndNoSwitchingEscape) {
  for (size_t joint = 0; joint < 4; ++joint) {
    const std::vector<double> left(1000, joint / 2 ? .001 : .02);
    const std::vector<double> right(1000, joint % 2 ? .001 : .02);
    const auto graph = contactSupportGraph(left, right);
    const auto result = graph.optimize(gtsam::Ordering::NATURAL);
    EXPECT_LONGS_EQUAL(5994, graph.size());
    for (size_t k = 0; k < 1000; ++k) {
      EXPECT_LONGS_EQUAL(joint / 2, result.at(2 * k));
      EXPECT_LONGS_EQUAL(joint % 2, result.at(2 * k + 1));
    }
  }
}

// All eight entries penalize only one-slice excursions, separately per foot.
TEST(ContactSupport, DurationTable) {
  ContactSupportParams params;
  params.singleSliceWeight = .2;
  const auto graph =
      contactSupportGraph({.001, .002, .003}, {.003, .002, .001}, params);
  EXPECT_LONGS_EQUAL(12, graph.size());
  for (size_t foot = 0; foot < 2; ++foot)
    for (size_t mask = 0; mask < 8; ++mask) {
      const size_t a = mask / 4, b = (mask / 2) % 2, c = mask % 2;
      const gtsam::DiscreteValues values{
          {foot, a}, {2 + foot, b}, {4 + foot, c}};
      DOUBLES_EQUAL(a == c && a != b ? .2 : 1, (*graph.at(10 + foot))(values),
                    1e-12);
    }
}

// A weak isolated stance OR swing observation is suppressed by the prior;
// setting the weight to 1 restores the unpenalized MAP solution.
TEST(ContactSupport, DurationSuppressesIsolatedClassification) {
  const double neutral = std::sqrt(2 * std::log(.020 / .003) /
                                   (1 / (.003 * .003) - 1 / (.020 * .020)));
  for (size_t surrounding = 0; surrounding < 2; ++surrounding)
    for (double weight : {1.0, .1}) {
      ContactSupportParams params;
      params.transition.setConstant(.25);
      params.noSupportWeight = 1;
      params.singleSliceWeight = weight;
      auto graph = contactSupportGraph(std::vector<double>(3, neutral),
                                       std::vector<double>(3, neutral), params);
      for (size_t k = 0; k < 3; ++k) {
        std::vector<double> evidence(2, 1);
        evidence[k == 1 ? 1 - surrounding : surrounding] = k == 1 ? 4 : 100;
        graph.add(gtsam::DiscreteKeys{{2 * k, 2}}, evidence);
      }
      const auto result = graph.optimize(gtsam::Ordering::NATURAL);
      EXPECT_LONGS_EQUAL(surrounding, result.at(0));
      EXPECT_LONGS_EQUAL(weight == 1 ? 1 - surrounding : surrounding,
                         result.at(2));
      EXPECT_LONGS_EQUAL(surrounding, result.at(4));
    }
}

// Reject invalid distances, scales, coupling weights, or transition matrices.
TEST(ContactSupport, InvalidInputs) {
  CHECK_EXCEPTION(contactSupportGraph({}, {}), std::invalid_argument);
  CHECK_EXCEPTION(contactSupportGraph({0}, {0}), std::invalid_argument);
  CHECK_EXCEPTION(contactSupportGraph({0, 0}, {0}), std::invalid_argument);
  CHECK_EXCEPTION(contactSupportGraph({0, -1}, {0, 0}), std::invalid_argument);
  CHECK_EXCEPTION(
      contactSupportGraph({0, 0}, {0, std::numeric_limits<double>::infinity()}),
      std::invalid_argument);
  ContactSupportParams params;
  params.swingSigma = params.supportSigma;
  CHECK_EXCEPTION(contactSupportGraph({0, 0}, {0, 0}, params),
                  std::invalid_argument);
  params = ContactSupportParams();
  params.transition(0, 0) = 1;
  CHECK_EXCEPTION(contactSupportGraph({0, 0}, {0, 0}, params),
                  std::invalid_argument);
  params = ContactSupportParams();
  params.noSupportWeight = 0;
  CHECK_EXCEPTION(contactSupportGraph({0, 0}, {0, 0}, params),
                  std::invalid_argument);
  params.noSupportWeight = std::numeric_limits<double>::quiet_NaN();
  CHECK_EXCEPTION(contactSupportGraph({0, 0}, {0, 0}, params),
                  std::invalid_argument);
  for (double weight :
       {0.0, -1.0, 1.01, std::numeric_limits<double>::quiet_NaN()}) {
    params = ContactSupportParams();
    params.singleSliceWeight = weight;
    CHECK_EXCEPTION(contactSupportGraph({0, 0}, {0, 0}, params),
                    std::invalid_argument);
  }
}
}  // namespace contact_support
/* ************************************************************************* */
namespace contact_velocity {
// Each unary uses its own binary foot key; zero speed is neutral.
TEST(ContactVelocity, DirectionAndOtherFootIndependence) {
  const auto graph = contactVelocityFactors({-.3, 0, .3}, {.3, 0, -.3});
  EXPECT_LONGS_EQUAL(6, graph.size());
  const double contact = 1 / (1 + std::exp(-3.0));
  for (size_t k = 0; k < 3; ++k) {
    const double p = k == 0 ? contact : k == 1 ? .5 : 1 - contact;
    for (size_t other = 0; other < 2; ++other) {
      DOUBLES_EQUAL(p,
                    (*graph.at(2 * k))(
                        gtsam::DiscreteValues{{2 * k, 1}, {2 * k + 1, other}}),
                    1e-12);
      DOUBLES_EQUAL(1 - p,
                    (*graph.at(2 * k + 1))(
                        gtsam::DiscreteValues{{2 * k, other}, {2 * k + 1, 1}}),
                    1e-12);
    }
  }
}

// Bounded velocities stay finite, neutral velocities preserve MAP, and
// opposite signed velocities favor left stance and right swing.
TEST(ContactVelocity, BoundsAndJointMap) {
  const auto extreme = contactVelocityFactors({-1e300}, {1e300});
  DOUBLES_EQUAL(.98, (*extreme.at(0))(gtsam::DiscreteValues{{0, 1}}), 1e-12);
  DOUBLES_EQUAL(.02, (*extreme.at(1))(gtsam::DiscreteValues{{1, 1}}), 1e-12);
  auto graph = contactSupportGraph(std::vector<double>(8, .007),
                                   std::vector<double>(8, .007));
  const auto baseline = graph.optimize(gtsam::Ordering::NATURAL);
  graph.push_back(contactVelocityFactors(std::vector<double>(8, 0),
                                         std::vector<double>(8, 0)));
  const auto neutral = graph.optimize(gtsam::Ordering::NATURAL);
  for (size_t key = 0; key < 16; ++key)
    EXPECT_LONGS_EQUAL(baseline.at(key), neutral.at(key));
  graph.push_back(contactVelocityFactors(std::vector<double>(8, -.3),
                                         std::vector<double>(8, .3)));
  const auto result = graph.optimize(gtsam::Ordering::NATURAL);
  for (size_t k = 0; k < 8; ++k) {
    EXPECT_LONGS_EQUAL(1, result.at(2 * k));
    EXPECT_LONGS_EQUAL(0, result.at(2 * k + 1));
  }
}

// Invalid arrays and sigmoid parameters fail before constructing factors.
TEST(ContactVelocity, InvalidInputs) {
  CHECK_EXCEPTION(contactVelocityFactors({}, {}), std::invalid_argument);
  CHECK_EXCEPTION(contactVelocityFactors({0}, {}), std::invalid_argument);
  CHECK_EXCEPTION(
      contactVelocityFactors({0}, {std::numeric_limits<double>::quiet_NaN()}),
      std::invalid_argument);
  CHECK_EXCEPTION(contactVelocityFactors({0}, {0}, 0), std::invalid_argument);
  CHECK_EXCEPTION(contactVelocityFactors({0}, {0}, .1, .5),
                  std::invalid_argument);
  CHECK_EXCEPTION(contactVelocityFactors({0}, {0}, .1, 0),
                  std::invalid_argument);
}
}  // namespace contact_velocity
/* ************************************************************************* */
namespace walking_constraints {
// Enumerate every two-slice assignment: masks have exact zeros and ones,
// excluding flight and direct exchanges but retaining double-stance bridges.
TEST(WalkingConstraints, ExactMasksAndBoundaries) {
  CHECK_EXCEPTION(contactWalkingConstraints(0), std::invalid_argument);
  EXPECT_LONGS_EQUAL(1, contactWalkingConstraints(1).size());
  const auto graph = contactWalkingConstraints(2);
  EXPECT_LONGS_EQUAL(3, graph.size());
  for (size_t a = 0; a < 4; ++a)
    for (size_t b = 0; b < 4; ++b) {
      const gtsam::DiscreteValues values{
          {0, a / 2}, {1, a % 2}, {2, b / 2}, {3, b % 2}};
      const bool exchange = (a == 1 && b == 2) || (a == 2 && b == 1);
      DOUBLES_EQUAL(a ? 1 : 0, (*graph.at(0))(values), 0);
      DOUBLES_EQUAL(b ? 1 : 0, (*graph.at(1))(values), 0);
      DOUBLES_EQUAL(exchange ? 0 : 1, (*graph.at(2))(values), 0);
      DOUBLES_EQUAL(a && b && !exchange ? 1 : 0, graph(values), 0);
    }
}

// Adding masks preserves existing factors and yields the best permitted MAP
// even when the measurements strongly favor flight or direct exchanges.
TEST(WalkingConstraints, AdditiveExactMap) {
  auto graph = contactSupportGraph({.001, .02, .02}, {.02, .02, .001});
  const size_t originalCount = graph.size();
  const auto originalFactor = graph.at(0);
  graph.push_back(contactWalkingConstraints(3));
  EXPECT_LONGS_EQUAL(originalCount + 5, graph.size());
  CHECK(graph.at(0) == originalFactor);
  const auto result = graph.optimize(gtsam::Ordering::NATURAL);
  double best = 0;
  for (size_t mask = 0; mask < 64; ++mask) {
    gtsam::DiscreteValues values;
    for (size_t key = 0; key < 6; ++key) values[key] = (mask >> key) & 1;
    best = std::max(best, graph(values));
  }
  CHECK(best > 0);
  DOUBLES_EQUAL(best, graph(result), best * 1e-10);
  DOUBLES_EQUAL(1, contactWalkingConstraints(3)(result), 0);
  const gtsam::DiscreteValues bridge{{0, 1}, {1, 0}, {2, 1},
                                     {3, 1}, {4, 0}, {5, 1}};
  DOUBLES_EQUAL(1, contactWalkingConstraints(3)(bridge), 0);
}
}  // namespace walking_constraints
/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
