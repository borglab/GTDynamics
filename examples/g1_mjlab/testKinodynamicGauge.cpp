/* GTDynamics Copyright 2026, Georgia Tech Research Corporation. See LICENSE. */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/Dynamics.h>
#include <gtdynamics/dynamics/WorldVelocityFactor.h>
#include <gtdynamics/factors/CollocationFactors.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/AttitudeFactor.h>

#include "KinodynamicGauge.h"
using namespace gtsam;
using namespace gtdynamics;

TEST(KinodynamicGauge, JacobiansAndFreeTilt) {
  const Pose3 anchor(Rot3::RzRyRx(.1, -.2, 3.13), Point3(1, 2, 3));
  const Pose3 changed(Rot3::RzRyRx(.4, .3, -3.12), Point3(2, 4, 3));
  auto graph = g1_example::gaugeFactors(1, anchor);
  auto yaw = std::dynamic_pointer_cast<g1_example::YawGaugeFactor>(graph[1]);
  Matrix H;
  auto error = yaw->evaluateError(changed, &H);
  std::function<Vector(const Pose3&)> f = [&](const Pose3& p) {
    return yaw->evaluateError(p);
  };
  EXPECT(assert_equal(numericalDerivative11(f, changed), H, 1e-8));
  EXPECT(std::abs(error(0)) < .04);  // wrapped across +/- pi
  auto translation =
      std::dynamic_pointer_cast<PoseTranslationPrior<Pose3>>(graph[0]);
  translation->evaluateError(changed, &H);
  f = [&](const Pose3& p) { return translation->evaluateError(p); };
  EXPECT(assert_equal(numericalDerivative11(f, changed), H, 1e-8));
  Values v;
  v.insert(Key(1), Pose3(Rot3::RzRyRx(.4, .3, 3.13), anchor.translation()));
  EXPECT_DOUBLES_EQUAL(0., graph.error(v), 1e-20);
}

TEST(KinodynamicGauge, PhysicalCostsAreTranslationYawInvariant) {
  auto root = std::make_shared<Link>(0, "root", 8., Matrix3::Identity(),
                                     Pose3(), Pose3());
  Robot robot(LinkMap{{"root", root}}, {});
  DynamicsParameters dp(1.);
  dp.gravity = Vector3(0, 0, -9.81);
  NonlinearFactorGraph graph;
  Values v, shifted;
  const Pose3 change(Rot3::Rz(.7), Point3(3, -2, 5));
  for (size_t k = 0; k < 2; ++k) {
    const Pose3 p(Rot3::RzRyRx(.1, .2, .3 + .1 * k), Point3(.1 * k, 0, .6));
    const Vector6 velocity{.1, .2, .3, .4, .5, .6};
    v.insert(PoseKey(0, k), p);
    v.insert(TwistKey(0, k), velocity);
    v.insert(TwistAccelKey(0, k), Vector6{.2, .3, .1, .4, .6, .5});
    v.insert(ContactWrenchKey(0, 0, k), Vector6{1, 2, 3, 4, 5, 80});
    v.insert(LinearVelocityKey(0, k), Vector3(.2, .3, .4));
    graph.add(Dynamics(dp).wrenchBalanceFactors(
        Slice(k), robot, {{0, {ContactWrenchKey(0, 0, k)}}}));
    graph.emplace_shared<WorldVelocityFactor>(PoseKey(0, k), TwistKey(0, k),
                                              LinearVelocityKey(0, k),
                                              noiseModel::Unit::Create(3));
    graph.emplace_shared<AttitudeFactor<Pose3>>(PoseKey(0, k), Unit3(0, 0, 1),
                                                noiseModel::Unit::Create(2));
  }
  graph.emplace_shared<FixTimeTrapezoidalPoseCollocationFactor>(
      PoseKey(0, 0), PoseKey(0, 1), TwistKey(0, 0), TwistKey(0, 1), .02,
      noiseModel::Unit::Create(6));
  shifted = v;
  for (size_t k = 0; k < 2; ++k) {
    shifted.update(PoseKey(0, k), change * v.at<Pose3>(PoseKey(0, k)));
    shifted.update(
        LinearVelocityKey(0, k),
        Vector3(change.rotation() * v.at<Vector3>(LinearVelocityKey(0, k))));
  }
  for (const auto& factor : graph)
    EXPECT_DOUBLES_EQUAL(factor->error(v), factor->error(shifted), 1e-10);
}
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
