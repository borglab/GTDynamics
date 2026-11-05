/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
/**
 * @file testKinodynamicInitializer.cpp
 * @brief Validate immutable layered initialization and linear dynamics
 * seeding.
 * @author GTDynamics contributors
 */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/dynamics/KinodynamicInitializer.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/utils/PoseContactConstraintSpec.h>
#include <gtdynamics/utils/Trajectory.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/TestableAssertions.h>
using namespace gtdynamics;
using namespace gtsam;
/* ************************************************************************* */
namespace layered_initialization {
struct Fixture {
  LinkSharedPtr root = std::make_shared<Link>(0, "base", 8, Matrix3::Identity(),
                                              Pose3(), Pose3());
  LinkSharedPtr left = std::make_shared<Link>(1, "left", 1, Matrix3::Identity(),
                                              Pose3(), Pose3());
  LinkSharedPtr right = std::make_shared<Link>(
      2, "right", 1, Matrix3::Identity(), Pose3(), Pose3());
  JointSharedPtr lj = std::make_shared<RevoluteJoint>(0, "lj", Pose3(), root,
                                                      left, Vector3::UnitZ());
  JointSharedPtr rj = std::make_shared<RevoluteJoint>(1, "rj", Pose3(), root,
                                                      right, Vector3::UnitZ());
  Robot robot{LinkMap{{"base", root}, {"left", left}, {"right", right}},
              JointMap{{"lj", lj}, {"rj", rj}}};
  PoseOnLink lc{left, Pose3(Rot3(), Point3(0, .1, -.5))};
  PoseOnLink rc{right, Pose3(Rot3(), Point3(0, -.1, -.5))};
  Trajectory trajectory{
      WalkCycle({Phase(0, 5,
                       std::make_shared<PoseContactConstraintSpec>(
                           std::vector<PoseOnLink>{lc, rc}))}),
      1};
  Pose3 anchor{Rot3(), Point3(0, 0, .5)};
  std::vector<Slice> slices;
  std::vector<double> times;
  Values measured;
  Fixture() {
    root->addJoint(lj);
    root->addJoint(rj);
    left->addJoint(lj);
    right->addJoint(rj);
    for (size_t k = 0; k < 6; ++k) {
      slices.emplace_back(k);
      times.push_back(78 + .02 * k);
      InsertJointAngle(&measured, 0, k, .01 * k);
      InsertJointAngle(&measured, 1, k, .01 * k);
    }
  }
  KinodynamicInitializationOptions options() const {
    KinodynamicInitializationOptions settings;
    settings.kinematics.contact.groundHeight = 0;
    settings.kinematics.contact.recordHistory = false;
    settings.dynamics.baseAngularAccelerationSigmaDegrees = .01;
    settings.dynamics.baseLinearAccelerationSigma = .001;
    settings.dynamics.jointAccelerationSigmaDegrees = .01;
    return settings;
  }
  std::vector<fixed_dynamics::Contact> contacts() const {
    return {{lc, .1, .04, .8}, {rc, .1, .04, .8}};
  }
};

// Pipeline retains nonzero GP velocities, leaves kinematics fixed through the
// linear seed, and performs no iterative dynamics optimization.
TEST(KinodynamicInitializer, PipelineStopsAtLinearSeed) {
  Fixture f;
  auto settings = f.options();
  const KinodynamicInitializer initializer(f.robot, settings);
  settings.kinematics.contact.groundHeight = 200;
  const auto before = f.measured;
  const auto result = initializer.run(
      "base", f.anchor, f.measured, f.trajectory, f.slices, f.times, .02,
      f.contacts(), Vector(), {Slice(0), Slice(2), Slice(5)});
  EXPECT(assert_equal(before, f.measured, 1e-15));
  EXPECT(result.dynamics.feasible);
  EXPECT(result.dynamics.equalityL2 < 1e-8);
  EXPECT_LONGS_EQUAL(0, result.dynamics.iterations);
  EXPECT_LONGS_EQUAL(0, result.dynamics.numALSolves);
  EXPECT_LONGS_EQUAL(0, result.dynamics.numFeasibleProjections);
  EXPECT_LONGS_EQUAL(0, result.dynamics.numFeasibleTrials);
  EXPECT_DOUBLES_EQUAL(0, result.dynamics.manifoldSeconds, 0);
  EXPECT_DOUBLES_EQUAL(0, result.dynamics.optimizationSeconds, 0);
  EXPECT(!result.dynamics.stationary);
  EXPECT(std::isinf(result.dynamics.projectedGradientNorm));
  EXPECT_LONGS_EQUAL(3, result.kinematics.contact.selectedSlices.size());
  EXPECT(Twist(result.values, 0, 2).head<3>().norm() > .2);
  EXPECT(result.values.at<Vector>(ContactWrenchKey(1, 0, 2))(5) > 1);
  for (const auto key : result.kinematics.values.keys())
    EXPECT(
        result.values.at(key).equals_(result.kinematics.values.at(key), 1e-12));
  Values accelerationPriors;
  for (size_t i = 0; i < f.slices.size(); ++i)
    accelerationPriors.insert(gpAccelerationPriors(
        *result.kinematics.interpolation, 0, f.times[i], f.slices[i].k, .02));
  const auto direct = fixedKinematicsDynamicsSeed(
      f.robot, f.trajectory, f.slices, f.times, result.kinematics.values,
      accelerationPriors, f.contacts(), Vector(), f.options().dynamics);
  EXPECT(assert_equal(direct.values, result.dynamics.values, 1e-7));
  EXPECT_DOUBLES_EQUAL(direct.objectiveFinal, result.dynamics.objectiveFinal,
                       1e-7);
  EXPECT(result.totalSeconds >= result.dynamicsSeconds);
}

// Kinematic strategies have no dynamics keys and can run repeatedly without
// retaining mutable solver state or modifying encoder measurements.
TEST(KinematicInitializer, ImmutableRepeatAndLayerBoundary) {
  Fixture f;
  auto settings = f.options().kinematics;
  const ContactInterpolationInitializer initializer(f.robot, settings);
  const auto a =
      initializer.run("base", f.anchor, f.measured, f.trajectory, f.slices,
                      f.times, .02, {Slice(0), Slice(2), Slice(5)});
  const auto b =
      initializer.run("base", f.anchor, f.measured, f.trajectory, f.slices,
                      f.times, .02, {Slice(0), Slice(2), Slice(5)});
  EXPECT(assert_equal(a.values, b.values, 1e-12));
  // The same tuning accepts another recording's timestamps, gauge and slices.
  auto laterTimes = f.times;
  for (auto& time : laterTimes) time += 10;
  const Pose3 shiftedAnchor(f.anchor.rotation(),
                            f.anchor.translation() + Point3(2, 0, 0));
  const auto c =
      initializer.run("base", shiftedAnchor, f.measured, f.trajectory, f.slices,
                      laterTimes, .02, {Slice(1), Slice(4)});
  EXPECT_LONGS_EQUAL(1, c.contact.selectedSlices.front().k);
  EXPECT_LONGS_EQUAL(4, c.contact.selectedSlices.back().k);
  EXPECT_DOUBLES_EQUAL(laterTimes.front(), c.interpolation->knotTimes().front(),
                       1e-12);
  EXPECT(assert_equal(shiftedAnchor, c.contact.poses.front(), 1e-12));
  EXPECT(assert_equal(a.values, b.values, 1e-12));
  for (const auto key : a.values.keys()) {
    const auto label = DynamicsSymbol(key).label();
    EXPECT(label == "p" || label == "V" || label == "q" || label == "v");
  }
  const auto fk = ForwardKinematicsInitializer(f.robot).run(
      "base", f.anchor, f.measured, f.slices);
  EXPECT(Twist(fk, 0, 2).norm() == 0);
  EXPECT(!fk.exists(JointAccelKey(0, 2)));
  const auto old = a.interpolation->interpolate(f.times[2]);
  f.measured.update(JointAngleKey(0, 2), 10.0);
  EXPECT_DOUBLES_EQUAL(old.jointAngles(0),
                       a.interpolation->interpolate(f.times[2]).jointAngles(0),
                       0);
}

// Buffered contact knots retain actual timestamps; endpoint coverage holds
// the nearest fitted pose rather than extrapolating outside the sparse span.
TEST(KinematicInitializer, BufferedEndpointKnots) {
  Fixture f;
  auto settings = f.options().kinematics;
  const auto result = ContactInterpolationInitializer(f.robot, settings)
                          .run("base", f.anchor, f.measured, f.trajectory,
                               f.slices, f.times, .02, {Slice(1), Slice(4)});
  const auto knots = result.interpolation->knotTimes();
  EXPECT_LONGS_EQUAL(4, knots.size());
  EXPECT_DOUBLES_EQUAL(f.times.front(), knots.front(), 1e-12);
  EXPECT_DOUBLES_EQUAL(f.times[1], knots[1], 1e-12);
  EXPECT_DOUBLES_EQUAL(f.times[4], knots[2], 1e-12);
  EXPECT_DOUBLES_EQUAL(f.times.back(), knots.back(), 1e-12);
  EXPECT(assert_equal(result.contact.poses.front(), Pose(result.values, 0, 0),
                      1e-10));
  EXPECT(assert_equal(result.contact.poses.back(), Pose(result.values, 0, 5),
                      1e-10));
}

// Central and clipped endpoint differences use their actual elapsed times,
// and reject invalid differentiation steps and nonincreasing timestamps.
TEST(KinodynamicInitializer, AccelerationStencilAndInputValidation) {
  Fixture f;
  const auto fit =
      ContactInterpolationInitializer(f.robot, f.options().kinematics)
          .run("base", f.anchor, f.measured, f.trajectory, f.slices, f.times,
               .02, {Slice(0), Slice(2), Slice(5)});
  for (double time : {f.times.front(), f.times[2], f.times.back()}) {
    const auto priors =
        gpAccelerationPriors(*fit.interpolation, 0, time, 8, .02);
    const double low = std::max(f.times.front(), time - .02),
                 high = std::min(f.times.back(), time + .02);
    const auto first = fit.interpolation->interpolate(low),
               last = fit.interpolation->interpolate(high);
    const Vector6 expected = (last.baseTwist - first.baseTwist) / (high - low);
    EXPECT(assert_equal(expected, TwistAccel(priors, 0, 8), 1e-12));
  }
  CHECK_EXCEPTION(gpAccelerationPriors(*fit.interpolation, 0, f.times[0], 0, 0),
                  std::invalid_argument);
  f.times[1] = f.times[0];
  CHECK_EXCEPTION(
      ContactInterpolationInitializer(f.robot).run(
          "base", f.anchor, f.measured, f.trajectory, f.slices, f.times, .02),
      std::invalid_argument);
}
}  // namespace layered_initialization
/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
