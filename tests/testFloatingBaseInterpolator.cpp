/* ----------------------------------------------------------------------------
 * GTDynamics Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <CppUnitLite/TestHarness.h>
#include <gtdynamics/universal_robot/RevoluteJoint.h>
#include <gtdynamics/utils/FloatingBaseInterpolator.h>
#include <gtdynamics/utils/values.h>
#include <gtsam/base/TestableAssertions.h>

using namespace gtdynamics;
using namespace gtsam;

/* ************************************************************************* */
namespace floating_base_interpolation {
Robot robot() {
  auto root = std::make_shared<Link>(3, "base", 1, Matrix3::Identity(), Pose3(),
                                     Pose3());
  auto child = std::make_shared<Link>(9, "tool", 1, Matrix3::Identity(),
                                      Pose3(), Pose3());
  auto joint = std::make_shared<RevoluteJoint>(7, "hinge", Pose3(), root, child,
                                               Vector3::UnitZ());
  root->addJoint(joint);
  child->addJoint(joint);
  return Robot({{"base", root}, {"tool", child}}, {{"hinge", joint}});
}

FloatingBaseInterpolator interpolator() {
  // Rotational process standard deviation: 30 degrees/s^(3/2).
  const double radiansPerDegree = std::acos(-1.0) / 180;
  return FloatingBaseInterpolator(
      robot(), "base", Vector6::Ones(),
      Vector::Constant(1, std::pow(30 * radiansPerDegree, 2)),
      Vector::Constant(1, 0.5 * radiansPerDegree));
}

// A floating robot may consist of a single free rigid body and no joints.
TEST(FloatingBaseInterpolator, FreeBodyWithoutJoints) {
  auto root = std::make_shared<Link>(3, "base", 1, Matrix3::Identity(), Pose3(),
                                     Pose3());
  const Robot body({{"base", root}}, {});
  FloatingBaseInterpolator gp(body, "base", Vector6::Ones(), Vector(),
                              Vector());
  gp.addKnot(0, Pose3(), Vector());
  gp.addKnot(1, Pose3(Rot3(), Point3(1, 0, 0)), Vector());
  const auto result = gp.solve();
  DOUBLES_EQUAL(.5, result.interpolate(.5).basePose.x(), 1e-9);
  DOUBLES_EQUAL(1, result.interpolate(.5).baseTwist[3], 1e-9);
  EXPECT_LONGS_EQUAL(0, result.interpolate(.5).jointAngles.size());
  EXPECT(result.values(.5).exists(PoseKey(3, 0)));
}

// Irregular knots exactly reproduce constant body twist and scalar joint rate;
// noncontiguous link/joint IDs survive the native FK output.
TEST(FloatingBaseInterpolator, ConstantMotionAndFrames) {
  auto gp = interpolator();
  const Pose3 anchor(Rot3::RzRyRx(.2, -.1, .4), Point3(1, 2, 3));
  const Vector6 twist = (Vector6() << 0, 0, .3, .4, -.1, .2).finished();
  for (double time : {0., .17, .58, 1.})
    gp.addKnot(time, anchor * Pose3::Expmap(time * twist),
               Vector::Constant(1, .2 + .7 * time));
  const auto result = gp.solve();
  for (double time : {0., .1, .17, .42, .58, .85, 1.}) {
    const auto state = result.interpolate(time);
    EXPECT(assert_equal(anchor * Pose3::Expmap(time * twist), state.basePose,
                        1e-8));
    EXPECT(assert_equal(twist, state.baseTwist, 1e-8));
    DOUBLES_EQUAL(.2 + .7 * time, state.jointAngles[0], 1e-9);
    DOUBLES_EQUAL(.7, state.jointVelocities[0], 1e-9);
    const auto values = result.values(time, 5);
    EXPECT(values.exists(PoseKey(9, 5)));
    EXPECT(assert_equal(state.baseTwist, values.at<Vector6>(TwistKey(3, 5))));
    DOUBLES_EQUAL(.7, values.atDouble(JointVelKey(7, 5)), 1e-9);
  }
  EXPECT_LONGS_EQUAL(7, result.jointIds()[0]);
  DOUBLES_EQUAL(0, result.processError(), 1e-12);
}

// Dense intermediate observations constrain knot velocities without moving
// fixed configurations. The inferred velocity is the derivative of the mean.
TEST(FloatingBaseInterpolator, DenseMeasurementsAndMeanDerivative) {
  auto gp = interpolator();
  gp.addKnot(0, Pose3(), Vector::Zero(1));
  gp.addKnot(1, Pose3(), Vector::Zero(1));
  const double baseline = .2;
  gp.addJointMeasurement(.25, Vector::Constant(1, baseline));
  gp.addJointMeasurement(.75, Vector::Constant(1, baseline));
  const auto result = gp.solve();
  EXPECT(result.jointMeasurementRms() < baseline);
  DOUBLES_EQUAL(0, result.interpolate(0).jointAngles[0], 0);
  DOUBLES_EQUAL(0, result.interpolate(1).jointAngles[0], 0);
  const double time = .4, epsilon = 1e-5;
  const Vector derivative = (result.interpolate(time + epsilon).jointAngles -
                             result.interpolate(time - epsilon).jointAngles) /
                            (2 * epsilon);
  EXPECT(
      assert_equal(derivative, result.interpolate(time).jointVelocities, 1e-8));
}

// Noncommuting orientation changes require right-Jacobian transport: the
// returned body twist must differentiate the actual Pose3 conditional mean.
TEST(FloatingBaseInterpolator, PoseMeanDerivativeAndKnotContinuity) {
  auto gp = interpolator();
  gp.addKnot(0, Pose3(), Vector::Zero(1));
  gp.addKnot(.3, Pose3(Rot3::RzRyRx(.2, -.1, .3), Point3(.2, .1, .05)),
             Vector::Zero(1));
  gp.addKnot(.8, Pose3(Rot3::RzRyRx(-.1, .3, .5), Point3(.4, .1, .1)),
             Vector::Zero(1));
  const auto result = gp.solve();
  const double epsilon = 1e-6;
  for (double time : {.1, .299, .301, .6}) {
    const Pose3 before = result.interpolate(time - epsilon).basePose;
    const Pose3 after = result.interpolate(time + epsilon).basePose;
    const Vector6 derivative =
        Pose3::Logmap(before.between(after)) / (2 * epsilon);
    EXPECT(assert_equal(derivative, result.interpolate(time).baseTwist, 1e-6));
  }
  EXPECT(assert_equal(result.interpolate(.3 - epsilon).baseTwist,
                      result.interpolate(.3 + epsilon).baseTwist, 1e-4));
}

// World linear-velocity observations are rotated into body-twist constraints;
// invalid input dimensions and query times fail explicitly.
TEST(FloatingBaseInterpolator, VelocityObservationsAndValidation) {
  auto gp = interpolator();
  CHECK_EXCEPTION(gp.solve(), std::invalid_argument);
  CHECK_EXCEPTION(gp.addKnot(0, Pose3(), Vector::Zero(2)),
                  std::invalid_argument);
  const Pose3 pose(Rot3::Rz(std::acos(-1.0) / 2), Point3(0, 0, 0));
  gp.addKnot(0, pose, Vector::Zero(1));
  CHECK_EXCEPTION(gp.addKnot(0, pose, Vector::Zero(1)), std::invalid_argument);
  gp.addKnot(1, pose, Vector::Zero(1));
  CHECK_EXCEPTION(gp.addJointMeasurement(2, Vector::Zero(1)),
                  std::out_of_range);
  gp.addBaseVelocityMeasurement(0, Vector3(1, 0, 0), 1e-4);
  const auto result = gp.solve();
  EXPECT(assert_equal(
      Vector3(1, 0, 0),
      pose.rotation().rotate(result.interpolate(0).baseTwist.tail<3>()), 1e-6));
  CHECK_EXCEPTION(result.interpolate(-.1), std::out_of_range);
  CHECK_EXCEPTION(gp.addBaseVelocityMeasurement(.5, Vector3::Zero(), .1),
                  std::invalid_argument);
}

// Solving through a const problem returns independent snapshots: later
// measurements, new knots, and repeated solves leave earlier results intact.
TEST(FloatingBaseInterpolator, IndependentSolveResults) {
  auto problem = interpolator();
  problem.addKnot(0, Pose3(), Vector::Zero(1));
  problem.addKnot(1, Pose3(), Vector::Zero(1));
  const auto& inputs = problem;
  const auto first = inputs.solve();
  const double firstTime = first.solveSeconds();
  problem.addJointMeasurement(.5, Vector::Constant(1, .2));
  const auto second = inputs.solve();
  const auto repeated = inputs.solve();
  EXPECT(second.interpolate(.5).jointAngles[0] > 0);
  EXPECT(assert_equal(second.interpolate(.5).jointVelocities,
                      repeated.interpolate(.5).jointVelocities));
  EXPECT(assert_equal(second.interpolate(.5).jointAngles,
                      repeated.interpolate(.5).jointAngles));
  EXPECT(second.processError() > first.processError());
  problem.addKnot(2, Pose3(), Vector::Zero(1));
  const auto third = inputs.solve();
  EXPECT_LONGS_EQUAL(2, first.knotTimes().size());
  EXPECT_LONGS_EQUAL(2, second.knotTimes().size());
  EXPECT_LONGS_EQUAL(3, third.knotTimes().size());
  CHECK_EXCEPTION(first.interpolate(1.5), std::out_of_range);
  DOUBLES_EQUAL(0, first.interpolate(.5).jointAngles[0], 0);
  DOUBLES_EQUAL(0, first.processError(), 0);
  DOUBLES_EQUAL(0, first.jointMeasurementRms(), 0);
  DOUBLES_EQUAL(firstTime, first.solveSeconds(), 0);
  auto sample = second.interpolate(.5);
  const double angle = sample.jointAngles[0];
  sample.jointAngles[0] = 100;
  DOUBLES_EQUAL(angle, second.interpolate(.5).jointAngles[0], 0);
}

// Interleaving knot and measurement additions builds the same estimation
// problem as adding all knots first, without buffering raw measurements.
TEST(FloatingBaseInterpolator, InterleavedFactorConstruction) {
  auto batch = interpolator(), incremental = interpolator();
  for (double time : {0., .3, 1.}) {
    const Pose3 pose(Rot3::Rz(.2 * time), Point3(.4 * time, 0, 0));
    const Vector angle = Vector::Constant(1, .7 * time);
    batch.addKnot(time, pose, angle);
    incremental.addKnot(time, pose, angle);
    incremental.addBaseVelocityMeasurement(time, Vector3(.4, 0, 0), .1);
    if (time > 0)
      incremental.addJointMeasurement(
          time == .3 ? .15 : .65, Vector::Constant(1, time == .3 ? .2 : .3));
  }
  batch.addJointMeasurement(.15, Vector::Constant(1, .2));
  batch.addJointMeasurement(.65, Vector::Constant(1, .3));
  for (double time : {0., .3, 1.})
    batch.addBaseVelocityMeasurement(time, Vector3(.4, 0, 0), .1);
  const auto first = batch.solve(), second = incremental.solve();
  for (double time : {0., .15, .3, .65, 1.}) {
    const auto a = first.interpolate(time), b = second.interpolate(time);
    EXPECT(assert_equal(a.basePose, b.basePose, 1e-9));
    EXPECT(assert_equal(a.baseTwist, b.baseTwist, 1e-9));
    EXPECT(assert_equal(a.jointAngles, b.jointAngles, 1e-9));
    EXPECT(assert_equal(a.jointVelocities, b.jointVelocities, 1e-9));
  }
  DOUBLES_EQUAL(first.jointMeasurementRms(), second.jointMeasurementRms(),
                1e-12);
  DOUBLES_EQUAL(first.processError(), second.processError(), 1e-9);
}

// The diagnostic uses physical-unit residuals from the stored encoder factors
// and includes exact-knot measurements even though they add no velocity factor.
TEST(FloatingBaseInterpolator, EncoderRmsIncludesFixedKnots) {
  auto problem = interpolator();
  problem.addKnot(0, Pose3(), Vector::Zero(1));
  problem.addKnot(1, Pose3(), Vector::Constant(1, .7));
  problem.addJointMeasurement(0, Vector::Constant(1, .1));
  problem.addJointMeasurement(.25, Vector::Constant(1, .3));
  problem.addJointMeasurement(1, Vector::Constant(1, .9));
  const auto result = problem.solve();
  const double interiorError = result.interpolate(.25).jointAngles[0] - .3;
  const double expected =
      std::sqrt((.1 * .1 + interiorError * interiorError + .2 * .2) / 3);
  DOUBLES_EQUAL(expected, result.jointMeasurementRms(), 1e-12);
  EXPECT(result.graphConstructionSeconds() > 0);
}

// A result keeps its numerical data and robot model alive after the local
// problem is destroyed, including the model needed for native FK queries.
TEST(FloatingBaseInterpolator, ResultOutlivesProblem) {
  const auto result = [] {
    auto problem = interpolator();
    problem.addKnot(0, Pose3(), Vector::Zero(1));
    problem.addKnot(1, Pose3(), Vector::Constant(1, .7));
    return problem.solve();
  }();
  DOUBLES_EQUAL(.35, result.interpolate(.5).jointAngles[0], 1e-9);
  const auto values = result.values(.5, 8);
  EXPECT(values.exists(PoseKey(9, 8)));
  DOUBLES_EQUAL(.7, values.atDouble(JointVelKey(7, 8)), 1e-9);
  EXPECT_LONGS_EQUAL(7, result.jointIds()[0]);
  EXPECT(result.jointNames()[0] == "hinge");
}

}  // namespace floating_base_interpolation
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
