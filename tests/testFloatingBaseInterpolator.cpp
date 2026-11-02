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
  gp.solve();
  DOUBLES_EQUAL(.5, gp.interpolate(.5).basePose.x(), 1e-9);
  DOUBLES_EQUAL(1, gp.interpolate(.5).baseTwist[3], 1e-9);
  EXPECT_LONGS_EQUAL(0, gp.interpolate(.5).jointAngles.size());
  EXPECT(gp.values(.5).exists(PoseKey(3, 0)));
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
  gp.solve();
  for (double time : {0., .1, .17, .42, .58, .85, 1.}) {
    const auto state = gp.interpolate(time);
    EXPECT(assert_equal(anchor * Pose3::Expmap(time * twist), state.basePose,
                        1e-8));
    EXPECT(assert_equal(twist, state.baseTwist, 1e-8));
    DOUBLES_EQUAL(.2 + .7 * time, state.jointAngles[0], 1e-9);
    DOUBLES_EQUAL(.7, state.jointVelocities[0], 1e-9);
    const auto values = gp.values(time, 5);
    EXPECT(values.exists(PoseKey(9, 5)));
    EXPECT(assert_equal(state.baseTwist, values.at<Vector6>(TwistKey(3, 5))));
    DOUBLES_EQUAL(.7, values.atDouble(JointVelKey(7, 5)), 1e-9);
  }
  EXPECT_LONGS_EQUAL(7, gp.jointIds()[0]);
  DOUBLES_EQUAL(0, gp.processError(), 1e-12);
}

// Dense intermediate observations constrain knot velocities without moving
// fixed configurations. The inferred velocity is the derivative of the mean.
TEST(FloatingBaseInterpolator, DenseMeasurementsAndMeanDerivative) {
  auto gp = interpolator();
  gp.addKnot(0, Pose3(), Vector::Zero(1));
  gp.addKnot(1, Pose3(), Vector::Zero(1));
  gp.solve();
  const double baseline = .2;
  gp.addJointMeasurement(.25, Vector::Constant(1, baseline));
  gp.addJointMeasurement(.75, Vector::Constant(1, baseline));
  gp.solve();
  EXPECT(gp.jointMeasurementRms() < baseline);
  DOUBLES_EQUAL(0, gp.interpolate(0).jointAngles[0], 0);
  DOUBLES_EQUAL(0, gp.interpolate(1).jointAngles[0], 0);
  const double time = .4, epsilon = 1e-5;
  const Vector derivative = (gp.interpolate(time + epsilon).jointAngles -
                             gp.interpolate(time - epsilon).jointAngles) /
                            (2 * epsilon);
  EXPECT(assert_equal(derivative, gp.interpolate(time).jointVelocities, 1e-8));
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
  gp.solve();
  const double epsilon = 1e-6;
  for (double time : {.1, .299, .301, .6}) {
    const Pose3 before = gp.interpolate(time - epsilon).basePose;
    const Pose3 after = gp.interpolate(time + epsilon).basePose;
    const Vector6 derivative =
        Pose3::Logmap(before.between(after)) / (2 * epsilon);
    EXPECT(assert_equal(derivative, gp.interpolate(time).baseTwist, 1e-6));
  }
  EXPECT(assert_equal(gp.interpolate(.3 - epsilon).baseTwist,
                      gp.interpolate(.3 + epsilon).baseTwist, 1e-4));
}

// World linear-velocity observations are rotated into body-twist constraints;
// invalid dimensions, times, and stale queries fail explicitly.
TEST(FloatingBaseInterpolator, VelocityObservationsAndValidation) {
  auto gp = interpolator();
  CHECK_EXCEPTION(gp.interpolate(0), std::logic_error);
  CHECK_EXCEPTION(gp.addKnot(0, Pose3(), Vector::Zero(2)),
                  std::invalid_argument);
  const Pose3 pose(Rot3::Rz(std::acos(-1.0) / 2), Point3(0, 0, 0));
  gp.addKnot(0, pose, Vector::Zero(1));
  CHECK_EXCEPTION(gp.addKnot(0, pose, Vector::Zero(1)), std::invalid_argument);
  gp.addKnot(1, pose, Vector::Zero(1));
  CHECK_EXCEPTION(gp.addJointMeasurement(2, Vector::Zero(1)),
                  std::out_of_range);
  gp.addBaseVelocityMeasurement(0, Vector3(1, 0, 0), 1e-4);
  gp.solve();
  EXPECT(assert_equal(
      Vector3(1, 0, 0),
      pose.rotation().rotate(gp.interpolate(0).baseTwist.tail<3>()), 1e-6));
  CHECK_EXCEPTION(gp.interpolate(-.1), std::out_of_range);
  gp.addJointMeasurement(.5, Vector::Zero(1));
  CHECK_EXCEPTION(gp.interpolate(.5), std::logic_error);
}
}  // namespace floating_base_interpolation
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}
